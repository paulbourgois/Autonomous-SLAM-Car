#include "sensors_pkg/sheer_data_node.hpp"

SheerDataNode::SheerDataNode() 
    : Node("lidar_sheer_data_node"), 
      serial_(io_), 
      baud_rate_(230400),
      port_("/dev/ttyUSB1")
{
    // Création du publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("lidar_sheer_data_topic", 10);

    // Ouvre le port série
    try {
        serial_.open(port_); // Vérifiez le port série correct dans platformio une fois l'arduino connectée
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        read_serial();
        std::thread([this]() {io_.run(); }).detach();
        RCLCPP_INFO(this->get_logger(), "Port série ouvert sur %s à %d bauds", port_.c_str(), baud_rate_);
    } catch (std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Erreur de connexion série : %s", e.what());
    }
}

void SheerDataNode::read_serial() {
    auto buf = std::make_shared<std::vector<char>>(100);  // Buffer plus grand pour accumuler les données
    serial_.async_read_some(boost::asio::buffer(*buf), [this, buf](std::error_code ec, std::size_t len) {
        if (ec) {
            RCLCPP_ERROR(this->get_logger(), "Erreur de lecture série : %s", ec.message().c_str());
            return;
        }
        if (!ec) {
            process_buffer(*buf, len);
            read_serial();
        }
    });
}

uint8_t SheerDataNode::CalCRC8(uint8_t *p, uint8_t len) {
    uint8_t crc = 0;
    uint16_t i;

    for (i = 0; i < len; i++) {
        crc = CrcTable[(crc ^ *p++) & 0xff];
    }

    return crc;
}


void SheerDataNode::process_buffer(const std::vector<char>& buf, size_t len) {
    static std::vector<char> accumulated_data;
    accumulated_data.insert(accumulated_data.end(), buf.begin(), buf.begin() + len);
    const uint8_t START_MARKER = 0x54;
    
    while (true) {
        // Cherche le prochain marqueur de début
        auto marker_pos = std::find(accumulated_data.begin(), accumulated_data.end(), START_MARKER);
        if (marker_pos == accumulated_data.end()) {
            // Pas de début de trame trouvé, on attend plus de données
            break;
        }
        // On cherche le prochain marqueur après celui trouvé (pour délimiter la trame)
        auto next_marker = std::find(marker_pos + 1, accumulated_data.end(), START_MARKER);
        if (next_marker == accumulated_data.end()) {
            // Pas de fin de trame trouvée, on attend plus de données
            break;
        }
        // On a une trame complète entre marker_pos et next_marker
        (*total_data_received_)++;
        size_t msg_start = std::distance(accumulated_data.begin(), marker_pos);
        size_t msg_end = std::distance(accumulated_data.begin(), next_marker);
        if (msg_end - msg_start < 3) {
            // Trame trop courte pour contenir des données + CRC
            accumulated_data.erase(accumulated_data.begin(), next_marker);
            continue;
        }
        // Les données utiles sont entre marker_pos et next_marker-2, la CRC est à next_marker-1, ils ont pris en compte le header dans le calcul du CRC
        std::vector<uint8_t> payload(marker_pos, next_marker - 1);
        uint8_t received_crc = static_cast<uint8_t>(*(next_marker - 1));
        uint8_t computed_crc = CalCRC8(payload.data(), payload.size());
        if (computed_crc == received_crc) {



            // Message valide, publication
            std_msgs::msg::String msg;
            std::ostringstream oss;
            for (auto b : payload) {
                oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)b;
            }

            std::string trame = oss.str();
            trame = trame.substr(4); // Enlève les 4 premiers caractères (START_MARKER et la var_len fixé à 2C)

            uint8_t timestamp = static_cast<uint8_t>(trame.substr(trame.length() - 2, 2)); //time stamp c'est les 2 derniers caractères
            uint8_t endAngle = static_cast<uint8_t>(trame.substr(trame.length() - 4, 2)); //end angle c'est les 2 avant derniers caractères

            if (trame.length() >= 16) { // 8 octets = 16 caractères hexadécimaux
                std::string last_8_bytes = trame.substr(trame.length() - 16, 16);
                msg.data = last_8_bytes;
            } else {
                msg.data = trame; // Si la trame est trop courte, on publie tout
            }

            publisher_->publish(msg);




        } else {
            (*total_data_not_ok_)++;
            *error_rate_ = static_cast<double>(*total_data_not_ok_) / (*total_data_received_);
            RCLCPP_WARN(this->get_logger(), "Taux d'erreurs: %f", *error_rate_);
            RCLCPP_WARN(this->get_logger(), "Trame CRC invalide: attendu %02X, reçu %02X", computed_crc, received_crc);
        }
        // On retire la trame traitée du buffer
        accumulated_data.erase(accumulated_data.begin(), next_marker);
    }
    // Limite de sécurité sur la taille du buffer
    const size_t MAX_BUFFER_SIZE = 8192;
    if (accumulated_data.size() > MAX_BUFFER_SIZE) {
        accumulated_data.clear();
        RCLCPP_WARN(this->get_logger(), "Buffer overflow - données effacées");
    }
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SheerDataNode>());
    rclcpp::shutdown();
    return 0;
}