#include "sensors_pkg/sheer_data_node.hpp"

SheerDataNode::SheerDataNode() 
    : Node("lidar_sheer_data_node"), 
      serial_(io_), 
      baud_rate_(230400),
      port_("/dev/ttyUSB0")
{
    // Création du publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("lidar_sheer_data_topic", 10);

    // Ouvre le port série
    try {
        serial_.open(port_); // Vérifiez le port série correct dans platformio une fois l'arduino connectée
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        read_serial();
        std::thread([this]() {io_.run(); }).detach();
    } catch (std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Erreur de connexion série : %s", e.what());
    }
}

void SheerDataNode::read_serial() {
    auto buf = std::make_shared<std::vector<char>>(100);  // Buffer plus grand pour accumuler les données
    serial_.async_read_some(boost::asio::buffer(*buf), [this, buf](std::error_code ec, std::size_t len) {
        if (!ec) {
            process_buffer(*buf, len);
            read_serial();
        }
    });
}

void SheerDataNode::process_buffer(const std::vector<char>& buf, size_t len) {
    static std::vector<char> accumulated_data;  // Buffer pour accumuler les données entre les appels
    
    // Ajouter les nouvelles données au buffer accumulé
    accumulated_data.insert(accumulated_data.end(), buf.begin(), buf.begin() + len);

    // Chercher les marqueurs dans le buffer accumulé
    const uint8_t START_MARKER = 0x54;  // Ajustez selon votre protocole

    // Chercher le prochain marqueur
    auto marker_pos = std::find(accumulated_data.begin() , accumulated_data.end(), START_MARKER);

    if (marker_pos != accumulated_data.end()) { // Si le marqueur n'est pas trouvé, marker_pos sera égal à accumulated_data.end()
        if (marker_pos > accumulated_data.begin()) { //Si le marqueur n'est pas au début du buffer, sinon le message entre 2 est vide, il existe au moins une position pour un checksum envoyé

            RCLCPP_WARN(this->get_logger(), "On a trouvé un marqueur");
            // On vérifie que la checksum est valide donc que le message est complet
            uint8_t checksum = 0;
            for (int i =0; i < std::distance(accumulated_data.begin(), marker_pos)-1; ++i) { 
                //La checksum envoyé à l'origine est contenu sur le dernier bit, juste avant le marquer_pos, donc il ne faut pas la prendre en compte
                // D'où la strict comparaison et le moins 1
                checksum += static_cast<uint8_t>(accumulated_data[i]);
                RCLCPP_WARN(this->get_logger(), "Checksum en cours de calcul: %02X", checksum);
            };

            RCLCPP_WARN(this->get_logger(), "La checksum envoyé est: %02X", *(marker_pos-1));
            checksum += static_cast<uint8_t>(START_MARKER); // Ajouter le marqueur au checksum
            checksum &= 0xFF; // Garde 1 octet
            RCLCPP_WARN(this->get_logger(), "Checksum calculée: %02X", checksum);


            bool valid = false;
            // Vérifie que marker_pos-1 est dans le buffer
            size_t checksum_pos = std::distance(accumulated_data.begin(), marker_pos) - 1;
            if (checksum_pos < accumulated_data.size()) {
                valid = (checksum == static_cast<uint8_t>(accumulated_data[checksum_pos]));
                RCLCPP_DEBUG(this->get_logger(), "Checksum: %02X, Validité: %s", 
                                checksum, valid ? "true" : "false");
            }
            

                        

            // Publier les données entre le dernier marqueur et celui-ci
            std_msgs::msg::String msg;
            std::ostringstream oss;
            
            for (auto it = accumulated_data.begin(); it != marker_pos; ++it) {
                oss << std::hex << std::setw(2) << std::setfill('0') 
                    << (static_cast<unsigned int>(*it) & 0xFF);
            }
            
            msg.data = oss.str();
            if (!msg.data.empty()) {
                publisher_->publish(msg);
            }
            // } else {
            //     RCLCPP_WARN(this->get_logger(), "Checksum invalide pour le message reçu");

        
    } else {
         // Pas d'autre marqueur trouvé
    }
    }

    // Conserver uniquement les données après le dernier marqueur trouvé
    if (marker_pos > accumulated_data.begin()) {
        accumulated_data.erase(accumulated_data.begin(), marker_pos);
    }

    // Option: limiter la taille du buffer accumulé pour éviter une croissance infinie
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