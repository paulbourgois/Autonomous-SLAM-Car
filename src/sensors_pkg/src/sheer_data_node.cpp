#include "sensors_pkg/sheer_data_node.hpp"

SheerDataNode::SheerDataNode() 
    : Node("lidar_data_publisher"), 
      serial_(io_), 
      baud_rate_(230400),
      port_("/dev/ttyUSB0")
{
    // Déclaration des paramètres ros2 pour les utiliser avec un fichier de configuration YAML pour launch
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 230400);
    this->declare_parameter<std::string>("frame_id", "laser_frame");
    // Donc on récupère les paramètres choisis
    this->get_parameter("serial_port", port_);
    this->get_parameter("baudrate", baud_rate_);
    this->get_parameter("frame_id", frame_id);

    // Création du publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

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

SheerDataNode::DecodedData SheerDataNode::DecodeData(const std::string& trame){
    // Extraction des données de la trametrame = trame.substr(4); // Enlève les 4 premiers caractères (START_MARKER et la var_len fixé à 2C)
    std::string timestamp_str = trame.substr(trame.length() - 4, 4);
    std::string end_angle_str = trame.substr(trame.length() - 8, 4);
    std::string radar_speed_str = trame.substr(0, 4);
    std::string start_angle_str = trame.substr(4, 4);
    std::string data_str = trame.substr(8, trame.length() - 8);
    int number_of_measuring_points = data_str.length() / 6; // Chaque point mesure 3 octets soit 6 caractères en hexa
    // RCLCPP_INFO(this->get_logger(), "End angle reçue : %s", end_angle_str.c_str());
    // RCLCPP_INFO(this->get_logger(), "Timestamp reçue : %s", timestamp_str.c_str());

    timestamp_str = timestamp_str.substr(2) + timestamp_str.substr(0, 2); // Correction de la longueur, on les mets dans le bon ordre (c'est envoyé : LSB, MSB) on décode en MSB, LSB
    end_angle_str = end_angle_str.substr(2) + end_angle_str.substr(0, 2); // Correction de la longueur, pareil que pour le timestamp
    radar_speed_str = radar_speed_str.substr(2) + radar_speed_str.substr(0, 2); // Correction de la longueur, pareil que pour le timestamp
    start_angle_str = start_angle_str.substr(2) + start_angle_str.substr(0, 2); // Correction de la longueur, pareil que pour le timestamp

    std::vector<std::pair<uint16_t, uint8_t>> data;
    for (int i=0; i < number_of_measuring_points; i++) {
        std::string point = data_str.substr(i * 6, 6);
        point = point.substr(2) + point.substr(0, 2) + point.substr(4, 2); // Correction de la longueur, pareil que pour le timestamp
        uint16_t distance_value = std::stoi(point.substr(0, 4), nullptr, 16); // Les 2 premiers octets sont la distance
        uint8_t signal_strength = std::stoi(point.substr(4, 2), nullptr, 16); // Les 2 derniers octets sont la force du signal
        data.push_back(std::make_pair(distance_value, signal_strength));
        // RCLCPP_INFO(this->get_logger(), "Point %d: Distance: %d, Signal Strength: %d", i, distance_value, signal_strength);
        
    }
    // RCLCPP_INFO(this->get_logger(), "Radar speed corrigé: %s", radar_speed_str.c_str());
    // RCLCPP_INFO(this->get_logger(), "Start angle corrigé: %s", start_angle_str.c_str());
    // RCLCPP_INFO(this->get_logger(), "End angle corrigé: %s", end_angle_str.c_str());
    // RCLCPP_INFO(this->get_logger(), "Timestamp corrigé: %s", timestamp_str.c_str());

    uint16_t timestamp = std::stoi(timestamp_str, nullptr, 16); //time stamp c'est les 2 derniers octets, donc 4 derniers caractères en hexa
    uint16_t endAngle = std::stoi(end_angle_str, nullptr, 16); //end angle c'est pareil avec les 2 avant derniers caractères
    uint16_t radarSpeed = std::stoi(radar_speed_str, nullptr, 16); // radar speed c'est les 2 premiers caractères
    uint16_t startAngle = std::stoi(start_angle_str, nullptr, 16);
    // RCLCPP_INFO(this->get_logger(), "Start Angle: %d, End Angle: %d", startAngle, endAngle);

    SheerDataNode::DecodedData decoded_data = {
        timestamp,
        endAngle,
        radarSpeed,
        startAngle,
        data,
    };
    return decoded_data;
    }

sensor_msgs::msg::LaserScan SheerDataNode::FromRawToLaserScan(const SheerDataNode::DecodedData& data) {
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.frame_id = frame_id; // Assurez-vous de définir le frame_id approprié
    scan_msg.angle_min = static_cast<float>(data.startAngle*M_PI/(100.0*180.0)); // Convertir en angle depuis des 0.01 angle puis radians
    scan_msg.angle_max = static_cast<float>(data.endAngle*M_PI/(100.0*180.0)); // Convertir en angle depuis des 0.01 angle puis radians
    scan_msg.range_min = 0.0; // Définir la distance minimale
    scan_msg.range_max = 100.0; // Définir la distance maximale
    // RCLCPP_INFO(this->get_logger(), "Timestamp: %d, Start Angle: %d, End Angle: %d, Radar Speed: %d", 
    //             data.timestamp, data.startAngle, data.endAngle, data.radarSpeed);
    if (data.timestamp >= last_timestamp_) {
        scan_msg.scan_time = (data.timestamp - last_timestamp_) / 1000.0; // Convertir le timestamp en secondes
    } else {
        // Gestion du débordement (overflow) si le compteur repart à 0 après 30000 ms
        scan_msg.scan_time = (30000 - last_timestamp_ + data.timestamp) / 1000.0; // Convertir le timestamp en secondes
    }
    scan_msg.time_increment = scan_msg.scan_time / data.points.size(); // Temps entre chaque point
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / data.points.size(); // Angle entre chaque point

    for (const auto& point : data.points) {
        scan_msg.ranges.push_back(static_cast<float>(point.first) / 1000.0);
        scan_msg.intensities.push_back(static_cast<float>(point.second));
    }

    return scan_msg;
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
            std::ostringstream oss;
            for (auto b : payload) {
                oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)b;
            }
            std::string trame = oss.str();


            if (trame.length() >= 12) {
                SheerDataNode::DecodedData data = DecodeData(trame);
                sensor_msgs::msg::LaserScan msg = FromRawToLaserScan(data);
                last_timestamp_ = data.timestamp; // Mettre à jour le dernier timestamp
                publisher_->publish(msg);

            } else {
                // RCLCPP_WARN(this->get_logger(), "Trame trop courte pour extraire le timestamp");
            }
            

            




        } else {
            (*total_data_not_ok_)++;
            *error_rate_ = static_cast<double>(*total_data_not_ok_) / (*total_data_received_);
            // RCLCPP_WARN(this->get_logger(), "Taux d'erreurs: %f", *error_rate_);
            // RCLCPP_WARN(this->get_logger(), "Trame CRC invalide: attendu %02X, reçu %02X", computed_crc, received_crc);
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