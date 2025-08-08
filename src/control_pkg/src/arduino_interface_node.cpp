#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/asio.hpp>

using std::placeholders::_1;

class MotorCommandNode : public rclcpp::Node
{
public:
    MotorCommandNode() : Node("motor_command_node"), io_(), serial_(io_)
    {
        // Ouvre le port série
        try {
            serial_.open("/dev/ttyACM0"); // Aller vérifier le port série correct dans platformio une fois l'arduino connectée
            serial_.set_option(boost::asio::serial_port_base::baud_rate(115200));
            RCLCPP_INFO(this->get_logger(), "Connexion série ouverte avec Arduino.");
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Erreur de connexion série : %s", e.what());
        }

        // Création du subscriber
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "motor_cmd",
            10,
            std::bind(&MotorCommandNode::motor_cmd_callback, this, _1)
        );
    }

private:
    void motor_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (serial_.is_open()) {
            std::string command = msg->data + "\n";  // ajoute un retour ligne si besoin
            boost::asio::write(serial_, boost::asio::buffer(command));
            RCLCPP_INFO(this->get_logger(), "Commande envoyée : '%s'", msg->data.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Port série non ouvert.");
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorCommandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
