#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/asio.hpp>

class SheerDataNode : public rclcpp::Node {
public:
    SheerDataNode();
private:
    
    

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    unsigned int baud_rate_;
    std::string port_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    void read_serial();
    void process_buffer(const std::vector<char>& buf, size_t len);
};