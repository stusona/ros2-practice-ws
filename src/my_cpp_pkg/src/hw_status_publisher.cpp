#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"
 
class HardwareStatusPublisherNode : public rclcpp::Node
{
public:
    HardwareStatusPublisherNode() : Node("hardware_status_publisher")
    {
        pub_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>(
            "hardware_status", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HardwareStatusPublisherNode::publishHardwareStatus, this));
        RCLCPP_INFO(this->get_logger(), "Template node has started");
    }
 
private:
    void publishHardwareStatus()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 45;
        msg.are_motors_ready = false;
        msg.debug_message = "Motors are a tad spicy";
        pub_->publish(msg);
    }

    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // initialize something?
    auto node = std::make_shared<HardwareStatusPublisherNode>(); // create node
    rclcpp::spin(node); // spin to allow for non-blocking waiting
    rclcpp::shutdown(); 
    return 0;
}