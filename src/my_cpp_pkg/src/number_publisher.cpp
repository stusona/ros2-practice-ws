#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
 
class NumberPubNode : public rclcpp::Node
{
public:
    NumberPubNode() : Node("number_publisher")
    {
        // create publisher that publishes an Int64 interface on the "number" topic
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);

        // create a timer that runs "publishNews" at 2 hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&NumberPubNode::publishNews, this));

        // print to console that we've begun
        RCLCPP_INFO(this->get_logger(), "Number publisher has started");
    }
 
private:

    // publish a number
    void publishNews()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = 10;
        publisher_->publish(msg);
    }

    // publisher and timer objects
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}