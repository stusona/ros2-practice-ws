#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
 
class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        // subscribe to number topic and call "addAndPrint" each time a new number is published to "number" topic
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10,
            std::bind(&NumberCounterNode::addAndPrint, this, std::placeholders::_1));

        // create publisher that publishes an Int64 interface on the "number_count" topic
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

        // print successful node creation to console
        RCLCPP_INFO(this->get_logger(), "Number Counter Node has started.");
    }
 
private:
    // subscribe
    void addAndPrint(const example_interfaces::msg::Int64::SharedPtr number)
    {
        // keep counter variable and add new number to it
        counter_ += number->data;
        RCLCPP_INFO(this->get_logger(), "%d", counter_);

        // publish counter to number_count topic
        example_interfaces::msg::Int64 new_msg;
        new_msg.data = counter_;
        publisher_->publish(new_msg);
    }

    // subscriber object
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    int counter_ = 0;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}