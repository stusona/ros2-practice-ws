#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        // subscribe to number topic and call "addAndPrint" each time a new number is published to "number" topic
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10,
            std::bind(&NumberCounterNode::addAndPrint, this, _1));

        // create publisher that publishes an Int64 interface on the "number_count" topic
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

        // create a server with an interface of type "example_interfaces/srv/SetBool"
        server_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter",
            std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));
        
        // print successful node creation to console
        RCLCPP_INFO(this->get_logger(), "Number Counter Node has started.");
    }
 
private:
    // subscriber function
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

    // function for server to reset counter if setbool request is true
    void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request, const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if(request->data)
        {
            counter_ = 0;
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Counter reset");
        }
        else
        {
            response->success = false;
            RCLCPP_INFO(this->get_logger(), "Counter has not been reset");
        }
    }

    // instantiate objects
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
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