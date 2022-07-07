#include "rclcpp/rclcpp.hpp"
 
class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
    MyCustomNode() : Node("node_name") // MODIFY NAME
    {
        RCLCPP_INFO(this->get_logger(), "Template node has started");
    }
 
private:
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // initialize something?
    auto node = std::make_shared<MyCustomNode>(); // create node MODIFY NAME
    rclcpp::spin(node); // spin to allow for non-blocking waiting
    rclcpp::shutdown(); 
    return 0;
}