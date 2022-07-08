#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")
    {
        // create a server with an interface of type "AddTwoInts"
        // This server runs a callback function when a client sends a request
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints", 
            std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, _2));
        // print to console
        RCLCPP_INFO(this->get_logger(), "Service server has started");
    }

private:
    // process the directions from the client
    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                            const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        // add a and b from the client request and store in "sum"
        response->sum = request->a + request->b;
        // print to console
        RCLCPP_INFO(this->get_logger(), "Server has processed: %d + %d = %d", request->a, request->b, response->sum);
    }

    // Instantiate server object
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}