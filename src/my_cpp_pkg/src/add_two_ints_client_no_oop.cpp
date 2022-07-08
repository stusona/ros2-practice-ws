#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // initialize something?
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_oop"); // create node 

    // Create client (called "add_two_ints") with an interface (or "service type") of "AddTwoInts"
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    // Wait for server to come online
    while(!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(node->get_logger(), "waiting for the server to be up");
    }

    // create request shared pointer object
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 3;
    request->b = 8;

    // send a request asynchronously
    auto future = client->async_send_request(request);
    // now wait until the future object from the request is "complete" and the flag is successful
    if(rclcpp::spin_until_future_complete(node, future) == rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "%d + %d = %d", request->a, request->b, future.get()->sum);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Error while calling service");
    }
    

    rclcpp::shutdown(); 
    return 0;
}