#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
 
class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {
        // start a thread that sends request and waits for completion
        // thread1_ = std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 2)));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 3, 4)));
        RCLCPP_INFO(this->get_logger(), "Template node has started");
    }

    void callAddTwoIntsService(int a, int b)
    {
        // Create client (called "add_two_ints") with an interface (or "service type") of "AddTwoInts"
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        // Wait for server to come online
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "waiting for the server to be up");
        }

        // create request shared pointer object
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        // send a request asynchronously
        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, response->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
 
private:
    // std::thread thread1_;
    std::vector<std::thread> threads_; // vector of threads
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // initialize something?
    auto node = std::make_shared<AddTwoIntsClientNode>(); // create node
    rclcpp::spin(node); // spin to allow for non-blocking waiting
    rclcpp::shutdown(); 
    return 0;
}