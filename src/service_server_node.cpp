#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_two_ints.hpp"

void Add(const std::shared_ptr<tutorial_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<tutorial_interfaces::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("tutorial_server"), "Incoming request\na: %ld"
                                                     " b: %ld",
              request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("tutorial_server"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("tutorial_server");

  rclcpp::Service<tutorial_interfaces::srv::AddTwoInts>::SharedPtr service =
      node->create_service<tutorial_interfaces::srv::AddTwoInts>("/tutorial/add_two_ints", &Add);

  RCLCPP_INFO(rclcpp::get_logger("tutorial_server"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
