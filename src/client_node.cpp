#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3)

  {
    RCLCPP_INFO(rclcpp::get_logger("tutorial_client"), "usage: add_two_ints_client X Y");
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("tutorial_client");
  rclcpp::Client<tutorial_interfaces::srv::AddTwoInts>::SharedPtr client =
      node->create_client<tutorial_interfaces::srv::AddTwoInts>("/tutorial/add_two_ints");

  auto request = std::make_shared<tutorial_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s))

  {
    if (!rclcpp::ok())

    {
      RCLCPP_ERROR(rclcpp::get_logger("tutorial_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("tutorial_client"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("tutorial_client"), "Sum: %ld", result.get()->sum);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("tutorial_client"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
