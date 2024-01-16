#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("tutorial_get_path")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/tutorial/topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::TimerCallback, this));
  }

private:
  void TimerCallback()
  {
    auto message = std_msgs::msg::String();
    message.data = std::string(ALICE3_PARAMETERS_PATH) + "/config/step_parameter.yaml";
    RCLCPP_INFO(this->get_logger(), "Path : '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
