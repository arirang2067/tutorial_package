#include <functional>
#include <memory>
#include <thread>

#include "tutorial_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace tutorial_package
{
  class FibonacciActionServer : public rclcpp::Node
  {
  public:
    using Fibonacci = tutorial_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    explicit FibonacciActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("fibonacci_action_server", options)
    {
      using namespace std::placeholders;

      this->action_server_ = rclcpp_action::create_server<Fibonacci>(
          this,
          "fibonacci",
          std::bind(&FibonacciActionServer::HandleGoal, this, _1, _2),
          std::bind(&FibonacciActionServer::HandleCancel, this, _1),
          std::bind(&FibonacciActionServer::HandleAccepted, this, _1));
    }

  private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

    rclcpp_action::GoalResponse HandleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse HandleCancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void HandleAccepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&FibonacciActionServer::Execute, this, _1), goal_handle}.detach();
    }

    void Execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing goal");
      rclcpp::Rate loop_rate(1);
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<Fibonacci::Feedback>();
      auto &sequence = feedback->partial_sequence;
      sequence.push_back(0);
      sequence.push_back(1);
      auto result = std::make_shared<Fibonacci::Result>();

      for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
      {
        // Check if there is a cancel request
        if (goal_handle->is_canceling())
        {
          result->sequence = sequence;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }
        // Update sequence
        sequence.push_back(sequence[i] + sequence[i - 1]);
        // Publish feedback
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        loop_rate.sleep();
      }

      // Check if goal is done
      if (rclcpp::ok())
      {
        result->sequence = sequence;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
    }
  }; // class FibonacciActionServer

} // namespace action_tutorials_cpp

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<tutorial_package::FibonacciActionServer>();
  rclcpp::spin(action_server);
  rclcpp::shutdown();
  return 0;
}
