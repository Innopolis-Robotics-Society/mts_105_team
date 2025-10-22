#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class GoClient : public rclcpp::Node {
public:
  GoClient() : rclcpp::Node("nav2_go_client") {
    // Параметры цели
    this->declare_parameter<double>("goal_x", 0.0);
    this->declare_parameter<double>("goal_y", 0.0);
    this->declare_parameter<double>("goal_yaw", 0.0);          // радианы
    this->declare_parameter<std::string>("goal_frame", "map");  // обычно "map"

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  bool wait_for_nav2() {
    RCLCPP_INFO(get_logger(), "Waiting for navigate_to_pose server...");
    return client_->wait_for_action_server(30s);
  }

  bool send_goal_from_params() {
    double x{}, y{}, yaw{};
    std::string frame;
    this->get_parameter("goal_x", x);
    this->get_parameter("goal_y", y);
    this->get_parameter("goal_yaw", yaw);
    this->get_parameter("goal_frame", frame);
    return send_goal(x, y, yaw, frame);
  }

private:
  static void yaw_to_quat(double yaw, double &qz, double &qw) {
    const double h = 0.5 * yaw;
    qz = std::sin(h);
    qw = std::cos(h);
  }

  bool send_goal(double x, double y, double yaw_rad, const std::string &frame) {
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = frame;
    goal.pose.header.stamp = this->now();

    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    double qz{}, qw{};
    yaw_to_quat(yaw_rad, qz, qw);
    goal.pose.pose.orientation.z = qz;
    goal.pose.pose.orientation.w = qw;

    typename rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback = [this](auto handle) {
      if (!handle) RCLCPP_ERROR(this->get_logger(), "Goal rejected");
      else RCLCPP_INFO(this->get_logger(), "Goal accepted");
    };
    opts.feedback_callback =
      [this](auto, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        const auto &p = fb->current_pose.pose.position;
        RCLCPP_DEBUG(this->get_logger(), "fb x=%.3f y=%.3f", p.x, p.y);
      };
    opts.result_callback = [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &res) {
      if (res.code == rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_INFO(this->get_logger(), "Result: SUCCEEDED");
      else
        RCLCPP_ERROR(this->get_logger(), "Result: FAILED (%d)", static_cast<int>(res.code));
    };

    auto gh_future = client_->async_send_goal(goal, opts);
    if (rclcpp::spin_until_future_complete(shared_from_this(), gh_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "send_goal failed");
      return false;
    }
    auto gh = gh_future.get();
    if (!gh) return false;

    auto result_future = client_->async_get_result(gh);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "get_result failed");
      return false;
    }
    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoClient>();

  if (!node->wait_for_nav2()) {
    RCLCPP_ERROR(node->get_logger(), "Nav2 action server unavailable");
    rclcpp::shutdown();
    return 1;
  }

  node->send_goal_from_params();
  rclcpp::shutdown();
  return 0;
}
