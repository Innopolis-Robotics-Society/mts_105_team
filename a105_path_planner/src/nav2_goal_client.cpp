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
    // goal params
    this->declare_parameter<double>("goal_x", 0.0);
    this->declare_parameter<double>("goal_y", 0.0);
    this->declare_parameter<double>("goal_yaw", 0.0);
    this->declare_parameter<std::string>("goal_frame", "map");

    // retry params
    repeat_sec_ = this->declare_parameter<double>("repeat_sec", 10.0);
    retry_on_aborted_  = this->declare_parameter<bool>("retry_on_aborted",  true);
    retry_on_canceled_ = this->declare_parameter<bool>("retry_on_canceled", true);
    retry_if_succeeded_= this->declare_parameter<bool>("retry_if_succeeded", false);
    max_retries_ = this->declare_parameter<int>("max_retries", 0); // 0 = бесконечно

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    wait_for_nav2();

    // первый пуск
    send_goal_from_params();

    // периодический контроль результата и возможный рестарт
    retry_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(repeat_sec_),
      std::bind(&GoClient::maybe_resend, this));
  }

private:
  void wait_for_nav2() {
    RCLCPP_INFO(get_logger(), "Waiting for navigate_to_pose server...");
    client_->wait_for_action_server();
  }

  void maybe_resend() {
    if (awaiting_) return;               // ещё идёт текущая миссия
    // Решаем, надо ли повторять
    if (last_code_ == rclcpp_action::ResultCode::SUCCEEDED && !retry_if_succeeded_) return;
    if (last_code_ == rclcpp_action::ResultCode::CANCELED  && !retry_on_canceled_)  return;
    if (last_code_ == rclcpp_action::ResultCode::ABORTED   && !retry_on_aborted_)   return;
    if (max_retries_ > 0 && retries_done_ >= max_retries_) return;

    RCLCPP_WARN(get_logger(), "Resending goal (retry #%d)", retries_done_+1);
    retries_done_++;
    send_goal_from_params();
  }

  bool send_goal_from_params() {
    double x{}, y{}, yaw{}; std::string frame;
    get_parameter("goal_x", x); get_parameter("goal_y", y);
    get_parameter("goal_yaw", yaw); get_parameter("goal_frame", frame);
    return send_goal(x, y, yaw, frame);
  }

  static void yaw_to_quat(double yaw, double &qz, double &qw) {
    const double h = 0.5 * yaw; qz = std::sin(h); qw = std::cos(h);
  }

  bool send_goal(double x, double y, double yaw_rad, const std::string &frame) {
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = frame;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    double qz{}, qw{}; yaw_to_quat(yaw_rad, qz, qw);
    goal.pose.pose.orientation.z = qz; goal.pose.pose.orientation.w = qw;

    typename rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback = [this](auto handle) {
      if (!handle) { RCLCPP_ERROR(get_logger(), "Goal rejected"); awaiting_ = false; }
      else { RCLCPP_INFO(get_logger(), "Goal accepted"); awaiting_ = true; }
    };
    opts.feedback_callback = [this](auto, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
      (void)fb; // при желании логируйте прогресс
    };
    opts.result_callback = [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &res) {
      awaiting_ = false;
      last_code_ = res.code;
      RCLCPP_INFO(get_logger(), "Result: %d", static_cast<int>(res.code));
      if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Goal reached");
      }
    };

    client_->async_send_goal(goal, opts);
    return true;
  }

  // state
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  std::atomic<bool> awaiting_{false};
  rclcpp_action::ResultCode last_code_{rclcpp_action::ResultCode::UNKNOWN};
  int retries_done_{0};

  // params
  double repeat_sec_{10.0};
  bool retry_on_aborted_{true}, retry_on_canceled_{true}, retry_if_succeeded_{false};
  int max_retries_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoClient>());
  rclcpp::shutdown();
  return 0;
}
