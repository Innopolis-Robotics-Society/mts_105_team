#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class EkfNavigator : public rclcpp::Node{
public:
  EkfNavigator(): Node("ekf_navigator"){
    goal_x_ = declare_parameter("goal_x", 0.0);
    goal_y_ = declare_parameter("goal_y", 0.0);
    k_lin_  = declare_parameter("k_lin", 0.8);
    k_ang_  = declare_parameter("k_ang", 2.0);
    vmax_   = declare_parameter("vmax",  0.5);
    wmax_   = declare_parameter("wmax",  1.2);
    arrive_dist_ = declare_parameter("arrive_dist", 0.1);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", 10,
      std::bind(&EkfNavigator::onOdom, this, std::placeholders::_1));
    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
private:
  double goal_x_, goal_y_, k_lin_, k_ang_, vmax_, wmax_, arrive_dist_;

  static inline double yawFromQuat(const geometry_msgs::msg::Quaternion& q){
    return std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
  }
  static inline double sat(double v,double m){ return std::copysign(std::min(std::fabs(v),m),v); }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double th= yawFromQuat(msg->pose.pose.orientation);

    double dx = goal_x_-x, dy=goal_y_-y;
    double dist = std::hypot(dx,dy);
    geometry_msgs::msg::Twist u;
    if(dist < arrive_dist_){
      pub_->publish(u);
      return;
    }
    double th_goal = std::atan2(dy,dx);
    double e_th = std::atan2(std::sin(th_goal-th), std::cos(th_goal-th));

    double v = k_lin_*dist;
    double w = k_ang_*e_th;
    u.linear.x  = sat(v, vmax_);
    u.angular.z = sat(w, wmax_);
    pub_->publish(u);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<EkfNavigator>());
  rclcpp::shutdown();
  return 0;
}
