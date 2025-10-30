#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class A105TFBroadcaster : public rclcpp::Node {
public:
  A105TFBroadcaster() : Node("a105_tf_broadcaster") {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&A105TFBroadcaster::broadcast, this));
  }

private:
  void broadcast() {
    // base_link -> base_laser
    geometry_msgs::msg::TransformStamped lidar_tf;
    lidar_tf.header.stamp = this->get_clock()->now();
    lidar_tf.header.frame_id = "base_link";
    lidar_tf.child_frame_id = "base_laser";
    lidar_tf.transform.translation.x = 0.0;
    lidar_tf.transform.translation.y = 0.0;
    lidar_tf.transform.translation.z = 0.15;
    lidar_tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(lidar_tf);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A105TFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
