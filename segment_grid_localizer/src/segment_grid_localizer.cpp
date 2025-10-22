#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <optional>
#include <algorithm>
#include <string>
#include <memory>

struct Pose2 { double x{0}, y{0}, yaw{0}; };

static inline double normang(double a){ return std::atan2(std::sin(a), std::cos(a)); }
static inline Pose2 compose(const Pose2& a, const Pose2& b){
  double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return { a.x + c*b.x - s*b.y, a.y + s*b.x + c*b.y, normang(a.yaw + b.yaw) };
}
static inline Pose2 inverse(const Pose2& p){
  double c = std::cos(p.yaw), s = std::sin(p.yaw);
  return { -( c*p.x + s*p.y),  -(-s*p.x + c*p.y), -p.yaw };
}
static inline double yawFromQuat(const geometry_msgs::msg::Quaternion& q){
  return std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

class SegmentGridLocalizer : public rclcpp::Node {
public:
  SegmentGridLocalizer(): Node("segment_grid_localizer"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(*tf_buffer_),
    tf_broadcaster_(this)
  {
    map_topic_  = declare_parameter<std::string>("map_topic", "/map");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    frame_map_  = declare_parameter<std::string>("map_frame", "map");
    frame_odom_ = declare_parameter<std::string>("odom_frame", "odom");
    frame_base_ = declare_parameter<std::string>("base_frame", "base_link");

    search_xy_max_    = declare_parameter("search_xy_max", 0.40);
    search_yaw_max_   = declare_parameter("search_yaw_max", 0.35);
    step_xy_coarse_   = declare_parameter("step_xy_coarse", 0.05);
    step_yaw_coarse_  = declare_parameter("step_yaw_coarse", 0.05);
    step_xy_fine_     = declare_parameter("step_xy_fine", 0.01);
    step_yaw_fine_    = declare_parameter("step_yaw_fine", 0.01);

    range_min_ = declare_parameter("range_min", 0.05);
    range_max_ = declare_parameter("range_max", 8.0);
    downsample_= declare_parameter("downsample", 2);

    occ_threshold_    = declare_parameter("occ_threshold", 95);
    sigma_hit_        = declare_parameter("sigma_hit", 0.05);
    penalty_unknown_  = declare_parameter("penalty_unknown", 0.2);
    penalty_outside_  = declare_parameter("penalty_outside", 0.0);
    alpha_smooth_     = declare_parameter("alpha_smooth", 1.0);

    zero_scan_rot_    = declare_parameter("zero_scan_rot", true);
    scan_yaw_bias_    = declare_parameter("scan_yaw_bias", 0.0); // рад

    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&SegmentGridLocalizer::onMap, this, std::placeholders::_1));

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(50),
      std::bind(&SegmentGridLocalizer::onOdom, this, std::placeholders::_1));

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SegmentGridLocalizer::onScan, this, std::placeholders::_1));

    pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("localized_pose", 10);
  }

private:
  // params
  std::string map_topic_, scan_topic_, odom_topic_;
  std::string frame_map_, frame_odom_, frame_base_;
  double search_xy_max_, search_yaw_max_;
  double step_xy_coarse_, step_yaw_coarse_;
  double step_xy_fine_,   step_yaw_fine_;
  double range_min_, range_max_; int downsample_;
  int occ_threshold_; double sigma_hit_, penalty_unknown_, penalty_outside_, alpha_smooth_;
  bool zero_scan_rot_; double scan_yaw_bias_;

  // data
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::vector<float> dist_; // метры
  rclcpp::Time map_stamp_;
  bool dist_ready_{false};

  std::optional<Pose2> odom_base_; // одометрия: odom->base_link
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // lidar в базе
  bool have_lidar_tf_{false};
  Pose2 base_T_lidar_{0,0,0};
  std::string last_scan_frame_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;

  // текущий map->odom
  Pose2 map_T_odom_{0,0,0};
  bool have_mo_{false};

  // utils
  static inline int idx(int x,int y,int W){ return y*W + x; }
  bool worldToGrid(double wx,double wy,int& gx,int& gy) const {
    if(!map_) return false;
    double ox = map_->info.origin.position.x;
    double oy = map_->info.origin.position.y;
    double res = map_->info.resolution;
    gx = static_cast<int>(std::floor((wx - ox)/res));
    gy = static_cast<int>(std::floor((wy - oy)/res));
    return gx>=0 && gy>=0 && gx<(int)map_->info.width && gy<(int)map_->info.height;
  }

  void computeDistanceField(){
    if(!map_) return;
    const int W = map_->info.width;
    const int H = map_->info.height;
    const double res = map_->info.resolution;
    dist_.assign(W*H, std::numeric_limits<float>::infinity());

    using Node = std::pair<float,int>; // (dist, id)
    auto cmp = [](const Node& a, const Node& b){ return a.first > b.first; };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> pq(cmp);

    // init из занятых
    for(int y=0;y<H;++y){
      for(int x=0;x<W;++x){
        int id = idx(x,y,W);
        int8_t v = map_->data[id];
        if(v >= occ_threshold_){
          dist_[id] = 0.0f;
          pq.emplace(0.0f, id);
        }
      }
    }

    const int dx8[8] = {+1,-1,0,0, +1,+1,-1,-1};
    const int dy8[8] = {0,0,+1,-1, +1,-1,+1,-1};
    const float cost8[8] = {1,1,1,1, std::sqrt(2.0f),std::sqrt(2.0f),std::sqrt(2.0f),std::sqrt(2.0f)};

    while(!pq.empty()){
      auto [d, id] = pq.top(); pq.pop();
      if(d>dist_[id]) continue;
      int x = id % W;
      int y = id / W;
      for(int k=0;k<8;++k){
        int nx = x + dx8[k], ny = y + dy8[k];
        if(nx<0||ny<0||nx>=W||ny>=H) continue;
        int nid = idx(nx,ny,W);
        float nd = d + cost8[k];
        if(nd < dist_[nid]){
          dist_[nid] = nd;
          pq.emplace(nd, nid);
        }
      }
    }
    for(auto& v: dist_) if(std::isfinite(v)) v *= res; // в метры
    dist_ready_ = true;
  }

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    map_ = msg;
    map_stamp_ = msg->header.stamp;
    computeDistanceField();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Map %ux%u res=%.3f", msg->info.width, msg->info.height, msg->info.resolution);
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
    Pose2 p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    p.yaw = yawFromQuat(msg->pose.pose.orientation);
    odom_base_ = p;
  }

  bool ensureLidarTF(const sensor_msgs::msg::LaserScan& scan){
    if(have_lidar_tf_ && scan.header.frame_id == last_scan_frame_) return true;
    try{
      auto T = tf_buffer_->lookupTransform(
        frame_base_, scan.header.frame_id, scan.header.stamp,
        rclcpp::Duration::from_seconds(0.1));
      base_T_lidar_.x = T.transform.translation.x;
      base_T_lidar_.y = T.transform.translation.y;
      double yaw_mount = yawFromQuat(T.transform.rotation);
      base_T_lidar_.yaw = (zero_scan_rot_ ? 0.0 : yaw_mount) + scan_yaw_bias_;
      have_lidar_tf_ = true;
      last_scan_frame_ = scan.header.frame_id;
      return true;
    }catch(const std::exception& e){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No TF %s->%s: %s",
                           frame_base_.c_str(), scan.header.frame_id.c_str(), e.what());
      return false;
    }
  }

  void scanPointsInBase(const sensor_msgs::msg::LaserScan& s,
                        std::vector<std::pair<double,double>>& pts) const {
    pts.clear();
    const int ds = std::max(1, downsample_);
    pts.reserve(s.ranges.size()/ds);
    const double yaw_l = base_T_lidar_.yaw;
    const double lx = base_T_lidar_.x, ly = base_T_lidar_.y;
    for(size_t k=0;k<s.ranges.size();k+=ds){
      float r = s.ranges[k];
      if(!std::isfinite(r) || r < std::max((double)s.range_min, range_min_) || r > std::min((double)s.range_max, range_max_))
        continue;
      double a = s.angle_min + k*s.angle_increment + yaw_l;
      double xb = r*std::cos(a) + lx;
      double yb = r*std::sin(a) + ly;
      pts.emplace_back(xb, yb);
    }
  }

  double scorePose(const Pose2& base_in_map,
                   const std::vector<std::pair<double,double>>& pts_base) const {
    if(!map_ || !dist_ready_) return -1e9;
    const double cs = std::cos(base_in_map.yaw), sn = std::sin(base_in_map.yaw);
    const double mx = base_in_map.x, my = base_in_map.y;
    const int W = map_->info.width;

    double sc = 0.0;
    for(const auto& pb: pts_base){
      double xm = mx + cs*pb.first - sn*pb.second;
      double ym = my + sn*pb.first + cs*pb.second;

      int gx, gy;
      if(!worldToGrid(xm, ym, gx, gy)){
        sc += penalty_outside_;
        continue;
      }
      int id = idx(gx,gy,W);
      int8_t occ = map_->data[id];
      if(occ == -1){
        sc += penalty_unknown_;
        continue;
      }
      float d = dist_[id]; // метры до ближайшей стены
      double w = std::exp(-0.5 * (d*d) / (sigma_hit_*sigma_hit_));
      sc += w;
    }
    return sc / std::max<size_t>(1, pts_base.size());
  }

  Pose2 coarseToFineSearch(const Pose2& seed,
                           const std::vector<std::pair<double,double>>& pts_base){
    Pose2 best = seed;
    double best_s = scorePose(best, pts_base);

    auto grid_search = [&](const Pose2& center, double rxy, double ryaw, double st_xy, double st_yaw){
      Pose2 cur_best = best;
      double cur_s = best_s;
      for(double dyaw=-ryaw; dyaw<=ryaw+1e-9; dyaw+=st_yaw){
        for(double dx=-rxy; dx<=rxy+1e-9; dx+=st_xy){
          for(double dy=-rxy; dy<=rxy+1e-9; dy+=st_xy){
            Pose2 cand { center.x + dx, center.y + dy, normang(center.yaw + dyaw) };
            double s = scorePose(cand, pts_base);
            if(s > cur_s){
              cur_s = s; cur_best = cand;
            }
          }
        }
      }
      best = cur_best; best_s = cur_s;
    };

    grid_search(seed,            search_xy_max_,  search_yaw_max_,  step_xy_coarse_, step_yaw_coarse_);
    grid_search(best,            step_xy_coarse_*1.5, step_yaw_coarse_*1.5, step_xy_fine_,   step_yaw_fine_);
    return best;
  }

  void publishMapToOdom(const rclcpp::Time& stamp, const Pose2& map_T_odom){
    geometry_msgs::msg::TransformStamped T;
    T.header.stamp = stamp;
    T.header.frame_id = frame_map_;
    T.child_frame_id  = frame_odom_;
    T.transform.translation.x = map_T_odom.x;
    T.transform.translation.y = map_T_odom.y;
    T.transform.translation.z = 0.0;
    tf2::Quaternion q; q.setRPY(0,0,map_T_odom.yaw);
    T.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_.sendTransform(T);
  }

  void publishPose(const rclcpp::Time& stamp, const Pose2& base_in_map){
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_map_;
    msg.pose.pose.position.x = base_in_map.x;
    msg.pose.pose.position.y = base_in_map.y;
    tf2::Quaternion q; q.setRPY(0,0,base_in_map.yaw);
    msg.pose.pose.orientation = tf2::toMsg(q);
    msg.pose.covariance.fill(0.0);
    msg.pose.covariance[0] = 0.05*0.05;
    msg.pose.covariance[7] = 0.05*0.05;
    msg.pose.covariance[35]= (5.0*M_PI/180.0)*(5.0*M_PI/180.0);
    pub_pose_->publish(msg);
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan){
    if(!map_ || !dist_ready_) return;
    if(!odom_base_.has_value()) return;
    if(!ensureLidarTF(*scan)) return;

    Pose2 od = *odom_base_;                              // odom->base_link
    Pose2 pred_base_in_map = compose(map_T_odom_, od);   // map->base_link

    std::vector<std::pair<double,double>> pts_base;
    scanPointsInBase(*scan, pts_base);
    if(pts_base.empty()) return;

    Pose2 best_base_in_map = coarseToFineSearch(pred_base_in_map, pts_base);

    Pose2 new_map_T_odom = compose(best_base_in_map, inverse(od)); // map->odom

    if(!have_mo_) { map_T_odom_ = new_map_T_odom; have_mo_ = true; }
    else{
      double a = std::clamp(alpha_smooth_, 0.0, 1.0);
      double dyaw = normang(new_map_T_odom.yaw - map_T_odom_.yaw);
      map_T_odom_.x += a * (new_map_T_odom.x - map_T_odom_.x);
      map_T_odom_.y += a * (new_map_T_odom.y - map_T_odom_.y);
      map_T_odom_.yaw = normang(map_T_odom_.yaw + a * dyaw);
    }

    publishMapToOdom(scan->header.stamp, map_T_odom_);
    publishPose(scan->header.stamp, best_base_in_map);
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SegmentGridLocalizer>());
  rclcpp::shutdown();
  return 0;
}
