// File: rgbd_to_cmd.cpp
// ROS 2 Humble: rgbd -> cmd_vel по белой дороге (C++), BEST_EFFORT
// Центр по distance transform ("балун") с уточнением до середины между краями.

#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>

using sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;
using geometry_msgs::msg::Twist;

static inline int clampi(int x, int a, int b) { return x < a ? a : (x > b ? b : x); }
static inline double clampd(double x, double a, double b) { return x < a ? a : (x > b ? b : x); }

class WhiteRoadNode : public rclcpp::Node {
public:
  WhiteRoadNode() : Node("white_road_node") {
    rmw_qos_profile_t be_qos = rmw_qos_profile_default;
    be_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    be_qos.durability  = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    be_qos.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    be_qos.depth       = 10;
    auto info_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    // Параметры
    this->declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    this->declare_parameter<bool>("enable_debug", true);

    this->declare_parameter<int>("hsv_S_max", 60);
    this->declare_parameter<int>("hsv_V_min", 180);
    this->declare_parameter<int>("morph_kernel", 5);
    this->declare_parameter<double>("roi_ymin_frac", 0.10);
    this->declare_parameter<double>("roi_ymax_frac", 0.76);
    this->declare_parameter<double>("near_cut_m", 0.40);

    this->declare_parameter<int>("step_px", 8);
    this->declare_parameter<int>("min_width_px", 12);

    this->declare_parameter<int>("erode_kernel", 7);
    this->declare_parameter<int>("erode_iter",   2);

    // Балун-трекинг
    this->declare_parameter<int>("balloon_track_win", 32);
    this->declare_parameter<int>("max_dx_px",        12);
    this->declare_parameter<double>("balloon_lambda", 1.0);
    this->declare_parameter<bool>("balloon_blur",    true);
    this->declare_parameter<double>("balloon_center_mu", 0.0); // мягкий сдвиг к центру кадра (0=выкл)

    this->declare_parameter<double>("lookahead_m", 0.60);
    this->declare_parameter<double>("min_v", 0.05);
    this->declare_parameter<double>("max_v", 0.13);

    this->declare_parameter<double>("w_gain", 3.5);
    this->declare_parameter<double>("w_max", 2.5);
    this->declare_parameter<double>("Ld_min", 0.25);
    this->declare_parameter<double>("Ld_alpha_k", 0.8);
    this->declare_parameter<double>("v_alpha_k", 1.4);

    const auto rgb_topic   = this->get_parameter("rgb_topic").as_string();
    const auto depth_topic = this->get_parameter("depth_topic").as_string();
    const auto info_topic  = this->get_parameter("camera_info_topic").as_string();
    enable_debug_          = this->get_parameter("enable_debug").as_bool();

    rgb_sub_.subscribe(this, rgb_topic, be_qos);
    depth_sub_.subscribe(this, depth_topic, be_qos);

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(30), rgb_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.15));
    sync_->registerCallback(std::bind(&WhiteRoadNode::synced_cb, this,
                                      std::placeholders::_1, std::placeholders::_2));

    cam_info_sub_ = this->create_subscription<CameraInfo>(
        info_topic, info_qos,
        std::bind(&WhiteRoadNode::info_cb, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<Twist>("/cmd_vel", 1);

    if (enable_debug_) {
      auto dbg_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
      mask_pub_    = this->create_publisher<Image>("white_mask", dbg_qos);
      viz_pub_     = this->create_publisher<Image>("centerline_viz", dbg_qos);
      overlay_pub_ = this->create_publisher<Image>("road_overlay", dbg_qos);
    }
  }

private:
  // Intrinsics
  double fx_{NAN};
  double cx_{NAN};
  bool enable_debug_{true};
  int last_cx_{-1};

  // Publishers
  rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<Image>::SharedPtr viz_pub_;
  rclcpp::Publisher<Image>::SharedPtr overlay_pub_;

  // Subscribers + sync
  message_filters::Subscriber<Image> rgb_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  std::unique_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<Image, Image>>> sync_;
  rclcpp::Subscription<CameraInfo>::SharedPtr cam_info_sub_;

  void info_cb(const CameraInfo::SharedPtr msg) {
    fx_ = static_cast<double>(msg->k[0]);
    cx_ = static_cast<double>(msg->k[2]);
  }

  void segment_white(const cv::Mat &bgr, const cv::Mat &depth_m,
                     cv::Mat &mask, int &ymin, int &ymax) {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    int S_max = static_cast<int>(this->get_parameter("hsv_S_max").as_int());
    int V_min = static_cast<int>(this->get_parameter("hsv_V_min").as_int());
    cv::inRange(hsv, cv::Scalar(0, 0, V_min), cv::Scalar(179, S_max, 255), mask);

    const int h = mask.rows, wcols = mask.cols;
    ymin = clampi(static_cast<int>(h * this->get_parameter("roi_ymin_frac").as_double()), 0, h);
    ymax = clampi(static_cast<int>(h * this->get_parameter("roi_ymax_frac").as_double()), 0, h);
    if (ymin > 0)  mask(cv::Rect(0, 0, wcols, ymin)).setTo(0);
    if (ymax < h)  mask(cv::Rect(0, ymax, wcols, h - ymax)).setTo(0);

    double near_cut = this->get_parameter("near_cut_m").as_double();
    if (!depth_m.empty()) {
      for (int y = 0; y < h; ++y) {
        const float* drow = depth_m.ptr<float>(y);
        uchar* mrow = mask.ptr<uchar>(y);
        for (int x = 0; x < wcols; ++x) {
          float z = drow[x];
          if (z > 0.0f && z < static_cast<float>(near_cut)) mrow[x] = 0;
        }
      }
    }

    int mk = static_cast<int>(this->get_parameter("morph_kernel").as_int());
    int k  = std::max(3, (mk | 1));
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);

    // CC -> оставляем крупнейший компонент
    cv::Mat labels, stats, centroids;
    int num = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    if (num > 1) {
      int best_idx = 1;
      int best_area = stats.at<int>(1, cv::CC_STAT_AREA);
      for (int i = 2; i < num; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > best_area) { best_area = area; best_idx = i; }
      }
      mask.setTo(0);
      for (int y = 0; y < h; ++y) {
        const int* lrow = labels.ptr<int>(y);
        uchar* mrow = mask.ptr<uchar>(y);
        for (int x = 0; x < wcols; ++x) mrow[x] = (lrow[x] == best_idx) ? 255 : 0;
      }
    }

    // Эрозия в конце
    int ek = std::max(1, static_cast<int>(this->get_parameter("erode_kernel").as_int()) | 1);
    int ei = std::max(1, static_cast<int>(this->get_parameter("erode_iter").as_int()));
    cv::Mat ker_er = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ek, ek));
    cv::erode(mask, mask, ker_er, cv::Point(-1,-1), ei);
  }

  // Балун-центролиния с уточнением до середины между краями
  std::vector<cv::Point> centerline_points_balloon(const cv::Mat &mask, int ymin, int ymax) {
    CV_Assert(mask.type() == CV_8UC1);
    const int h = mask.rows, w = mask.cols;

    cv::Mat dist;
    cv::distanceTransform(mask, dist, cv::DIST_L2, 3); // CV_32F
    if (this->get_parameter("balloon_blur").as_bool()) {
      cv::GaussianBlur(dist, dist, cv::Size(0,0), 1.0);
    }

    const int step_px   = static_cast<int>(this->get_parameter("step_px").as_int());
    const int max_dx    = static_cast<int>(this->get_parameter("max_dx_px").as_int());
    const int track_win = static_cast<int>(this->get_parameter("balloon_track_win").as_int());
    const double lambda = this->get_parameter("balloon_lambda").as_double();
    const double mu     = this->get_parameter("balloon_center_mu").as_double();

    std::vector<cv::Point> centers;
    int y_start = std::min(h - 1, std::max(0, ymax - 1));
    int y_end   = std::max(0, ymin);

    int cx_prev = (last_cx_ >= 0 ? last_cx_ : w/2);

    for (int y = y_start; y >= y_end; y -= step_px) {
      const float* drow = dist.ptr<float>(y);
      const uchar* mrow = mask.ptr<uchar>(y);

      int xL = std::max(0, cx_prev - track_win);
      int xR = std::min(w - 1, cx_prev + track_win);

      int   best_x = -1;
      float best_score = -1.0f;

      for (int x = xL; x <= xR; ++x) {
        float score = drow[x]
                    - static_cast<float>(lambda * std::abs(x - cx_prev))
                    - static_cast<float>(mu * std::abs(x - w/2));
        if (score > best_score) { best_score = score; best_x = x; }
      }

      if (best_x < 0 || best_score <= 0.0f) {
        double val; cv::Point pmax;
        cv::minMaxLoc(dist.row(y), nullptr, &val, nullptr, &pmax);
        if (val <= 0.0) continue;
        best_x = pmax.x;
      }

      // уточнение: середина между реальными краями вокруг best_x
      int xLedge = best_x, xRedge = best_x;
      while (xLedge > 0   && mrow[xLedge] > 0) --xLedge;
      if (xLedge < best_x && mrow[xLedge] == 0) ++xLedge;
      while (xRedge < w-1 && mrow[xRedge] > 0) ++xRedge;
      if (xRedge > best_x && mrow[xRedge] == 0) --xRedge;

      int cx_mid = best_x;
      if (xLedge < best_x && xRedge > best_x) {
        cx_mid = (xLedge + xRedge) / 2;
      }

      int cx = std::clamp(cx_mid, cx_prev - max_dx, cx_prev + max_dx);
      centers.emplace_back(cx, y);
      cx_prev = cx;
    }

    if (!centers.empty()) last_cx_ = centers.front().x;
    return centers;
  }

  inline double depth_at(const cv::Mat &depth, int y, int x, bool is_u16) {
    const int h = depth.rows, w = depth.cols;
    x = clampi(x, 0, w - 1);
    y = clampi(y, 0, h - 1);
    if (is_u16) {
      uint16_t z = depth.at<uint16_t>(y, x);
      return static_cast<double>(z) * 0.001;
    } else {
      float z = depth.at<float>(y, x);
      return static_cast<double>(z);
    }
  }

  std::pair<double,double> control_pp(int u, int v_px, const cv::Mat &depth, bool depth_is_u16,
                                      double Ld0, double v_min, double v_max) {
    double z = depth_at(depth, v_px, u, depth_is_u16);
    if (!std::isfinite(z) || z <= 0.05) z = Ld0;

    double x = (static_cast<double>(u) - cx_) * z / fx_;
    double alpha = std::atan2(x, z);

    double Ld_min     = this->get_parameter("Ld_min").as_double();
    double Ld_alpha_k = this->get_parameter("Ld_alpha_k").as_double();
    double v_alpha_k  = this->get_parameter("v_alpha_k").as_double();
    double w_gain     = this->get_parameter("w_gain").as_double();
    double w_max      = this->get_parameter("w_max").as_double();

    double Ld = std::max(Ld_min, Ld0 * (1.0 - Ld_alpha_k * std::min(1.0, std::abs(alpha))));
    double v_cmd = clampd(v_max * (1.0 - v_alpha_k * std::abs(alpha)), v_min, v_max);
    double w     = - w_gain * 2.0 * v_cmd * std::sin(alpha) / std::max(Ld, 1e-3);
    w = clampd(w, -w_max, w_max);
    return {v_cmd, w};
  }

  void synced_cb(const Image::ConstSharedPtr &rgb_msg, const Image::ConstSharedPtr &depth_msg) {
    if (!std::isfinite(fx_)) return;

    cv_bridge::CvImageConstPtr rgb_cv_ptr;
    try { rgb_cv_ptr = cv_bridge::toCvShare(rgb_msg, "bgr8"); }
    catch (const cv_bridge::Exception &e) { RCLCPP_WARN(this->get_logger(), "cv_bridge rgb: %s", e.what()); return; }
    const cv::Mat &bgr = rgb_cv_ptr->image;

    bool depth_is_u16 = (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                         depth_msg->encoding == "16UC1");
    cv_bridge::CvImageConstPtr depth_cv_ptr;
    try {
      if (depth_is_u16) depth_cv_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      else              depth_cv_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (const cv_bridge::Exception &e) { RCLCPP_WARN(this->get_logger(), "cv_bridge depth: %s", e.what()); return; }
    const cv::Mat &depth = depth_cv_ptr->image;

    cv::Mat depth_m;
    if (depth_is_u16) depth.convertTo(depth_m, CV_32F, 0.001);
    else              depth_m = depth;

    cv::Mat mask;
    int ymin = 0, ymax = 0;
    segment_white(bgr, depth_m, mask, ymin, ymax);

    auto centers = centerline_points_balloon(mask, ymin, ymax);

    Twist twist;
    if (!centers.empty()) {
      double Ld    = this->get_parameter("lookahead_m").as_double();
      double v_min = this->get_parameter("min_v").as_double();
      double v_max = this->get_parameter("max_v").as_double();

      // цель по глубине к Ld
      auto best_it = std::min_element(centers.begin(), centers.end(),
        [&](const cv::Point &a, const cv::Point &b) {
          double da = std::abs(depth_at(depth, a.y, a.x, depth_is_u16) - Ld);
          double db = std::abs(depth_at(depth, b.y, b.x, depth_is_u16) - Ld);
          return da < db;
        });
      cv::Point tgt = *best_it;

      auto [v_cmd, w_cmd] = control_pp(tgt.x, tgt.y, depth, depth_is_u16, Ld, v_min, v_max);
      twist.linear.x  = v_cmd;
      twist.angular.z = w_cmd;

      if (enable_debug_) {
        try {
          auto msg = cv_bridge::CvImage(rgb_msg->header, "mono8", mask).toImageMsg();
          mask_pub_->publish(*msg);
        } catch (...) {}
        try {
          cv::Mat viz = cv::Mat::zeros(bgr.size(), bgr.type());
          for (const auto &p : centers) cv::circle(viz, p, 3, cv::Scalar(0, 0, 255), -1);
          cv::circle(viz, tgt, 6, cv::Scalar(255, 0, 0), -1);
          auto msg = cv_bridge::CvImage(rgb_msg->header, "bgr8", viz).toImageMsg();
          viz_pub_->publish(*msg);
        } catch (...) {}
        try {
          cv::Mat overlay = build_overlay(bgr, mask, centers, &tgt, ymin, ymax);
          auto msg = cv_bridge::CvImage(rgb_msg->header, "bgr8", overlay).toImageMsg();
          overlay_pub_->publish(*msg);
        } catch (...) {}
      }
    }

    cmd_pub_->publish(twist);
  }

  cv::Mat build_overlay(const cv::Mat &bgr, const cv::Mat &mask,
                        const std::vector<cv::Point> &centers,
                        const cv::Point *tgt, int ymin, int ymax) {
    cv::Mat ov = bgr.clone();

    cv::Mat color_mask = cv::Mat::zeros(bgr.size(), bgr.type());
    std::vector<cv::Mat> ch;
    cv::split(color_mask, ch);
    mask.copyTo(ch[1]);
    cv::merge(ch, color_mask);
    cv::addWeighted(ov, 1.0, color_mask, 0.35, 0.0, ov);

    cv::line(ov, {0, ymin}, {ov.cols - 1, ymin}, {255, 255, 0}, 1);
    cv::line(ov, {0, ymax}, {ov.cols - 1, ymax}, {255, 255, 0}, 1);

    if (centers.size() > 1) {
      cv::polylines(ov, std::vector<std::vector<cv::Point>>{centers}, false, {0, 0, 255}, 2);
    }
    for (const auto &p : centers) cv::circle(ov, p, 2, {0, 0, 255}, -1);
    if (tgt) cv::circle(ov, *tgt, 6, {255, 0, 0}, -1);

    return ov;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WhiteRoadNode>());
  rclcpp::shutdown();
  return 0;
}
