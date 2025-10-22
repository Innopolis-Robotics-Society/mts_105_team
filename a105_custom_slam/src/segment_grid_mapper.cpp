#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <string>

struct Pose2{ double x{0}, y{0}, yaw{0}; };

struct GridParams{
  // Segment grid
  double cell{0.5};
  double range_max{8.0};
  double fov_pad{0.0};
  double hit_eps{0.06};
  double cover_frac_min{0.6};
  double min_wall_frac{0.5};
  double p_hit{0.7}, p_miss{0.35};
  double l0{0.0}, lmin{-4.0}, lmax{4.0};
  double L_thresh{0.0};
  double map_radius{6.0};
  double x0{0.0}, y0{0.0};

  // Binary map
  double grid_res{0.05};
  bool   persist_map{true};

  // Hysteresis for segments
  int    stroke_cells{1};
  double free_cover_min{0.95};
  int    miss_streak_forget{5};

  // Corridors and rays
  double corridor_margin{0.02};
  bool   ray_clear{true};
  double ray_margin{0.03};
  bool   ray_override_occupied{false}; // не затирать стены по умолчанию
  int    free_stroke_cells{0};
};

struct SegRec{
  int i{0}, j{0}; bool hor{false};
  double logit{0.0};
  int miss_streak{0};
  SegRec()=default;
  SegRec(int ii,int jj,bool h,double L,int ms):i(ii),j(jj),hor(h),logit(L),miss_streak(ms){}
};

static inline long long Hkey(int i,int j,bool hor){
  return ( ( (long long)i<<33) ^ ( (long long)j<<1) ^ (long long)hor );
}
static inline double normang(double a){ return std::atan2(std::sin(a), std::cos(a)); }
static inline double logit_from_p(double p){ return std::log(p/(1.0-p)); }
static inline void clamp(double& v,double lo,double hi){ if(v<lo) v=lo; if(v>hi) v=hi; }

class SegmentGridMapper : public rclcpp::Node{
public:
  SegmentGridMapper(): Node("segment_grid_mapper"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // params
    P_.cell            = declare_parameter("cell", 0.5);
    P_.x0              = declare_parameter("x0", -0.5 * P_.cell);
    P_.y0              = declare_parameter("y0", -0.5 * P_.cell);
    P_.range_max       = declare_parameter("range_max", 8.0);
    P_.fov_pad         = declare_parameter("fov_pad", 0.0);
    P_.hit_eps         = declare_parameter("hit_eps", 0.06);
    P_.cover_frac_min  = declare_parameter("cover_frac_min", 0.6);
    P_.min_wall_frac   = declare_parameter("min_wall_frac", 0.5);
    P_.p_hit           = declare_parameter("p_hit", 0.7);
    P_.p_miss          = declare_parameter("p_miss", 0.35);
    P_.l0              = declare_parameter("l0", 0.0);
    P_.lmin            = declare_parameter("lmin", -4.0);
    P_.lmax            = declare_parameter("lmax",  4.0);
    P_.L_thresh        = declare_parameter("L_thresh", 0.0);
    P_.map_radius      = declare_parameter("map_radius", 6.0);

    P_.grid_res        = declare_parameter("grid_res", 0.05);
    P_.persist_map     = declare_parameter("persist_map", true);

    P_.stroke_cells        = declare_parameter("stroke_cells", 1);
    P_.free_cover_min      = declare_parameter("free_cover_min", 0.95);
    P_.miss_streak_forget  = declare_parameter("miss_streak_forget", 5);

    P_.corridor_margin       = declare_parameter("corridor_margin", 0.02);
    P_.ray_clear             = declare_parameter("ray_clear", true);
    P_.ray_margin            = declare_parameter("ray_margin", 0.03);
    P_.ray_override_occupied = declare_parameter("ray_override_occupied", false);
    P_.free_stroke_cells     = declare_parameter("free_stroke_cells", 0);

    frame_map_   = declare_parameter("map_frame",   std::string("map"));
    frame_base_  = declare_parameter("base_frame",  std::string("base_link"));
    scan_topic_  = declare_parameter("scan_topic",  std::string("scan"));
    map_topic_   = declare_parameter("map_topic",   std::string("map"));

    init_occupancy_grid();

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SegmentGridMapper::onScan, this, std::placeholders::_1));

    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("segment_grid_markers", 10);
    rclcpp::QoS qos( rclcpp::KeepLast(1) );
    qos.transient_local().reliable();
    pub_occ_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, qos);
    bootstrap_initial_walls();
  }

private:



  void bootstrap_initial_walls(){
    // смещение всех точек на (-0.25, -0.25)
    rasterize_world_line(-0.25, -0.25, -0.25,  0.25); // вертикальная
    rasterize_world_line(-0.25, -0.25,  0.25, -0.25); // горизонтальная

    occ_msg_.header.frame_id = frame_map_;
    occ_msg_.header.stamp = now();
    pub_occ_->publish(occ_msg_);
  }

  GridParams P_;
  std::string frame_map_, frame_base_, scan_topic_, map_topic_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unordered_map<long long, SegRec> segs_;

  // Binary occupancy grid
  double origin_x_{0.0}, origin_y_{0.0};
  int width_{0}, height_{0};
  nav_msgs::msg::OccupancyGrid occ_msg_;

  struct EvalOut{
    int near_hits{0};
    double cover_frac{0.0};
    double max_run_len{0.0};
    double t0{0.0}, t1{0.0};
  };

  static inline int idx(int x,int y,int W){ return y*W + x; }
  bool inBounds(int gx,int gy) const { return gx>=0 && gy>=0 && gx<width_ && gy<height_; }

  void init_occupancy_grid(){
    const double side = 2.0*P_.map_radius;
    width_  = static_cast<int>(std::ceil(side / P_.grid_res));
    height_ = static_cast<int>(std::ceil(side / P_.grid_res));
    origin_x_ = P_.x0 - P_.map_radius;
    origin_y_ = P_.y0 - P_.map_radius;

    occ_msg_.info.resolution = P_.grid_res;
    occ_msg_.info.width = width_;
    occ_msg_.info.height = height_;
    occ_msg_.info.origin.position.x = origin_x_;
    occ_msg_.info.origin.position.y = origin_y_;
    occ_msg_.info.origin.orientation.w = 1.0;
    occ_msg_.header.frame_id = frame_map_;
    occ_msg_.data.assign(width_*height_, 0);
  }

  bool worldToGrid(double x,double y,int& gx,int& gy) const{
    gx = static_cast<int>(std::floor((x - origin_x_) / P_.grid_res));
    gy = static_cast<int>(std::floor((y - origin_y_) / P_.grid_res));
    return inBounds(gx,gy);
  }

  static inline double yawFromQuat(const geometry_msgs::msg::Quaternion& q){
    return std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
  }

  bool getPose(const rclcpp::Time& t, Pose2& out){
    try{
      geometry_msgs::msg::TransformStamped T =
        tf_buffer_.lookupTransform(frame_map_, frame_base_, t, rclcpp::Duration::from_seconds(0.05));
      out.x = T.transform.translation.x;
      out.y = T.transform.translation.y;
      out.yaw = yawFromQuat(T.transform.rotation);
      return true;
    }catch(const std::exception& e){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF map->base_link unavailable: %s", e.what());
      return false;
    }
  }

  void segEndpoints(int i,int j,bool hor,double& x1,double& y1,double& x2,double& y2){
    const double L = P_.cell;
    const double X = P_.x0, Y = P_.y0;
    if(hor){ x1=X+i*L; y1=Y+j*L; x2=X+(i+1)*L; y2=Y+j*L; }
    else   { x1=X+i*L; y1=Y+j*L; x2=X+i*L;     y2=Y+(j+1)*L; }
  }

  SegRec& getSeg(int i,int j,bool hor){
    long long key=Hkey(i,j,hor);
    auto it = segs_.find(key);
    if(it==segs_.end()){
      auto ins = segs_.emplace(key, SegRec{i,j,hor,P_.l0,0});
      return ins.first->second;
    }
    return it->second;
  }
  bool isActive(int i,int j,bool hor) const{
    auto it = segs_.find(Hkey(i,j,hor));
    return (it!=segs_.end() && it->second.logit > P_.L_thresh);
  }

  EvalOut evalSegment(const Pose2& pose,
                      const std::vector<std::pair<double,double>>& pts,
                      int i,int j,bool hor,
                      double ang_min,double ang_max)
  {
    double x1,y1,x2,y2; segEndpoints(i,j,hor,x1,y1,x2,y2);

    // coverage
    const int M=10;
    int covered=0;
    for(int m=0;m<M;++m){
      double t=(m+0.5)/M;
      double xm=x1+t*(x2-x1), ym=y1+t*(y2-y1);
      double dx=xm-pose.x, dy=ym-pose.y;
      double r=std::hypot(dx,dy);
      if(r>P_.range_max) continue;
      double a = normang(std::atan2(dy,dx)-pose.yaw);
      if(a>=ang_min-P_.fov_pad && a<=ang_max+P_.fov_pad) covered++;
    }
    EvalOut out;
    out.cover_frac = static_cast<double>(covered)/M;

    // near-hits + best t-interval
    double dx=x2-x1, dy=y2-y1;
    double Lseg = std::hypot(dx,dy) + 1e-9;
    double denom = dx*dx+dy*dy + 1e-9;

    std::vector<double> ts; ts.reserve(32);
    for(const auto& p: pts){
      double t = ((p.first-x1)*dx + (p.second-y1)*dy)/denom;
      if(t<0.0 || t>1.0) continue;
      double px=x1+t*dx, py=y1+t*dy;
      double d=std::hypot(p.first-px,p.second-py);
      if(d<P_.hit_eps){ out.near_hits++; ts.push_back(t); }
    }
    if(ts.empty()){ out.max_run_len=0.0; out.t0=0.0; out.t1=0.0; return out; }

    std::sort(ts.begin(), ts.end());
    const double gap_t = std::max(2.0*P_.hit_eps / Lseg, 0.05);

    double run_s=ts.front(), prev=ts.front();
    double best_t=0.0, best_s=run_s, best_e=run_s;
    for(size_t k=1;k<ts.size();++k){
      if(ts[k]-prev>gap_t){
        double len=prev-run_s;
        if(len>best_t){ best_t=len; best_s=run_s; best_e=prev; }
        run_s=ts[k];
      }
      prev=ts[k];
    }
    double len=prev-run_s;
    if(len>best_t){ best_t=len; best_s=run_s; best_e=prev; }

    out.max_run_len = best_t * Lseg;
    out.t0=best_s; out.t1=best_e;
    return out;
  }

  // ======== occupied painting ========
  inline void set_cell_occ(int gx,int gy){
    if(!inBounds(gx,gy)) return;
    occ_msg_.data[idx(gx,gy,width_)] = 100;
  }
  void paint_brush(int gx,int gy){
    for(int dy=-P_.stroke_cells; dy<=P_.stroke_cells; ++dy)
      for(int dx=-P_.stroke_cells; dx<=P_.stroke_cells; ++dx)
        set_cell_occ(gx+dx, gy+dy);
  }
  void rasterize_world_line(double x1,double y1,double x2,double y2){
    int gx1,gy1,gx2,gy2;
    if(!worldToGrid(x1,y1,gx1,gy1) || !worldToGrid(x2,y2,gx2,gy2)) return;
    int x=gx1, y=gy1, dx=std::abs(gx2-gx1), sx=gx1<gx2?1:-1;
    int dy=-std::abs(gy2-gy1), sy=gy1<gy2?1:-1, err=dx+dy;
    while(true){
      paint_brush(x,y);
      if(x==gx2 && y==gy2) break;
      int e2=2*err;
      if(e2>=dy){ err+=dy; x+=sx; }
      if(e2<=dx){ err+=dx; y+=sy; }
    }
  }

  // ======== free carving (не трогаем стены) ========
  void clear_brush(int gx,int gy){
    const int r = std::max(0, P_.free_stroke_cells);
    for(int dy=-r; dy<=r; ++dy){
      for(int dx=-r; dx<=r; ++dx){
        int x = gx+dx, y = gy+dy;
        if(!inBounds(x,y)) continue;
        int id = idx(x,y,width_);
        if(occ_msg_.data[id] != 100) // не затираем стену
          occ_msg_.data[id] = 0;
      }
    }
  }

  void clear_rect_world(double x1,double y1,double x2,double y2){
    if(x2<x1) std::swap(x1,x2);
    if(y2<y1) std::swap(y1,y2);
    const double m = std::max(P_.corridor_margin, P_.grid_res);
    x1 += m; x2 -= m; y1 += m; y2 -= m;
    if(x2<=x1 || y2<=y1) return;

    int gx1 = (int)std::floor((x1 - origin_x_) / P_.grid_res);
    int gx2 = (int)std::floor((x2 - origin_x_) / P_.grid_res);
    int gy1 = (int)std::floor((y1 - origin_y_) / P_.grid_res);
    int gy2 = (int)std::floor((y2 - origin_y_) / P_.grid_res);
    gx1 = std::max(0, std::min(gx1, width_-1));
    gx2 = std::max(0, std::min(gx2, width_-1));
    gy1 = std::max(0, std::min(gy1, height_-1));
    gy2 = std::max(0, std::min(gy2, height_-1));

    for(int gy=gy1; gy<=gy2; ++gy){
      for(int gx=gx1; gx<=gx2; ++gx){
        int id = idx(gx,gy,width_);
        if(occ_msg_.data[id] != 100) // не затираем стену
          occ_msg_.data[id] = 0;
      }
    }
  }

  // лучевая очистка: до стены, не затирая занятое (если не разрешено параметром)
  void clear_ray_world(double x0,double y0,double x1,double y1){
    if(!P_.ray_clear) return;

    int gx0,gy0,gx1,gy1;
    if(!worldToGrid(x0,y0,gx0,gy0) || !worldToGrid(x1,y1,gx1,gy1)) return;

    int x=gx0, y=gy0, dx=std::abs(gx1-gx0), sx=gx0<gx1?1:-1;
    int dy=-std::abs(gy1-gy0), sy=gy0<gy1?1:-1, err=dx+dy;

    while(true){
      double cx = origin_x_ + (x + 0.5)*P_.grid_res;
      double cy = origin_y_ + (y + 0.5)*P_.grid_res;
      double dist_to_end = std::hypot(cx - x1, cy - y1);
      if(dist_to_end <= P_.ray_margin) break;

      if(inBounds(x,y)){
        int id = idx(x,y,width_);
        if(P_.ray_override_occupied || occ_msg_.data[id] != 100){
          clear_brush(x,y);
        }
      }

      if(x==gx1 && y==gy1) break;
      int e2=2*err;
      if(e2>=dy){ err+=dy; x+=sx; }
      if(e2<=dx){ err+=dx; y+=sy; }
    }
  }

  // corner quadrant free
  void clear_corner_quadrant(double xc,double yc,int sx,int sy){
    const double L = P_.cell;
    double x1 = xc + (sx>0 ? 0.0 : -L);
    double x2 = xc + (sx>0 ?  L  :  0.0);
    double y1 = yc + (sy>0 ? 0.0 : -L);
    double y2 = yc + (sy>0 ?  L  :  0.0);
    clear_rect_world(x1,y1,x2,y2);
  }

  // при активации сегмента: рисуем стену и чистим коридоры/углы
  void on_segment_activated(int i,int j,bool hor){
    double x1,y1,x2,y2; segEndpoints(i,j,hor,x1,y1,x2,y2);
    rasterize_world_line(x1,y1,x2,y2);

    const double L = P_.cell, X=P_.x0, Y=P_.y0;
    // параллельные соседние -> очистка прямоугольника между ними
    if(hor){
      if(isActive(i, j-1, true)) clear_rect_world(X+i*L, Y+(j-1)*L, X+(i+1)*L, Y+j*L);
      if(isActive(i, j+1, true)) clear_rect_world(X+i*L, Y+j*L,     X+(i+1)*L, Y+(j+1)*L);
    }else{
      if(isActive(i-1, j, false)) clear_rect_world(X+(i-1)*L, Y+j*L, X+i*L,     Y+(j+1)*L);
      if(isActive(i+1, j, false)) clear_rect_world(X+i*L,     Y+j*L, X+(i+1)*L, Y+(j+1)*L);
    }

    // углы
    double x_i = X + i*L, x_ip1 = X + (i+1)*L;
    double y_j = Y + j*L, y_jp1 = Y + (j+1)*L;
    if(hor){
      if(isActive(i,   j,   false)) clear_corner_quadrant(x_i,  y_j,   +1, +1);
      if(isActive(i+1, j,   false)) clear_corner_quadrant(x_ip1,y_j,   -1, +1);
      if(isActive(i,   j+1, false)) clear_corner_quadrant(x_i,  y_jp1, +1, -1);
      if(isActive(i+1, j+1, false)) clear_corner_quadrant(x_ip1,y_jp1, -1, -1);
    }else{
      if(isActive(i,   j,   true)) clear_corner_quadrant(x_i,  y_j,   +1, +1);
      if(isActive(i-1, j,   true)) clear_corner_quadrant(x_i,  y_j,   -1, +1);
      if(isActive(i,   j+1, true)) clear_corner_quadrant(x_i,  y_jp1, +1, -1);
      if(isActive(i-1, j+1, true)) clear_corner_quadrant(x_i,  y_jp1, -1, -1);
    }
  }

  void update_from_segment(int i,int j,bool hor,const EvalOut& ev){
    if(ev.cover_frac < P_.cover_frac_min) return;

    const bool wall_by_run = (ev.max_run_len >= P_.min_wall_frac * P_.cell);
    const double logit_hit_seg  = logit_from_p(P_.p_hit);
    const double logit_miss_seg = logit_from_p(P_.p_miss);

    auto& s = getSeg(i,j,hor);
    const bool was_active = (s.logit > P_.L_thresh);

    if(wall_by_run){
      s.logit += logit_hit_seg * ev.cover_frac;
      clamp(s.logit, P_.lmin, P_.lmax);
      s.miss_streak = 0;
    }else{
      if(ev.cover_frac >= P_.free_cover_min){
        s.miss_streak++;
        if(s.miss_streak >= P_.miss_streak_forget){
          s.logit += logit_miss_seg * ev.cover_frac;
          clamp(s.logit, P_.lmin, P_.lmax);
          s.miss_streak = 0;
        }
      }
    }

    const bool now_active = (s.logit > P_.L_thresh);
    if(!was_active && now_active){
      on_segment_activated(i,j,hor); // стена рисуется и коридор/углы чистятся сразу
    }
  }

  void publishMarkers(const rclcpp::Time& t){
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id=frame_map_; m.header.stamp=t;
    m.ns="segments"; m.id=0;
    m.type=visualization_msgs::msg::Marker::LINE_LIST;
    m.action=visualization_msgs::msg::Marker::ADD;
    m.scale.x=0.03;
    m.color.a=1.0; m.color.r=0.2; m.color.g=0.8; m.color.b=0.2;

    for(const auto& kv: segs_){
      const auto& s = kv.second;
      if(s.logit <= P_.L_thresh) continue;
      double x1,y1,x2,y2; segEndpoints(s.i,s.j,s.hor,x1,y1,x2,y2);
      geometry_msgs::msg::Point p1, p2;
      p1.x=x1; p1.y=y1; p2.x=x2; p2.y=y2;
      m.points.push_back(p1); m.points.push_back(p2);
    }
    arr.markers.push_back(m);
    pub_markers_->publish(arr);
  }

  void build_map_and_publish(const rclcpp::Time& t,
                             const Pose2& pose,
                             const std::vector<std::pair<double,double>>& pts)
  {
    occ_msg_.header.stamp = t;

    if(!P_.persist_map){
      // Перестраиваем кадр: стены, коридоры/углы, лучи
      std::fill(occ_msg_.data.begin(), occ_msg_.data.end(), 0);

      // стены из активных сегментов
      for(const auto& kv: segs_){
        const auto& s = kv.second;
        if(s.logit <= P_.L_thresh) continue;
        double x1,y1,x2,y2; segEndpoints(s.i,s.j,s.hor,x1,y1,x2,y2);
        rasterize_world_line(x1,y1,x2,y2);
      }

      // параллельные коридоры
      const double L = P_.cell, X=P_.x0, Y=P_.y0;
      for(const auto& kv: segs_){
        const auto& s = kv.second;
        if(s.logit<=P_.L_thresh) continue;
        if(s.hor){
          if(isActive(s.i, s.j+1, true)) clear_rect_world(X+s.i*L, Y+s.j*L, X+(s.i+1)*L, Y+(s.j+1)*L);
        }else{
          if(isActive(s.i+1, s.j, false)) clear_rect_world(X+s.i*L, Y+s.j*L, X+(s.i+1)*L, Y+(s.j+1)*L);
        }
      }
      // углы
      for(const auto& kv: segs_){
        const auto& s = kv.second;
        if(s.logit<=P_.L_thresh) continue;
        double x_i = P_.x0 + s.i*P_.cell, x_ip1 = P_.x0 + (s.i+1)*P_.cell;
        double y_j = P_.y0 + s.j*P_.cell, y_jp1 = P_.y0 + (s.j+1)*P_.cell;
        if(s.hor){
          if(isActive(s.i,   s.j,   false)) clear_corner_quadrant(x_i,  y_j,   +1, +1);
          if(isActive(s.i+1, s.j,   false)) clear_corner_quadrant(x_ip1,y_j,   -1, +1);
          if(isActive(s.i,   s.j+1, false)) clear_corner_quadrant(x_i,  y_jp1, +1, -1);
          if(isActive(s.i+1, s.j+1, false)) clear_corner_quadrant(x_ip1,y_jp1, -1, -1);
        }else{
          if(isActive(s.i,   s.j,   true)) clear_corner_quadrant(x_i,  y_j,   +1, +1);
          if(isActive(s.i-1, s.j,   true)) clear_corner_quadrant(x_i,  y_j,   -1, +1);
          if(isActive(s.i,   s.j+1, true)) clear_corner_quadrant(x_i,  y_jp1, +1, -1);
          if(isActive(s.i-1, s.j+1, true)) clear_corner_quadrant(x_i,  y_jp1, -1, -1);
        }
      }

      // лучи до наблюдаемых точек
      if(P_.ray_clear){
        for(const auto& p: pts){
          clear_ray_world(pose.x, pose.y, p.first, p.second);
        }
      }
    }else{
      // Накопительный режим: стены уже нарисованы при активации, чистим лучами
      if(P_.ray_clear){
        for(const auto& p: pts){
          clear_ray_world(pose.x, pose.y, p.first, p.second);
        }
      }
    }

    pub_occ_->publish(occ_msg_);
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    Pose2 pose;
    if(!getPose(msg->header.stamp, pose)) return;

    // scan points in map
    std::vector<std::pair<double,double>> pts;
    pts.reserve(msg->ranges.size());
    const double ang_min = msg->angle_min;
    const double ang_inc = msg->angle_increment;
    const double ang_max = msg->angle_max;
    for(size_t k=0;k<msg->ranges.size();++k){
      double r = msg->ranges[k];
      if(!std::isfinite(r) || r<=msg->range_min || r>=P_.range_max) continue;
      double a_map = pose.yaw + (ang_min + k*ang_inc);
      pts.emplace_back(pose.x + r*std::cos(a_map),
                       pose.y + r*std::sin(a_map));
    }

    // segment indices around the robot
    const double L = P_.cell;
    const double gx = (pose.x - P_.x0)/L;
    const double gy = (pose.y - P_.y0)/L;
    const double R  = P_.map_radius/L;
    int Imin = (int)std::floor(gx - R) - 1;
    int Imax = (int)std::ceil (gx + R) + 1;
    int Jmin = (int)std::floor(gy - R) - 1;
    int Jmax = (int)std::ceil (gy + R) + 1;

    // update segments (и возможная активация со стеной)
    for(int i=Imin;i<=Imax;++i){
      for(int j=Jmin;j<=Jmax;++j){
        auto e1 = evalSegment(pose, pts, i,j, true,  ang_min,ang_max);
        update_from_segment(i,j, true,  e1);
        auto e2 = evalSegment(pose, pts, i,j, false, ang_min,ang_max);
        update_from_segment(i,j, false, e2);
      }
    }

    publishMarkers(msg->header.stamp);
    build_map_and_publish(msg->header.stamp, pose, pts);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occ_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<SegmentGridMapper>());
  rclcpp::shutdown();
  return 0;
}
