// localization_node: where the robot is in a map of the room's walls and
// furniture, from the two crossed lidars (docs/LOCALIZATION.md).
//
// Subscribes (relative, /dog namespace):
//   lidar_left/scan, lidar_right/scan  sensor_msgs/LaserScan
//   imu/data                           roll and pitch to level the points
//   odom                               dead reckoning (remap in simulation: there
//                                      "odom" is the true pose)
//   state                              locomotion state: points only on the legs;
//                                      the end of a survey starts the relocalization
//   localization/command               std_msgs/String: save | relocalize | reset
// Publishes:
//   localization/pose     geometry_msgs/PoseStamped  base_link in the map frame (at odom rate)
//   localization/status   std_msgs/String  JSON {mode, status, inliers, points, cells, x, y, yaw}
//   localization/map      nav_msgs/OccupancyGrid  the walls (latched, when they change)
//   tf                    map -> odom
//
// Modes (localization.mode): mapping - start an empty map where the robot
// stands, grow it as it walks and save it on exit or on "save";
// localize - load the map, find the robot in it (best after a survey: the
// swaying lidars sweep the whole room), then follow it; auto - localize if
// the map file exists, else mapping.
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "dog_perception/core.hpp"
#include "dog_perception/localization.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace dog_perception
{

namespace
{
double stampSec(const builtin_interfaces::msg::Time & t) {return t.sec + t.nanosec * 1e-9;}

double yawOf(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

std::string expandHome(const std::string & p)
{
  if (p.rfind("~/", 0) == 0) {
    const char * home = std::getenv("HOME");
    return std::string(home ? home : "") + p.substr(1);
  }
  return p;
}

struct OdomSample
{
  double t;
  Pose2 pose;
  double z;
};
}  // namespace

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode()
  : rclcpp::Node("localization")
  {
    SensorParams s;
    s.x_lidar_x = getD("sensors.x_lidar_x", s.x_lidar_x);
    s.x_lidar_y = getD("sensors.x_lidar_y", s.x_lidar_y);
    s.x_lidar_z = getD("sensors.x_lidar_z", s.x_lidar_z);
    s.x_lidar_tilt_deg = getD("sensors.x_lidar_tilt_deg", s.x_lidar_tilt_deg);
    s.x_lidar_yaw_deg = getD("sensors.x_lidar_yaw_deg", s.x_lidar_yaw_deg);
    s.tof = false;
    s.gs2 = false;
    mounts_ = mountsFromParams(s);

    const double res = getD("localization.resolution", 0.05);
    min_hits_ = getI("localization.min_hits", 2);
    z_min_ = getD("localization.z_min", 0.10);
    z_max_ = getD("localization.z_max", 2.0);
    range_min_ = getD("localization.range_min", 0.30);
    range_max_ = getD("localization.range_max", 8.0);
    cloud_time_ = getD("localization.cloud_time", 0.3);
    min_points_ = getI("localization.min_points", 30);
    min_inliers_ = getD("localization.min_inliers", 0.5);
    lost_after_ = getD("localization.lost_after", 3.0);
    reloc_wait_ = getD("localization.reloc_wait", 3.0);
    match_.prior_xy = getD("localization.prior_xy", match_.prior_xy);
    match_.prior_yaw = getD("localization.prior_yaw_deg", 5.0) * M_PI / 180.0;
    map_path_ = expandHome(declare_parameter("localization.map", std::string("")));
    save_on_exit_ = declare_parameter("localization.save_on_exit", true);
    const auto mode = declare_parameter("localization.mode", std::string("auto"));

    grid_ = WallGrid(res, min_hits_);
    const bool have_map = !map_path_.empty() && std::ifstream(map_path_ + ".yaml").good();
    if (mode == "localize" || (mode == "auto" && have_map)) {
      if (!have_map || !grid_.load(map_path_)) {
        RCLCPP_ERROR(get_logger(), "cannot load the map %s(.yaml/.pgm) - mapping instead", map_path_.c_str());
        grid_ = WallGrid(res, min_hits_);
      } else {
        mapping_ = false;
        status_ = "relocalizing";
        RCLCPP_INFO(get_logger(), "map %s: %d x %d cells, %d walls - finding the robot in it "
          "(a survey helps: command \"survey\")", map_path_.c_str(), grid_.width(), grid_.height(),
          grid_.occupiedCount());
      }
    }
    if (mapping_) {
      status_ = "tracking";
      RCLCPP_INFO(get_logger(), "mapping from here%s", map_path_.empty() ? " (no localization.map: not saved)" :
        (" into " + map_path_).c_str());
    }

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    const auto sensor = rclcpp::SensorDataQoS();
    subs_.push_back(create_subscription<std_msgs::msg::String>("state", latched,
      [this](std_msgs::msg::String::ConstSharedPtr m) {onState(m->data);}));
    subs_.push_back(create_subscription<sensor_msgs::msg::Imu>("imu/data", sensor,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        const auto & q = m->orientation;
        imu_hist_.emplace_back(stampSec(m->header.stamp), quatToRot(q.x, q.y, q.z, q.w));
        while (imu_hist_.size() > 400) {imu_hist_.pop_front();}
      }));
    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>("odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {onOdom(*m);}));
    for (const std::string name : {"lidar_left", "lidar_right"}) {
      subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(name + "/scan", sensor,
        [this, name](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {onScan(name, *m);}));
    }
    subs_.push_back(create_subscription<std_msgs::msg::String>("localization/command", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) {onCommand(m->data);}));
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("localization/pose", 10);
    pub_status_ = create_publisher<std_msgs::msg::String>("localization/status", latched);
    pub_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>("localization/map", latched);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
        publishStatus();
        if (map_changed_) {publishMap();}
      });
    publishMap();
  }

  ~LocalizationNode() override
  {
    if (mapping_ && save_on_exit_ && !map_path_.empty() && !grid_.empty()) {save();}
  }

private:
  double getD(const std::string & n, double d) {return declare_parameter(n, d);}
  int getI(const std::string & n, int d) {return static_cast<int>(declare_parameter<int64_t>(n, d));}

  // ---------------------------------------------------------- inputs
  void onState(const std::string & s)
  {
    // the survey is over: the lidars have swept the room - where are we?
    if (state_ == "survey" && s != "survey" && !mapping_ && status_ != "tracking") {
      reloc_now_ = true;
    }
    state_ = s;
  }

  bool legsOn() const {return state_ == "stand" || state_ == "walk" || state_ == "survey";}

  void onOdom(const nav_msgs::msg::Odometry & m)
  {
    const auto & p = m.pose.pose;
    const double t = stampSec(m.header.stamp);
    const Pose2 o{p.position.x, p.position.y,
      yawOf(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)};
    odom_hist_.push_back({t, o, p.position.z});
    while (!odom_hist_.empty() && odom_hist_.front().t < t - 5.0) {odom_hist_.pop_front();}
    // pose in the map: map <- odom <- base
    const Pose2 b = T_.compose(o);
    const double half = 0.5 * T_.yaw;
    const double qw = std::cos(half), qz = std::sin(half);  // rotation about z, times the odom attitude
    const auto & q = p.orientation;
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = m.header.stamp;
    ps.header.frame_id = "map";
    ps.pose.position.x = b.x;
    ps.pose.position.y = b.y;
    ps.pose.position.z = p.position.z;
    ps.pose.orientation.w = qw * q.w - qz * q.z;
    ps.pose.orientation.x = qw * q.x - qz * q.y;
    ps.pose.orientation.y = qw * q.y + qz * q.x;
    ps.pose.orientation.z = qw * q.z + qz * q.w;
    if (status_ == "tracking") {pub_pose_->publish(ps);}
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = m.header.stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = m.header.frame_id.empty() ? "odom" : m.header.frame_id;
    tf.transform.translation.x = T_.x;
    tf.transform.translation.y = T_.y;
    tf.transform.rotation.w = qw;
    tf.transform.rotation.z = qz;
    tf_->sendTransform(tf);
  }

  std::optional<M3> R_at(double t) const
  {
    if (imu_hist_.empty()) {return std::nullopt;}
    auto it = std::lower_bound(imu_hist_.begin(), imu_hist_.end(), t,
      [](const std::pair<double, M3> & h, double v) {return h.first < v;});
    if (it == imu_hist_.end()) {return imu_hist_.back().second;}
    if (it == imu_hist_.begin()) {return it->second;}
    const auto prev = std::prev(it);
    return (it->first - t < t - prev->first) ? it->second : prev->second;
  }

  /// Dead-reckoned pose at t (linear between samples).
  std::optional<OdomSample> odomAt(double t) const
  {
    if (odom_hist_.empty() || t > odom_hist_.back().t + 0.2 || t < odom_hist_.front().t) {return std::nullopt;}
    auto it = std::lower_bound(odom_hist_.begin(), odom_hist_.end(), t,
      [](const OdomSample & s, double v) {return s.t < v;});
    if (it == odom_hist_.end()) {return odom_hist_.back();}
    if (it == odom_hist_.begin()) {return *it;}
    const auto & a = *std::prev(it);
    const auto & b = *it;
    const double k = (t - a.t) / std::max(b.t - a.t, 1e-6);
    OdomSample s{t, {a.pose.x + k * (b.pose.x - a.pose.x), a.pose.y + k * (b.pose.y - a.pose.y),
        wrapAngle(a.pose.yaw + k * wrapAngle(b.pose.yaw - a.pose.yaw))}, a.z + k * (b.z - a.z)};
    return s;
  }

  void onScan(const std::string & name, const sensor_msgs::msg::LaserScan & msg)
  {
    if (!legsOn()) {return;}
    const double t = stampSec(msg.header.stamp);
    const auto R = R_at(t);
    const auto od = odomAt(t);
    if (!R || !od) {return;}
    // level the body by the IMU's roll and pitch (its yaw is not needed: the
    // heading comes from the odometry)
    const double yaw = std::atan2((*R)[1][0], (*R)[0][0]);
    const M3 level = mul(rotRpy(0.0, 0.0, -yaw), *R);
    std::vector<P2> pts;
    for (const auto & p : scanToBody(mounts_.at(name), msg.ranges, msg.angle_min, msg.angle_increment,
        msg.range_min, msg.range_max))
    {
      const double r = std::hypot(p.x, p.y);
      if (r < range_min_ || r > range_max_) {continue;}
      const V3 q = mul(level, p);
      const double above_floor = q.z + od->z;
      if (above_floor < z_min_ || above_floor > z_max_) {continue;}  // the floor, the ceiling
      pts.push_back(od->pose.apply({q.x, q.y}));
    }
    recent_.push_back({t, pts});
    while (!recent_.empty() && recent_.front().first < t - cloud_time_) {recent_.pop_front();}
    if (mapping_) {
      mapStep(t, pts);
    } else {
      localizeStep(t, pts);
    }
  }

  std::vector<P2> recentCloud() const
  {
    std::vector<P2> c;
    for (const auto & r : recent_) {c.insert(c.end(), r.second.begin(), r.second.end());}
    return c;
  }

  void mapStep(double t, const std::vector<P2> & pts)
  {
    if (!grid_.empty() && grid_.fieldValid()) {
      const auto cloud = recentCloud();
      if (static_cast<int>(cloud.size()) >= min_points_) {
        const auto r = match(grid_, cloud, T_, match_);
        last_ = r;
        if (r.ok) {T_ = r.pose;}
      }
    }
    std::vector<P2> w;
    w.reserve(pts.size());
    for (const auto & p : pts) {w.push_back(T_.apply(p));}
    grid_.insert(w);
    if (grid_.dirty() && (t - last_field_ > 0.5 || !grid_.fieldValid())) {
      grid_.updateField();
      last_field_ = t;
      map_changed_ = true;
    }
  }

  void localizeStep(double t, const std::vector<P2> & pts)
  {
    if (status_ == "tracking") {
      const auto cloud = recentCloud();
      if (static_cast<int>(cloud.size()) < min_points_) {return;}  // nothing but floor in view
      const auto r = match(grid_, cloud, T_, match_);
      last_ = r;
      if (r.ok && r.inlier_fraction >= min_inliers_) {
        T_ = r.pose;
        last_good_ = t;
      } else if (t - last_good_ > lost_after_) {
        status_ = "lost";
        RCLCPP_WARN(get_logger(), "lost: %.0f %% of %zu points on the walls for %.1f s - "
          "relocalizing (a survey helps)", 100.0 * r.inlier_fraction, cloud.size(), t - last_good_);
        reloc_.clear();
        reloc_t0_ = t;
        last_try_ = t;
      }
      return;
    }
    // relocalizing / lost: collect a cloud (dead reckoning holds it together)
    if (reloc_.empty()) {reloc_t0_ = t;}
    reloc_.insert(reloc_.end(), pts.begin(), pts.end());
    if (reloc_.size() > 20000) {reloc_ = thin(reloc_, 0.03);}
    const bool waited = state_ != "survey" && t - reloc_t0_ > 2.0 && t - last_try_ > reloc_wait_;
    if (!reloc_now_ && !waited) {return;}
    reloc_now_ = false;
    last_try_ = t;
    const auto cloud = thin(reloc_, grid_.resolution());
    const auto g = globalSearch(grid_, cloud, global_, match_);
    RCLCPP_INFO(get_logger(), "relocalization over %zu points: %s (%.0f %% on the walls, next best %.0f %%) "
      "at x %.2f y %.2f yaw %.0f deg", cloud.size(), g.ok ? "found" : "not sure", 100.0 * g.score,
      100.0 * g.second, g.best.pose.x, g.best.pose.y, g.best.pose.yaw * 180.0 / M_PI);
    if (g.ok) {
      T_ = g.best.pose;
      last_ = g.best;
      status_ = "tracking";
      last_good_ = t;
      reloc_.clear();
    } else if (t - reloc_t0_ > 30.0) {
      reloc_ = thin(recentCloud(), 0.03);  // start collecting afresh
      reloc_t0_ = t;
    }
  }

  void onCommand(const std::string & c)
  {
    if (c == "save") {
      save();
    } else if (c == "relocalize" && !mapping_) {
      status_ = "relocalizing";
      reloc_.clear();
      last_try_ = 0.0;
      RCLCPP_INFO(get_logger(), "relocalizing");
    } else if (c == "reset") {
      grid_ = WallGrid(grid_.resolution(), min_hits_);
      T_ = Pose2{};
      if (!odom_hist_.empty()) {T_ = odom_hist_.back().pose.inverse();}  // the map starts where the robot is
      mapping_ = true;
      status_ = "tracking";
      map_changed_ = true;
      RCLCPP_INFO(get_logger(), "new map from here");
    } else {
      RCLCPP_WARN(get_logger(), "unknown command '%s' (save | relocalize | reset)", c.c_str());
    }
  }

  void save()
  {
    if (map_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "no localization.map path - not saved");
      return;
    }
    if (grid_.save(map_path_)) {
      RCLCPP_INFO(get_logger(), "map saved: %s.pgm/.yaml/.walls (%d walls)", map_path_.c_str(), grid_.occupiedCount());
    } else {
      RCLCPP_ERROR(get_logger(), "cannot write %s", map_path_.c_str());
    }
  }

  // ---------------------------------------------------------- outputs
  void publishStatus()
  {
    const Pose2 b = odom_hist_.empty() ? T_ : T_.compose(odom_hist_.back().pose);
    std::ostringstream o;
    o.precision(4);
    o << "{\"mode\": \"" << (mapping_ ? "mapping" : "localize") << "\", \"status\": \"" << status_
      << "\", \"inliers\": " << last_.inlier_fraction << ", \"points\": " << last_.points
      << ", \"rms\": " << last_.rms << ", \"cells\": " << grid_.occupiedCount()
      << ", \"x\": " << b.x << ", \"y\": " << b.y << ", \"yaw\": " << b.yaw << "}";
    std_msgs::msg::String m;
    m.data = o.str();
    pub_status_->publish(m);
  }

  void publishMap()
  {
    map_changed_ = false;
    nav_msgs::msg::OccupancyGrid g;
    g.header.stamp = now();
    g.header.frame_id = "map";
    g.info.resolution = static_cast<float>(grid_.resolution());
    g.info.width = static_cast<uint32_t>(grid_.width());
    g.info.height = static_cast<uint32_t>(grid_.height());
    g.info.origin.position.x = grid_.origin().x;
    g.info.origin.position.y = grid_.origin().y;
    g.info.origin.orientation.w = 1.0;
    g.data = grid_.occupancy();
    pub_map_->publish(g);
  }

  std::map<std::string, SensorMount> mounts_;
  WallGrid grid_;
  MatchParams match_;
  GlobalParams global_;
  Pose2 T_;  // map <- odom
  MatchResult last_;
  bool mapping_{true};
  std::string status_{"tracking"}, state_, map_path_;
  bool save_on_exit_{true}, map_changed_{true}, reloc_now_{false};
  int min_hits_{2}, min_points_{30};
  double z_min_{0.1}, z_max_{2.0}, range_min_{0.3}, range_max_{8.0}, cloud_time_{0.3};
  double min_inliers_{0.5}, lost_after_{3.0}, reloc_wait_{3.0};
  double last_field_{-1e9}, last_good_{0.0}, reloc_t0_{0.0}, last_try_{0.0};
  std::deque<std::pair<double, M3>> imu_hist_;
  std::deque<OdomSample> odom_hist_;
  std::deque<std::pair<double, std::vector<P2>>> recent_;
  std::vector<P2> reloc_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

}  // namespace dog_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<dog_perception::LocalizationNode>();
    rclcpp::spin(node);
  }  // saves the map (mapping) before shutdown
  rclcpp::shutdown();
  return 0;
}
