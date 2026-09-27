#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tram_vehicle_msgs/msg/velocity_sensor.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <GeographicLib/UTMUPS.hpp>

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

class MetricsEvaluator : public rclcpp::Node
{
public:
  MetricsEvaluator() : Node("metrics_evaluator")
  {
    match_tolerance_sec_ = declare_parameter<double>("match_tolerance_sec", 0.05);
    csv_path_ = declare_parameter<std::string>("csv_path", "metrics_samples.csv");
    summary_path_ = declare_parameter<std::string>("summary_path", "metrics_summary.csv");

    // Geometry supplied by organizers, expressed in base_link.
    master_x_ = declare_parameter<double>("master_x", -9.873);
    rover_x_ = declare_parameter<double>("rover_x", 2.563);
    antenna_z_ = declare_parameter<double>("antenna_z", 3.0);

    // Organizer's local metric frame: x = UTM_E - 300000, y = UTM_N - 6100000.
    origin_easting_ = declare_parameter<double>("origin_easting", 300000.0);
    origin_northing_ = declare_parameter<double>("origin_northing", 6100000.0);

    result_velocity_sub_ = create_subscription<tram_vehicle_msgs::msg::VelocitySensor>(
      "/result/velocity", 100,
      std::bind(&MetricsEvaluator::OnResultVelocity, this, std::placeholders::_1));

    result_position_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/result/position", 100,
      std::bind(&MetricsEvaluator::OnResultPosition, this, std::placeholders::_1));

    // Used only for an approximate observed input->result delay measurement.
    front_input_sub_ = create_subscription<tram_vehicle_msgs::msg::VelocitySensor>(
      "/vehicle/front_bogie_velocity", 100,
      std::bind(&MetricsEvaluator::OnFrontInput, this, std::placeholders::_1));

    master_fix_sub_.subscribe(this, "/sensing/gnss/master/fix", rmw_qos_profile_sensor_data);
    rover_fix_sub_.subscribe(this, "/sensing/gnss/rover/fix", rmw_qos_profile_sensor_data);
    fix_sync_ = std::make_shared<FixSynchronizer>(FixSyncPolicy(50), master_fix_sub_, rover_fix_sub_);
    fix_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.10));
    fix_sync_->registerCallback(
      std::bind(&MetricsEvaluator::OnGnssFixPair, this,
                std::placeholders::_1, std::placeholders::_2));

    master_vel_sub_.subscribe(this, "/sensing/gnss/master/vel", rmw_qos_profile_sensor_data);
    rover_vel_sub_.subscribe(this, "/sensing/gnss/rover/vel", rmw_qos_profile_sensor_data);
    vel_sync_ = std::make_shared<VelSynchronizer>(VelSyncPolicy(50), master_vel_sub_, rover_vel_sub_);
    vel_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.10));
    vel_sync_->registerCallback(
      std::bind(&MetricsEvaluator::OnGnssVelPair, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Metrics evaluator started. Results will be written on shutdown.");
  }

  ~MetricsEvaluator() override
  {
    try {
      EvaluateAndSave();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to save metrics: %s", e.what());
    }
  }

private:
  using Fix = sensor_msgs::msg::NavSatFix;
  using Vel = geometry_msgs::msg::TwistStamped;
  using FixSyncPolicy = message_filters::sync_policies::ApproximateTime<Fix, Fix>;
  using VelSyncPolicy = message_filters::sync_policies::ApproximateTime<Vel, Vel>;
  using FixSynchronizer = message_filters::Synchronizer<FixSyncPolicy>;
  using VelSynchronizer = message_filters::Synchronizer<VelSyncPolicy>;
  using SteadyTime = std::chrono::steady_clock::time_point;

  struct ScalarSample { int64_t t_ns; double value; };
  struct PositionSample { int64_t t_ns; double x; double y; double z; };
  struct VelocityErrorRow { double t; double est; double gt; double error; };
  struct PositionErrorRow { double t; double est_x; double est_y; double est_z; double gt_x; double gt_y; double gt_z; double e2d; double e3d; };

  static int64_t StampNs(const builtin_interfaces::msg::Time &stamp)
  {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
  }

  static int64_t MeanStampNs(const builtin_interfaces::msg::Time &a,
                             const builtin_interfaces::msg::Time &b)
  {
    return StampNs(a) / 2 + StampNs(b) / 2 + (StampNs(a) % 2 + StampNs(b) % 2) / 2;
  }

  static double SpeedMagnitude(const geometry_msgs::msg::Vector3 &v)
  {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  }

  void GnssToMap(double lat, double lon, double &x, double &y) const
  {
    int zone = 0;
    bool northp = true;
    double easting = 0.0;
    double northing = 0.0;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, easting, northing);
    x = easting - origin_easting_;
    y = northing - origin_northing_;
  }

  void OnGnssVelPair(const Vel::ConstSharedPtr &master,
                     const Vel::ConstSharedPtr &rover)
  {
    const double vm = SpeedMagnitude(master->twist.linear);
    const double vr = SpeedMagnitude(rover->twist.linear);
    gt_velocity_.push_back({MeanStampNs(master->header.stamp, rover->header.stamp), 0.5 * (vm + vr)});
  }

  void OnGnssFixPair(const Fix::ConstSharedPtr &master,
                     const Fix::ConstSharedPtr &rover)
  {
    if (!std::isfinite(master->latitude) || !std::isfinite(master->longitude) ||
        !std::isfinite(rover->latitude) || !std::isfinite(rover->longitude)) {
      return;
    }

    double mx, my, rx, ry;
    GnssToMap(master->latitude, master->longitude, mx, my);
    GnssToMap(rover->latitude, rover->longitude, rx, ry);

    const double yaw = std::atan2(ry - my, rx - mx);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    // Antenna offsets are [x_offset, 0] in base_link, therefore
    // P_base = P_antenna - R(yaw) * [x_offset, 0].
    const double base_m_x = mx - c * master_x_;
    const double base_m_y = my - s * master_x_;
    const double base_r_x = rx - c * rover_x_;
    const double base_r_y = ry - s * rover_x_;

    const double base_x = 0.5 * (base_m_x + base_r_x);
    const double base_y = 0.5 * (base_m_y + base_r_y);

    double base_z = std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(master->altitude) && std::isfinite(rover->altitude)) {
      base_z = 0.5 * (master->altitude + rover->altitude) - antenna_z_;
    }

    gt_position_.push_back({
      MeanStampNs(master->header.stamp, rover->header.stamp), base_x, base_y, base_z});
  }

  void OnFrontInput(const tram_vehicle_msgs::msg::VelocitySensor::SharedPtr msg)
  {
    const int64_t t = StampNs(msg->header.stamp);
    input_arrival_[t] = std::chrono::steady_clock::now();
    TryLatency(t);
  }

  void OnResultVelocity(const tram_vehicle_msgs::msg::VelocitySensor::SharedPtr msg)
  {
    const int64_t t = StampNs(msg->header.stamp);
    result_velocity_.push_back({t, msg->velocity});
    result_arrival_[t] = std::chrono::steady_clock::now();
    TryLatency(t);
  }

  void OnResultPosition(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    result_position_.push_back({
      StampNs(msg->header.stamp),
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z});
  }

  void TryLatency(int64_t stamp_ns)
  {
    auto in = input_arrival_.find(stamp_ns);
    auto out = result_arrival_.find(stamp_ns);
    if (in == input_arrival_.end() || out == result_arrival_.end()) return;

    const double ms = std::chrono::duration<double, std::milli>(out->second - in->second).count();
    // Independent DDS delivery to this observer can occasionally make this negative.
    if (ms >= 0.0 && ms < 10000.0) observed_latency_ms_.push_back(ms);
    input_arrival_.erase(in);
    result_arrival_.erase(out);
  }

  template<class Sample>
  static const Sample *Nearest(const std::vector<Sample> &samples, int64_t t_ns, int64_t tolerance_ns)
  {
    if (samples.empty()) return nullptr;
    auto it = std::lower_bound(samples.begin(), samples.end(), t_ns,
      [](const Sample &s, int64_t t) { return s.t_ns < t; });

    const Sample *best = nullptr;
    int64_t best_dt = std::numeric_limits<int64_t>::max();
    if (it != samples.end()) {
      const int64_t dt = std::llabs(it->t_ns - t_ns);
      if (dt < best_dt) { best = &*it; best_dt = dt; }
    }
    if (it != samples.begin()) {
      --it;
      const int64_t dt = std::llabs(it->t_ns - t_ns);
      if (dt < best_dt) { best = &*it; best_dt = dt; }
    }
    return best_dt <= tolerance_ns ? best : nullptr;
  }

  static double Mean(const std::vector<double> &v)
  {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
  }

  static double Rmse(const std::vector<double> &v)
  {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    double sum = 0.0;
    for (double x : v) sum += x * x;
    return std::sqrt(sum / static_cast<double>(v.size()));
  }

  static double Mae(const std::vector<double> &v)
  {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    double sum = 0.0;
    for (double x : v) sum += std::abs(x);
    return sum / static_cast<double>(v.size());
  }

  static double MaxAbs(const std::vector<double> &v)
  {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    double m = 0.0;
    for (double x : v) m = std::max(m, std::abs(x));
    return m;
  }

  template<class Sample>
  static double RateHz(const std::vector<Sample> &v)
  {
    if (v.size() < 2) return std::numeric_limits<double>::quiet_NaN();
    const double dt = static_cast<double>(v.back().t_ns - v.front().t_ns) * 1e-9;
    return dt > 0.0 ? static_cast<double>(v.size() - 1) / dt
                    : std::numeric_limits<double>::quiet_NaN();
  }

  void EvaluateAndSave()
  {
    if (saved_) return;
    saved_ = true;

    auto by_time = [](const auto &a, const auto &b) { return a.t_ns < b.t_ns; };
    std::sort(gt_velocity_.begin(), gt_velocity_.end(), by_time);
    std::sort(gt_position_.begin(), gt_position_.end(), by_time);
    std::sort(result_velocity_.begin(), result_velocity_.end(), by_time);
    std::sort(result_position_.begin(), result_position_.end(), by_time);

    const int64_t tol_ns = static_cast<int64_t>(match_tolerance_sec_ * 1e9);
    const int64_t t0_ns = !result_velocity_.empty() ? result_velocity_.front().t_ns :
                          (!result_position_.empty() ? result_position_.front().t_ns : 0);

    std::vector<VelocityErrorRow> velocity_rows;
    std::vector<double> velocity_errors;
    for (const auto &est : result_velocity_) {
      const auto *gt = Nearest(gt_velocity_, est.t_ns, tol_ns);
      if (!gt) continue;
      const double error = est.value - gt->value;
      velocity_rows.push_back({(est.t_ns - t0_ns) * 1e-9, est.value, gt->value, error});
      velocity_errors.push_back(error);
    }

    std::vector<PositionErrorRow> position_rows;
    std::vector<double> position_e2d;
    std::vector<double> position_e3d;
    for (const auto &est : result_position_) {
      const auto *gt = Nearest(gt_position_, est.t_ns, tol_ns);
      if (!gt) continue;
      const double dx = est.x - gt->x;
      const double dy = est.y - gt->y;
      const double e2d = std::hypot(dx, dy);
      double e3d = std::numeric_limits<double>::quiet_NaN();
      if (std::isfinite(est.z) && std::isfinite(gt->z)) {
        const double dz = est.z - gt->z;
        e3d = std::sqrt(dx * dx + dy * dy + dz * dz);
        position_e3d.push_back(e3d);
      }
      position_rows.push_back({(est.t_ns - t0_ns) * 1e-9,
                               est.x, est.y, est.z, gt->x, gt->y, gt->z, e2d, e3d});
      position_e2d.push_back(e2d);
    }

    const double vel_mae = Mae(velocity_errors);
    const double vel_rmse = Rmse(velocity_errors);
    const double vel_bias = Mean(velocity_errors);
    const double vel_max = MaxAbs(velocity_errors);

    const double pos_mae_2d = Mean(position_e2d);
    const double pos_rmse_2d = Rmse(position_e2d);
    const double pos_max_2d = position_e2d.empty() ? std::numeric_limits<double>::quiet_NaN()
                                                   : *std::max_element(position_e2d.begin(), position_e2d.end());
    const double pos_final_2d = position_e2d.empty() ? std::numeric_limits<double>::quiet_NaN()
                                                     : position_e2d.back();
    const double pos_rmse_3d = Rmse(position_e3d);

    const double velocity_rate = RateHz(result_velocity_);
    const double position_rate = RateHz(result_position_);
    const double latency_mean = Mean(observed_latency_ms_);
    const double latency_max = observed_latency_ms_.empty() ? std::numeric_limits<double>::quiet_NaN()
                                                            : *std::max_element(observed_latency_ms_.begin(), observed_latency_ms_.end());

    std::ofstream csv(csv_path_);
    csv << std::setprecision(12);
    csv << "kind,time_s,est_velocity,gt_velocity,velocity_error,est_x,est_y,est_z,gt_x,gt_y,gt_z,error_2d,error_3d\n";
    for (const auto &r : velocity_rows) {
      csv << "velocity," << r.t << ',' << r.est << ',' << r.gt << ',' << r.error
          << ",,,,,,,,\n";
    }
    for (const auto &r : position_rows) {
      csv << "position," << r.t << ",,,," << r.est_x << ',' << r.est_y << ',' << r.est_z
          << ',' << r.gt_x << ',' << r.gt_y << ',' << r.gt_z << ',' << r.e2d << ',' << r.e3d << '\n';
    }

    std::ofstream summary(summary_path_);
    summary << std::setprecision(12);
    summary << "metric,value,unit\n";
    summary << "matched_velocity," << velocity_rows.size() << ",samples\n";
    summary << "velocity_mae," << vel_mae << ",m/s\n";
    summary << "velocity_rmse," << vel_rmse << ",m/s\n";
    summary << "velocity_bias," << vel_bias << ",m/s\n";
    summary << "velocity_max_abs_error," << vel_max << ",m/s\n";
    summary << "matched_position," << position_rows.size() << ",samples\n";
    summary << "position_mae_2d," << pos_mae_2d << ",m\n";
    summary << "position_rmse_2d," << pos_rmse_2d << ",m\n";
    summary << "position_rmse_3d," << pos_rmse_3d << ",m\n";
    summary << "position_max_2d," << pos_max_2d << ",m\n";
    summary << "position_final_2d," << pos_final_2d << ",m\n";
    summary << "velocity_rate," << velocity_rate << ",Hz\n";
    summary << "position_rate," << position_rate << ",Hz\n";
    summary << "observed_latency_mean," << latency_mean << ",ms\n";
    summary << "observed_latency_max," << latency_max << ",ms\n";

    RCLCPP_INFO(get_logger(), "========== ODOMETRY EVALUATION ==========");
    RCLCPP_INFO(get_logger(), "Velocity matched: %zu", velocity_rows.size());
    RCLCPP_INFO(get_logger(), "  MAE  = %.6f m/s", vel_mae);
    RCLCPP_INFO(get_logger(), "  RMSE = %.6f m/s", vel_rmse);
    RCLCPP_INFO(get_logger(), "  Bias = %.6f m/s", vel_bias);
    RCLCPP_INFO(get_logger(), "  Max  = %.6f m/s", vel_max);
    RCLCPP_INFO(get_logger(), "Position matched: %zu", position_rows.size());
    RCLCPP_INFO(get_logger(), "  MAE 2D   = %.3f m", pos_mae_2d);
    RCLCPP_INFO(get_logger(), "  RMSE 2D  = %.3f m", pos_rmse_2d);
    RCLCPP_INFO(get_logger(), "  RMSE 3D  = %.3f m", pos_rmse_3d);
    RCLCPP_INFO(get_logger(), "  Max 2D   = %.3f m", pos_max_2d);
    RCLCPP_INFO(get_logger(), "  Final 2D = %.3f m", pos_final_2d);
    RCLCPP_INFO(get_logger(), "Output rate: velocity %.3f Hz, position %.3f Hz", velocity_rate, position_rate);
    RCLCPP_INFO(get_logger(), "Observed input->result delay: mean %.3f ms, max %.3f ms", latency_mean, latency_max);
    RCLCPP_WARN(get_logger(), "Observed delay is measured by an external ROS observer; it is not pure callback execution time.");
    RCLCPP_INFO(get_logger(), "Samples CSV: %s", csv_path_.c_str());
    RCLCPP_INFO(get_logger(), "Summary CSV: %s", summary_path_.c_str());
    RCLCPP_INFO(get_logger(), "=========================================");
  }

  double match_tolerance_sec_{};
  std::string csv_path_;
  std::string summary_path_;
  double master_x_{};
  double rover_x_{};
  double antenna_z_{};
  double origin_easting_{};
  double origin_northing_{};
  bool saved_{false};

  rclcpp::Subscription<tram_vehicle_msgs::msg::VelocitySensor>::SharedPtr result_velocity_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr result_position_sub_;
  rclcpp::Subscription<tram_vehicle_msgs::msg::VelocitySensor>::SharedPtr front_input_sub_;

  message_filters::Subscriber<Fix> master_fix_sub_;
  message_filters::Subscriber<Fix> rover_fix_sub_;
  message_filters::Subscriber<Vel> master_vel_sub_;
  message_filters::Subscriber<Vel> rover_vel_sub_;
  std::shared_ptr<FixSynchronizer> fix_sync_;
  std::shared_ptr<VelSynchronizer> vel_sync_;

  std::vector<ScalarSample> gt_velocity_;
  std::vector<PositionSample> gt_position_;
  std::vector<ScalarSample> result_velocity_;
  std::vector<PositionSample> result_position_;

  std::unordered_map<int64_t, SteadyTime> input_arrival_;
  std::unordered_map<int64_t, SteadyTime> result_arrival_;
  std::vector<double> observed_latency_ms_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<MetricsEvaluator>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
