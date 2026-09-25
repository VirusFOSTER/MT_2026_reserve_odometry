#include "parser_node.hpp"
#include <fstream>


CDataParser::CDataParser(const rclcpp::NodeOptions & options) : rclcpp::Node("data_parser_node", options) {
    this->gnss_master_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "input_nav_sat_fix", rclcpp::SensorDataQoS().keep_last(1),
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg){ this->process_gnss_master_fix(msg); });
    {
      std::ofstream out_data_(this->gnss_master_fix_file_, std::ios_base::trunc | std::ios_base::out);
      out_data_ << "time_sec,frame_id,status,service,"
                << "latitude,longitude,altitude,"
                << "cov_xx,cov_xy,cov_xz,cov_yx,cov_yy,cov_yz,cov_zx,cov_zy,cov_zz,"
                << "position_covariance_type\n";
      out_data_.close();
    }


    this->gnss_master_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "input_gnss_master_vel", rclcpp::SensorDataQoS().keep_last(1),
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg){ this->process_gnss_master_vel(msg); });
    {
      std::ofstream out_data_(this->gnss_master_vel_file_, std::ios_base::trunc | std::ios_base::out);
      out_data_ << "time_sec,frame_id,"
                << "linear_x,linear_y,linear_z,"
                << "angular_x,angular_y,angular_z\n";
      out_data_.close();
    }

    this->gnss_rover_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "input_gnss_rover_fix", rclcpp::SensorDataQoS().keep_last(1),
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg){ this->process_gnss_rover_fix(msg); });
    {
      std::ofstream out_data_(this->gnss_rover_fix_file_, std::ios_base::trunc | std::ios_base::out);
      out_data_ << "time_sec,frame_id,status,service,"
                << "latitude,longitude,altitude,"
                << "cov_xx,cov_xy,cov_xz,cov_yx,cov_yy,cov_yz,cov_zx,cov_zy,cov_zz,"
                << "position_covariance_type\n";
      out_data_.close();
    }

    this->gnss_rover_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "input_gnss_rover_vel", rclcpp::SensorDataQoS().keep_last(1),
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg){ this->process_gnss_rover_vel(msg); });
    {
      std::ofstream out_data_(this->gnss_rover_vel_file_, std::ios_base::trunc | std::ios_base::out);
      out_data_ << "time_sec,frame_id,"
                << "linear_x,linear_y,linear_z,"
                << "angular_x,angular_y,angular_z\n";
      out_data_.close();
    }

    this->rear_bogie_vel_sub_ = this->create_subscription<tram_vehicle_msgs::msg::VelocitySensor>(
      "input_rear_bogie_vel", rclcpp::SensorDataQoS().keep_last(1),
      [this](const tram_vehicle_msgs::msg::VelocitySensor::SharedPtr msg){ this->process_rear_bogie_vel(msg); });
    {
      std::ofstream out_data_(this->rear_bogie_vel_file_, std::ios_base::trunc | std::ios_base::out);
      out_data_ << "time_sec,frame_id,"
      << "velocity\n";
      out_data_.close();
    }

    this->driver_position_cmd_sub_ = this->create_subscription<tram_vehicle_msgs::msg::DriverControllerCommand>(
      "input_driver_position_cmd", rclcpp::SensorDataQoS().keep_last(1),
      [this](const tram_vehicle_msgs::msg::DriverControllerCommand::SharedPtr msg){ this->process_driver_position_cmd(msg); });
    {
      std::ofstream out_data_(this->driver_position_cmd_file_, std::ios_base::trunc | std::ios_base::out);
      out_data_ << "time_sec,frame_id,"
      << "type\n";
      out_data_.close();
    }

    this->front_bogie_velocity_sub_ = this->create_subscription<tram_vehicle_msgs::msg::VelocitySensor>(
      "input_front_bogie_vel", rclcpp::SensorDataQoS().keep_last(1),
      [this](const tram_vehicle_msgs::msg::VelocitySensor::SharedPtr msg){ this->process_front_bogie_velocity(msg); });
    {
      std::ofstream out_data_(this->front_bogie_velocity_file_, std::ios_base::trunc | std::ios_base::out);
      out_data_ << "time_sec,frame_id,"
      << "velocity\n";
      out_data_.close();
    }
}

void CDataParser::process_gnss_master_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),"Get gnss master fix");
    std::ofstream out_data_(this->gnss_master_fix_file_, std::ios_base::app | std::ios_base::out);

    const auto& stamp = msg->header.stamp;
    const double time_sec = static_cast<double>(stamp.sec) +
    static_cast<double>(stamp.nanosec) * 1e-9;

    out_data_ << std::fixed << std::setprecision(9)
        << time_sec << ","
        << msg->header.frame_id << ","
        << static_cast<int>(msg->status.status) << ","
        << msg->status.service << ","
        << msg->latitude << ","
        << msg->longitude << ","
        << msg->altitude << ",";

    for (size_t i = 0; i < 9; ++i) {
        out_data_ << msg->position_covariance[i];
        if (i < 8) out_data_ << ",";
    }

    out_data_ << "," << static_cast<int>(msg->position_covariance_type) << "\n";

    out_data_.close();
}

void CDataParser::process_gnss_master_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),"Get gnss master vel");
    std::ofstream out_data_(this->gnss_master_vel_file_, std::ios_base::app | std::ios_base::out);

    const auto& stamp = msg->header.stamp;
    const double time_sec = static_cast<double>(stamp.sec) +
    static_cast<double>(stamp.nanosec) * 1e-9;

    out_data_<< std::fixed << std::setprecision(9)
        << time_sec << ","
        << msg->header.frame_id << ","
        << msg->twist.linear.x << ","
        << msg->twist.linear.y << ","
        << msg->twist.linear.z << ","
        << msg->twist.angular.x << ","
        << msg->twist.angular.y << ","
        << msg->twist.angular.z << "\n";

    out_data_.close();
}

void CDataParser::process_gnss_rover_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),"Get gnss rover fix");
  std::ofstream out_data_(this->gnss_rover_fix_file_, std::ios_base::app | std::ios_base::out);

  const auto& stamp = msg->header.stamp;
  const double time_sec = static_cast<double>(stamp.sec) +
  static_cast<double>(stamp.nanosec) * 1e-9;

  out_data_ << std::fixed << std::setprecision(9)
      << time_sec << ","
      << msg->header.frame_id << ","
      << static_cast<int>(msg->status.status) << ","
      << msg->status.service << ","
      << msg->latitude << ","
      << msg->longitude << ","
      << msg->altitude << ",";

  for (size_t i = 0; i < 9; ++i) {
    out_data_ << msg->position_covariance[i];
    if (i < 8) out_data_ << ",";
  }

  out_data_ << "," << static_cast<int>(msg->position_covariance_type) << "\n";

  out_data_.close();
}

void CDataParser::process_gnss_rover_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),"Get gnss rover vel");
  std::ofstream out_data_(this->gnss_rover_vel_file_, std::ios_base::app | std::ios_base::out);

  const auto& stamp = msg->header.stamp;
  const double time_sec = static_cast<double>(stamp.sec) +
  static_cast<double>(stamp.nanosec) * 1e-9;

  out_data_ << std::fixed << std::setprecision(9)
      << time_sec << ","
      << msg->header.frame_id << ","
      << msg->twist.linear.x << ","
      << msg->twist.linear.y << ","
      << msg->twist.linear.z << ","
      << msg->twist.angular.x << ","
      << msg->twist.angular.y << ","
      << msg->twist.angular.z << "\n";

  out_data_.close();
}

void CDataParser::process_rear_bogie_vel(const tram_vehicle_msgs::msg::VelocitySensor::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),"Get rear_bogie_vel");
  std::ofstream out_data_(this->rear_bogie_vel_file_, std::ios_base::app | std::ios_base::out);

  const auto& stamp = msg->header.stamp;
  const double time_sec = static_cast<double>(stamp.sec) +
  static_cast<double>(stamp.nanosec) * 1e-9;

  out_data_ << time_sec << ","
      << msg->header.frame_id << ","
      << msg->velocity << "\n";

  out_data_.close();
}

void CDataParser::process_driver_position_cmd(const tram_vehicle_msgs::msg::DriverControllerCommand::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),"Get driver_position_cmd");
  std::ofstream out_data_(this->driver_position_cmd_file_, std::ios_base::app | std::ios_base::out);

  const auto& stamp = msg->header.stamp;
  const double time_sec = static_cast<double>(stamp.sec) +
  static_cast<double>(stamp.nanosec) * 1e-9;

  out_data_ << time_sec << ","
  << msg->header.frame_id << ","
  << (int)msg->position << "\n";

  out_data_.close();
}

void CDataParser::process_front_bogie_velocity(const tram_vehicle_msgs::msg::VelocitySensor::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),"Get front_bogie_vel");
  std::ofstream out_data_(this->front_bogie_velocity_file_, std::ios_base::app | std::ios_base::out);

  const auto& stamp = msg->header.stamp;
  const double time_sec = static_cast<double>(stamp.sec) +
  static_cast<double>(stamp.nanosec) * 1e-9;

  out_data_ << time_sec << ","
      << msg->header.frame_id << ","
      << msg->velocity << "\n";

  out_data_.close();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CDataParser)
