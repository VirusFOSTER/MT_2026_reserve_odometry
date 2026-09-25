#ifndef DATA_PARSER_HPP
#define DATA_PARSER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tram_vehicle_msgs/msg/velocity_sensor.hpp"
#include "tram_vehicle_msgs/msg/driver_controller_command.hpp"


class CDataParser : public rclcpp::Node {
public:
    explicit CDataParser(const rclcpp::NodeOptions & options);
    ~CDataParser() = default;

private:
    void process_gnss_master_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void process_gnss_master_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void process_gnss_rover_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void process_gnss_rover_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void process_rear_bogie_vel(const tram_vehicle_msgs::msg::VelocitySensor::SharedPtr msg);
    void process_driver_position_cmd(const tram_vehicle_msgs::msg::DriverControllerCommand::SharedPtr msg);
    void process_front_bogie_velocity(const tram_vehicle_msgs::msg::VelocitySensor::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_master_fix_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gnss_master_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_rover_fix_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gnss_rover_vel_sub_;
    rclcpp::Subscription<tram_vehicle_msgs::msg::VelocitySensor>::SharedPtr rear_bogie_vel_sub_;
    rclcpp::Subscription<tram_vehicle_msgs::msg::DriverControllerCommand>::SharedPtr driver_position_cmd_sub_;
    rclcpp::Subscription<tram_vehicle_msgs::msg::VelocitySensor>::SharedPtr front_bogie_velocity_sub_;

    std::string gnss_master_fix_file_ = "results/gnss_master_fix.csv";
    std::string gnss_master_vel_file_ = "results/gnss_master_vel.csv";
    std::string gnss_rover_fix_file_ = "results/gnss_rover_fix.csv";
    std::string gnss_rover_vel_file_ = "results/gnss_rover_vel.csv";
    std::string rear_bogie_vel_file_ = "results/rear_bogie_vel.csv";
    std::string driver_position_cmd_file_ = "results/driver_position_cmd.csv";
    std::string front_bogie_velocity_file_ = "results/front_bogie_velocity.csv";
};

#endif
