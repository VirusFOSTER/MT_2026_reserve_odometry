#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tram_vehicle_msgs/msg/driver_controller_command.hpp>
#include <tram_vehicle_msgs/msg/velocity_sensor.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>

class ReserveOdometry : public rclcpp::Node {
    public:
    ReserveOdometry(): Node("reserve_odometry") {
        /*
            Подписывается  на топики:
            /vehicle/front_bogie_velocity -> tram_vehicle_msgs/msg/VelocitySensor
            /vehicle/rear_bogie_velocity -> tram_vehicle_msgs/msg/VelocitySensor
            /vehicle/driver_position_cmd -> tram_vehicle_msgs/msg/DriverControllerCommand

            Подписывается на топики по GNSS:
            /sensing/gnss/master/fix -> sensor_msgs/msg/NavSatFix
            /sensing/gnss/master/vel -> geometry_msgs/msg/TwistStamped
            /sensing/gnss/rover/fix -> sensor_msgs/msg/NavSatFix
            /sensing/gnss/rover/vel -> geometry_msgs/msg/TwistStamped
        
            Публикует в топики:
            /result/velocity -> tram_vehicle_msgs/msg/VelocitySensor
            /result/position -> nav_msgs/msg/Odometry
        
        */

        RCLCPP_INFO(get_logger(),
            "Запуск ноды резервной одометрии!");

        //Синхронное получение входных данных    
        front_bogie_sub_ = std::make_shared<message_filters::Subscriber<tram_vehicle_msgs::msg::VelocitySensor>>(
            this, "/vehicle/front_bogie_velocity");
        rear_bogie_sub_ = std::make_shared<message_filters::Subscriber<tram_vehicle_msgs::msg::VelocitySensor>>(
            this, "/vehicle/rear_bogie_velocity");
        cmd_sub_  = std::make_shared<message_filters::Subscriber<tram_vehicle_msgs::msg::DriverControllerCommand>>(
            this, "/vehicle/driver_position_cmd");
        
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            tram_vehicle_msgs::msg::VelocitySensor, tram_vehicle_msgs::msg::VelocitySensor,
            tram_vehicle_msgs::msg::DriverControllerCommand>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(static_cast<uint32_t>(sync_queue_size_)),
            *front_bogie_sub_, *rear_bogie_sub_, *cmd_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
        sync_->registerCallback(std::bind(&ReserveOdometry::SyncCallback, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        //Подписка на топики GNSS для начальной коррекции    
        master_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
                "/sensing/gnss/master/fix", 10,
                std::bind(&ReserveOdometry::MasterFixCallback, this, std::placeholders::_1));

        master_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
                "/sensing/gnss/master/vel", 10,
                std::bind(&ReserveOdometry::MasterVelCallback, this, std::placeholders::_1));
        
        rover_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
                "/sensing/gnss/rover/fix", 10,
                std::bind(&ReserveOdometry::RoverFixCallback, this, std::placeholders::_1));

        rover_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
                "/sensing/gnss/rover/vel", 10,
                std::bind(&ReserveOdometry::RoverVelCallback, this, std::placeholders::_1));
           
        //Паблишеры для отправки одометрии        
        vel_pub_ = create_publisher<tram_vehicle_msgs::msg::VelocitySensor>("/result/velocity", 10);        
        pos_pub_ = create_publisher<nav_msgs::msg::Odometry>("/result/position", 10);   
    }

private:

    void MasterFixCallback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        return;
    }

    void RoverFixCallback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        return;
    }

    void MasterVelCallback(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        return;
    }

    void RoverVelCallback(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        return;
    }

    void SyncCallback(
        const tram_vehicle_msgs::msg::VelocitySensor::ConstSharedPtr& front_bogie_vel,
        const tram_vehicle_msgs::msg::VelocitySensor::ConstSharedPtr& rear_bogie_vel,
        const tram_vehicle_msgs::msg::DriverControllerCommand::ConstSharedPtr& cmd
    ) {
        RCLCPP_INFO(
            get_logger(),
            "Входные данные получены!"
        );

        const rclcpp::Time stamp(front_bogie_vel->header.stamp);

        const double velocity = 
            0.5 * (
                front_bogie_vel->velocity +
                rear_bogie_vel->velocity
            );
        
        if (!initialized_) {
            last_stamp_ = stamp;
            last_velocity_ = velocity;
            initialized_ = true;

            PublishVelocity(velocity, 
                        "base_link",
                        stamp);
        
            PublishPosition(velocity,
                            "odom",
                            "base_link",
                            stamp);

            return;
        }

        const double dt = (stamp - last_stamp_).seconds();

        if (dt > 0.0 && dt < 1.0) {
            const double distance = 0.5 * (last_velocity_ + velocity) * dt; 
            x_ += distance;
        }

        last_stamp_ = stamp;
        last_velocity_ = velocity;

        PublishVelocity(velocity, 
                        "base_link",
                        stamp);
        
        PublishPosition(velocity,
                        "odom",
                        "base_link",
                        stamp);
        
        RCLCPP_INFO(
            get_logger(),
            "v=%.3f m/s, x=%.3f m, dt=%.4f, cmd=%d",
            velocity,
            x_,
            dt,
            static_cast<int>(cmd->position)
        );
    }

    void PublishVelocity(const double velocity,
                         const std::string& frame_id,
                         const rclcpp::Time& stamp) {
        tram_vehicle_msgs::msg::VelocitySensor velocity_msg;

        velocity_msg.header.stamp = stamp;
        velocity_msg.header.frame_id = frame_id;
        velocity_msg.velocity = velocity;
        
        vel_pub_->publish(velocity_msg);
    }

    void PublishPosition(const double velocity,
                         const std::string& frame_id,
                         const std::string& child_frame_id,
                         const rclcpp::Time& stamp
    ) {
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = frame_id;
        odom_msg.child_frame_id = child_frame_id;
        
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = z_;

        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
        odom_msg.pose.pose.orientation.w = 0.0;

        odom_msg.twist.twist.linear.x = velocity;
        
        pos_pub_->publish(odom_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr master_fix_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr rover_fix_sub_;
    rclcpp::Subscription<geometry_msgs::msg:: TwistStamped>::SharedPtr master_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr rover_vel_sub_;

    rclcpp::Publisher<tram_vehicle_msgs::msg::VelocitySensor>::SharedPtr vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pos_pub_;

    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        tram_vehicle_msgs::msg::VelocitySensor, tram_vehicle_msgs::msg::VelocitySensor, tram_vehicle_msgs::msg::DriverControllerCommand>>> sync_;

    std::shared_ptr<message_filters::Subscriber<tram_vehicle_msgs::msg::VelocitySensor>> front_bogie_sub_, rear_bogie_sub_;
    std::shared_ptr<message_filters::Subscriber<tram_vehicle_msgs::msg::DriverControllerCommand>> cmd_sub_;
    int sync_queue_size_ = 30;
    double sync_slop_sec_ = 0.2;

    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;

    double last_velocity_ = 0.0;

    rclcpp::Time last_stamp_{0,0,RCL_ROS_TIME};

    bool initialized_ = false;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReserveOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    node.reset();
    return 0;
}
