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
        RCLCPP_INFO(get_logger(),
            "Получил входные данные!");

        //Дальше обработка данных и определение местоположения трамвая...

        return;
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
