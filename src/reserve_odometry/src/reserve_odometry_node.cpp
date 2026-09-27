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
#include <vector>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>

#include <nlohmann/json.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <Eigen/Dense>

struct TrackPoint {
    double x;
    double y;
    double z;

    double tang;
    double curv;

    double s;
};

using Track = std::vector<TrackPoint>;

Track LoadTrack(const std::string& path) {
    std::ifstream file(path);
    Track result_track;

    if (!file.is_open()) {
        throw std::runtime_error("Не удалось открыть карту");
    }

    nlohmann::json json;
    file >> json;

    double accumulated_s = 0.0;
    bool first = true;
    TrackPoint prev{};

    for (const auto& item : json["points"]) {
        TrackPoint point{};

        point.x = item["x"].get<double>();
        point.y = item["y"].get<double>();
        point.z = item["z"].get<double>();

        point.tang = item["tang"].get<double>();
        point.curv = item["curv"].get<double>();

        if (first) {
            point.s = 0.0;
            first = false;
        } else {
            const double dx = point.x - prev.x;
            const double dy = point.y - prev.y;
            const double dz = point.z - prev.z;

            accumulated_s += std::sqrt(
                dx * dx + dy * dy + dz * dz
            );

            point.s = accumulated_s;
        }

        result_track.push_back(point);
        prev = point;
    }
    return result_track;
}

struct MapPoint {
    double x;
    double y;
    double z;
};

MapPoint GnssToMap(
    double latitude, 
    double longitude,
    double altitude
) {
    int zone;
    bool northp;
   
    double easting;
    double northing;

    GeographicLib::UTMUPS::Forward(
        latitude,
        longitude,
        zone,
        northp,
        easting,
        northing
    );

    if (zone != 37 || !northp) {
        throw std::runtime_error(
            "GNSS point is outside expected UTM zone 37N"
        );
    }

    const double MGRS_ORIGIN_EASTING = 300000.0;
    const double MGRS_ORIGIN_NORTHING = 6100000.0;

    MapPoint result;
    result.x = easting - MGRS_ORIGIN_EASTING;
    result.y = northing - MGRS_ORIGIN_NORTHING;
    result.z = altitude;

    return result;
}

size_t FindNearestPoint(
    const std::vector<TrackPoint>& track,
    double x, 
    double y
) {
    size_t best_index = 0;
    double best_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < track.size(); ++i) {
        const double dx = track[i].x - x;
        const double dy = track[i ].y - y;
        const double distance = dx*dx + dy*dy;

        if (distance < best_distance) {
            best_distance = distance;
            best_index = i;
        }
    }

    return best_index;
}

double NormalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }

    return angle;
}

TrackPoint InterpolateTrack(
    const std::vector<TrackPoint>& track,
    double s
) {
    TrackPoint result;

    if (track.front().s >= s) {
        return track.front();
    }

    if (track.back().s <= s) {
        return track.back();
    }

    auto it = std::lower_bound(
        track.begin(),
        track.end(),
        s,
        [](const TrackPoint& p, double value) {
            return p.s < value;
        }
    );

    int i1 = std::distance(track.begin(), it);
    int i0 = i1 - 1;

    const TrackPoint& p1 = track[i1];
    const TrackPoint& p0 = track[i0];

    const double segment = p1.s - p0.s;
    const double alpha = (segment > 0 ? (s - p0.s)/segment : 0.0);

    result.s = s;
    result.x = p0.x + alpha * (p1.x - p0.x);
    result.y = p0.y + alpha * (p1.y - p0.y);
    result.z = p0.z + alpha * (p1.z - p0.z);
    result.curv = p0.curv + alpha * (p1.curv - p0.curv);
    result.tang = p0.tang + alpha * NormalizeAngle(p1.tang - p0.tang);

    return result;
}   


class TramSpeedEKF
{
public:

    struct Result
    {
        double velocity;          // m/s
        double acceleration;      // m/s^2

        double front_slip;        // m/s
        double rear_slip;         // m/s

        double traction_gain;
        double brake_gain;
    };


    TramSpeedEKF()
    {
        x_.setZero();

        // Начальные оценки параметров.
        x_(KT) = 1.0;
        x_(KB) = 1.5;
        x_(DRAG) = 0.003;

        P_.setZero();

        P_(V, V) = 1.0;
        P_(A, A) = 1.0;

        P_(KT, KT) = 1.0;
        P_(KB, KB) = 1.0;

        P_(DRAG, DRAG) = 1e-4;

        // Большая неопределённость slip в начале.
        P_(BF, BF) = 1.0;
        P_(BR, BR) = 1.0;
    }


    Result Update(
        double front_velocity_kmh,
        double rear_velocity_kmh,
        int8_t driver_position,
        double dt)
    {

        // -----------------------------------------------------
        // km/h -> m/s
        // -----------------------------------------------------

        const double z_front =
            front_velocity_kmh / 3.6;

        const double z_rear =
            rear_velocity_kmh / 3.6;


        // -----------------------------------------------------
        // Первая инициализация
        // -----------------------------------------------------

        if (!initialized_) {

            x_(V) =
                0.5 * (z_front + z_rear);

            x_(A) = 0.0;

            x_(BF) = 0.0;
            x_(BR) = 0.0;

            initialized_ = true;

            return GetResult();
        }

        // Защита от плохого dt.
        if (dt <= 0.0 || dt > 0.5) {
            return GetResult();
        }

        // -----------------------------------------------------
        // Управление
        // -----------------------------------------------------

        const int command =
            std::clamp(
                static_cast<int>(driver_position),
                -15,
                15
            );

        const double u_traction =
            static_cast<double>(
                std::max(command, 0)
            ) / 15.0;

        const double u_brake =
            static_cast<double>(
                std::max(-command, 0)
            ) / 15.0;


        // -----------------------------------------------------
        // Predict
        // -----------------------------------------------------

        Predict(
            u_traction,
            u_brake,
            z_front,
            z_rear,
            dt
        );


        // -----------------------------------------------------
        // Correct
        // -----------------------------------------------------

        CorrectWheel(
            z_front,
            BF
        );

        CorrectWheel(
            z_rear,
            BR
        );


        // -----------------------------------------------------
        // Физические ограничения
        // -----------------------------------------------------

        ApplyConstraints();


        return GetResult();
    }


private:

    enum StateIndex
    {
        V = 0,
        A = 1,
        KT = 2,
        KB = 3,
        DRAG = 4,
        BF = 5,
        BR = 6
    };

    static constexpr int N = 7;

    using StateVector =
        Eigen::Matrix<double, N, 1>;

    using StateMatrix =
        Eigen::Matrix<double, N, N>;


    StateVector x_;

    StateMatrix P_;

    bool initialized_ = false;


    // ---------------------------------------------------------
    // Настройки модели
    // ---------------------------------------------------------

    // Инерционность изменения ускорения.
    double acceleration_tau_ = 0.8;

    // За сколько slip обычно затухает после исчезновения причины.
    double slip_tau_ = 1.5;

    // Приблизительное постоянное сопротивление.
    double rolling_acceleration_ = 0.025;

    // Сглаживание sign(v) около нуля.
    double velocity_epsilon_ = 0.3;

    // Стандартное отклонение датчика скорости.
    double wheel_sigma_ = 0.08;


    void Predict(
        double u_traction,
        double u_brake,
        double z_front,
        double z_rear,
        double dt)
    {
        const double v = x_(V);
        const double a = x_(A);

        const double kt = x_(KT);
        const double kb = x_(KB);

        const double drag = x_(DRAG);


        // -----------------------------------------------------
        // Сопротивление движению
        // -----------------------------------------------------

        const double rolling =
            rolling_acceleration_ *
            std::tanh(
                v / velocity_epsilon_
            );

        const double aerodynamic =
            drag * v * std::abs(v);


        // -----------------------------------------------------
        // Желаемое ускорение от модели
        // -----------------------------------------------------

        const double target_acceleration =
            kt * u_traction
            - kb * u_brake
            - rolling
            - aerodynamic;


        // -----------------------------------------------------
        // State prediction
        // -----------------------------------------------------

        StateVector predicted = x_;

        predicted(V) =
            v + a * dt;

        predicted(A) =
            a +
            dt / acceleration_tau_ *
            (target_acceleration - a);


        // KT, KB, DRAG — random walk:
        // математическое ожидание не меняется.

        const double slip_decay =
            std::exp(
                -dt / slip_tau_
            );

        predicted(BF) =
            slip_decay * x_(BF);

        predicted(BR) =
            slip_decay * x_(BR);


        // -----------------------------------------------------
        // Jacobian F = df/dx
        // -----------------------------------------------------

        StateMatrix F =
            StateMatrix::Identity();


        // v(k+1) = v + a*dt

        F(V, A) = dt;


        // Производная rolling resistance.

        const double q =
            v / velocity_epsilon_;

        const double tanh_q =
            std::tanh(q);

        const double sech2 =
            1.0 - tanh_q * tanh_q;


        const double d_rolling_dv =
            rolling_acceleration_
            / velocity_epsilon_
            * sech2;


        // d(v*|v|)/dv = 2|v|

        const double d_a_target_dv =
            -d_rolling_dv
            -2.0 * drag * std::abs(v);


        F(A, V) =
            dt / acceleration_tau_
            * d_a_target_dv;


        F(A, A) =
            1.0 -
            dt / acceleration_tau_;


        F(A, KT) =
            dt / acceleration_tau_
            * u_traction;


        F(A, KB) =
            -dt / acceleration_tau_
            * u_brake;


        F(A, DRAG) =
            -dt / acceleration_tau_
            * v * std::abs(v);


        F(BF, BF) =
            slip_decay;

        F(BR, BR) =
            slip_decay;


        // -----------------------------------------------------
        // Process noise Q
        // -----------------------------------------------------

        StateMatrix Q =
            StateMatrix::Zero();


        // Модель скорости достаточно надёжна,
        // но не идеальна.
        Q(V, V) =
            0.03 * 0.03 * dt;


        // Неизвестный реальный момент в первую очередь
        // компенсируем через неопределённость ускорения.
        Q(A, A) =
            0.50 * 0.50 * dt;


        // KT и KB должны изменяться медленно.
        // Именно сюда, в частности, попадает изменение массы.
        Q(KT, KT) =
            0.04 * 0.04 * dt;

        Q(KB, KB) =
            0.05 * 0.05 * dt;


        // Сопротивление меняется очень медленно.
        Q(DRAG, DRAG) =
            0.0001 * 0.0001 * dt;


        // -----------------------------------------------------
        // Adaptive slip noise
        // -----------------------------------------------------

        const double wheel_difference =
            std::abs(
                z_front - z_rear
            );

        const double command_intensity =
            std::max(
                u_traction,
                u_brake
            );


        // Чем сильнее расходятся тележки и чем больше
        // тяга/торможение, тем охотнее разрешаем состояниям
        // BF/BR быстро изменяться.
        const double slip_sigma =
            0.15
            + 0.50 * wheel_difference
            + 0.30 * command_intensity;


        Q(BF, BF) =
            slip_sigma *
            slip_sigma *
            dt;

        Q(BR, BR) =
            slip_sigma *
            slip_sigma *
            dt;


        // -----------------------------------------------------
        // Covariance prediction
        // -----------------------------------------------------

        x_ = predicted;

        P_ =
            F * P_ * F.transpose()
            + Q;
    }


    void CorrectWheel(
        double measurement,
        int slip_index)
    {
        // -----------------------------------------------------
        // h(x) = v + slip
        // -----------------------------------------------------

        Eigen::Matrix<double, 1, N> H;

        H.setZero();

        H(0, V) = 1.0;
        H(0, slip_index) = 1.0;


        const double predicted_measurement =
            x_(V) +
            x_(slip_index);


        const double innovation =
            measurement -
            predicted_measurement;


        // -----------------------------------------------------
        // Measurement covariance
        // -----------------------------------------------------

        double R =
            wheel_sigma_ *
            wheel_sigma_;


        // -----------------------------------------------------
        // Innovation covariance
        // -----------------------------------------------------

        double S =
            (H * P_ * H.transpose())(0, 0)
            + R;


        // -----------------------------------------------------
        // Robustification
        //
        // Очень большой innovation может означать резкое
        // буксование/юз или выброс датчика.
        // -----------------------------------------------------

        const double normalized_innovation =
            innovation * innovation /
            std::max(S, 1e-9);


        constexpr double NIS_LIMIT = 16.0;


        if (normalized_innovation > NIS_LIMIT) {

            const double scale =
                normalized_innovation /
                NIS_LIMIT;

            R *= scale;

            S =
                (H * P_ * H.transpose())(0, 0)
                + R;
        }


        // -----------------------------------------------------
        // Kalman gain
        // -----------------------------------------------------

        const Eigen::Matrix<double, N, 1> K =
            P_ * H.transpose() / S;


        // -----------------------------------------------------
        // State correction
        // -----------------------------------------------------

        x_ += K * innovation;


        // -----------------------------------------------------
        // Joseph covariance update
        // -----------------------------------------------------

        const StateMatrix I =
            StateMatrix::Identity();

        const StateMatrix KH =
            K * H;


        P_ =
            (I - KH) *
            P_ *
            (I - KH).transpose()
            +
            K * R * K.transpose();
    }


    void ApplyConstraints()
    {
        // Мы оцениваем модуль продольной скорости.
        if (x_(V) < 0.0) {
            x_(V) = 0.0;
        }


        // Физически коэффициенты тяги и торможения
        // отрицательными быть не могут.
        x_(KT) =
            std::clamp(
                x_(KT),
                0.05,
                4.0
            );

        x_(KB) =
            std::clamp(
                x_(KB),
                0.05,
                6.0
            );


        x_(DRAG) =
            std::clamp(
                x_(DRAG),
                0.0,
                0.1
            );


        // Ограничиваем совсем патологические оценки slip.
        x_(BF) =
            std::clamp(
                x_(BF),
                -20.0,
                20.0
            );

        x_(BR) =
            std::clamp(
                x_(BR),
                -20.0,
                20.0
            );
    }


    Result GetResult() const
    {
        Result result{};

        result.velocity =
            std::max(0.0, x_(V));

        result.acceleration =
            x_(A);

        result.front_slip =
            x_(BF);

        result.rear_slip =
            x_(BR);

        result.traction_gain =
            x_(KT);

        result.brake_gain =
            x_(KB);

        return result;
    }
};

using GnssSyncPolicy = 
            message_filters::sync_policies::ApproximateTime<
                sensor_msgs::msg::NavSatFix,
                sensor_msgs::msg::NavSatFix>;

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

        // Объявление параметров
        declare_parameter<std::string>("forward_track_path", 
                                       "/home/epyur/mt_hackathon_ws/src/reserve_odometry/resource/таллинская - щукинская.json");
        declare_parameter<std::string>("backward_track_path",
                                       "/home/epyur/mt_hackathon_ws/src/reserve_odometry/resource/щукинская - таллинская.json");

        // Получение параметров
        std::string forward_track_path = get_parameter("forward_track_path").as_string();
        std::string backward_track_path = get_parameter("backward_track_path").as_string();

        // Выгрузка треков
        forward_track_ = LoadTrack(forward_track_path);
        backward_track_ = LoadTrack(backward_track_path);
        RCLCPP_INFO(
            get_logger(),
            "Загружены карты: 1: %zu точек; 2: %zu точек.",
            forward_track_.size(),
            backward_track_.size()
        );

        master_fix_filter_ =
            std::make_shared<
                message_filters::Subscriber<
                    sensor_msgs::msg::NavSatFix>>(
                        this,
                        "/sensing/gnss/master/fix");

        rover_fix_filter_ =
            std::make_shared<
                message_filters::Subscriber<
                    sensor_msgs::msg::NavSatFix>>(
                        this,
                        "/sensing/gnss/rover/fix");

        gnss_sync_ =
            std::make_shared<
                message_filters::Synchronizer<
                    GnssSyncPolicy>>(
                            GnssSyncPolicy(20),
                            *master_fix_filter_,
                            *rover_fix_filter_);

        gnss_sync_->setMaxIntervalDuration(
            rclcpp::Duration::from_seconds(0.2));

        gnss_sync_->registerCallback(
            std::bind(
                &ReserveOdometry::GnssInitCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));


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
           
        //Паблишеры для отправки одометрии        
        vel_pub_ = create_publisher<tram_vehicle_msgs::msg::VelocitySensor>("/result/velocity", 10);        
        pos_pub_ = create_publisher<nav_msgs::msg::Odometry>("/result/position", 10);   
    }

private:

    void GnssInitCallback(
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr& master_msg,
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr& rover_msg
    ) {
        if (pose_initialized_) {
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Получены GNSS для инициализации"
        );


        // Переподим коордианты атенн в метричеую плоскую с.к.
        const MapPoint master = GnssToMap(
            master_msg->latitude,
            master_msg->longitude,
            master_msg->altitude
        );

        const MapPoint rover = GnssToMap(
            rover_msg->latitude,
            rover_msg->longitude,
            rover_msg->altitude
        );


        // Определяем направление трамвая
        const double dx = rover.x - master.x;
        const double dy = rover.y - master.y;
        const double yaw = std::atan2(dy, dx);

        // Вычисляем положение base_link
        const double ROVER_X = 2.563; //Координата rover относительно base_link
        init_base_x_ = rover.x - ROVER_X * std::cos(yaw);
        init_base_y_ = rover.y - ROVER_X * std::sin(yaw);
        init_base_z_ = rover.z - 3.0; //Антенна Rover находится выше base_link на 3 м.

        // Находим ближайшие точки в двух направлениях
        const size_t forward_index = FindNearestPoint(
            forward_track_, init_base_x_, init_base_y_
        );

        const size_t backward_index = FindNearestPoint(
            backward_track_, init_base_x_, init_base_y_
        );
        
        const double forward_dist = std::hypot(
            init_base_x_ - forward_track_[forward_index].x,
            init_base_y_ - forward_track_[forward_index].y
        );

        const double backward_dist = std::hypot(
            init_base_x_ - backward_track_[backward_index].x,
            init_base_y_ - backward_track_[backward_index].y
        );

        RCLCPP_INFO(
            get_logger(),
            "MAP MATCH: F dist=%.3f m, B dist=%.3f m",
            forward_dist,
            backward_dist
        );

        if (std::min(forward_dist, backward_dist) > 20.0) {
            RCLCPP_WARN(
                get_logger(),
                "GNSS position is OUTSIDE TRACK MAP! "
                "Nearest track point is %.1f m away",
                std::min(forward_dist, backward_dist)
            );
        }       

        // Вычисляем ощибки по углам
        const double forward_error = std::abs(
            NormalizeAngle(yaw - forward_track_[forward_index].tang)
        );

        const double backward_error = std::abs(
            NormalizeAngle(yaw - backward_track_[backward_index].tang)
        );

        // Определяем направление маршрута
        if (forward_error < backward_error) {
            active_track_ = &forward_track_;
            track_s_ = forward_track_[forward_index].s;

            RCLCPP_INFO(
                get_logger(),
                "Node inited. Forward s=%.3f", track_s_
            );
        } else {
            active_track_ = &backward_track_;
            track_s_ = backward_track_[backward_index].s;

            RCLCPP_INFO(
                get_logger(),
                "Node inited. Forward s=%.3f", track_s_
            );
        }

        // Инициализация ноды закончилась
        pose_initialized_ = true;

        const double baseline =
            std::hypot(dx, dy);

        RCLCPP_INFO(
            get_logger(),
            "GNSS antennas: "
            "master=(%.3f, %.3f), "
            "rover=(%.3f, %.3f), "
            "baseline=%.3f m, "
            "yaw=%.6f rad",
            master.x,
            master.y,
            rover.x,
            rover.y,
            baseline,
            yaw
        );

        RCLCPP_INFO(
            get_logger(),
            "GNSS INIT DIAGNOSTICS: "
            "base=(%.3f, %.3f, %.3f), "
            "yaw=%.6f rad | "
            "F: idx=%zu s=%.3f tang=%.6f err=%.6f | "
            "B: idx=%zu s=%.3f tang=%.6f err=%.6f",
            init_base_x_,
            init_base_y_,
            init_base_z_,
            yaw,

            forward_index,
            forward_track_[forward_index].s,
            forward_track_[forward_index].tang,
            forward_error,

            backward_index,
            backward_track_[backward_index].s,
            backward_track_[backward_index].tang,
            backward_error
        );
    }   

    void SyncCallback(
        const tram_vehicle_msgs::msg::VelocitySensor::ConstSharedPtr& front_bogie_vel,
        const tram_vehicle_msgs::msg::VelocitySensor::ConstSharedPtr& rear_bogie_vel,
        const tram_vehicle_msgs::msg::DriverControllerCommand::ConstSharedPtr& cmd
    ) {
        if (!pose_initialized_ || active_track_ == nullptr) {
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Входные данные получены!"
        );

        const rclcpp::Time stamp(front_bogie_vel->header.stamp);

        const double front_velocity =
            front_bogie_vel->velocity;

        const double rear_velocity =
            rear_bogie_vel->velocity;
        
        if (!time_initialized_) {
            const auto estimate = velocity_ekf_.Update(
                front_velocity,
                rear_velocity,
                cmd->position,
                0.0
            );

            const double velocity = estimate.velocity;
            const TrackPoint pose = 
                InterpolateTrack(
                    *active_track_,
                    track_s_
                );


            last_stamp_ = stamp;
            last_velocity_ = velocity;
            time_initialized_ = true;

            PublishVelocity(velocity, 
                        "base_link",
                        stamp);
        
            PublishPosition({pose.x, pose.y, pose.z},
                            velocity,
                            "odom",
                            "base_link",
                            stamp);

            return;
        }

        const double dt = (stamp - last_stamp_).seconds();

        if (dt <= 0.0) {
            // const double ds = 0.5 * (last_velocity_ + velocity) * dt; 
            // track_s_ += ds;
            return;
        }

        const auto estimate = velocity_ekf_.Update(
                front_velocity,
                rear_velocity,
                cmd->position,
                dt
            );
        const double velocity = estimate.velocity;
        
        if (dt < 0.5) {
            const double ds = 0.5 * (last_velocity_ + velocity) * dt;
            AdvanceAlongTrack(ds);
        }

        const TrackPoint pose = InterpolateTrack(*active_track_, track_s_);

        last_stamp_ = stamp;
        last_velocity_ = velocity;

        PublishVelocity(velocity, 
                        "base_link",
                        stamp);
        
        PublishPosition({pose.x, pose.y, pose.z},
                        velocity,
                        "odom",
                        "base_link",
                        stamp);
        
        RCLCPP_INFO(
            get_logger(),

            "EKF: "
            "v=%.3f m/s, "
            "a=%.3f m/s2, "
            "front=%.3f km/h, "
            "rear=%.3f km/h, "
            "slipF=%+.3f, "
            "slipR=%+.3f, "
            "kT=%.3f, "
            "kB=%.3f, "
            "s=%.3f, "
            "dt=%.3f, "
            "cmd=%d",

            estimate.velocity,
            estimate.acceleration,

            front_velocity,
            rear_velocity,

            estimate.front_slip,
            estimate.rear_slip,

            estimate.traction_gain,
            estimate.brake_gain,

            track_s_,
            dt,

            static_cast<int>(
                cmd->position
            )
        );
        
    }

    void AdvanceAlongTrack(double ds)
    {
        if (active_track_ == nullptr || active_track_->empty()) {
            return;
        }

        track_s_ += ds;

        while (track_s_ > active_track_->back().s) {
            const double overflow =
                track_s_ - active_track_->back().s;

            if (active_track_ == &forward_track_) {
                active_track_ = &backward_track_;
                RCLCPP_INFO(
                    get_logger(),
                    "Reached end of FORWARD track -> switching to BACKWARD");
            } else {
                active_track_ = &forward_track_;
                RCLCPP_INFO(
                    get_logger(),
                    "Reached end of BACKWARD track -> switching to FORWARD");
            }

            track_s_ = active_track_->front().s + overflow;
        }
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

    void PublishPosition(const MapPoint& pose,
                         const double velocity,
                         const std::string& frame_id,
                         const std::string& child_frame_id,
                         const rclcpp::Time& stamp
    ) {
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = frame_id;
        odom_msg.child_frame_id = child_frame_id;
        
        odom_msg.pose.pose.position.x = pose.x;
        odom_msg.pose.pose.position.y = pose.y;
        odom_msg.pose.pose.position.z = pose.z;

//       const double half_yaw = pose.tang / 2; 

        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0; //std::sin(half_yaw);
        odom_msg.pose.pose.orientation.w = 1.0; //std::cos(half_yaw);

        odom_msg.twist.twist.linear.x = velocity;
//        odom_msg.twist.twist.angular.z = velocity * pose.curv;
        
        pos_pub_->publish(odom_msg);
    }

    std::shared_ptr<
        message_filters::Subscriber<
        sensor_msgs::msg::NavSatFix>>
        master_fix_filter_;

    std::shared_ptr<
        message_filters::Subscriber<
        sensor_msgs::msg::NavSatFix>>
        rover_fix_filter_;

    std::shared_ptr<
        message_filters::Synchronizer<GnssSyncPolicy>>
        gnss_sync_;

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

    double init_base_x_ = 0.0;
    double init_base_y_ = 0.0;
    double init_base_z_ = 0.0;
    double track_s_ = 0.0;

    double last_velocity_ = 0.0;

    rclcpp::Time last_stamp_{0,0,RCL_ROS_TIME};

    bool pose_initialized_ = false;
    bool time_initialized_ = false;
    TramSpeedEKF velocity_ekf_;

    std::vector<TrackPoint> forward_track_;
    std::vector<TrackPoint> backward_track_;

    std::vector<TrackPoint>* active_track_ = nullptr;
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
