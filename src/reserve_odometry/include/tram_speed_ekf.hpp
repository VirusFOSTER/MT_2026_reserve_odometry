#pragma once

#include <algorithm>
#include <Eigen/Dense>

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


    TramSpeedEKF();


    Result Update(
        double front_velocity_kmh,
        double rear_velocity_kmh,
        int8_t driver_position,
        double dt);


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

    using StateVector = Eigen::Matrix<double, N, 1>;

    using StateMatrix = Eigen::Matrix<double, N, N>;


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
        double dt);


    void CorrectWheel(
        double measurement,
        int slip_index);


    void ApplyConstraints();

    Result GetResult() const
    {
        Result result{};

        result.velocity = std::max(0.0, x_(V));

        result.acceleration = x_(A);

        result.front_slip = x_(BF);

        result.rear_slip = x_(BR);

        result.traction_gain = x_(KT);

        result.brake_gain = x_(KB);

        return result;
    }
};