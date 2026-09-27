#include "tram_speed_ekf.hpp"


TramSpeedEKF::TramSpeedEKF()
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


TramSpeedEKF::Result TramSpeedEKF::Update(
        double front_velocity_kmh,
        double rear_velocity_kmh,
        int8_t driver_position,
        double dt)
{

    // -----------------------------------------------------
    // km/h -> m/s
    // -----------------------------------------------------

    const double z_front = front_velocity_kmh / 3.6;

    const double z_rear = rear_velocity_kmh / 3.6;


    // -----------------------------------------------------
    // Первая инициализация
    // -----------------------------------------------------

    if (!initialized_) {
        x_(V) = 0.5 * (z_front + z_rear);

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


void TramSpeedEKF::Predict(
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

    const double q = v / velocity_epsilon_;

    const double tanh_q = std::tanh(q);

    const double sech2 = 1.0 - tanh_q * tanh_q;


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

void TramSpeedEKF::CorrectWheel(
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

        S = (H * P_ * H.transpose())(0, 0) + R;
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

    const StateMatrix I = StateMatrix::Identity();

    const StateMatrix KH = K * H;


    P_ = (I - KH) * P_ * (I - KH).transpose() + K * R * K.transpose();
}

void TramSpeedEKF::ApplyConstraints()
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