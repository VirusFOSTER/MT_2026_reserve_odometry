#ifndef MOVING_MODEL_1_HPP
#define MOVING_MODEL_1_HPP

#include "../model_concept.hpp"

namespace filter {
namespace kalman {
namespace model {
/**
 * @brief The moving_model_1 class - класс, описывающий модель движения объекта наблюдения
 * Данная модель используется для фильтрации данных (фильтр Калмана)
 * Вектор состояния в данной модели описывается системой следующих уравнений
                ---------------------------------------------
                | x_{k+1}   = x_k + vx_k * cos(yaw_k) * dt  |
                | y_{k+1}   = y_k + vx_k * sin(yaw_k) * dt  |
                | vx_{k+1}  = vx_k                          |
                | yaw_{k+1} = yaw_k                         |
                | wz_{k+1}  = wz_k                          |
                ---------------------------------------------
 * Применение: объекты, ориентация которых не может быть установлена
 */
template <class T_object>
class moving_model_1 : public m_concept::model_concept<T_object> {
public:
    using object_t = T_object;

    enum IDX
    {
        X   = 0,        /// <--- индекс координаты X в матрицах
        Y   = 1,        /// <--- индекс координаты Y в матрицах
        VX  = 2,        /// <--- индекс скорости в матрицах
        YAW = 3,        /// <--- индекс ориентации в матрицах
        WZ  = 4,        /// <--- индекс угловой скорости в матрицах
    };

    explicit moving_model_1() : m_concept::model_concept<T_object>() {}

    ~moving_model_1() = default;

    void matricies_init(const object_t& obj_);

    void make_X(double dt_, float ref_v_);
    Eigen::MatrixXd make_A(double dt_);
    Eigen::MatrixXd make_Q(double dt_);
    Eigen::MatrixXd make_Y(const object_t& obj_);
    Eigen::MatrixXd make_C();
    Eigen::MatrixXd make_R();

    void update(object_t* object_);

    bool equal(m_concept::model_concept<T_object> *concept_);

private:
    Eigen::MatrixXd CX_;
};


template <class T_object>
void moving_model_1<T_object>::matricies_init(const object_t& obj_) {
    this->X_ = Eigen::MatrixXd::Zero(5,1);

    this->X_(IDX::X) = obj_.get_position().get_x();
    this->X_(IDX::Y) = obj_.get_position().get_y();
    this->X_(IDX::YAW) = obj_.get_orientation();
    this->X_(IDX::VX) = obj_.get_velocity().to_spherical()[0];


}
}               /// <---model
}           /// <--- kalman
}       /// <--- filter

#endif
