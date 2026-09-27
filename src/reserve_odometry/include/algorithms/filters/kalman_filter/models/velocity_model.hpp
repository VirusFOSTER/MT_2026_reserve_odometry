#ifndef MOVING_MODEL_1_HPP
#define MOVING_MODEL_1_HPP

#include "algorithms/filters/kalman_filter/model_concept.hpp"

namespace filter {
namespace kalman {
namespace model {
/** ------------------------------------------------------------------------------------------
 * @brief The velocity_model class - класс, описывающий модель движения объекта наблюдения
 * Данная модель используется для фильтрации данных (фильтр Калмана)
 * Вектор состояния в данной модели описывается системой следующих уравнений
 * Применение: объекты, ориентация которых не может быть установлена
 -------------------------------------------------------------------------------------------*/
template <class T_object>
class velocity_model : public m_concept::model_concept<T_object> {
public:
    using object_t = T_object;
    using this_t = velocity_model<object_t>;

    enum IDX
    {
    	// Индекс матриц - для удобства
    };

    explicit velocity_model() : m_concept::model_concept<T_object>() {}

    ~velocity_model() = default;

    void matricies_init(const object_t& obj_);

    void make_X(double dt_);
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


//-----------------------------------------------------------------------------------

template <class T_object>
void velocity_model<T_object>::matricies_init(const object_t& obj_) { }

//-----------------------------------------------------------------------------------

template <class T_object>
void velocity_model<T_object>::make_X(double dt_) { }

//-----------------------------------------------------------------------------------

template <class T_object>
Eigen::MatrixXd velocity_model<T_object>::make_A(double dt_) {}

//-----------------------------------------------------------------------------------

template <class T_object>
Eigen::MatrixXd velocity_model<T_object>::make_Q(double dt_) {}

//-----------------------------------------------------------------------------------

template <class T_object>
Eigen::MatrixXd velocity_model<T_object>::make_Y(const object_t& obj_) {}

//-----------------------------------------------------------------------------------

template <class T_object>
Eigen::MatrixXd velocity_model<T_object>::make_C() {}

//-----------------------------------------------------------------------------------

template <class T_object>
Eigen::MatrixXd velocity_model<T_object>::make_R() {}

//-----------------------------------------------------------------------------------


template <class T_object>
void velocity_model<T_object>::update(object_t* object_) { 
    // object_->speed_ = X(IDX::X);     Пример обновления вектора состояния значениями
}

//-----------------------------------------------------------------------------------

template <class T_object>
bool velocity_model<T_object>::equal(m_concept::model_concept<T_object> *concept_) {
    return concept_ && dynamic_cast<this_t*>(concept_);
}
}               /// <---model
}           /// <--- kalman
}       /// <--- filter

#endif

