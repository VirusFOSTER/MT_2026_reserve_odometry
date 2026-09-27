#ifndef MODULE_KALMAN_FILTER_MODEL_CONCEPT_HPP
#define MODULE_KALMAN_FILTER_MODEL_CONCEPT_HPP

#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/Dense"

namespace filter {
namespace kalman {
namespace m_concept {
/**
 * @brief The model_concept class - концепция построения модели фильтров Калмана
 * В качестве шаблона передается тип сопровождаемого объекта
 */
template <class T_object>
struct model_concept {
    using object_t = T_object;

    /**
     * @brief matricies_init - инициализация вектора состояния и матрицы ковариации
     * @param obj_ - объект, при помощи параметров которого заполняются матрицы
     */
    virtual void matricies_init(const object_t& obj_) = 0;

    /**
     * @brief make_X - формирование вектора состояния через интервал времени dt_
     * @param dt_ - интервал времени для прогнозирования
     */
    virtual void make_X(double dt_) = 0;

    /**
     * @brief make_A - формирование матрицы перехода состояния
     * @param dt_ - интервал времени для прогнозирования
     */
    virtual Eigen::MatrixXd make_A(double dt_) = 0;

    /**
     * @brief make_Q - формирование ковариационной матрицы шума процесса
     * @param dt_ - интервал времени для прогнозирования
     */
    virtual Eigen::MatrixXd make_Q(double dt_) = 0;

    /**
     * @brief make_Y - формирование иновационной матрицы состояния
     * @param obj_ - объект, параметрами которого формируется инновационная матрица состояния
     */
    virtual Eigen::MatrixXd make_Y(const object_t& obj_) = 0;

    /**
     * @brief make_C - формирование матрицы измерений
     */
    virtual Eigen::MatrixXd make_C() = 0;

    /**
     * @brief make_R - формирование матрицы шума измерений
     */
    virtual Eigen::MatrixXd make_R() = 0;

    /**
     * @brief update - обновление объекта наблюдения
     */
    virtual void update(object_t* object_) = 0;

    /**
     * @brief equal
     * Сравнение текущей модели с передаваемой
     * По умолчанию сравниваемые модели совпадают
     * @param concept_ - указатель на сравниваемую модель 
     * @return true - модели совпадают
     * @return false - модели не совпадают
     */
    virtual bool equal(model_concept<T_object>*) { return true; }

    Eigen::MatrixXd X_;     /// <--- матрица вектора состояния фильтруемого объекта
    Eigen::MatrixXd P_;     /// <--- ковариационная матрица фильтруемого объекта
};
}               /// <--- concept
}           /// <--- kalman
}       /// <--- filter

#endif
