#ifndef MODULE_KALMAN_OBJECT_HPP
#define MODULE_KALMAN_OBJECT_HPP

#include <boost/shared_ptr.hpp>
#include "model_concept.hpp"

namespace sys {
namespace filter {
namespace kalman {
/**--------------------------------------------------------------------------------------------------------------------------
 * @brief The kalman_object class - объект наблюдения, сопровождаемый фильтром Калмана
 * Используется расширенный фильтр Калмана
 * В качестве шаблона передается тип сопровождаемого объекта
-------------------------------------------------------------------------------------------------------------------------- */
template <class T_object>
class kalman_object {
public:
    using object_t = T_object;
    using concept_t = boost::shared_ptr<m_concept::model_concept<object_t>>;

    /**
     * @brief kalman_object - конструктор
     * @param model_ - модель фильтра Калмана
     */
    explicit kalman_object(concept_t model_) : concept_(model_) {}

    /**
     * Деструктор
     */
    ~kalman_object() = default;

    /**
     * @brief Set the model object
     * Установление модели для фильтра Калмана
     * @param model_ - устанавливаемя модель фильтра Калмана
     */
    void set_model(concept_t model_);

    /**
     * @brief init_matricies
     * Инициализация матрицы вектора состояния и матрицы вектора состояния сопровождаемого ОН
     */
    void init_matricies();

    /**
     * @brief predict - прогнозирование вектора состояния на момент через интервал времени dt_
     * @param dt_ - интервал времени для прогнозирования
     * @return результат прогнозирования
     */
    bool predict(double dt_, float ref_v_ = 0.0);

    /**
     * @brief update - обновление вектора состояния
     * @param stage_ - измеряемое состояние объекта наблюдения
     * @return результат обновления
     */
    bool update(const object_t& stage_);

    /**
     * @brief object - получение указателя на сопровождаемый объект
     * @return указатель на сопровождаемый объект
     */
    object_t* object();

    /**
     * @brief X - получение вектора состояния ОН 
     * @return Eigen::MatrixXd& вектор состояния ОН
     */
    inline Eigen::MatrixXd& X() const { return this->concept_->X_; }

    /**
     * @brief P - получение матрицы ковариации ОН
     * @return Eigen::MatrixXd& матрица ковариации
     */
    inline Eigen::MatrixXd& P() const { return this->concept_->P_; }

    /**
     * @brief Get the model object
     * Получение модели фильтра Калмана
     * @return concept_t указатель на модель Фильтра Калмана
     */
    inline concept_t get_model() const { return this->concept_; }

protected:
    enum sign_tag {
        tg_none_ = 0x00,
        tg_predict_ = 0x01,
        tg_update_ = 0x02
    };

    uint8_t sign_ = sign_tag::tg_none_;

private:
    concept_t concept_ = nullptr;       /// <--- указатель на модель
};



template <class T_object>
void kalman_object<T_object>::set_model(typename kalman_object<T_object>::concept_t model_) {
    this->concept_ = model_;
}


template <class T_object>
void kalman_object<T_object>::init_matricies() {
    this->concept_->matricies_init(*this->object());
}


template <class T_object>
T_object* kalman_object<T_object>::object() {
    return static_cast<object_t*>(this);
}


template <class T_object>
bool kalman_object<T_object>::predict(double dt_, float ref_v_) {
    this->concept_->make_X(dt_);
    auto A_ = this->concept_->make_A(dt_);
    auto Q_ = this->concept_->make_Q(dt_);
    this->CQ_ = Q_;
    

    if (this->concept_->X_.rows() == this->concept_->X_.rows() &&
            A_.cols() == this->concept_->P_.rows() && Q_.cols() == Q_.rows() &&
            A_.rows() == Q_.cols()) {
        this->concept_->P_ = A_ * this->concept_->P_ * A_.transpose() + Q_;

        this->concept_->update(this->object());

        return true;
    }

    return false;
}


template <class T_object>
bool kalman_object<T_object>::update(const T_object& stage_) {
    auto C_ = this->concept_->make_C();
    auto Y_ = this->concept_->make_Y(stage_);
    auto R_ = this->concept_->make_R();

    if (C_.cols() == this->concept_->X_.rows() && this->concept_->P_.cols() == C_.cols() && R_.rows() == R_.cols()
            && R_.rows() == C_.rows() && Y_.rows() == C_.rows()) {
        Eigen::MatrixXd Y_pr_ = C_ * this->concept_->X_;
        Eigen::MatrixXd PCT_ = this->concept_->P_ * C_.transpose();
        Eigen::MatrixXd K_ = PCT_ * ((R_ + C_ * PCT_).inverse());

        this->concept_->X_ = this->concept_->X_ + K_ * (Y_ - Y_pr_);
        this->concept_->P_ = this->concept_->P_ - K_ * (C_ * this->concept_->P_);

        this->concept_->update(this->object());

        return true;
    }

    return false;
}
}           /// <--- kalman
}       /// <--- filter
}   /// <--- sys

#endif
