#ifndef OTSU_FILTER_HPP
#define OTSU_FILTER_HPP

#include "gauss_blur.hpp"

namespace sys {
namespace algorithms {
namespace otsu {
/** -------------------------------------------------------------------------------------------------------------------
     * @brief The filter_concept class - абстрактынй класс фильтрации методом оцу с использованием различных методов
     * сглаживания пороговых значений весов сигналов
     ----------------------------------------------------------------------------------------------------------------------*/
struct filter_concept {
    virtual void filtering(float& thr_,
                           std::vector<std::reference_wrapper<types::tp_signal>>& signals_,
                           std::vector<float>& weights_) = 0;
};

/** ------------------------------------------------------------------------------------------------------------------------
 * @brief The otsu_filter class - класс фильтрации сигналов методом Оцу
 --------------------------------------------------------------------------------------------------------------------------*/
class otsu_filter : public gauss::gauss_smoothing  {
public:
    /**
     * @brief otsu_filter - Конструктор
     */
    explicit otsu_filter();

    /**
     * Деструктор
     */
    virtual ~otsu_filter() {}

    /**
     * @brief exec_filter - выполнение фильтра Оцу для поиска ОН
     */
    void exec_filter();

private:
    /**
     * @brief float_to_uint8_t - перевод значения веса из float в формат uint8
     * Веса должны быть нормированы (от 0 до 1)
     * @param p_ - вычисленный вес сигнала
     * @return значение веса в формате uint8
     */
    inline uint8_t float_to_uint8_t(float& p_) {
        return (p_ >= 1.0) ? 255 : p_ * 256;
    }

protected:
    /**
     * @brief otsu_threshold - вычисление порогового значения весла методом Оцу
     * @return пороговое значение фильтра Оцу
     */
    float otsu_threshold();

protected:
    std::vector<uint16_t> hist_ = {};                       /// <--- гистограмма распределения весов сигналов
    boost::shared_ptr<filter_concept> filter_ = nullptr;    /// <--- фильтр сигналов

public:     // DEBUG
    int numb_signals_frame_ = 0;
    float otsu_threshold_ = 0.0;
    float real_otsu_threshold_ = 0.0;
};
}           /// <--- otsu
}       /// <--- algorithms
}   /// <--- sys

#endif
