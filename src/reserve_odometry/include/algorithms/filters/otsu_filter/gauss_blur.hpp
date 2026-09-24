#ifndef GAUSS_SMOOTHING_HPP
#define GAUSS_SMOOTHING_HPP

#include <functional>
#include <set>
#include "../../../common/type_traits/reference_wrapper.hpp"
#include "../../../types/tp_signal.hpp"
#include "../../../configuration/configuration.hpp"

namespace sys {
namespace algorithms {
namespace gauss {
/** ------------------------------------------------------------------------------------------------------------------------
 * @brief The gauss_smoothing class - класс для размытия сигналов функцией гаусса
 --------------------------------------------------------------------------------------------------------------------------*/
class gauss_smoothing {
protected:
    /**
     * @brief The tg_work_mode enum - метки режима работы размытия по гауссу
     */
    enum tg_work_mode {
        _tg_filter_no_active_   = 0b00000000,       /// <--- размытие по гауссу не активно
        _tg_filter_active_      = 0b00000001,       /// <--- размытие по гауссу активно
        _tg_smooth_range_       = 0b00000010,       /// <--- метка размытия по расстоянию
        _tg_smooth_speed_       = 0b00000100,       /// <--- метка размытия по скорости
        _tg_priority_move_      = 0b00001000,       /// <--- метка приоритета движущихся целей
        _tg_store_effect_       = 0b00010000        /// <--- метка буферного эффекта
    };

    using itr_t_ = std::multiset<sys::type_traits::reference_wrapper<types::tp_signal>>::iterator;

public:
    /**
     * @brief gauss_smoothing - Конструктор
     */
    explicit gauss_smoothing() = default;

    /**
     * Деструктор
     */
    virtual ~gauss_smoothing() {}

    /**
     * @brief exec - Сглаживание функцией гаусса
     * Параметры размытия задаются в конфигурации системы детекции ОН
     */
    void exec_gauss_smoothing_vector();

    std::vector<std::reference_wrapper<types::tp_signal>>& all_signals() { return this->signals_; }


private:
    /**
     * @brief push_weights - использование накопительного эффекта
     */
    void push_weights();

protected:
#ifndef SET_LINEAR_BUFFER
    std::vector<std::reference_wrapper<types::tp_signal>> signals_ = {};        /// <--- Ссылки на все сигналы
#else
    std::multiset<sys::type_traits::reference_wrapper<types::tp_signal>> m_signals_ = {};
#endif
    std::vector<float> weights_ = {};                /// <--- Текущие значения весов (используются при storage_effect)
    float multiple_coefficient_ = 1.0;               /// <--- множитель весов
    uint8_t mode_work_ = tg_work_mode::_tg_filter_no_active_;   /// <--- режим работы размытия
    config::radar_otsu_filter_configuration* ptr_otsu_cfg_ = nullptr;   /// <--- указатель на конфигурацию фильтра Оцу
    config::radar_clustering_configuration* ptr_cluster_cfg_ = nullptr; /// <--- указатель на конфигурацию кластеризации
    float weight_minimum_ = 0.0;                    /// <--- минимальное текущее значение веса сигналов
    float weight_maximum_ = 0.0;                    /// <--- максимальное текущее значение веса сигнала
    float k1_ = 0.0;                                /// <--- коэффициенты определения весов сигналов
    float k2_ = 0.0;                                /// <--- коэффициенты определения весов сигналов
};
}           /// <--- gauss
}       /// <--- algorithms
}   /// <--- sys

#endif
