#include "../../../../../include/detection/algorithms/filters/otsu_filter/otsu_filter.hpp"

#include <chrono>
#include <thread>

using namespace sys;
using namespace algorithms;
using namespace otsu;


/** ------------------------------------------------------------------------------------------------------------------------
 * @brief The average_moving_model class - модель фильтрации с использованием сколльзящего среднего
 --------------------------------------------------------------------------------------------------------------------------*/
struct average_moving_model : filter_concept {
    float real_threshold_ = 0.0;

    /**
     * @brief average_moving_model - конструктор
     * @param ptr_otsu_cfg_ - указатель на конфигурацию фильтра Оцу
     */
    explicit average_moving_model(config::radar_otsu_filter_configuration* ptr_otsu_cfg_) :
        ptr_cfg_(ptr_otsu_cfg_) {
        this->thrs_.resize(this->ptr_cfg_->window_size(),0.0);
    }


    /**
     * @brief filtering - фильтрация сигналов
     * @param thr_ - пороговое значение (вычисленное)
     * @param signals_ - массив ссылок на сигналы
     */
    void filtering(float& thr_,
                   std::vector<std::reference_wrapper<types::tp_signal>>& signals_,
                   std::vector<float>& weights_) {
        if (ptr_cfg_ && ptr_cfg_->window_size()) {
            // Инкрементируем размер окна до тех пор, пока не достигнет конфигурационного значения
            if (this->size_wind_ < ptr_cfg_->window_size()) {
                this->size_wind_++;
            }

            // Определяем текущее пороговое значение фильтра
            this->current_threshold_ -= this->thrs_[this->current_filter_ % ptr_cfg_->window_size()];
            this->current_threshold_ += thr_;
            this->thrs_[this->current_filter_ % ptr_cfg_->window_size()] = thr_;
            this->real_threshold_ = this->current_threshold_ / this->size_wind_;
            this->current_filter_++;

            //(?>) Фильтруем сигналы со слабыми весами
            this->signals_numb_ = 0;        // DEBUG
            for (uint16_t i = 0; i < signals_.size(); ++i) {
                auto &s_ = signals_[i].get();
                auto pose_ = s_.um_signal_clone().um_position();

                s_.set_buffer_weight(weights_[i]);

                if (weights_[i] >= this->real_threshold_) {
                    signals_[i].get().activate();
                }
                if (!signals_[i].get().is_filtered()) {
                    this->signals_numb_++;
                }
            }
        }
    }

    std::vector<float> thrs_ = {};                                  /// <--- окно пороговых значений
    float current_threshold_ = 0.0;                                 /// <--- текущая сумма порогового значения фильтра
    uint16_t current_filter_ = 0;                                   /// <--- текущая итерация фильтра
    uint16_t size_wind_ = 0;                                        /// <--- текущий размер окна
    config::radar_otsu_filter_configuration* ptr_cfg_ = nullptr;    /// <--- указатель на конфигурацию фильтра Оцу

    uint16_t signals_numb_ = 0;     /// DEBUG
};


#define TYPE_STORAGE_MOVING_AVERAGE (std::string)"moving_average"
#define TYPE_STORAGE_EXPONENTIAL_SMOOTHING (std::string)"exponential_smoothing"


otsu_filter::otsu_filter() {
    // Инициализация гистограммы распределения весов сигналов
    this->hist_.resize(256,0);

    if (configuration_ && configuration_->detection_configuration() &&
        configuration_->detection_configuration()->filter_configuration()) {
        auto fcfg_ = configuration_->detection_configuration()->filter_configuration();
        if (fcfg_->otsu_filter_configuration()) {
            if (fcfg_->otsu_filter_configuration()->filter_active() &&
                fcfg_->otsu_filter_configuration()->type_storage() == TYPE_STORAGE_MOVING_AVERAGE) {
                this->filter_ = boost::shared_ptr<filter_concept>(
                    new average_moving_model(fcfg_->otsu_filter_configuration().get()));
            }
        }
    }
}


void otsu_filter::exec_filter() {
    //(?) Если фильтр не определен, прерываем фильтрацию
    if (!this->filter_) {
        return;
    }

    // Выполняем сглаживание весов по гауссу
    this->exec_gauss_smoothing_vector();

    // Вычисляем пороговое значение фильтра
    auto thr_ = this->otsu_threshold();
    this->otsu_threshold_ = thr_;       // DEBUG

    // Фильтруем сигналы
    this->filter_->filtering(thr_,this->signals_,this->weights_);
    this->real_otsu_threshold_ = dynamic_cast<average_moving_model*>(this->filter_.get())->real_threshold_;

    this->numb_signals_frame_ = dynamic_cast<average_moving_model*>(this->filter_.get())->signals_numb_;    /// DEBUG
}


float otsu_filter::otsu_threshold() {
    uint8_t min_value_ = this->float_to_uint8_t(this->weight_minimum_);
    uint8_t max_value_ = this->float_to_uint8_t(this->weight_maximum_);

    if (min_value_ == max_value_) {
        return 0.0;
    }

    std::for_each(this->hist_.begin(),this->hist_.end(),[ & ](auto& el_){ el_ = 0; });
    for (auto& weight_ : this->weights_) {
        auto cur_weight_ = this->float_to_uint8_t(weight_);
        hist_[cur_weight_]++;
    }

    int32_t m = 0, n = 0;
    for (int32_t t = 0; t <= max_value_ - min_value_; t++) {
        m += t * hist_[t];
        n += hist_[t];
    }

    float max_sigma = -1.0;
    int32_t thr_ = 0;
    int32_t alpha_ = 0;
    int32_t betha_ = 0;

    for (int32_t t = 0; t < max_value_ - min_value_; t++) {
        alpha_ += t * hist_[t];
        betha_ += hist_[t];
        float w1 = (float)betha_ / n;
        float a = (float)alpha_ / betha_ - (float)(m - alpha_) / (n - betha_);
        float sigma = w1 * (1 - w1) * a * a;

        if (sigma > max_sigma) {
            max_sigma = sigma;
            thr_ = t;
        }
    }

    thr_ += min_value_;

    return thr_ / 256.0;
}
