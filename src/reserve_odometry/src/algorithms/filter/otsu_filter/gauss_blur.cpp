#include "../../../../../include/detection/algorithms/filters/otsu_filter/gauss_blur.hpp"

#include <chrono>
#include <thread>


using namespace sys;
using namespace algorithms;
using namespace gauss;


void gauss_smoothing::exec_gauss_smoothing_vector() {
    //(?) Проверяем наличие конфигурации фильтра Оцу и выставленный режим работы
    if (!(this->mode_work_ & tg_work_mode::_tg_filter_active_) || !this->ptr_otsu_cfg_) {
        return;
    }

    //(?) Если сигналы были зарегистрированы, то выполняем размытие в соответствии с конфигурацией...
    if (this->signals_.size()) {
        // Сбрасываем значения минимального и максимального весов
        this->weight_minimum_ = std::numeric_limits<float>::max();
        this->weight_maximum_ = -std::numeric_limits<float>::max();

        // Инициализируем веса
        this->weights_.resize(this->signals_.size(), 1.0);

        // Сортируем сигналы по углу визирования
        std::sort(this->signals_.begin(), this->signals_.end(),
                  [ & ](auto& s1_, auto& s2_){
                      return
                          s1_.get().um_signal_clone().um_position()[2] <
                          s2_.get().um_signal_clone().um_position()[2];
                  });

        //(?>) Вычисляем веса сигналов в соответствии с конфигурации фильтра Оцу
        for (uint16_t i = 0; i < this->signals_.size(); ++i) {
            auto &s1_ = this->signals_[i].get();
            auto ss1_ = s1_.um_signal_clone();
            for (uint16_t j = i+1; j < this->signals_.size(); ++j) {
                auto s2_ = this->signals_[j].get().um_signal_clone();
                if (s2_.um_position()[2] > s1_.angle_search_maximum()) {
                    break;
                }
                float dist_ = (this->mode_work_ & tg_work_mode::_tg_smooth_range_) ?
                                  ss1_.um_position().distance(s2_.um_position()) : 0.0;
                if (dist_ > this->ptr_otsu_cfg_->range_gauss_sigma()) {
                    continue;
                }
                float v_error_ = (this->mode_work_ & tg_work_mode::_tg_smooth_speed_) ?
                                     ss1_.um_speed().distance(s2_.um_speed()) : 0.0;
                float w1_ = exp(dist_ * dist_ * k1_);
                float w2_ = exp(v_error_ * v_error_ * k2_);

                weights_[i] += w1_ * w2_;
                weights_[j] += w1_ * w2_;
            }
            float w3_ = (this->mode_work_ & tg_work_mode::_tg_priority_move_) ?
                            1.0 /
                                (1.0 + exp((-1)*pow(s1_.um_speed()[0],2.0) /
                                           (2.0 * pow(this->ptr_otsu_cfg_->coeff_priority_moving(),2.0)))) :
                            1.0;
            this->weights_[i] *= w3_;
        }

        for (auto& w_ : this->weights_) {
            this->weight_minimum_ = (this->weight_minimum_ > w_) ? w_ : this->weight_minimum_;
            this->weight_maximum_ = (this->weight_maximum_ < w_) ? w_ : this->weight_maximum_;
        }

        //(?>) Нормируем веса сигналов (при условии, что выявлены максимальное и минимальное значение весов)
        float k_ = this->weight_maximum_ - this->weight_minimum_;
        if (k_) {
            std::for_each(this->weights_.begin(), this->weights_.end(),
                          [&](auto& w_){ w_ = (w_ - this->weight_minimum_) / k_; });
        }

        if (this->mode_work_ & tg_work_mode::_tg_store_effect_) {
            this->push_weights();

            for (auto& w_ : this->weights_) {
                this->weight_minimum_ = (this->weight_minimum_ > w_) ? w_ : this->weight_minimum_;
                this->weight_maximum_ = (this->weight_maximum_ < w_) ? w_ : this->weight_maximum_;
            }

            //(?>) Нормируем веса сигналов (при условии, что выявлены максимальное и минимальное значение весов)
            float k_ = this->weight_maximum_ - this->weight_minimum_;
            if (k_) {
                std::for_each(this->weights_.begin(), this->weights_.end(),
                              [&](auto& w_){ w_ = (w_ - this->weight_minimum_) / k_; });
            }
        }
        this->multiple_coefficient_ = k_;
    }
}

void gauss_smoothing::push_weights() {
    // Сбрасываем значения минимального и максимального весов
    this->weight_minimum_ = std::numeric_limits<float>::max();
    this->weight_maximum_ = -std::numeric_limits<float>::max();

    for (uint16_t i = 0; i < this->signals_.size(); ++i) {
        /*if (this->signals_[i].get().is_filtered())  {
            continue;
        }*/
        this->weights_[i] = 1 - (1 - weights_[i]) * (1 - this->signals_[i].get().buffer_weight());
        //this->weights_[i] *= (this->signals_[i].get().buffer_weight());
        this->weight_minimum_ = (this->weight_maximum_ > this->weights_[i]) ?
                                    this->weights_[i] : this->weight_minimum_;
        this->weight_maximum_ = (this->weight_maximum_ < this->weights_[i]) ?
                                    this->weights_[i] : this->weight_maximum_;
    }
}
