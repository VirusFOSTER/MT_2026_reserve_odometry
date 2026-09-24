#include "../../../../include/detection/algorithms/clustering/dbscan.hpp"


using namespace sys;
using namespace algorithms;
using namespace dbscan;


bool tclusters_test_1(sys::algorithms::dbscan::list_cluster_t* list_);
bool tclusters_test_2(sys::algorithms::dbscan::list_cluster_t* list_);
bool tclusters_test_3(sys::algorithms::dbscan::list_cluster_t* list_);
bool rclusters_test_4(sys::algorithms::dbscan::list_cluster_t* list_,
                      std::vector<std::reference_wrapper<types::tp_signal> > &signals_);



cluster_t::~cluster_t() {
    if (this != this->list_pointer_->first_cluster_ && this != this->list_pointer_->last_cluster_) {
        this->prev_cluster_->next_cluster_ = this->next_cluster_;
        this->next_cluster_->prev_cluster_ = this->prev_cluster_;
    } else {
        if (this == this->list_pointer_->first_cluster_) {
            if (this->next_cluster_) {
                this->next_cluster_->prev_cluster_ = nullptr;
            }
            this->list_pointer_->first_cluster_ = this->next_cluster_;
        }
        if (this == this->list_pointer_->last_cluster_) {
            if (this->prev_cluster_) {
                this->prev_cluster_->next_cluster_ = nullptr;
            }
            this->list_pointer_->last_cluster_ = this->prev_cluster_;
        }
    }
}


DBSCAN::DBSCAN() {
    if (configuration_ && configuration_->detection_configuration() &&
        configuration_->detection_configuration()->buffer_configuration()) {
        this->ptr_cfg_ = configuration_->detection_configuration()->buffer_configuration()->clustering_configuration().get();
    }
    if (this->ptr_cfg_) {
        this->clustering_active_ = this->ptr_cfg_->search_by_radius() || this->ptr_cfg_->search_by_speed();
    }
}


void DBSCAN::exec_clustering(time_t &time_, std::vector<std::reference_wrapper<types::tp_signal> > &signals_) {
    if (!this->clustering_active_) {
        return;
    }

    this->capture_signals(signals_);

    for (uint16_t i = 0; i < signals_.size(); ++i) {
        auto& s1_ = signals_[i].get();
        if (s1_.is_filtered() || s1_.captured()) {
            continue;
        }

        auto& ss1_ = s1_.um_signal_clone();

        // (?) Формируем кластер, если его нет
        if (!s1_.get_cluster_pointer()) {
            if (this->clusters_list_) {
                if (this->clusters_list_->last_cluster_) {
                    this->clusters_list_->last_cluster_->next_cluster_ =
                        new cluster_t(s1_,this->clusters_numb_,this->clusters_list_);
                    this->clusters_list_->last_cluster_->next_cluster_->prev_cluster_ = this->clusters_list_->last_cluster_;
                    this->clusters_list_->last_cluster_ = this->clusters_list_->last_cluster_->next_cluster_;
                } else {
                    this->clusters_list_->first_cluster_  = new cluster_t(s1_,this->clusters_numb_,this->clusters_list_);
                    this->clusters_list_->last_cluster_ = this->clusters_list_->first_cluster_;
                }
            } else {
                this->clusters_list_ = new list_cluster_t;
                this->clusters_list_->first_cluster_ = new cluster_t(s1_,this->clusters_numb_,this->clusters_list_);
                this->clusters_list_->last_cluster_ = this->clusters_list_->first_cluster_;
            }
            this->clusters_numb_++;
        }

        for (uint16_t j = i+1; j < signals_.size(); ++j) {
            auto& s2_ = signals_[j].get();
            if (s2_.is_filtered() || !s2_.frame()->time_detect()->equal(time_) || s2_.captured()) {
                continue;
            }

            auto& ss2_ = s2_.um_signal_clone();

            if (ss2_.um_position()[2] > s1_.angle_search_maximum()) {
                break;
            }

            if (this->control_distance(ss1_,ss2_) && this->control_speed(s1_,s2_)) {
                s1_.get_cluster_pointer()->add_new_element(s2_);
            }
        }
    }
}


bool DBSCAN::control_distance(types::tp_base_signal &s1_, types::tp_base_signal &s2_) {
    return (this->ptr_cfg_->radius_search()) ?
               s1_.um_position().distance(s2_.um_position()) <= this->ptr_cfg_->radius_search() : true;
}


bool DBSCAN::control_speed(types::tp_signal &s1_, types::tp_signal &s2_) {
    return (this->ptr_cfg_->speed_error()) ?
               s1_.um_signal_clone().um_speed().distance(s2_.um_signal_clone().um_speed()) <= this->ptr_cfg_->speed_error() : true;
}


void DBSCAN::capture_signals(std::vector<std::reference_wrapper<types::tp_signal> > &signals_) {
    if (!this->clusters_list_) {
        return;
    }

    for (uint16_t i = 0; i < signals_.size(); ++i) {
        if (signals_[i].get().is_filtered() || signals_[i].get().get_cluster_pointer()) {
            continue;
        }

        auto& s_ = signals_[i].get();

        auto ptr_cluster_ = this->clusters_list_->first_cluster_;
        while (ptr_cluster_) {
            if (ptr_cluster_->capture_signal(s_)) {
                if (s_.get_cluster_pointer()) {
                    s_.get_cluster_pointer()->remove_element(s_);
                }
                ptr_cluster_->add_new_element(s_);
                s_.capture_element();
            }

            ptr_cluster_ = ptr_cluster_->next_cluster_;
        }
    }
}


void DBSCAN::capture_clusters() {
    auto ptr_1_ = this->clusters_list_->first_cluster_;
    while (ptr_1_) {
        auto ptr_2_ = ptr_1_->next_cluster_;
        while (ptr_2_) {
            if (ptr_1_->capture_cluster(*ptr_2_)) {
                auto temp_ptr_ = ptr_2_->next_cluster_;
                if (ptr_2_->first_element()) {
                    ptr_1_->add_new_element(*ptr_2_->first_element());
                }
                ptr_2_ = temp_ptr_;
                continue;
            }
            ptr_2_ = ptr_2_->next_cluster_;
        }
        ptr_1_ = ptr_1_->next_cluster_;
    }
}


uint16_t DBSCAN::clusters_numb() {
    uint16_t cl_numb_ = 0;
    if (this->clusters_list_) {
        auto ptr_ = this->clusters_list_->first_cluster_;
        while (ptr_) {
            cl_numb_++;
            ptr_ = ptr_->next_cluster_;
        }
    }
    return cl_numb_;
}



//-----------------TESTS-----------------------
bool tclusters_test_1(sys::algorithms::dbscan::list_cluster_t* list_) {
    if (!list_) {
        return true;
    }
    auto ptr_ = list_->first_cluster_;
    while (ptr_) {
        if (ptr_->total_length() >= 10.0) {
            uint16_t numb_positions_ = 0;
            auto el_ = ptr_->track();
            while (el_) {
                numb_positions_++;
                el_ = el_->next_position_;
            }
            if (numb_positions_ < 3) {
                return false;
            }
        }
        ptr_ = ptr_->next_cluster_;
    }
    return true;
}

bool tclusters_test_2(sys::algorithms::dbscan::list_cluster_t* list_) {
    if (!list_) {
        return true;
    }
    auto ptr_ = list_->first_cluster_;
    while (ptr_) {
        if (!ptr_->dynamic()) {
            ptr_ = ptr_->next_cluster_;
            continue;
        }
        auto next_ptr_ = ptr_->next_cluster_;
        while (next_ptr_) {
            if (!next_ptr_->dynamic()) {
                next_ptr_ = next_ptr_->next_cluster_;
                continue;
            }
            auto el1_ = ptr_->first_element();
            while (el1_) {
                auto el2_ = next_ptr_->first_element();
                while (el2_) {
                    if (el1_->signal_position().distance(el2_->signal_position()) <=
                            configuration_->detection_configuration()->buffer_configuration()->clustering_configuration()->radius_search() &&
                        el1_->signal_speed().distance(el2_->signal_speed()) <=
                            configuration_->detection_configuration()->buffer_configuration()->clustering_configuration()->speed_error()) {
                        return false;
                    }
                    el2_ = el2_->get_next_cluster_element();
                }
                el1_ = el1_->get_next_cluster_element();
            }
            next_ptr_ = next_ptr_->next_cluster_;
        }
        ptr_ = ptr_->next_cluster_;
    }

    return true;
}

bool tclusters_test_3(sys::algorithms::dbscan::list_cluster_t* list_) {
    if (!list_) {
        return true;
    }
    auto ptr_ = list_->first_cluster_;
    while (ptr_) {
        auto next_ptr_ = ptr_->next_cluster_;
        while (next_ptr_) {
            auto el1_ = ptr_->first_element();
            while (el1_) {
                auto el2_ = next_ptr_->first_element();
                while (el2_) {
                    if (el1_->signal_position().distance(el2_->signal_position()) <=
                            configuration_->detection_configuration()->buffer_configuration()->clustering_configuration()->radius_search() &&
                        el1_->signal_speed().distance(el2_->signal_speed()) <=
                            configuration_->detection_configuration()->buffer_configuration()->clustering_configuration()->speed_error()) {
                        return false;
                    }
                    el2_ = el2_->get_next_cluster_element();
                }
                el1_ = el1_->get_next_cluster_element();
            }
            next_ptr_ = next_ptr_->next_cluster_;
        }
        ptr_ = ptr_->next_cluster_;
    }

    return true;
}

bool rclusters_test_4(sys::algorithms::dbscan::list_cluster_t* list_,
                      std::vector<std::reference_wrapper<types::tp_signal> > &signals_) {
    if (!list_) {
        return true;
    }

    for (uint16_t i = 0; i < signals_.size(); ++i) {
        if (signals_[i].get().is_filtered() || signals_[i].get().get_cluster_pointer()) {
            continue;
        }

        auto& s_ = signals_[i].get();

        auto ptr_cluster_ = list_->first_cluster_;
        while (ptr_cluster_) {
            if (ptr_cluster_->capture_signal(s_)) {
                return false;
            }

            ptr_cluster_ = ptr_cluster_->next_cluster_;
        }
    }

    return true;
}
//-----------------TESTS-----------------------
