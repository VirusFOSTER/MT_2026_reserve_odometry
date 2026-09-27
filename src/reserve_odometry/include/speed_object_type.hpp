#pragma once

#include "algorithms/filters/kalman_filter/kalman_object.hpp"
#include "algorithms/filters/kalman_filter/models/velocity_model.hpp"

//-------------------------------KALMAN OBJECT------------------------------------------------
struct speed_object_type : public filter::kalman::kalman_object<speed_object_type> {
    double speed_ = 0.0f;            
    double ds_ = 0.0f;

    explicit speed_object_type(boost::shared_ptr<filter::kalman::model_concept<speed_object_type>> model_) :
        filter::kalman::kalman_object<speed_object_type>(model_){
        if (model_) {
            this->init_matricies();
        }
    }
};

//--------------------------------------------------------------------------------------------