/* 
 * File:   TimeData.cpp
 * Author: alexander-ubuntu
 * 
 * Created on June 16, 2013, 3:36 PM
 */

#include "rexos_datatypes/TimeData.h"
namespace rexos_datatypes{
    TimeData::TimeData() {
    }

    int TimeData::getDuration(){
        return this->duration;
    }
    void TimeData::setDuration(int duration){
        this->duration = duration;
    }
}