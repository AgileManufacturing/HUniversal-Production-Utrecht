#ifndef STLNODE_H
#define STLNODE_H

#include "ros/ros.h"
#include "rexos_logger/rexos_logger.h"

#include <opencv2/core/core.hpp>

class StlNode{
public:
    StlNode();

    void handleFrame(cv::Mat& frame);

private:
};


#endif
