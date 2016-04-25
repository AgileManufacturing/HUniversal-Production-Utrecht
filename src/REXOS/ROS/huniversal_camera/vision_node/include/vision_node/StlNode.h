#ifndef STLNODE_H
#define STLNODE_H

#include "ros/ros.h"
#include "rexos_logger/rexos_logger.h"

#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>

class StlNode{
public:
    StlNode(image_transport::ImageTransport& imageTransport);

    void handleFrame(cv::Mat& frame);

private:
    image_transport::Publisher debugImagePublisher;

};


#endif
