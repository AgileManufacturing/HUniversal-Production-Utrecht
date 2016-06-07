#ifndef STLNODE_H
#define STLNODE_H

#include "ros/ros.h"
#include "rexos_logger/rexos_logger.h"
#include "vision_node/FoundPublisher.h"

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>

/**
 * @brief The StlNode class
 *
 * This class handles the camera frames that are used for STL based part
 * recognition.
 */
class StlNode{
public:

    /**
     * @brief StlNode constructor
     * @param imageTransport Used for transporting images in ROS.
     */
    StlNode(image_transport::ImageTransport& imageTransport);
    /**
     * @brief handleFrame
     *
     * This function applies feature extraction and matching on the given
     * camera frame. The methods used are explained on the STL vision wiki page.
     * @param frame The frame that is processed.
     */
    void handleFrame(cv::Mat& frame);

private:
    image_transport::Publisher debugImagePublisher;
    FoundPublisher part_publisher;
};


#endif
