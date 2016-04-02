#ifndef VISIONOBJECT_H
#define VISIONOBJECT_H
#include <vector>
#include <opencv2/core/mat.hpp>

/**
 * @brief The VisionObject struct
 *
 * Contains the information of and object that is extracted
 * fromt a camera frame.
 */
struct VisionObject{
	cv::Mat objectImage;
	std::vector<cv::Point> data;
};

#endif // VISIONOBJECT_H
