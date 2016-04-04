#ifndef _VISION_OBJECT_H
#define _VISION_OBJECT_H
#include <vector>
#include <opencv2/core/mat.hpp>

class VisionObject
{
public:
	cv::Mat objectImage;
	std::vector<cv::Point> data;
	std::string typeName;
};

#endif
