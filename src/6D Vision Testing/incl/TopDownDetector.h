#ifndef _TOP_DOWN_DETECTOR_H
#define _TOP_DOWN_DETECTOR_H
#include <opencv2/core/mat.hpp>

class VisionObject;

class TopDownDetector
{
public:
	static std::vector<int> getYaw(VisionObject& part);
	static cv::Point getXY(const VisionObject& part);
private:
	static cv::Mat loadTemplateImage(const VisionObject& part);
};
#endif