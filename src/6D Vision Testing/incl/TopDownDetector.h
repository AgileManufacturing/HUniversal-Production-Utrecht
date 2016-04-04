#ifndef _TOP_DOWN_DETECTOR_H
#define _TOP_DOWN_DETECTOR_H
#include <opencv2/core/mat.hpp>

class VisionObject;

class TopDownDetector
{
public:
	static float getYaw(const VisionObject& part);
	static cv::Point getXY(const VisionObject& part);
private:
	static cv::Point getCenterOfMass(const cv::Mat& partImage);
	static cv::Mat loadTemplateImage(const std::string typeName);
};
#endif