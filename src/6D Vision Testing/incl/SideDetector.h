#ifndef _SIDE_DETECTOR_H
#define _SIDE_DETECTOR_H
#include <opencv2/core/mat.hpp>

class VisionObject;

class SideDetector
{
public:
	static int getZ(const VisionObject& part);
private:
	static cv::Point getCenterOfMass(const cv::Mat& partImage);
};

#endif