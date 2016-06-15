#ifndef _SIDE_DETECTOR_H
#define _SIDE_DETECTOR_H
#include <opencv2/core/mat.hpp>

class VisionObject;

class SideDetector
{
public:
	static int getZ(const VisionObject& part);
	static cv::Point3i getRollPitchYaw(VisionObject part, std::vector<int> yaws);
private:
	static cv::Point getCenterOfMass(const cv::Mat& partImage);
	static std::vector<cv::Point> getPossibleRollPitchValues(const VisionObject& part);
};

#endif