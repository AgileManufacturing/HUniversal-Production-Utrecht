#define UTILITIES_H
#ifdef UTILITIES_H

#include "opencv2\core\core.hpp"

class Utilities{
public:
	static cv::Mat rotateImage(const cv::Mat& image, cv::Mat& dstImage, double angle, double scale = 1.0);
	static cv::Point getCenterOfMass(const cv::Mat& partImage);
};

#endif