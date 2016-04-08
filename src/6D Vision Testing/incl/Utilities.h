#define _UTILITIES_H
#ifdef _UTILITIES_H
#include <opencv2/core/mat.hpp>

class Utilities
{
public:
	static cv::Mat rotateImage(const cv::Mat& image, cv::Mat& dstImage, double angle, double scale = 1.0);
	static cv::Point getCenterOfMass(const cv::Mat& partImage);
	static void exitWithMessage(std::string message);
};

std::istream& operator>>(std::istream& input, cv::Point& point);
#endif