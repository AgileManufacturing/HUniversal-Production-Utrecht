#include "Utilities.h"
#include "VisionObject.h"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <iterator>


cv::Mat Utilities::rotateImage(const cv::Mat& srcImage,cv::Mat& dstImage, double angle, double scale) 
{
	auto centrePoint = getCenterOfMass(srcImage);
	auto rotMat = cv::getRotationMatrix2D(centrePoint, angle, scale);
	cv::warpAffine(srcImage, dstImage, rotMat, srcImage.size());
	return dstImage;
}

cv::Point Utilities::getCenterOfMass(const cv::Mat& partImage)
{
	cv::Moments moments;
	cv::Point centerOfMass;

	//Use the first order moments to calculate the 
	//center of mass of the object.
	moments = cv::moments(partImage, true);
	centerOfMass.x = moments.m10 / moments.m00;
	centerOfMass.y = moments.m01 / moments.m00;

	return centerOfMass;
}

void Utilities::exitWithMessage(std::string message)
{
	std::cerr << message << std::endl;
	std::cerr << "Terminating program";
	exit(EXIT_FAILURE);
}

int Utilities::addDegrees(int rotation, int degrees)
{
	int newAngle = rotation + degrees;
	if (newAngle > 360) {
		newAngle -= 360;
	}
	return newAngle;
}

std::vector<int> Utilities::filterYaw(const std::vector<std::pair<int, int>>& matchList)
{
	std::vector<int> yawList;
	bool newYaw;
	for (auto p : matchList) {
		if (p.first >= 80) {
				newYaw = true;
				for (auto y : yawList) {
					if (abs(100 - ((y * 100) / p.second)) <= 20) {
						newYaw = false;
					}
				}
				if (newYaw) {
					yawList.push_back(p.second);
				}
			}
		}
	return yawList;
}

cv::Point2i Utilities::getPartOffsets(VisionObject part)
{
	int minx = 10000, miny = 10000;
	for (cv::Point p : part.data) {
		if (p.x < minx) {
			minx = p.x;
		}
		if (p.y < miny) {
			miny = p.y;
		}
	}
	return cv::Point2i(minx,miny);
}

std::istream& operator>>(std::istream& input, cv::Point& point)
{
	char discardC;
	//Read x
	input >> point.x;
	//Discard ,
	input >> discardC;
	//Read y
	input >> point.y;
	//Discard ; and possible newline
	input >> discardC;
	return input;
}

