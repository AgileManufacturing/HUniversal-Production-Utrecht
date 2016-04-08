#include "Utilities.h"
#include <opencv2/imgproc.hpp>
#include <iostream>

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