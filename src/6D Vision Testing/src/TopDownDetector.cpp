#include "TopDownDetector.h"
#include "VisionObject.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgcodecs.hpp>

float TopDownDetector::getYaw(const VisionObject& part)
{
	auto templateImage = loadTemplateImage(part.typeName);
}

cv::Point TopDownDetector::getXY(const VisionObject& part)
{
	cv::Point centerOfMass;
	cv::Point partImageOrigin;
	cv::Point absoluteCenter;

	//Calculate center of mass
	centerOfMass = getCenterOfMass(part.objectImage);

	//Find the top left corner of the part by drawing
	//a bounding rectangle around the part data
	//and taking the tl. The part data is relative to the
	//full image from which it was extracted.
	partImageOrigin = cv::boundingRect(part.data).tl();

	//The absolute center of the image is the center of mass
	//added to the origin of the part image relative to its original image
	absoluteCenter = partImageOrigin + centerOfMass;
	return absoluteCenter;
}

cv::Point TopDownDetector::getCenterOfMass(const cv::Mat& partImage)
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

cv::Mat TopDownDetector::loadTemplateImage(const std::string typeName)
{
	auto path = "templateImages/topDown/" + typeName;
	return cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
}