#include "TopDownDetector.h"
#include "VisionObject.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgcodecs.hpp>
#include "ObjectDetector.h"
#include "Utilities.h"

float TopDownDetector::getYaw(VisionObject& part)
{
	auto templateImage = loadTemplateImage(part);
	part.objectImage = ObjectDetector::applyOtsuThreshold(part.objectImage);

	int posCount = 0;
	int minx = 1000, miny = 1000;
	for (cv::Point p : part.data) {
		if (p.x < minx) {
			minx = p.x;
		}
		if (p.y < miny) {
			miny = p.y;
		}
	}
	cv::Mat tempImage;
	int bestMatch = 0;
	int highestMatch = 0;
	for (int i = 0; i < 360;i+=2) {
		Utilities::rotateImage(templateImage, tempImage, i);
		auto connectedComponents = ObjectDetector::findConnectedComponents(tempImage);
		auto objects = ObjectDetector::filterObjects(connectedComponents, tempImage);
		cv::resize(objects[0].objectImage, objects[0].objectImage, cv::Size(part.objectImage.size().width, part.objectImage.size().height));
		for (cv::Point p : part.data) {
			if ((int)objects[0].objectImage.at<uchar>(cv::Point(p.x - minx, p.y - miny)) > 0) {
				++posCount;
			}
		}		
		if (highestMatch < ((posCount * 100) / part.data.size())) {
			highestMatch = (posCount * 100) / part.data.size();
			bestMatch = i;
		}
		posCount = 0;
	}
	Utilities::rotateImage(templateImage, tempImage, bestMatch);
	auto connectedComponents = ObjectDetector::findConnectedComponents(tempImage);
	auto objects = ObjectDetector::filterObjects(connectedComponents, tempImage);
	cv::resize(objects[0].objectImage, objects[0].objectImage, cv::Size(part.objectImage.size().width, part.objectImage.size().height));
	for (cv::Point p : part.data) {
		if ((int)objects[0].objectImage.at<uchar>(cv::Point(p.x - minx, p.y - miny)) > 0) {
			cv::line(part.objectImage, cv::Point(p.x - minx, p.y - miny), cv::Point(p.x - minx, p.y - miny), cv::Scalar(125, 125, 125));
		}
	}
	return bestMatch;
}

cv::Point TopDownDetector::getXY(const VisionObject& part)
{
	cv::Point centerOfMass;
	cv::Point partImageOrigin;
	cv::Point absoluteCenter;

	//Calculate center of mass
	centerOfMass = Utilities::getCenterOfMass(part.objectImage);

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

cv::Mat TopDownDetector::loadTemplateImage(const VisionObject& part)
{
	auto path = "data/" + part.typeName + ".jpg";
	auto image = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
	if(image.data == nullptr)
	{
		Utilities::exitWithMessage("Unable to find template file for part type: " + part.typeName + " in data folder.");
	}
	return image;
}