#include "TopDownDetector.h"
#include "VisionObject.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgcodecs.hpp>
#include "ObjectDetector.h"
#include "Utilities.h"
#include <iostream>

std::vector<int> TopDownDetector::getYaw(VisionObject& part)
{
	auto templateImage = loadTemplateImage(part);
	part.objectImage = ObjectDetector::applyOtsuThreshold(part.objectImage);
	cv::imshow("Test", part.objectImage);
	
	cv::Point2i partOffsets = Utilities::getPartOffsets(part);

	cv::Mat tempImage;
	int bestMatch = 0;
	std::vector<std::pair<int, int>> matchList(180);
	int highestMatch = 0;
	for (int i = 0; i < 360;i+=2) {
		Utilities::rotateImage(templateImage, tempImage, i);
		auto connectedComponents = ObjectDetector::findConnectedComponents(tempImage);
		auto objects = ObjectDetector::filterObjects(connectedComponents, tempImage);
		cv::resize(objects[0].objectImage, objects[0].objectImage, cv::Size(part.objectImage.size().width, part.objectImage.size().height));

		int posCount = 0;
		for (cv::Point p : part.data) {
			if ((int)objects[0].objectImage.at<uchar>(cv::Point(p.x - partOffsets.x, p.y - partOffsets.y)) > 0) {
				++posCount;
			}
		}
		int matchPercentage = (posCount * 100) / part.data.size();
		Utilities::insertMatch(matchList, std::make_pair(matchPercentage, i));
	}
	return Utilities::filterYaw(matchList);
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