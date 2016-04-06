#include "TopDownDetector.h"
#include "VisionObject.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ObjectDetector.h"
#include <iostream>

float TopDownDetector::getYaw(VisionObject& part){
	auto templateImage = cv::imread(std::string("images/stlImages/stlfloe3.jpg"), CV_LOAD_IMAGE_GRAYSCALE);

	auto connectedComponents = ObjectDetector::findConnectedComponents(templateImage);
	auto objects = ObjectDetector::filterObjects(connectedComponents, templateImage);

	cv::resize(objects[0].objectImage, objects[0].objectImage, cv::Size(part.objectImage.size().width, part.objectImage.size().height));
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
	cv::Point centrePoint = cv::Point(part.objectImage.cols / 2, part.objectImage.rows / 2);
	cv::Mat rotMat;
	cv::Mat tempImage;
	int bestMatch = 0;
	int highestMatch = 0;
	for (int i = 0; i < 360;++i) {
		rotMat = cv::getRotationMatrix2D(centrePoint, i, 1);
		cv::warpAffine(objects[0].objectImage, tempImage, rotMat, part.objectImage.size());
		for (cv::Point p : part.data) {
			if ((int)tempImage.at<uchar>(cv::Point(p.x - minx, p.y - miny)) > 0) {
				++posCount;
			}
		}		
		if (highestMatch < ((posCount * 100) / part.data.size())) {
			highestMatch = (posCount * 100) / part.data.size();
			bestMatch = i;
			std::cout << "Best match: " << i << " Match percentage: " << highestMatch << std::endl;
		}
		posCount = 0;
	}
	rotMat = cv::getRotationMatrix2D(centrePoint, bestMatch, 1);
	cv::warpAffine(objects[0].objectImage, tempImage, rotMat, part.objectImage.size());
	for (cv::Point p : part.data) {
		if ((int)tempImage.at<uchar>(cv::Point(p.x - minx, p.y - miny)) > 0) {
			cv::line(part.objectImage, cv::Point(p.x - minx, p.y - miny), cv::Point(p.x - minx, p.y - miny), cv::Scalar(125, 125, 125));
		}
	}
	cv::imshow("Plswork", part.objectImage);
	return bestMatch;
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