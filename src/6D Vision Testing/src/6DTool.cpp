#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ObjectDetector.h>
#include "TopDownDetector.h"
#include <SideDetector.h>

cv::Mat topDownImage;
cv::Mat sideImage;
VisionObject part;

void checkParameters(int argc, char** argv)
{
	//The program expects two parameters from the user. Since C++ adds it's own first
	//runtime parameter to argc the total equals 3.
	if (argc != 3)
	{
		std::cerr << "Incorrect number of parameters entered, expected 2. Terminating program.";
		exit(EXIT_FAILURE);
	}
}

void readImages(char** argv)
{
	topDownImage = cv::imread(std::string("images/testImages/") + argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	//sideMatrix = cv::imread(std::string("images/testImages/") + argv[2], CV_LOAD_IMAGE_GRAYSCALE);
	part.typeName = "Floe3";
}

void detectPart()
{
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	auto connectedComponents = ObjectDetector::findConnectedComponents(topDownImage);
	auto objects = ObjectDetector::filterObjects(connectedComponents, topDownImage);
	part.objectImage = objects[0].objectImage;
	part.data = objects[0].data;
}

void topDownCalculations()
{
	auto center = TopDownDetector::getXY(part);
	//auto yaw = TopDownDetector::getYaw(part);
	std::cout << "XY: " << center << std::endl;
	//std::cout << "Yaw: " << yaw << std::endl;
}

void sideCalculations()
{
	auto Height = SideDetector::getZ(part);
	std::cout << "Z: " << Height << std::endl;
}

int main(int argc, char** argv)
{
	//checkParameters(argc, argv);
	readImages(argv);
	detectPart();
	topDownCalculations();	
	sideCalculations();

	cv::imshow("Part", part.objectImage);
	cv::imshow("Original", topDownImage);
	cv::waitKey(0);
    return EXIT_SUCCESS;
}