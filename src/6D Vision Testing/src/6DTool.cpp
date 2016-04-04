#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ObjectDetector.h>
#include "TopDownDetector.h"

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

void topDownCalculations(cv::Mat& topDownImage)
{
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	auto connectedComponents = ObjectDetector::findConnectedComponents(topDownImage);
	auto objects = ObjectDetector::filterObjects(connectedComponents, topDownImage);

	cv::imshow("Test", objects[0].objectImage);

	auto center = TopDownDetector::getXY(objects[0]);
}

int main(int argc, char** argv)
{
	//checkParameters(argc, argv);

	//auto sideMatrix = cv::imread(std::string("images/testImages/") + argv[2], CV_LOAD_IMAGE_GRAYSCALE);
	auto topDownImage = cv::imread(std::string("images/testImages/") + argv[1], CV_LOAD_IMAGE_GRAYSCALE);

	topDownCalculations(topDownImage);	

	namedWindow("Output image", cv::WINDOW_AUTOSIZE);
	cv::imshow("Output image", topDownImage);
	cv::waitKey(0);
    return EXIT_SUCCESS;
}