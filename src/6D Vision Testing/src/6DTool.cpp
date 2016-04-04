#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ObjectDetector.h>
#include <opencv2/imgproc/imgproc.hpp>

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

void preprocessImages(char** argv)
{
	//auto sideMatrix = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
	auto topDownMatrix = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	auto connectedComponents = ObjectDetector::findConnectedComponents(topDownMatrix);
	auto objects = ObjectDetector::filterObjects(connectedComponents, topDownMatrix);
	auto objectImage = objects[0].objectImage;

	cv::Moments moments = cv::moments(objectImage, true);
	cv::Point objectCenter{ moments.m10 / moments.m00, moments.m01 / moments.m00 };

	findContours(objectImage.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	std::vector<std::vector<cv::Point>> hull(contours.size());
	for (size_t i = 0; i < contours.size(); i++)
	{
		convexHull(cv::Mat(contours[i]), hull[i], false);
	}

	//Draw stuff
	for (size_t i = 0; i< contours.size(); i++)
	{
		drawContours(objectImage, hull, i, cv::Scalar(128, 128, 128, 255));
	}
	cv::rectangle(objectImage, objectCenter - cv::Point(1, 1), objectCenter + cv::Point(1, 1), cv::Scalar(0, 0, 0, 255), 3);

	namedWindow("Output image", cv::WINDOW_AUTOSIZE);
	cv::imshow("Output image", objectImage);
	cv::waitKey(0);
}

int main(int argc, char** argv)
{
	//checkParameters(argc, argv);
	preprocessImages(argv);
    return EXIT_SUCCESS;
}
