#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ObjectDetector.h>

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

	/*cv::Mat cannyImage;
	cv::Canny(topDownMatrix, cannyImage, 100, 255);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Mat contourInput = cannyImage.clone();
	findContours(contourInput, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	std::vector<cv::Rect> boundRect(contours.size());
	std::vector<std::vector<cv::Point>> contours_poly(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
	}

	cv::Mat contourImage = cv::Mat::zeros(contourInput.size(), CV_8UC3);
	for(int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(255, 255, 255);
		//drawContours(contourImage, contours, i, color);
		cv::rectangle(contourImage, boundRect[i].tl(), boundRect[i].br(), color);
	}

	for(auto& point : hierarchy)
	{
		std::cout << point << std::endl;;
	}*/

	auto topDownMatrix = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	auto components = ObjectDetector::findConnectedComponents(topDownMatrix);
	auto objects = ObjectDetector::filterObjects(components, topDownMatrix);
	for(auto& objectImage : objects)
	{
		namedWindow("Without thresh", cv::WINDOW_AUTOSIZE);
		namedWindow("With thresh", cv::WINDOW_AUTOSIZE);
		cv::imshow("Without thresh", objectImage.objectImage);
		cv::imshow("With thresh", ObjectDetector::applyOtsuThreshold(objectImage.objectImage));
	}
	cv::waitKey(0);
}

int main(int argc, char** argv)
{
	//checkParameters(argc, argv);
	preprocessImages(argv);
    return EXIT_SUCCESS;
}
