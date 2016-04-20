#include "TopDownDetector.h"
#include "SideDetector.h"
#include "Utilities.h"
#include "basetimer.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ObjectDetector.h>

cv::Mat topDownImage;
cv::Mat sideImage;
VisionObject topDownPart;
VisionObject sidePart;

std::vector<int> possibleYaws;

cv::Point3i position;
cv::Point3f rotation;

std::string savePath;

void initialise(int argc, char** argv)
{
	//The program expects two parameters from the user. Since C++ adds it's own first
	//runtime parameter to argc the total equals 3.
	if (argc != 5)
	{
		Utilities::exitWithMessage("Incorrect number of parameters entered. Expected 4 got " + std::to_string(argc-1) + ".");
	}

	topDownImage = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	if(topDownImage.data == nullptr)
	{
		Utilities::exitWithMessage("Path to top-down view image is invalid.");
	}

	sideImage = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
	if(sideImage.data == nullptr)
	{
		Utilities::exitWithMessage("Path to side view image is invalid.");
	}

	topDownPart.typeName = argv[3];
	sidePart.typeName = argv[3];
	savePath = argv[4];
}

void detectParts()
{
	auto connectedComponents = ObjectDetector::findConnectedComponents(topDownImage);
	auto objects = ObjectDetector::filterObjects(connectedComponents, topDownImage);
	if(objects.size() == 0)
	{
		Utilities::exitWithMessage("Unable to locate part in top-down view image.");
	}
	topDownPart.objectImage = objects[0].objectImage;
	topDownPart.data = objects[0].data;

	connectedComponents = ObjectDetector::findConnectedComponents(sideImage);
	objects = ObjectDetector::filterObjects(connectedComponents, sideImage);
	if(objects.size() == 0)
	{
		Utilities::exitWithMessage("Unable to locate part in side view image");
	}
	sidePart.objectImage = objects[0].objectImage;
	sidePart.data = objects[0].data;
}

void calculateTopDownValues()
{
	auto xy = TopDownDetector::getXY(topDownPart);
	auto yaw = TopDownDetector::getYaw(topDownPart);
	position.x = xy.x;
	position.y = xy.y;
	possibleYaws = yaw;
}

void calculateSideValues()
{
	auto z = SideDetector::getZ(sidePart);
	auto rollPitchYaw = SideDetector::getRollPitchYaw(sidePart, possibleYaws);
	position.z = z;
	rotation.x = rollPitchYaw.x;
	rotation.y = rollPitchYaw.y;
	rotation.z = rollPitchYaw.z;
}

void saveValues()
{
	std::ofstream outputFile{ savePath + "/output.txt" };
	if(!outputFile.is_open())
	{
		Utilities::exitWithMessage("Unable to create output file.");
	}
	outputFile << "X: " << position.x << std::endl;
	outputFile << "Y: " << position.y << std::endl;
	outputFile << "Z: " << position.z << std::endl;
	outputFile << "Roll: " << rotation.x << std::endl;
	outputFile << "Pitch: " << rotation.y << std::endl;
	outputFile << "Yaw: " << rotation.z << std::endl;
	outputFile.close();

	cv::Mat templateImage = cv::imread("dataset/Test0"
		"r" + std::to_string((int)rotation.x) +
		"p" + std::to_string((int)rotation.y) +
		"y" + std::to_string(Utilities::addDegrees(rotation.z, 180)) + ".jpg",
		CV_LOAD_IMAGE_GRAYSCALE);
	std::cout << "dataset/Test0"
		"r" + std::to_string((int)rotation.x) +
		"p" + std::to_string((int)rotation.y) +
		"y" + std::to_string(Utilities::addDegrees(rotation.z, 180)) + ".jpg" << std::endl;
	cv::imshow("Side image", sideImage);
	cv::imshow("Best matching template image", templateImage);
}

int main(int argc, char** argv)
{
	BaseTimer timer;
	timer.start();
	initialise(argc, argv);
	detectParts();
	calculateTopDownValues();	
	calculateSideValues();
	saveValues();
	timer.stop();
	std::cout << "Elapsed time in milliseconds: " << timer.elapsedMilliSeconds() << std::endl;
	cv::waitKey(0);
    return EXIT_SUCCESS;
}