#include "TopDownDetector.h"
#include "SideDetector.h"
#include "Utilities.h"
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <ObjectDetector.h>

cv::Mat topDownImage;
cv::Mat sideImage;
VisionObject part;

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

	part.typeName = argv[3];
	savePath = argv[4];
}

void detectPart()
{
	auto connectedComponents = ObjectDetector::findConnectedComponents(topDownImage);
	auto objects = ObjectDetector::filterObjects(connectedComponents, topDownImage);
	if(objects.size() == 0)
	{
		Utilities::exitWithMessage("Unable to locate part in top-down view image.");
	}
	part.objectImage = objects[0].objectImage;
	part.data = objects[0].data;
}

void calculateTopDownValues()
{
	auto xy = TopDownDetector::getXY(part);
	auto yaw = TopDownDetector::getYaw(part);
	position.x = xy.x;
	position.y = xy.y;
	rotation.x = yaw;
}

void calculateSideValues()
{
	auto z = SideDetector::getZ(part);
	auto rollPitch = SideDetector::getRollPitch(part);
	position.z = z;
	rotation.y = rollPitch.x;
	rotation.z = rollPitch.y;
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
	outputFile << "Yaw: " << rotation.x << std::endl;
	outputFile << "Roll: " << rotation.y << std::endl;
	outputFile << "Pitch: " << rotation.z << std::endl;
	outputFile.close();
}

int main(int argc, char** argv)
{
	initialise(argc, argv);
	detectPart();
	calculateTopDownValues();	
	calculateSideValues();
	saveValues();

    return EXIT_SUCCESS;
}