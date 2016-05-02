#include "SideDetector.h"
#include "VisionObject.h"
#include "Utilities.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <sstream>
#include "ObjectDetector.h"

cv::Point3i SideDetector::getRollPitchYaw(VisionObject part, std::vector<int> yaws)
{
	auto values = getPossibleRollPitchValues(part);
	cv::Point2i partOffsets = Utilities::getPartOffsets(part);
	//creating list of decent size
	std::vector<std::pair<int, cv::Point3i>> matchList(values.size() * yaws.size());
	for (auto yaw : yaws) {
		for (auto v : values) {
			cv::Mat currentImage = cv::imread("dataset/" + part.typeName + "/" + part.typeName +
				"r" + std::to_string(v.x) +
				"p" + std::to_string(v.y) +
				"y" + std::to_string(Utilities::addDegrees(yaw, 0)) + ".jpg",
				CV_LOAD_IMAGE_GRAYSCALE);
		
			auto connectedComponents = ObjectDetector::findConnectedComponents(currentImage);
			auto templateObjects = ObjectDetector::filterObjects(connectedComponents, currentImage);

			cv::resize(templateObjects[0].objectImage, templateObjects[0].objectImage,
				cv::Size(part.objectImage.size().width,
					part.objectImage.size().height));
			int posCount = 0;
			for (cv::Point p : part.data) {
				if ((int)templateObjects[0].objectImage.at<uchar>(cv::Point(p.x - partOffsets.x, p.y - partOffsets.y)) > 0) {
					++posCount;
				}
			}
			int matchPercentage = (posCount * 100) / part.data.size();
			Utilities::insertMatch(matchList, std::make_pair(matchPercentage, cv::Point3i(v.x, v.y, yaw)));
		}
	}
	return matchList[0].second;
}

int SideDetector::getZ(const VisionObject & part)
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
	return absoluteCenter.y;
}

cv::Point SideDetector::getCenterOfMass(const cv::Mat & partImage)
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

std::vector<cv::Point> SideDetector::getPossibleRollPitchValues(const VisionObject& part)
{
	std::vector<cv::Point> values;
	std::string line;
	std::ifstream dataFile{ "dataset/" + part.typeName + "/rollPitchData.txt" };
	if(!dataFile.is_open())
	{
		Utilities::exitWithMessage("Unable to open rollPitchData.txt in data folder.");
	}
	
	//Loop until we find the line corresponding to the part
	while(std::getline(dataFile, line))
	{
		//Build a stream for the line and read every possible
		//roll pitch combination.
		std::stringstream stream{ line };
		cv::Point nextPoint;
		std::string discardS;
		char discardC;
		//Discard the partname
		stream >> discardS;
		//Loop until the stream is empty to read all roll-pitch pairs
		while(stream)
		{
			stream >> nextPoint;
			//Put roll-pitch pair in list
			values.push_back(nextPoint);
		}
	}
	dataFile.close();

	if(values.size() == 0)
	{
		Utilities::exitWithMessage("No roll and pitch values found for type: " + part.typeName + '.');
	}
	return values;
}