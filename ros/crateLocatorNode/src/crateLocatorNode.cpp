//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        VisionNode
// File:           CrateLocatorNode.cpp
// Description:    his vision RosNode detects the crates and publishes events on the crateEvent topic.
// Author:         Kasper van Nieuwland en Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of VisionNode.
//
// VisionNode is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// VisionNode is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with VisionNode.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <CrateLocatorNode/crateLocatorNode.h>
#include <DataTypes/Crate.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const char WINDOW_NAME[] = "Image window";

//on mouse click event, print the real life coordinate at the clicked pixel
void on_mouse(int event, int x, int y, int flags, void* param) {
	if (event == CV_EVENT_LBUTTONDOWN) {
		Vision::PixelToRealCoordinateTransformer* cordTransformer = (Vision::PixelToRealCoordinateTransformer*) param;
		DataTypes::Point2D result = cordTransformer->to_rc(DataTypes::Point2D(x, y));
		ROS_INFO("RX: %f, RY:%f", result.x, result.y);
		ROS_INFO("PX: %d, PY:%d", x, y);
		std::cout.flush();
	}
}

/**
 * Subscribes to the camera node, starts the QR detector and opens a window to show the output.
 **/
CrateLocatorNode::CrateLocatorNode( ) :
		measurementCount(0), measurements(0), failCount(0), imageTransport(node) {

	//setup the QR detector
	qrDetector = new Vision::QRCodeDetector();

	//setup the fiducial detector
	fidDetector = new Vision::FiducialDetector();
	// minimum for the radius of the circle = 10, so a diameter of 20 pixels
	fidDetector->minRad = 10;
	// maximum for the radius of the circle = 40, so a diameter of 80 pixels
	fidDetector->maxRad = 40;
	// minimum distance between two circles, 30 pixels
	fidDetector->distance = 30;
	fidDetector->minDist = 2.0f;
	fidDetector->maxDist = 5.0f;
	fidDetector->verbose = true;
	fidDetector->circleVotes = 60;

	//setup the coordinate transformation(from pixel to real life)
	//The real-life coordinates of the fiducials in mm.
	//0,0 is center of the delta robot
	std::vector<DataTypes::Point2D> rc;
	rc.push_back(DataTypes::Point2D(-75, 115));
	rc.push_back(DataTypes::Point2D(25, 115));
	rc.push_back(DataTypes::Point2D(25, 65));

	cordTransformer = new Vision::PixelToRealCoordinateTransformer(rc, rc);

	//GUI stuff
	cv::namedWindow(WINDOW_NAME);
	cvSetMouseCallback(WINDOW_NAME, &on_mouse, cordTransformer);
}

CrateLocatorNode::~CrateLocatorNode( ) {
	delete qrDetector;
	delete fidDetector;
	delete cordTransformer;
	cv::destroyWindow(WINDOW_NAME);
}

bool xComp(cv::Point2f a, cv::Point2f b) {
	return (a.x < b.x);
}

bool yComp(cv::Point2f a, cv::Point2f b) {
	return (a.y < b.y);
}

inline float averageX(std::vector<cv::Point2f> points) {
	int pointsCounter = 0;
	double total = 0;
	for (std::vector<cv::Point2f>::iterator it = points.begin(); it != points.end(); ++it, ++pointsCounter) {
		total += (*it).x;
	}
	return total / pointsCounter;
}

inline float averageY(std::vector<cv::Point2f> points) {
	int pointsCounter = 0;
	double total = 0;
	for (std::vector<cv::Point2f>::iterator it = points.begin(); it != points.end(); ++it, ++pointsCounter) {
		total += (*it).y;
	}
	return total / pointsCounter;
}

bool CrateLocatorNode::calibrate(unsigned int measurements, unsigned int maxErrors) {
	ROS_INFO("Updating calibration markers...");

	fid1_buffer.clear();
	fid2_buffer.clear();
	fid3_buffer.clear();

	measurementCount = 0;
	this->measurements = measurements;
	failCount = 0;

	// ROS says: Once a subscriber is removed (aka out of context) the subscription is removed.
	// This allows us to use a temporary callback handler.
	{
		std::cout << "[DEBUG] Starting calibration" << std::endl;
		image_transport::Subscriber subscriber = imageTransport.subscribe("camera/image", 1,
		        &CrateLocatorNode::calibrateCallback, this);
		while (ros::ok() && (measurementCount < measurements && failCount < maxErrors)) {
			ros::spinOnce();
		}
		std::cout << "[DEBUG] Done calibrating, removing subscription." << std::endl;
	}

	if (measurementCount >= measurements) {
		std::cout << "[DEBUG] Computing average X and Y coordinate for each marker." << std::endl;
		DataTypes::Point2D fid1(averageX(fid1_buffer), averageY(fid1_buffer));
		DataTypes::Point2D fid2(averageX(fid2_buffer), averageY(fid2_buffer));
		DataTypes::Point2D fid3(averageX(fid3_buffer), averageY(fid3_buffer));

		markers.push_back(DataTypes::Point2D(fid1.x, fid1.y));
		markers.push_back(DataTypes::Point2D(fid2.x, fid2.y));
		markers.push_back(DataTypes::Point2D(fid3.x, fid3.y));
		cordTransformer->set_fiducials_pixel_coordinates(markers);

		// TODO: Determine usefulness?
		// It was used in the ROS_INFO below..
		/*
		// Determine mean deviation
		double totalDistance = 0;
		for (std::vector<DataTypes::Point2D>::iterator it = fid1_buffer.begin(); it != fid1_buffer.end(); ++it)
			totalDistance += fid1.distance(*it);
		for (std::vector<DataTypes::Point2D>::iterator it = fid2_buffer.begin(); it != fid2_buffer.end(); ++it)
			totalDistance += fid2.distance(*it);
		for (std::vector<DataTypes::Point2D>::iterator it = fid3_buffer.begin(); it != fid3_buffer.end(); ++it)
			totalDistance += fid3.distance(*it);
		float meanDeviation = totalDistance / double(fid1_buffer.size() + fid2_buffer.size() + fid3_buffer.size());*/

		ROS_INFO( "Calibration markers updated.\nMeasured: %d Failed: %d", measurements, failCount);
		return true;
	}

	ROS_INFO(
	        "Calibration timed out, too many failed attempts. Measurements needed: %d Measured: %d", measurements, measurementCount);
	return false;
}

void CrateLocatorNode::calibrateCallback(const sensor_msgs::ImageConstPtr& msg) {
	// Receive image
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// First copy the image to a gray scale image.
	cv::Mat gray;
	cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

	// Locate all fiducial points
	std::vector<cv::Point2f> fiducialPoints;
	fidDetector->detect(gray, fiducialPoints, &cv_ptr->image);

	// If three fiducials (all on the workin area) have been found, sort them on distance.
	// The fiducials are put into their own buffer as they are sorted.
	if (fiducialPoints.size() == 3) {
		measurementCount++;
		Vision::FiducialDetector::order(fiducialPoints);
		fid1_buffer.push_back(fiducialPoints[0]);
		fid2_buffer.push_back(fiducialPoints[1]);
		fid3_buffer.push_back(fiducialPoints[2]);
	} else {
		failCount++;
//size() on a vector returns a size_type, which has a different size on a 64-bit platform
#if defined(__LP64__) || defined(_LP64)
		ROS_WARN("Incorrect number of markers. Needed: 3 Saw: %lu", fiducialPoints.size());
#else
		ROS_WARN("Incorrect number of markers. Needed: 3 Saw: %u", fiducialPoints.size());
#endif

	}

	// Show the debug image and progress status.
	cv::imshow(WINDOW_NAME, cv_ptr->image);
	cv::waitKey(3);
	std::cout << "Measures " << measurementCount << "/" << measurements << std::endl;
	std::cout.flush();
}

void CrateLocatorNode::crateLocateCallback(const sensor_msgs::ImageConstPtr& msg) {
	// Receive image
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// First copy the image to a gray scale image.
	cv::Mat gray;
	cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

	// Draw the calibration points for visual debugging.
	for (std::vector<DataTypes::Point2D>::iterator it = markers.begin(); it != markers.end(); ++it)
		cv::circle(cv_ptr->image, cv::Point(cv::saturate_cast<int>(it->x), cv::saturate_cast<int>(it->y)), 1,
		        cv::Scalar(0, 0, 255), 2);

	for (std::vector<DataTypes::Point2D>::iterator it = markers.begin(); it != markers.end(); ++it) {
		cv::circle(cv_ptr->image,
		        cv::Point(
		                cordTransformer->to_pc(
		                        cordTransformer->to_rc(
		                                DataTypes::Point2D(cv::saturate_cast<int>(it->x),
		                                        cv::saturate_cast<int>(it->y)))).x,
		                cordTransformer->to_pc(
		                        cordTransformer->to_rc(
		                                DataTypes::Point2D(cv::saturate_cast<int>(it->x),
		                                        cv::saturate_cast<int>(it->y)))).y), 7, cv::Scalar(255, 0, 255), 1);
	}

	// Detect all QR crates in the image.
	std::vector<DataTypes::Crate> crates;
	qrDetector->detectQRCodes(gray, crates);

	// Transform crate coordinates
	for (std::vector<DataTypes::Crate>::iterator it = crates.begin(); it != crates.end(); ++it) {
		it->draw(cv_ptr->image);
	}

	cv::imshow(WINDOW_NAME, cv_ptr->image);
	cv::waitKey(3);
}

void CrateLocatorNode::run( ) {
	//run initial calibration. If that fails, this node will shut down.
	if (!calibrate()) {
		ros::shutdown();
	} else {

		std::cout << "[DEBUG] Waiting for subscription" << std::endl;
		// subscribe example: (poorly documented on ros wiki)
		// Images are transported in JPEG format to decrease tranfer time per image.
		// imageTransport.subscribe(<base image topic>, <queue_size>, <callback>, <tracked object>, <TransportHints(<transport type>)>)
		sub = imageTransport.subscribe("camera/image", 1, &CrateLocatorNode::crateLocateCallback, this, image_transport::TransportHints("compressed"));
		std::cout << "[DEBUG] Starting crateLocateCallback loop" << std::endl;

		while (ros::ok()) {
			ros::spinOnce();
		}
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "vision");
	CrateLocatorNode crateLocatorNode;
	crateLocatorNode.run();

	return 0;
}
