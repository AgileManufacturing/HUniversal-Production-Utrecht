/**
 * @file CrateLocatorNode.cpp
 * @brief Detects the crates and publishes events on the crateEvent topic.
 * @date Created: 2011-11-11
 * @date Revisioned: 2012-10-22
 *
 * @author Kasper van Nieuwland
 * @author Zep Mouris
 * @author Koen Braham
 * @author Daan Veltman
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <CrateLocatorNode/CrateLocatorNode.h>
#include <DataTypes/Crate.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <CrateLocatorNode/Topics.h>
#include <CrateLocatorNode/Services.h>

static const char WINDOW_NAME[] = "Image window";

//on mouse click event, print the real life coordinate at the clicked pixel
void on_mouse(int event, int x, int y, int flags, void* param) {
	if (event == CV_EVENT_LBUTTONDOWN) {
		Vision::PixelAndRealCoordinateTransformer* cordTransformer = (Vision::PixelAndRealCoordinateTransformer*) param;
		DataTypes::Point2D result = cordTransformer->pixelToRealCoordinate(DataTypes::Point2D(x, y));
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

	cordTransformer = new Vision::PixelAndRealCoordinateTransformer(rc, rc);

	//crate tracking configuration
	//the amount of mm a point has to move before we mark it as moving.
	// When not moving we found a deviation of ~0.5 pixel.
	double crateMovementThreshold = 0.75;
	//the number of frames before a change is marked definite.
	int numberOfStableFrames = 10;
	crateTracker = new Vision::CrateTracker(numberOfStableFrames, crateMovementThreshold);

	//ROS things
	crateEventPublisher = node.advertise<crateLocatorNode::CrateEventMsg>(CrateLocatorNodeTopics::CRATE_EVENT, 100);
	getCrateService = node.advertiseService(CrateLocatorNodeServices::GET_CRATE, &CrateLocatorNode::getCrate, this);
	getAllCratesService = node.advertiseService(CrateLocatorNodeServices::GET_ALL_CRATES, &CrateLocatorNode::getAllCrates, this);

	//GUI stuff
	cv::namedWindow(WINDOW_NAME);
	cvSetMouseCallback(WINDOW_NAME, &on_mouse, cordTransformer);
}

CrateLocatorNode::~CrateLocatorNode( ) {
	delete qrDetector;
	delete fidDetector;
	delete cordTransformer;
	delete crateTracker;
	cv::destroyWindow(WINDOW_NAME);
}

bool CrateLocatorNode::getCrate(crateLocatorNode::getCrate::Request &req, crateLocatorNode::getCrate::Response &res) {
	DataTypes::Crate crate;
	bool succeeded = crateTracker->getCrate(req.name, crate);
	if (succeeded) {
		res.state = crate.getState();
		crateLocatorNode::CrateMsg msg;
		msg.name = crate.name;
		msg.x = crate.rect().center.x;
		msg.y = crate.rect().center.y;
		msg.angle = crate.rect().angle;
		res.crate = msg;
	} else {
		res.state = DataTypes::Crate::state_non_existing;
		crateLocatorNode::CrateMsg msg;
		msg.name = "";
		msg.x = 0;
		msg.y = 0;
		msg.angle = 0;
		res.crate = msg;
	}
	return true;
}

bool CrateLocatorNode::getAllCrates(crateLocatorNode::getAllCrates::Request &req,
        crateLocatorNode::getAllCrates::Response &res) {
	std::vector<DataTypes::Crate> allCrates = crateTracker->getAllCrates();
	for (std::vector<DataTypes::Crate>::iterator it = allCrates.begin(); it != allCrates.end(); ++it) {
		res.states.push_back(it->getState());
		crateLocatorNode::CrateMsg msg;
		msg.name = it->name;
		msg.x = it->rect().center.x;
		msg.y = it->rect().center.y;
		msg.angle = it->rect().angle;
		res.crates.push_back(msg);
	}
	return true;
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
		        &CrateLocatorNode::calibrateCallback, this, image_transport::TransportHints("compressed"));
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

		double maxDeviation = 0.0;
		double minDeviation = 0.0;
		for (std::vector<cv::Point2f>::iterator it = fid1_buffer.begin(); it != fid1_buffer.end(); ++it) {
			std::cout << (*it).x << std::endl;
			if(fid1.x - (*it).x > maxDeviation){
				maxDeviation = fid1.x - (*it).x;
			}
			if(fid1.x - (*it).x < minDeviation){
				minDeviation = fid1.x - (*it).x;
			}
		}

		std::cout << "deviation " << fabs(maxDeviation - minDeviation) << std::endl;

		markers.push_back(DataTypes::Point2D(fid1.x, fid1.y));
		markers.push_back(DataTypes::Point2D(fid2.x, fid2.y));
		markers.push_back(DataTypes::Point2D(fid3.x, fid3.y));
		cordTransformer->setFiducialPixelCoordinates(markers);

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
		                cordTransformer->realToPixelCoordinate(
		                        cordTransformer->pixelToRealCoordinate(
		                                DataTypes::Point2D(cv::saturate_cast<int>(it->x),
		                                        cv::saturate_cast<int>(it->y)))).x,
		                cordTransformer->realToPixelCoordinate(
		                        cordTransformer->pixelToRealCoordinate(
		                                DataTypes::Point2D(cv::saturate_cast<int>(it->x),
		                                        cv::saturate_cast<int>(it->y)))).y), 7, cv::Scalar(255, 0, 255), 1);
	}

	// Detect all QR crates in the image.
	std::vector<DataTypes::Crate> crates;
	qrDetector->detectQRCodes(gray, crates);

	// Transform crate coordinates
	for (std::vector<DataTypes::Crate>::iterator it = crates.begin(); it != crates.end(); ++it) {
		it->draw(cv_ptr->image);

		std::vector<cv::Point2f> points = it->getPoints();
		for (int n = 0; n < 3; n++) {
			DataTypes::Point2D coordinate(points[n].x, points[n].y);
			coordinate = cordTransformer->pixelToRealCoordinate(coordinate);
			points[n].x = coordinate.x;
			points[n].y = coordinate.y;
			//std::cout << "[DEBUG] " << it->name << " " << points[n].x << ", " << points[n].y << std::endl;
		}
		it->setPoints(points);
	}

	//inform the crate tracker about the seen crates
	std::vector<Vision::CrateEvent> events = crateTracker->update(crates);

	//publish events
	for (std::vector<Vision::CrateEvent>::iterator it = events.begin(); it != events.end(); ++it) {
		crateLocatorNode::CrateEventMsg msg;
		msg.event = it->type;
		msg.crate.name = it->name;
		msg.crate.x = it->x;
		msg.crate.y = it->y;
		msg.crate.angle = it->angle;

		ROS_INFO("%s", it->toString().c_str());
		crateEventPublisher.publish(msg);
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
		sub = imageTransport.subscribe("camera/image", 1, &CrateLocatorNode::crateLocateCallback, this,
		        image_transport::TransportHints("compressed"));
		std::cout << "[DEBUG] Starting crateLocateCallback loop" << std::endl;

		while (ros::ok()) {
			ros::spinOnce();
		}
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "crateLocator");
	CrateLocatorNode crateLocatorNode;
	crateLocatorNode.run();

	return 0;
}
