//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        VisionNode
// File:           visionNode.cpp
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

#include <cv_bridge/cv_bridge.h>
    #include <sensor_msgs/image_encodings.h>
    #include <opencv2/imgproc/imgproc.hpp>
    #include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

static const char WINDOW[] = "Image  window";

//on mouse click event, print the real life coordinate at the clicked pixel
void on_mouse(int event, int x, int y, int flags, void* param){
	if(event == CV_EVENT_LBUTTONDOWN){
//		pc_rc_transformer* cordTransformer = (pc_rc_transformer*)param;
//		point2f result = cordTransformer->to_rc(point2f(x, y));
//		ROS_INFO("RX: %f, RY:%f", result.x, result.y);
		ROS_INFO("PX: %d, PY:%d", x,y);
	}
}


visionNode::visionNode() : imageTransport(node){
	std::cout << "[DEBUG] Waiting for subscribe" << std::endl;
		sub = imageTransport.subscribe("camera/image", 1, &visionNode::imageCallback, this);
		std::cout << "[DEBUG] Starting callback loop" << std::endl;

		//GUI stuff
		cv::namedWindow(WINDOW);
		cvSetMouseCallback(WINDOW, &on_mouse);
}
visionNode::~visionNode(){
	cv::destroyWindow(WINDOW);
}



void visionNode::imageCallback(const sensor_msgs::ImageConstPtr& msg){


	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}


	// Find qr codes

	cv::Mat gray;
	cv::cvtColor( cv_ptr->image, gray, CV_BGR2GRAY);

	std::vector<Crate> crates;
	qrDetector->detectCrates(gray, crates);

	//transform crate coordinates
	for(std::vector<Crate>::iterator it=crates.begin(); it!=crates.end(); ++it){
		it->draw( cv_ptr->image);
	}

	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){
		cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
	}

	cv::imshow(WINDOW, cv_ptr->image);
	cv::waitKey(3);
}

bool xComp(cv::Point2f i, cv::Point2f j) { return (i.x<j.x); }
bool yComp(cv::Point2f i, cv::Point2f j) { return (i.y<j.y); }

inline float medianX(std::vector<cv::Point2f> points){
       	std::vector<cv::Point2f>::iterator n = points.begin()+points.size()/2;
	if(points.size()%2 == 0) {
		nth_element(points.begin(), n, points.end(), xComp);
		return ((n->x + (n+1)->x)/2.0);
	} else {
	        nth_element(points.begin(), n, points.end(), xComp);
        	return n->x;
	}
}
inline float medianY(std::vector<cv::Point2f> points){
        std::vector<cv::Point2f>::iterator n = points.begin()+points.size()/2;
        if(points.size()%2 == 0) {
                nth_element(points.begin(), n, points.end(), yComp);
                return ((n->y + (n+1)->y)/2.0);
        } else {
                nth_element(points.begin(), n, points.end(), yComp);
                return n->y;
        }
}
/*
bool visionNode::calibrate(unsigned int measurements, unsigned int maxErrors){
	ROS_INFO("Updating calibration markers...");

	std::vector<cv::Point2f> fid1_buffer;
    	std::vector<cv::Point2f> fid2_buffer;
    	std::vector<cv::Point2f> fid3_buffer;

	unsigned int measurementCount = 0;
	unsigned int failCount = 0;
	while(measurementCount<measurements && failCount<maxErrors){
		cam->get_frame(&camFrame);
		rectifier->rectify(camFrame, rectifiedCamFrame);
		cv::Mat gray;
		cv::cvtColor(rectifiedCamFrame, gray, CV_BGR2GRAY);

		std::vector<cv::Point2f> fiducialPoints;
		fidDetector->detect(gray, fiducialPoints);
		if(fiducialPoints.size() == 3) {
			measurementCount++;
			Crate::order(fiducialPoints);
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
	}

	if(measurementCount == measurements) {
		cv::Point2f fid1(medianX(fid1_buffer), medianY(fid1_buffer));
		cv::Point2f fid2(medianX(fid2_buffer), medianY(fid2_buffer));
		cv::Point2f fid3(medianX(fid3_buffer), medianY(fid3_buffer));

		markers.push_back(point2f(fid1.x, fid1.y));
		markers.push_back(point2f(fid2.x, fid2.y));
		markers.push_back(point2f(fid3.x, fid3.y));
		cordTransformer->set_fiducials_pixel_coordinates(markers);

		// Determine mean deviation
		double totalDistance = 0;
		for(std::vector<cv::Point2f>::iterator it=fid1_buffer.begin(); it!=fid1_buffer.end(); ++it) totalDistance += Crate::distance(fid1, *it);
		for(std::vector<cv::Point2f>::iterator it=fid2_buffer.begin(); it!=fid2_buffer.end(); ++it) totalDistance += Crate::distance(fid2, *it);
		for(std::vector<cv::Point2f>::iterator it=fid3_buffer.begin(); it!=fid3_buffer.end(); ++it) totalDistance += Crate::distance(fid3, *it);
		float meanDeviation = totalDistance / double(fid1_buffer.size()+fid2_buffer.size()+fid3_buffer.size());

		ROS_INFO("Calibration markers updated.\nMeasured: %d Failed: %d Mean deviation: %f", measurements, failCount, meanDeviation);
		return true;
	}
	ROS_INFO("Calibration timed out, too many failed attempts. Measurements needed: %d Measured: %d", measurements, measurementCount);
	return false;
}
*/
void visionNode::run(){
	//run initial calibration. If that fails, this node will shut down.
	//if(!calibrate()) ros::shutdown();
	
			
	//vision::registerNode rn;
	//rn.action = "VISION";
	//rn.form = "??";
	//nodeRegistrationPublisher.publish(rn);	
	ros::spinOnce();
	
	
	
	//VideoWriter outputVideo;
	//Size S = cv::Size(cam->get_img_width(),cam->get_img_height());
	//outputVideo.open("/home/lcv/output.avi" , CV_FOURCC('M','P','2','V'), 30, S, true);

	//main loop
	while(ros::ok()){

		//if calibration was manualy invoked by call on the service
		/*if(invokeCalibration) {
			invokeCalibration = false;
			calibrate();
		}*/

		//grab frame from camera
		//cam->get_frame(&camFrame);

		//correct the lens distortion
		//rectifier->rectify(camFrame, rectifiedCamFrame);

		//create a duplicate grayscale frame
			/*
		cv::Mat gray;
		cv::cvtColor(rectifiedCamFrame, gray, CV_BGR2GRAY);

		//draw the calibration points
		for(point2f::point2fvector::iterator it=markers.begin(); it!=markers.end(); ++it)
			cv::circle(rectifiedCamFrame, cv::Point(cv::saturate_cast<int>(it->x), cv::saturate_cast<int>(it->y)), 1, cv::Scalar(0, 0, 255), 2);
			
		for(point2f::point2fvector::iterator it=markers.begin(); it!=markers.end(); ++it){
			cv::circle(rectifiedCamFrame, 
			cv::Point(
				cordTransformer->to_pc(cordTransformer->to_rc(point2f(cv::saturate_cast<int>(it->x), cv::saturate_cast<int>(it->y)))).x, 
				cordTransformer->to_pc(cordTransformer->to_rc(point2f(cv::saturate_cast<int>(it->x), cv::saturate_cast<int>(it->y)))).y
			), 7, cv::Scalar(255, 0, 255), 1);
		}
		
		/*
		point2f testgrid_pix_1, testgrid_real_1;
		
		testgrid_real_1.x = -100;
	   testgrid_real_1.y = -120;
		
		for (int y = 0; y < 200; y+=10){
			for (int x = 0; x < 200; x+=10){
				
				testgrid_pix_1 = cordTransformer->to_pc(testgrid_real_1);
				if(testgrid_pix_1.x > 0 && testgrid_pix_1.x < cam->get_img_width() && testgrid_pix_1.y > 0 && testgrid_pix_1.y < cam->get_img_height()){
					cv::circle(rectifiedCamFrame, cv::Point(testgrid_pix_1.x,testgrid_pix_1.y), 4, cv::Scalar(0, 0, 255), 1);
				}
				testgrid_real_1.x = -100 + x;
			}
			testgrid_real_1.y = -100 + y;
		}

*/
		//detect crates
		/*
		std::vector<Crate> crates;
		qrDetector->detectCrates(gray, crates);

		//transform crate coordinates
		for(std::vector<Crate>::iterator it=crates.begin(); it!=crates.end(); ++it)
		{
			it->draw(rectifiedCamFrame);

			std::vector<cv::Point2f> points;
			for(int n = 0; n <3; n++){
				point2f result = cordTransformer->to_rc(point2f(it->getPoints()[n].x, it->getPoints()[n].y));
				points.push_back(cv::Point2f(result.x, result.y));
			}
			it->setPoints(points);
		}

		//inform the crate tracker about the seen crates
		std::vector<CrateEvent> events = crateTracker->update(crates);

		//publish events
		for(std::vector<CrateEvent>::iterator it = events.begin(); it != events.end(); ++it)
		{
			vision::CrateEventMsg msg;
			msg.event = it->type;
			msg.crate.name = it->name;
			msg.crate.x = it->x;
			msg.crate.y = it->y;
			msg.crate.angle = it->angle;

			ROS_INFO("%s", it->toString().c_str());
			crateEventPublisher.publish(msg);
		}

		//update GUI
		//outputVideo.write(rectifiedCamFrame);
		imshow("image",rectifiedCamFrame);
		waitKey(1000/30);
*/
		//let ROS do it's magical things
		ros::spinOnce();
	}
}


int main(int argc, char* argv[]) {
	ros::init(argc, argv, "vision");

	visionNode vn;
	vn.run();

	return 0;
}