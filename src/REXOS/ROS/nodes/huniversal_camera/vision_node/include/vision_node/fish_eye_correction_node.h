#include <camera/RectifyImage.h>

#include <opencv/cvwimage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


	bool setCorrectionMatrices(vision_node::setCorrectionMatrices::Request& request,
			vision_node::setCorrectionMatrices::Response& response);
	bool getCorrectionMatrices(vision_node::getCorrectionMatrices::Request& request,
			vision_node::getCorrectionMatrices::Response& response);


	Camera::RectifyImage * rectifier;
	cv::Mat rectifiedCamFrame;


	ros::ServiceServer fishEyeCorrectionService = nodeHandle.advertiseService(vision_node_services::FISH_EYE_CORRECTION,
	        &VisionNode::fishEyeCorrection, this);
