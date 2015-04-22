#include <camera/SimulatedCamera.h>
#include <rexos_utilities/Utilities.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdexcept>

using namespace camera;

SimulatedCamera::SimulatedCamera(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, CameraListener* listener, double fps, 
		ros::NodeHandle& nodeHandle) : 
		Camera(equipletName, identifier, listener, fps), nodeHandle(nodeHandle), frameSize(0, 0)
{
	cameraFeedSubscriber = nodeHandle.subscribe(equipletName + "/" + 
			identifier.getManufacturer() + "/" + identifier.getTypeNumber() + "/" + identifier.getSerialNumber() + "/camera/image_raw",
			1, &SimulatedCamera::handleFrame, this);
}

SimulatedCamera::~SimulatedCamera() {
}

cv::Size SimulatedCamera::getFrameSize() {
	if(frameSize != cv::Size(0, 0)){
		return frameSize;
	} else {
		boost::shared_ptr<sensor_msgs::CameraInfo const> cameraInfoPtr;
		
		std::string path = equipletName + "/" + identifier.getManufacturer() + "/" + identifier.getTypeNumber() + "/" + 
				identifier.getSerialNumber() + "/camera/camera_info/";
		REXOS_INFO_STREAM("Waiting for camera info at " << path);
		cameraInfoPtr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(path, nodeHandle, ros::Duration(30.0));
		if(cameraInfoPtr == NULL) {
			REXOS_ERROR("Unable to determine frame size after waiting 30 seconds");
			return cv::Size();
		} else {
			return cv::Size(cameraInfoPtr->width, cameraInfoPtr->height);
		}
	}
}

int SimulatedCamera::getFrameFormat() {
	return CV_8UC3;
}

void SimulatedCamera::enableCamera(bool enabled) {
	Camera::enableCamera(enabled);
}

void SimulatedCamera::handleFrame(const sensor_msgs::ImageConstPtr& msg) {
	if(isNewFrameRequired() == true) {
		// someone is waiting for this frame, lets store the frame and notify him
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		camFrame = cv_ptr->image;
		frameSize = camFrame.size();
		
		onNewFrame();
	} else {
		// nobody wants this frame, so just disregard it
	}
}
