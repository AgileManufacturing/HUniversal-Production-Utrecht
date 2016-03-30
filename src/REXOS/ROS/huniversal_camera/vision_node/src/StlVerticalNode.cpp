#include "vision_node/StlVerticalNode.h"
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "vision_node/ObjectDetector.h"
#include "vision_node/PartMatcher.h"

using namespace cv;
using namespace std;

StlVerticalNode::StlVerticalNode(image_transport::ImageTransport& imageTransport){
    //debugImagePublisher = imageTransport.advertise("camera/stl_debug_image", 1);
}

void StlVerticalNode::handleFrame(cv::Mat& frame){
    //CvImage is used to convert cv images to publishable images
    cv_bridge::CvImage cvi;
    Mat grayFrame;
    cvtColor(frame,grayFrame,CV_RGB2GRAY);
    vector<vector<Point>> blobs = ObjectDetector::findConnectedComponents(grayFrame);
    vector<VisionObject> objects = ObjectDetector::filterObjects(blobs,grayFrame,frame);
    for(int p = 0; p < objects.size();++p){
        //Draws a rotated box around each object
        RotatedRect rect = minAreaRect(objects[p].data);
        Point2f vertices[4];
        rect.points(vertices);
        for(int i = 0; i < 4;++i){
            line(frame,vertices[i],vertices[(i+1)%4],Scalar(0,0,255),5);
        }

        pair<Part,double> match = PartMatcher::matchPart(PartMatcher::createParameterMap(objects[p]));
        if(match.second > 80){
            string matchResult = match.first.name + " - " + to_string(match.second);
            REXOS_WARN_STREAM(matchResult);
        }
    }
    if(!frame.empty()){
        ros::Time time = ros::Time::now();
        cvi.header.stamp = time;
        cvi.header.frame_id = "stl_debug_image";
        cvi.encoding = sensor_msgs::image_encodings::BGR8;
        cvi.image = frame;
        debugImagePublisher.publish(cvi.toImageMsg());
    }else{
        REXOS_INFO("STL VISION: Given clone frame was empty.");
    }


}
