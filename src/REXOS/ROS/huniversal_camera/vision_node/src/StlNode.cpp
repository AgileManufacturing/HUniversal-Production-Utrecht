#include "vision_node/StlNode.h"

#include <iostream>

StlNode::StlNode(){}

void StlNode::handleFrame(cv::Mat& frame){
    REXOS_INFO("STL NODE IS TRYING TO DO ITS JOB, CALM DOWN!");
    cv::namedWindow("Testing node");
    imshow("Testing node",frame);
}
