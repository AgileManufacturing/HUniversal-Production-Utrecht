/**
 * @file CameraNode.h
 * @brief Header for the camera node class
 * @date Created: 2012-10-09
 * 
 * @author Arjan Groenewegen
 * 
 * @section LICENSE 
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
 *
 **/

#ifndef IMAGESTREAMNODE_H
#define IMAGESTREAMNODE_H

#include "ros/ros.h"
#include "rexos_logger/rexos_logger.h"

#include <opencv/cvwimage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>

#include <iostream>
#include <sstream>

/**
 * @brief This class enables the abillity to simulate a camera by streaming a specific image. 
 * This makes debugging without using the camera possible.
 */
class ImageStreamNode {
public:
	ImageStreamNode(int argc, char * argv[]);
	~ImageStreamNode();
	void run();

private:
	ros::NodeHandle nodeHandle;
	image_transport::ImageTransport imageTransport;
	image_transport::Publisher publisher;
	
	char* pathToImageFile;
};

#endif
