/**
 * @file ImageTransformationNode.h
 * @brief Transforms the image from the camera with a GUI and adjustable adaptive threshold. It sends out the image on a topic.
 * @date 2012-11-06
 *
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

#ifndef ImageTransformationNode_H
#define ImageTransformationNode_H

#include "iostream"

#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "Topics.h"
#include "Utilities/Utilities.h"
#include "DotMatrixPrinterNode/DotMatrixPrinterNodeSettings.h"

/**
 * Transforms an image to the correct pixel size and a black and white format so it can be printed by the DotMatrixPrinterNode.
 **/
class ImageTransformationNode
{
	public:
		ImageTransformationNode(std::string path);
		cv::Mat transformImage();
		void publishImage();
		void run();
	private:
		/**
		 * @var cv::Mat inputImage
		 * The image to be transformed.
		 **/
		cv::Mat inputImage;

		/**
		 * @var cv::Mat outputImage
		 * The image sent out after transformation.
		 **/
		cv::Mat outputImage;
		
		/**
		 * @var ros::NodeHandle nodeHandle
		 * roscpp's interface for creating subscribers, publishers, etc. 
		 **/
		ros::NodeHandle nodeHandle;
		
		/**
		 * @var image_transport::ImageTransport imageTransport
		 * Prepares the image for transport.
		 **/
		image_transport::ImageTransport imageTransport;
		
		/**
		 * @var image_transport::Publisher pub
		 * Publisher that publishes the image.
		 **/
		image_transport::Publisher pub;
};

#endif
