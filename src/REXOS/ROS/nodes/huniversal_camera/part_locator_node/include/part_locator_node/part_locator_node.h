/**
 * @file part_locator_node.cpp
 * @brief locates objects and rotates points.
 * @date Created: 2013-09-20
 *
 * @author Garik hakopian
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

#pragma once

#include "ros/ros.h"
#include "rexos_logger/rexos_logger.h"

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <vision_node/QrCodes.h>

#include <iostream>
#include <string>

#include <matrices/Matrices.h>
#include <vectors/Vectors.h>
#include <rexos_statemachine/ModuleStateMachine.h>
#include <rexos_statemachine/Transitions.h>
#include "equiplet_node/RegisterModule.h"
#include <rexos_coordinates/Module.h>
#include <rexos_knowledge_database/Module.h>
#include <rexos_knowledge_database/ModuleIdentifier.h>

#include <vision_node/QrCodes.h>

#include <boost/circular_buffer.hpp>
#include <map>

class PartLocatorNode : public rexos_knowledge_database::Module,
		public rexos_coordinates::Module, public rexos_statemachine::ModuleStateMachine {
public:
	struct QrCode {
		Vector2 location;
		double angle;
	};
protected:
	static const Vector2 EXPECTED_DIRECTION;
	static const Vector2 EXPECTED_ITEM_DIRECTION;
	static const int minCornerSamples;
	static const int minItemSamples;
	std::string topLeftValue;
	std::string topRightValue;
	std::string bottomRightValue;
	double workPlaneWidth;
	double workPlaneHeight;
	double workSpaceHeight;
	
	double actualWorkPlaneWidth;
	double actualWorkPlaneHeight;
	double topLeftOffsetX, topLeftOffsetY;
	double topRightOffsetX, topRightOffsetY;
	double bottomRightOffsetX, bottomRightOffsetY;
	
	std::string equipletName;
	std::string equipletIdentifier;
	
	std::map<std::string, boost::circular_buffer<QrCode> > smoothBuffer;

public:
	PartLocatorNode(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier, std::string equipletIdentifier);
	void run();
	
	virtual bool transitionInitialize();
	virtual bool transitionDeinitialize();
	/**
	 * MAST transition from safe to standby is handled by this method. 
	 * see http://wiki.agilemanufacturing.nl/index.php/Vision_system#camera_control_node
	 **/
	virtual bool transitionSetup();
	/**
	 * MAST transition from standby to safe is handled by this method. 
	 * see http://wiki.agilemanufacturing.nl/index.php/Vision_system#camera_control_node
	 **/
	virtual bool transitionShutdown();
	/**
	 * MAST transition from standby to normal is handled by this method. 
	 * see http://wiki.agilemanufacturing.nl/index.php/Vision_system#camera_control_node
	 **/
	virtual bool transitionStart();
	/**
	 * MAST transition from normal to standby is handled by this method. 
	 * see http://wiki.agilemanufacturing.nl/index.php/Vision_system#camera_control_node
	 **/
	virtual bool transitionStop();
private:
	/**
	 * The first known position of the top left corner
	 **/
	QrCode originalTopLeftCoor;
	/**
	 * The first known position of the top right corner
	 **/
	QrCode originalTopRightCoor;
	/**
	 * The first known position of the bottom right corner
	 **/
	QrCode originalBottomRightCoor;
	/**
	 * The last known position of the top left corner
	 **/
	QrCode currentTopLeftCoor;
	/**
	 * The last known position of the top right corner
	 **/
	QrCode currentTopRightCoor;
	/**
	 * The last known position of the bottom right corner
	 **/
	QrCode currentBottomRightCoor;
	Matrix3 totalMatrix;
	Matrix3 postCorrectionTotalMatrix;
	
	boost::circular_buffer<QrCode> samplesTopLeft;
	boost::circular_buffer<QrCode> samplesTopRight;
	boost::circular_buffer<QrCode> samplesBottomRight;
	
	ros::NodeHandle nodeHandle;
	/**
	 * The service client used for storing QR codes in the enviroment_cache
	 **/
	ros::ServiceClient environmentCacheClient;
	
	/**
	 * Handles the input from the vision_node
	 * It converts the QR code coordinates from screen space to workplane space (by multiplying it with totalMatrix). 
	 **/
	void qrCodeCallback(const vision_node::QrCodes & message);
	
	/**
	 * Extracts the QR codes from the message and selects the QR codes of the corners (if present). If so, it will call updateMatrices().
	 **/
	void detectCorners(const vision_node::QrCodes & message);
	/**
	 * Submits the QR code to the enviroment_cache
	 * @param the value on the QR code
	 * @param the location of the QR code
	 * @param the rotation angle of the QR code
	 **/
	void storeInEnviromentCache(std::string value, Vector3 location, double angle);
	/**
	 * Calculates the rotation angle for an QR code by comparing the input vector to the desired vector.
	 * @return the rotation angle
	 **/
	double getItemRotationAngle(Vector2 lineA2B);
	
	/**
	 * Updates the totalMatrix by calling calculateOffsetMatrix(), calculateRotationMatrix(), and calculateScaleMatrix().
	 **/
	void updateMatrices();
	
	/**
	 * Calculates a new translation matrix which will match the origin of the workplane as seen on screen to the desired origin (at 0, 0).
	 * @return The translation matrix
	 **/
	Matrix3 calculateOffsetMatrix();
	/**
	 * Calculates a new rotation matrix which will match the rotation of the workplane as seen on screen to the desired rotation.
	 * @return The rotation matrix
	 **/
	Matrix3 calculateRotationMatrix();
	/**
	 * Calculates a new translation matrix which will scale the workplane as seen on screen to the desired size.
	 * @return The scale matrix
	 **/
	Matrix3 calculateScaleMatrix();
	
	QrCode calculateSmoothPos(std::string name, QrCode lastPosition);
	QrCode calculateSmoothPos(boost::circular_buffer<QrCode> buffer);
};
