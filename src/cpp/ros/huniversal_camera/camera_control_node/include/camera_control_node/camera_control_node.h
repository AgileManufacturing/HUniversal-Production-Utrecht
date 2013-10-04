/** @file cameraControlNode.h
 * @brief Remote interface to adjust the camera settings in runtime.
 * @date Created: 2012-10-18
 *
 * @author Koen Braham
 * @author Daan Veltman
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

#ifndef CAMERACONTROLNODE_H_
#define CAMERACONTROLNODE_H_

#include "ros/ros.h"
#include <camera_node/camera_node.h>
#include <rexos_statemachine/ModuleStateMachine.h>
#include <rexos_statemachine/Transitions.h>
#include "equiplet_node/RegisterModule.h"


class CameraControlNode : public rexos_statemachine::ModuleStateMachine {
public:
	CameraControlNode(int equipletID, int moduleID);

	void increaseExposureCall();
	void decreaseExposureCall();
	void autoWhiteBalanceCall(bool enabled);

	inline std::string printResult(bool result);

	void run();
	
		virtual void transitionSetup(rexos_statemachine::TransitionActionServer* as);
		virtual void transitionShutdown(rexos_statemachine::TransitionActionServer* as);
		virtual void transitionStart(rexos_statemachine::TransitionActionServer* as);
		virtual void transitionStop(rexos_statemachine::TransitionActionServer* as);

private:
	ros::NodeHandle nodeHandle;
	ros::ServiceClient increaseExposureClient;
	ros::ServiceClient decreaseExposureClient;
	ros::ServiceClient autoWhiteBalanceClient;
	ros::ServiceClient fishEyeCorrectionClient;
	
	ros::ServiceClient calibrateLensClient;
	std_srvs::Empty emptyService;
};

#endif /* CAMERACONTROLNODE_H_ */
