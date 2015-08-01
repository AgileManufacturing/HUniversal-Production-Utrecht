/**
 * @file MotorManagerPlugin.h
 * @brief The motor manager plugin is used to control the motors of a model. It publishes the services to allow the REXOS system to configure and activate the motors.
 * The motor manager plugin is designed to be used with the @link rexos_motor library. It does NOT use the modbus RTU protocol used by the CRD514-KD
 * @date Created: 2015-06-00
 *
 * @author Tommas Bakker
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

#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <ros/ros.h>

#include <motor_manager_plugin/Motor.h>
#include <motor_manager_plugin/isMotorReady.h>
#include <motor_manager_plugin/setMinSpeed.h>
#include <motor_manager_plugin/setLowerAngleLimit.h>
#include <motor_manager_plugin/setUpperAngleLimit.h>
#include <motor_manager_plugin/setMotorMode.h>
#include <motor_manager_plugin/setPowerStatus.h>
#include <motor_manager_plugin/startMotor.h>
#include <motor_manager_plugin/stopMotor.h>
#include <motor_manager_plugin/writeRotationData.h>

#include <vector>

using namespace gazebo;

namespace motor_manager_plugin {
	class MotorManagerPlugin : public ModelPlugin {
	public: 
		MotorManagerPlugin();
	public: 
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		/**
		 * This method is called just BEFORE the world in gazebo is updated
		 */
		void onUpdate(const common::UpdateInfo & info);
		
		bool isMotorReady(motor_manager_plugin::isMotorReady::Request& request, motor_manager_plugin::isMotorReady::Response& response);
		bool setMinSpeed(motor_manager_plugin::setMinSpeed::Request& request, motor_manager_plugin::setMinSpeed::Response& response);
		bool setLowerAngleLimit(motor_manager_plugin::setLowerAngleLimit::Request& request, motor_manager_plugin::setLowerAngleLimit::Response& response);
		bool setUpperAngleLimit(motor_manager_plugin::setUpperAngleLimit::Request& request, motor_manager_plugin::setUpperAngleLimit::Response& response);
		bool setMotorMode(motor_manager_plugin::setMotorMode::Request& request, motor_manager_plugin::setMotorMode::Response& response);
		bool setPowerStatus(motor_manager_plugin::setPowerStatus::Request& request, motor_manager_plugin::setPowerStatus::Response& response);
		bool startMotor(motor_manager_plugin::startMotor::Request& request, motor_manager_plugin::startMotor::Response& response);
		bool stopMotor(motor_manager_plugin::stopMotor::Request& request, motor_manager_plugin::stopMotor::Response& response);
		bool writeRotationData(motor_manager_plugin::writeRotationData::Request& request, motor_manager_plugin::writeRotationData::Response& response);
	protected:
		std::vector<Motor> motors;
		ros::NodeHandle* nodeHandle;
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;
		
		ros::ServiceServer isMotorReadyServer;
		ros::ServiceServer setMinSpeedServer;
		ros::ServiceServer setLowerAngleLimitServer;
		ros::ServiceServer setUpperAngleLimitServer;
		ros::ServiceServer setMotorModeServer;
		ros::ServiceServer setPowerStatusServer;
		ros::ServiceServer startMotorServer;
		ros::ServiceServer stopMotorServer;
		ros::ServiceServer writeRotationDataServer;
	};
	GZ_REGISTER_MODEL_PLUGIN(motor_manager_plugin::MotorManagerPlugin)
}
