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
