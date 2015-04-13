#include "motor_manager_plugin/MotorManagerPlugin.h"

#include <gazebo/physics/World.hh>
#include <ros/callback_queue.h>
#include <boost/algorithm/string.hpp>

using namespace gazebo;

namespace motor_manager_plugin {
	MotorManagerPlugin::MotorManagerPlugin() : ModelPlugin() {
		std::cout << "MotorManagerPlugin constructed" << std::endl;
	}
	void MotorManagerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
		std::cout << "MotorManagerPlugin loading" << std::endl;
		
		if(_sdf->HasElement("motors") == false) {
			std::cerr << "Missing motors" << std::endl;
			return;
		}
		std::string sdfValue = _sdf->GetElement("motors")->GetValue()->GetAsString();
		ROS_INFO_STREAM("motors sdfValue: " << sdfValue);
		
		std::vector<std::string> motorStrings;
		boost::split(motorStrings, sdfValue, boost::is_any_of(" "));
		
		for(int i = 0; i < motorStrings.size(); i++) {
			ROS_INFO_STREAM("motors entry: " << motorStrings[i]);
			Motor motor;
			motor.joint = _model->GetJoint(motorStrings[i]);
			if(motor.joint == NULL) {
				ROS_FATAL_STREAM("motor " << motorStrings[i] << " does not exist in this model");
			}
			motors.push_back(motor);
		}
		
		if (!ros::isInitialized()) {
			ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
			<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			return;
		}
		
		model = _model;
		nodeHandle = new ros::NodeHandle();
		isMotorReadyServer = nodeHandle->advertiseService("isMotorReady", 
				&MotorManagerPlugin::isMotorReady, this);
		setMinSpeedServer = nodeHandle->advertiseService("setMinSpeed", 
				&MotorManagerPlugin::setMinSpeed, this);
		setUpperAngleLimitServer = nodeHandle->advertiseService("setUpperAngleLimit", 
				&MotorManagerPlugin::setUpperAngleLimit, this);
		setLowerAngleLimitServer = nodeHandle->advertiseService("setLowerAngleLimit", 
				&MotorManagerPlugin::setLowerAngleLimit, this);
		setMotorModeServer = nodeHandle->advertiseService("setMotorMode", 
				&MotorManagerPlugin::setMotorMode, this);
		setPowerStatusServer = nodeHandle->advertiseService("setPowerStatus", 
				&MotorManagerPlugin::setPowerStatus, this);
		startMotorServer = nodeHandle->advertiseService("startMotor", 
				&MotorManagerPlugin::startMotor, this);
		stopMotorServer = nodeHandle->advertiseService("stopMotor", 
				&MotorManagerPlugin::stopMotor, this);
		writeRotationDataServer = nodeHandle->advertiseService("writeRotationData", 
				&MotorManagerPlugin::writeRotationData, this);
		
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&MotorManagerPlugin::onUpdate, this, _1));
	}
	void MotorManagerPlugin::onUpdate(const common::UpdateInfo & info) {
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
		
		common::Time currTime = model->GetWorld()->GetSimTime();
		for(auto iter = motors.begin(); iter < motors.end(); iter++) {
			iter->updateMotor(currTime);
		}
	}
	
	bool MotorManagerPlugin::isMotorReady(motor_manager_plugin::isMotorReady::Request& request, 
			motor_manager_plugin::isMotorReady::Response& response) {
		if(request.motorIndex >= motors.size() || request.motorIndex < 0) {
			return false;
		}
		Motor& motor = motors[request.motorIndex];
		response.isMotorReady = !motor.isMotorReady();
		return true;
	}
	bool MotorManagerPlugin::setMinSpeed(motor_manager_plugin::setMinSpeed::Request& request, 
			motor_manager_plugin::setMinSpeed::Response& response) {
		if(request.motorIndex >= motors.size() || request.motorIndex < 0) {
			return false;
		}
		Motor& motor = motors[request.motorIndex];
		motor.minSpeed = request.velocity;
		return true;
	}
	bool MotorManagerPlugin::setLowerAngleLimit(motor_manager_plugin::setLowerAngleLimit::Request& request, 
			motor_manager_plugin::setLowerAngleLimit::Response& response) {
		if(request.motorIndex >= motors.size() || request.motorIndex < 0) {
			return false;
		}
		Motor& motor = motors[request.motorIndex];
		motor.lowerAngleLimit = request.angle;
		return true;
	}
	bool MotorManagerPlugin::setUpperAngleLimit(motor_manager_plugin::setUpperAngleLimit::Request& request, 
			motor_manager_plugin::setUpperAngleLimit::Response& response) {
		if(request.motorIndex >= motors.size() || request.motorIndex < 0) {
			return false;
		}
		Motor& motor = motors[request.motorIndex];
		motor.upperAngleLimit = request.angle;
		return true;
	}
	bool MotorManagerPlugin::setMotorMode(motor_manager_plugin::setMotorMode::Request& request, 
			motor_manager_plugin::setMotorMode::Response& response) {
		if(request.motorIndex >= motors.size() || request.motorIndex < 0) {
			return false;
		}
		Motor& motor = motors[request.motorIndex];
		motor.relativeMode = request.relativeMode;
		return true;
	}
	bool MotorManagerPlugin::setPowerStatus(motor_manager_plugin::setPowerStatus::Request& request, 
			motor_manager_plugin::setPowerStatus::Response& response) {
		if(request.motorIndex >= motors.size() || request.motorIndex < 0) {
			return false;
		}
		Motor& motor = motors[request.motorIndex];
		motor.setPowerStatus(request.powerStatus);
		return true;
	}
	bool MotorManagerPlugin::startMotor(motor_manager_plugin::startMotor::Request& request, 
			motor_manager_plugin::startMotor::Response& response) {
		if(request.motorIndex >= motors.size()) {
			return false;
		}
		if(request.motorIndex < 0) {
			for(int i = 0; i < motors.size(); i++) {
				Motor& motor = motors[i];
				motor.startMotor(model->GetWorld()->GetSimTime());
			}
		} else {
			Motor& motor = motors[request.motorIndex];
			motor.startMotor(model->GetWorld()->GetSimTime());
		}
		return true;
	}
	bool MotorManagerPlugin::stopMotor(motor_manager_plugin::stopMotor::Request& request, 
			motor_manager_plugin::stopMotor::Response& response) {
		if(request.motorIndex >= motors.size()) {
			return false;
		}
		if(request.motorIndex < 0) {
			for(int i = 0; i < motors.size(); i++) {
				Motor& motor = motors[i];
				motor.stopMotor(model->GetWorld()->GetSimTime());
			}
		} else {
			Motor& motor = motors[request.motorIndex];
			motor.stopMotor(model->GetWorld()->GetSimTime());
		}
		return true;
	}
	bool MotorManagerPlugin::writeRotationData(motor_manager_plugin::writeRotationData::Request& request, 
			motor_manager_plugin::writeRotationData::Response& response) {
		if(request.motorIndex >= motors.size() || request.motorIndex < 0) {
			return false;
		}
		Motor& motor = motors[request.motorIndex];
		motor.maxSpeed = request.maxSpeed;
		motor.maxAcceleration = request.maxAcceleration;
		motor.maxDecceleration = request.maxDecelleration;
		if(motor.relativeMode == true) {
			// relative mode
			motor.targetAngle += request.angle;
		} else {
			// absolute mode
			motor.targetAngle = request.angle;
		}
		return true;
	}
}
