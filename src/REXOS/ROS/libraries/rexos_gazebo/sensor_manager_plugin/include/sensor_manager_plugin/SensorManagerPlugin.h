#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <ros/ros.h>

#include <sensor_manager_plugin/ContactSensor.h>
#include <rexos_io/readU16.h>

#include <vector>

using namespace gazebo;

namespace sensor_manager_plugin {
	class SensorManagerPlugin : public ModelPlugin {
	public: 
		SensorManagerPlugin();
	public: 
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		void onUpdate(const common::UpdateInfo & info);
		
		bool readSensors(rexos_io::readU16::Request& request, 
				rexos_io::readU16::Response& response);
	protected:
		std::vector<ContactSensor> contactSensors;
		ros::NodeHandle* nodeHandle;
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;
		
		ros::ServiceServer readSensorsServer;
	};
	GZ_REGISTER_MODEL_PLUGIN(sensor_manager_plugin::SensorManagerPlugin)
}
