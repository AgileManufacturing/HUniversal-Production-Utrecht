#include "sensor_manager_plugin/SensorManagerPlugin.h"

#include <gazebo/physics/World.hh>
#include <ros/callback_queue.h>
#include <boost/algorithm/string.hpp>

using namespace gazebo;

namespace sensor_manager_plugin {
	SensorManagerPlugin::SensorManagerPlugin() : ModelPlugin() {
		std::cout << "SensorManagerPlugin constructed" << std::endl;
	}
	void SensorManagerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
		std::cout << "SensorManagerPlugin loading" << std::endl;
		
		if(_sdf->HasElement("contactSensors") == false) {
			std::cerr << "Missing contactSensors" << std::endl;
			return;
		}
		std::string sdfValue = _sdf->GetElement("contactSensors")->GetValue()->GetAsString();
		
		std::vector<std::string> contactSensorsStrings;
		boost::split(contactSensorsStrings, sdfValue, boost::is_any_of(" "));
		
		for(uint i = 0; i < contactSensorsStrings.size() - 1; i += 2) {
			ContactSensor contactSensor;
			contactSensor.joint = _model->GetJoint(contactSensorsStrings[i]);
			contactSensor.angle = boost::lexical_cast<double>(contactSensorsStrings[i + 1]);
			if(contactSensor.joint == NULL) {
				ROS_FATAL_STREAM("contactSensor " << contactSensorsStrings[i] << " does not exist in this model");
			}
			contactSensors.push_back(contactSensor);
		}
		
		if(_sdf->HasElement("rosNamespace") == false) {
			std::cerr << "Missing rosNamespace" << std::endl;
			return;
		}
		std::string rosNamespace = _sdf->GetElement("rosNamespace")->GetValue()->GetAsString();
		ROS_INFO_STREAM("Advertising services at " << rosNamespace);
		
		if(_sdf->HasElement("address") == false) {
			std::cerr << "Missing address" << std::endl;
			return;
		}
		std::string address = _sdf->GetElement("address")->GetValue()->GetAsString();
		ROS_INFO_STREAM("Advertising at address " << address);
		
		if (!ros::isInitialized()) {
			ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
			<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			return;
		}
		
		model = _model;
		nodeHandle = new ros::NodeHandle();
		readSensorsServer = nodeHandle->advertiseService(rosNamespace + address, 
				&SensorManagerPlugin::readSensors, this);
		
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&SensorManagerPlugin::onUpdate, this, _1));
	}
	void SensorManagerPlugin::onUpdate(const common::UpdateInfo & info) {
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
	}
	
	bool SensorManagerPlugin::readSensors(rexos_io::readU16::Request& request, 
			rexos_io::readU16::Response& response) {
		for(uint i = 0; i < contactSensors.size(); i++) {
			ContactSensor& contactSensor = contactSensors[i];
			// the real sensors are inverted. Mimic this behaviour
			response.value |= (!contactSensor.isTriggered()) << i;
		}
		return true;
	}
}
