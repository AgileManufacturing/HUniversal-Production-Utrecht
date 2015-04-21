#include "gripper_plugin/GripperPlugin.h"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <ros/callback_queue.h>
#include <boost/algorithm/string.hpp>
#include <map>

using namespace gazebo;

namespace gripper_plugin {
	GripperPlugin::GripperPlugin() : ModelPlugin(), contactSub(NULL) {
		std::cout << "GripperPlugin constructed" << std::endl;
	}
	GripperPlugin::~GripperPlugin() {
	}
	void GripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
		std::cout << "GripperPlugin loading" << std::endl;
		
		model = _model;
		
		if(_sdf->HasElement("grippers") == false) {
			std::cerr << "Missing grippers" << std::endl;
			return;
		}
		std::string sdfValue = _sdf->GetElement("grippers")->GetValue()->GetAsString();
		
		std::vector<std::string> grippersStrings;
		boost::split(grippersStrings, sdfValue, boost::is_any_of(" "));
		
		// temp map used for subscription
		std::map<std::string, physics::CollisionPtr> collisions;
		for(uint i = 0; i < grippersStrings.size() - 1; i += 2) {
			physics::LinkPtr link = _model->GetLink(grippersStrings[i]);
			if(link == NULL) {
				ROS_FATAL_STREAM("link " << grippersStrings[i] << " does not exist in this model");
				return;
			}
			physics::CollisionPtr collision = link->GetCollision(grippersStrings[i + 1]);
			if(collision == NULL) {
				ROS_FATAL_STREAM("collision " << grippersStrings[i + 1] << " does not exist in this model");
				return;
			}
			
			collisions[link->GetName() + "::" + collision->GetName()] = collision;
			Gripper gripper(link, collision);
			grippers.push_back(gripper);
		}
		
		// Subscribing to contact topics
		node = transport::NodePtr(new transport::Node());
		node->Init(model->GetWorld()->GetName());
		
		physics::ContactManager* mgr = model->GetWorld()->GetPhysicsEngine()->GetContactManager();
		std::string topic = mgr->CreateFilter(model->GetName(), collisions);
		std::cout << "Subcribing to contact topic " << topic << std::endl;
		contactSub = node->Subscribe(topic, &GripperPlugin::onContacts, this);
		
		// Advertising ROS services
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
		
		nodeHandle = new ros::NodeHandle();
		writeGripperActiveServer = nodeHandle->advertiseService(rosNamespace + address + "/write/", 
				&GripperPlugin::writeGripperActive, this);
		readGripperActiveServer = nodeHandle->advertiseService(rosNamespace + address + "/read/", 
				&GripperPlugin::readGripperActive, this);
		
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&GripperPlugin::onUpdate, this, _1));
	}
	void GripperPlugin::onUpdate(const common::UpdateInfo& info) {
		common::Time currTime = model->GetWorld()->GetSimTime();
		for(uint i = 0; i < grippers.size(); i++) {
			grippers[i].handleUpdate(currTime);
		}
	}
	void GripperPlugin::onContacts(ConstContactsPtr& msg) {
		for(uint i = 0; i < grippers.size(); i++) {
			for (int j = 0; j < msg->contact_size(); j++) {
				physics::CollisionPtr collision = grippers[i].getCollision();
				physics::CollisionPtr collision1 = boost::dynamic_pointer_cast<physics::Collision>(
					model->GetWorld()->GetEntity(msg->contact(j).collision1()));
				physics::CollisionPtr collision2 = boost::dynamic_pointer_cast<physics::Collision>(
					model->GetWorld()->GetEntity(msg->contact(j).collision2()));
			
				if(collision1 == collision || collision2 == collision) {
					grippers[i].handleContact(msg->contact(j), model->GetWorld());
				}
			}
		}
	}
	
	bool GripperPlugin::writeGripperActive(rexos_io::writeU16::Request& request, 
			rexos_io::writeU16::Response& response) {
		ROS_INFO_STREAM("writing gripper " << request.value);
		for(uint i = 0; i < grippers.size(); i++) {
			// The gripper valve works inverted, mimic that behavior
			if((request.value >> i) & 1) {
				grippers[i].setActive(true);
			} else {
				grippers[i].setActive(false);
			}
		}
		return true;
	}
	bool GripperPlugin::readGripperActive(rexos_io::readU16::Request& request, 
			rexos_io::readU16::Response& response) {
		response.value = 0;
		for(uint i = 0; i < grippers.size(); i++) {
			if(grippers[i].getIsActive() == true) {
				response.value |= 1 << i;
			}
		}
		ROS_INFO_STREAM("reading gripper " << response.value);
		return true;
	}
}
