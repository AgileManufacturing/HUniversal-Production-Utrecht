#include "attach_plugin/AttachPlugin.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>

#include <iostream>

namespace gazebo {
	AttachPlugin::AttachPlugin() : ModelPlugin(){
		std::cout << "AttachPlugin constructed";
	}
	void AttachPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
		std::cout << "AttachPlugin loading";
		
		if(_sdf->HasElement("parentModel") == false) {
			std::cerr << "Missing parentModel" << std::endl;
			return;
		}
		std::string parentModel = _sdf->GetElement("parentModel")->GetValueString();
		
		if(_sdf->HasElement("parentLink") == false) {
			std::cerr << "Missing parentLink" << std::endl;
			return;
		}
		std::string parentLink = _sdf->GetElement("parentLink")->GetValueString();
		
		if(_sdf->HasElement("childLink") == false) {
			std::cerr << "Missing childLink" << std::endl;
			return;
		}
		std::string childLink = _sdf->GetElement("childLink")->GetValueString();
		
		
		if(_model->GetWorld()->GetModel(parentModel) == NULL) {
			std::cerr << "The parentModel " << parentModel << " does noet exist" << std::endl;
			return;
		}
		
		if(_model->GetWorld()->GetModel(parentModel)->GetLink(parentLink) == NULL) {
			std::cerr << "The parentLink " << parentLink << " does noet exist" << std::endl;
			return;
		}
		
		if(_model->GetLink(childLink) == NULL) {
			std::cerr << "The childLink " << childLink << " does noet exist" << std::endl;
			return;
		}
		
		std::cerr << "Attaching..." << std::endl;
		_model->GetJoint("linkToParent")->Attach(
				_model->GetWorld()->GetModel(parentModel)->GetLink(parentLink), 
				_model->GetLink(childLink));
	}
}
