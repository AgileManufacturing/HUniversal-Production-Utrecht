#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>

#include <iostream>

namespace gazebo {
	class AttachPlugin : public ModelPlugin {
	public: 
		AttachPlugin();
	public: 
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	};
	GZ_REGISTER_MODEL_PLUGIN(AttachPlugin)
}
