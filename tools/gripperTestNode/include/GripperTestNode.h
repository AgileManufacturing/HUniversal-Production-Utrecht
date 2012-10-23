#include <rosMast/StateMachine.h>
#include "ros/ros.h"
#include "gripperTestNode/Grip.h"
#include "gripperTestNode/Release.h"
#include "Services.h"

class GripperTestNode: public rosMast::StateMachine
{
	public:
		GripperTestNode(int equipletID, int moduleID);
		~GripperTestNode();
		int transitionSetup();
		int transitionShutdown();
		int transitionStart();
		int transitionStop();
		static void error();
		bool grip(gripperTestNode::Grip::Request &req, gripperTestNode::Grip::Response &res);	
		bool release(gripperTestNode::Release::Request &req, gripperTestNode::Release::Response &res);
	private:
		Gripper * gripper;
		ros::ServiceServer gripService;
		ros::ServiceServer releaseService;
};