/**
* @file DeltaRobotNode.cpp
* @brief Provide the services to move the DeltaRobot.
* @date Created: 2012-09-19
*
* @author Dick van der Steen
* @author Dennis Koole
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
**/


#include "DeltaRobotNode/deltaRobotNode.h"

#define NODE_NAME "DeltaRobotNode"

deltaRobotNodeNamespace::DeltaRobotNode::DeltaRobotNode(int equipletID, int moduleID) : rosMast::StateMachine(equipletID, moduleID)
{	
	ros::NodeHandle nodeHandle;

	// Advertise the services
	ros::ServiceServer moveToPointService =
		nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_POINT, &deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint, this);

	ros::ServiceServer movePathService =
		nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_PATH, &deltaRobotNodeNamespace::DeltaRobotNode::movePath, this);

	ros::ServiceServer moveToRelativePointService =
		nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_RELATIVE_POINT, &deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint, this);

	ros::ServiceServer moveRelativePathService =
		nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_RELATIVE_PATH, &deltaRobotNodeNamespace::DeltaRobotNode::moveRelativePath, this);

	ros::ServiceServer calibrateService =
		nodeHandle.advertiseService(DeltaRobotNodeServices::CALIBRATE, &deltaRobotNodeNamespace::DeltaRobotNode::calibrate, this); 

	// Initialize modbus for IO controller
    modbus_t* modbusIO = modbus_new_tcp("192.168.0.2", 502);
    if(modbusIO == NULL)
    {
        throw std::runtime_error("Unable to allocate libmodbus context");
    }
    if(modbus_connect(modbusIO) == -1)
    {
        throw std::runtime_error("Modbus connection to IO controller failed");
    }
    assert(modbusIO != NULL);

    DataTypes::DeltaRobotMeasures drm;
    drm.base = DeltaRobot::Measures::BASE;
    drm.hip = DeltaRobot::Measures::HIP;
    drm.effector = DeltaRobot::Measures::EFFECTOR;
    drm.ankle = DeltaRobot::Measures::ANKLE;
    drm.maxAngleHipAnkle = DeltaRobot::Measures::HIP_ANKLE_ANGLE_MAX;


    ModbusController::ModbusController* modbus = new ModbusController::ModbusController(modbus_new_rtu(
        "/dev/ttyS0",
        Motor::CRD514KD::RtuConfig::BAUDRATE,
        Motor::CRD514KD::RtuConfig::PARITY,
        Motor::CRD514KD::RtuConfig::DATA_BITS,
        Motor::CRD514KD::RtuConfig::STOP_BITS));


    Motor::StepperMotor* motors[3];
    motors[0] = new Motor::StepperMotor(modbus, Motor::CRD514KD::Slaves::MOTOR_0, DeltaRobot::Measures::MOTOR_ROT_MIN, DeltaRobot::Measures::MOTOR_ROT_MAX);
    motors[1] = new Motor::StepperMotor(modbus, Motor::CRD514KD::Slaves::MOTOR_1, DeltaRobot::Measures::MOTOR_ROT_MIN, DeltaRobot::Measures::MOTOR_ROT_MAX);
    motors[2] = new Motor::StepperMotor(modbus, Motor::CRD514KD::Slaves::MOTOR_2, DeltaRobot::Measures::MOTOR_ROT_MIN, DeltaRobot::Measures::MOTOR_ROT_MAX);

    Motor::MotorManager* motorManager = new Motor::MotorManager(modbus, motors, 3);

	// Create a deltarobot	
    deltaRobot = new DeltaRobot::DeltaRobot(drm, motorManager, motors, modbusIO);

	StateMachine::StateEngine();
}

/**
 * Starts the (re)calibration of the robot
 * @param req The request for this service as defined in Calibrate.srv 
 * @param res The response for this service as defined in Calibrate.srv
 * 
 * @return true if the calibration was succesfully. false otherwise.
 */
bool deltaRobotNodeNamespace::DeltaRobotNode::calibrate(deltaRobotNode::Calibrate::Request &req,
	deltaRobotNode::Calibrate::Response &res) {

	if(currentState != rosMast::normal) {
		return false;
	}

    // Calibrate the motors
    if(!deltaRobot->calibrateMotors()){
    	ROS_ERROR("Calibration FAILED. EXITING.");
    	return false;
    }
	return true;	
}

/**
 * Move to an absolute point. Will be implemented in a later release.
 *
 * @param req The request for this service as defined in MoveToPoint.srv 
 * @param res The response for this service as defined in MoveToPoint.srv
 * 
 * @return true
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint(deltaRobotNode::MoveToPoint::Request &req,
	deltaRobotNode::MoveToPoint::Response &res) {
	ROS_INFO("moveToPoint called");

	if(currentState != rosMast::normal) {
		return false;
	}

	DataTypes::Point3D<double>& effectorLocation = deltaRobot->getEffectorLocation();
	deltaRobotNode::Motion motion = req.motion;
	/**
	 * Check if the DeltaRobot can move from the current effector location
	 * to the absolute point given as argument for this service.
	 **/
	if(!deltaRobot->checkPath(
		DataTypes::Point3D<double>(effectorLocation.x, effectorLocation.y, effectorLocation.z),
		DataTypes::Point3D<double>(motion.x, motion.y, motion.z)))
	{
		res.succeeded = false;
		return res.succeeded;
	}
	ROS_INFO("moveTo: (%f, %f, %f) speed=%f", motion.x, motion.y,motion.z, motion.speed);
	deltaRobot->moveTo(DataTypes::Point3D<double>(motion.x, motion.y, motion.z),motion.speed);
	res.succeeded = true;
	return res.succeeded;
}

/**
 * Move to a number of absolute points.
 *
 * @param req The request for this service as defined in MovePath.srv 
 * @param res The response for this service as defined in MovePath.srv
 * 
 * @return true
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::movePath(deltaRobotNode::MovePath::Request &req,
	deltaRobotNode::MovePath::Response &res) {
	ROS_INFO("movePath called");

	if(currentState != rosMast::normal) {
		return false;
	}

	deltaRobotNode::Motion currentMotion;
	deltaRobotNode::Motion nextMotion;
	try
	{
		unsigned int n;
		for(n = 0; n < req.motion.size() -1; n++)
		{
			currentMotion = req.motion[n];			
			nextMotion = req.motion[n+1];
			if(!deltaRobot->checkPath(
				DataTypes::Point3D<double>(currentMotion.x, currentMotion.y, currentMotion.z),
				DataTypes::Point3D<double>(nextMotion.x, nextMotion.y, nextMotion.z)))
			{
				res.succeeded = false;
				return res.succeeded;
			}
		}
		for(n = 0; n < req.motion.size(); n++)
		{	
			currentMotion = req.motion[n];
			ROS_INFO("moveTo: (%f, %f, %f) speed=%f", currentMotion.x, currentMotion.y,currentMotion.z, currentMotion.speed);
			deltaRobot->moveTo(DataTypes::Point3D<double>(currentMotion.x, currentMotion.y, currentMotion.z),currentMotion.speed);
		}
	}
	catch(std::runtime_error& ex)
	{
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		res.succeeded = false;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
	}
	res.succeeded = true;
	return res.succeeded;
}

/**
 * Move to a point that is relative to the current effector location
 *
 * @param req The request for this service as defined in MoveToRelativePoint.srv 
 * @param res The response for this service as defined in MoveToRelativePoint.srv
 * 
 * @return true
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint(deltaRobotNode::MoveToRelativePoint::Request &req,
	deltaRobotNode::MoveToRelativePoint::Response &res) {
	ROS_INFO("moveToRelativePoint called");

	if(currentState != rosMast::normal) {
		return false;
	}

	deltaRobotNode::Motion currentMotion;
	try {
		currentMotion = req.motion;
		DataTypes::Point3D<double>& effectorLocation = deltaRobot->getEffectorLocation();
		ROS_INFO("Current effector location: x: %f y: %f z: %f", effectorLocation.x, effectorLocation.y, effectorLocation.z);
		double relativeX = effectorLocation.x + currentMotion.x;
		double relativeY = effectorLocation.y + currentMotion.y;
		double relativeZ = effectorLocation.z + currentMotion.z;
		ROS_INFO("Current motion z: %f", currentMotion.z);

		if(!deltaRobot->checkPath(
				DataTypes::Point3D<double>(effectorLocation.x, effectorLocation.y, effectorLocation.z),
				DataTypes::Point3D<double>(relativeX, relativeY, relativeZ)))
		{
			res.succeeded = false;
			return res.succeeded;
		}
		deltaRobot->moveTo(DataTypes::Point3D<double>(relativeX, relativeY, relativeZ), currentMotion.speed);
		//deltaRobot->waitForReady();

	} catch(std::runtime_error& ex) {
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		res.succeeded = false;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
	}

    res.succeeded = true;
	return res.succeeded;
}

/**
 * Move to a number of relative points. 
 *
 * @param req The request for this service as defined in MoveRelativePath.srv 
 * @param res The response for this service as defined in MoveRelativePath.srv
 *
 * @return true if path is allowed else return false.
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveRelativePath(deltaRobotNode::MoveRelativePath::Request &req,
	deltaRobotNode::MoveRelativePath::Response &res) {
	ROS_INFO("moveRelativePath called");

	if(currentState != rosMast::normal) {
		return false;
	}

	deltaRobotNode::Motion currentMotion;
	double relativeX;
	double relativeY;
	double relativeZ;
	DataTypes::Point3D<double> effectorLocation;
	try
	{
		effectorLocation = deltaRobot->getEffectorLocation();
		unsigned int n;
		for(n = 0; n < req.motion.size(); n++)
		{
			currentMotion = req.motion[n];			
			relativeX = effectorLocation.x + currentMotion.x;
			relativeY = effectorLocation.y + currentMotion.y;
			relativeZ = effectorLocation.z + currentMotion.z;
			if(!deltaRobot->checkPath(
				DataTypes::Point3D<double>(effectorLocation.x, effectorLocation.y, effectorLocation.z),
				DataTypes::Point3D<double>(relativeX, relativeY, relativeZ)))
			{
				res.succeeded = false;
				ROS_INFO("FROM %f, %f, %f TO %f, %f, %f Not allowed",effectorLocation.x,effectorLocation.z,effectorLocation.y,relativeX,relativeY,relativeZ );
				return res.succeeded;
			}
			effectorLocation.x = relativeX;
			effectorLocation.y = relativeY;
			effectorLocation.z = relativeZ;
		}
		for(n = 0; n < req.motion.size(); n++)
		{	
			currentMotion = req.motion[n];			
			effectorLocation = deltaRobot->getEffectorLocation();
			relativeX = effectorLocation.x + currentMotion.x;
			relativeY = effectorLocation.y + currentMotion.y;
			relativeZ = effectorLocation.z + currentMotion.z;
			ROS_INFO("moveTo: (%f, %f, %f) speed=%f", relativeX, relativeY,relativeZ, currentMotion.speed);
			deltaRobot->moveTo(DataTypes::Point3D<double>(relativeX, relativeY, relativeZ),currentMotion.speed);
		}
		//deltaRobot->waitForReady();
	}
	catch(std::runtime_error& ex)
	{
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		res.succeeded = false;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
	}
	res.succeeded = true;
	return res.succeeded;
	
}
/**
 * Transition from Safe to Standby state

 @return 0 if everything went OK else error
**/
int deltaRobotNodeNamespace::DeltaRobotNode::transitionSetup() {
	setState(rosMast::setup);

	ROS_INFO("Setup transition called");	
	
    // Generate the effector boundaries with voxel size 2
    deltaRobot->generateBoundaries(2);

	// Power on the deltarobot and calibrate the motors.
    deltaRobot->powerOn();

    // Calibrate the motors
    if(!deltaRobot->calibrateMotors()){
    	ROS_ERROR("Calibration FAILED. EXITING.");
    	return 1;
    } 
	return 0; 
}

int deltaRobotNodeNamespace::DeltaRobotNode::transitionShutdown() {
	
	ROS_INFO("Shutdown transition called");
	
	setState(rosMast::shutdown);
	deltaRobot->powerOff();
	return 0;
}

int deltaRobotNodeNamespace::DeltaRobotNode::transitionStart() {
	setState(rosMast::start);
	
	ROS_INFO("Start transition called");
    
    // Calibrate the motors
    if(!deltaRobot->calibrateMotors()){
    	ROS_ERROR("Calibration FAILED. EXITING.");
    	return 1;
    }	
	return 0;
}

int deltaRobotNodeNamespace::DeltaRobotNode::transitionStop() {
	ROS_INFO("Stop transition called");
	
	setState(rosMast::stop);

	return 0;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, NODE_NAME);

	int equipletID = atoi(argv[1]);
	int moduleID = atoi(argv[2]);

	deltaRobotNodeNamespace::DeltaRobotNode drn(equipletID, moduleID);    

	ROS_INFO("DeltaRobotNode ready..."); 	
	return 0;
}


