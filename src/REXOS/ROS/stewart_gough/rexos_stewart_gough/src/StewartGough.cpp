/**
 * @file DeltaRobot.cpp
 * @brief Symbolizes an entire deltarobot.
 *
 * @author Garik Hakopian
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

#include <sstream>
#include <iostream>
#include <string>
#include <cstdio>
#include <stdlib.h> 
#include <stdexcept>
#include <cmath>

#include <rexos_stewart_gough/StewartGough.h>
#include <rexos_motor/MotorException.h>
#include <rexos_motor/SimulatedMotor.h>
#include <rexos_motor/StepperMotor.h>
#include <rexos_motor/SimulatedMotorManager.h>
#include <rexos_motor/StepperMotorManager.h>
#include <rexos_sensor/SimulatedContactSensor.h>
#include <rexos_sensor/ContactSensor.h>
#include <rexos_utilities/Utilities.h>

#include "ros/ros.h"

#include "rexos_stewart_gough/SixAxisCalculations.h"

namespace rexos_stewart_gough{
	void StewartGough::createModbus() {
		REXOS_INFO("Creating modbus");
		modbus = new rexos_modbus::ModbusController(modbus_new_rtu(
			"/dev/ttyS0",
			rexos_motor::CRD514KD::RtuConfig::BAUDRATE,
			rexos_motor::CRD514KD::RtuConfig::PARITY,
			rexos_motor::CRD514KD::RtuConfig::DATA_BITS,
			rexos_motor::CRD514KD::RtuConfig::STOP_BITS));
	}
	
	void StewartGough::createMotorManager() {
		REXOS_INFO_STREAM("Creating motor manager " << &modbus);
		if(isSimulated == false) {
			createModbus();
		}
		
		// populate motors
		if(isSimulated == true) {
			motors.push_back(new rexos_motor::SimulatedMotor(equipletName, identifier, 0, stepperMotorProperties));
			motors.push_back(new rexos_motor::SimulatedMotor(equipletName, identifier, 1, stepperMotorProperties));
			motors.push_back(new rexos_motor::SimulatedMotor(equipletName, identifier, 2, stepperMotorProperties));
			motors.push_back(new rexos_motor::SimulatedMotor(equipletName, identifier, 3, stepperMotorProperties));
			motors.push_back(new rexos_motor::SimulatedMotor(equipletName, identifier, 4, stepperMotorProperties));
			motors.push_back(new rexos_motor::SimulatedMotor(equipletName, identifier, 5, stepperMotorProperties));
		} else {
			motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_0, stepperMotorProperties));
			motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_1, stepperMotorProperties));
			motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_2, stepperMotorProperties));
			motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_3, stepperMotorProperties));
			motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_4, stepperMotorProperties));
			motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_5, stepperMotorProperties));
		}
		
		REXOS_INFO("part 2");
		motorMap[0] = MotorMap(0,3);
		motorMap[1] = MotorMap(1,2);
		motorMap[2] = MotorMap(2,1);
		motorMap[3] = MotorMap(3,0);
		motorMap[4] = MotorMap(4,4);
		motorMap[5] = MotorMap(5,5);
		
		if(isSimulated == true) {
			motorManager = new rexos_motor::SimulatedMotorManager(equipletName, identifier, motors);
		} else {
			motorManager = new rexos_motor::StepperMotorManager(modbus, motors);
		}
	}
	
	
    /**
     * Constructor of a sixaxis robot.
     * 
     * @param steqartGoughMeasures The measures of the steqart gough configuration in use.
     * @param motorManager The manager that allows all motors to be simultaneously activated.
     * @param motors The motor array with the three motor objects.
     * @param modbusIO The TCP modbus connection for the IO controller.
     **/
    StewartGough::StewartGough(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, bool isSimulated, Json::Value node) :
			equipletName(equipletName), identifier(identifier), isSimulated(isSimulated), 
			effectorLocation(), motors(), currentMotionSlot(1), modbus(NULL), 
			stewartGoughMeasures(node["stewartGoughMeasures"]), stepperMotorProperties(node["stepperMotorProperties"]),
			motorManager(NULL) {
		REXOS_INFO("StewartGough constructor entering...");
		readJSONNode(node);
		
		REXOS_INFO("Configuring motors");
		createMotorManager();
		
		REXOS_INFO("Configuring sensors");
		if(isSimulated == true) {
			sensors.push_back(new rexos_sensor::SimulatedContactSensor(equipletName, identifier, 0));
			sensors.push_back(new rexos_sensor::SimulatedContactSensor(equipletName, identifier, 1));
			sensors.push_back(new rexos_sensor::SimulatedContactSensor(equipletName, identifier, 2));
			sensors.push_back(new rexos_sensor::SimulatedContactSensor(equipletName, identifier, 3));
			sensors.push_back(new rexos_sensor::SimulatedContactSensor(equipletName, identifier, 4));
			sensors.push_back(new rexos_sensor::SimulatedContactSensor(equipletName, identifier, 5));
		} else {
			// Initialize modbus for IO controller
			modbusIO = modbus_new_tcp(modbusIp.c_str(), modbusPort);
			if(modbusIO == NULL){
				throw std::runtime_error("Unable to allocate libmodbus context");
			}
			if(modbus_connect(modbusIO) == -1) {
				throw std::runtime_error("Modbus connection to IO controller failed");
			}
			
			sensors.push_back(new rexos_sensor::ContactSensor(0, modbusIO));
			sensors.push_back(new rexos_sensor::ContactSensor(1, modbusIO));
			sensors.push_back(new rexos_sensor::ContactSensor(2, modbusIO));
			sensors.push_back(new rexos_sensor::ContactSensor(3, modbusIO));
			sensors.push_back(new rexos_sensor::ContactSensor(4, modbusIO));
			sensors.push_back(new rexos_sensor::ContactSensor(5, modbusIO));
		}
		
		sixAxisCalculations = new SixAxisCalculations(
			100, 300.00,
			101.3, 40.93,
			31.05, 32.59,
			0.26
			);

		REXOS_INFO_STREAM(" max angle: " << stewartGoughMeasures.maxAngleHipAnkle << std::endl); 
	

       // kinematics = new InverseKinematics;
		
		REXOS_INFO("end of constructor reached");
    }

    /**
     * Deconstructor of a stewart gough. Turns off the motors and deletes the kinematics model.
     **/
    StewartGough::~StewartGough(void){
        if(motorManager->isPoweredOn()){
            motorManager->powerOff();
        }
    }
    
	void StewartGough::readJSONNode(const Json::Value node) {
		modbusIp = node["modbusIp"].asString();
		REXOS_INFO_STREAM("found modbusIp " << modbusIp);
		
		modbusPort = node["modbusPort"].asInt();
		REXOS_INFO_STREAM("found modbusPort " << modbusPort);
		
		calibrationBigStepFactor = node["calibrationBigStepFactor"].asInt();
		REXOS_INFO_STREAM("found calibrationBigStepFactor " << calibrationBigStepFactor);
	}
	
    /**
     * Checks the validity of an angle for a motor.
     *
     * @param motorIndex The index for the motor from 0 to (amount of motors - 1) to be checked for.
     * @param angle The angle in degrees where 0 degrees is directly opposite of the center of the imaginary circle for the engines.
     *
     * @return If the angle is valid for the motor.
     **/
    bool StewartGough::isValidAngle(int motorIndex, double angle) {
        assert(motorIndex >= 0 && motorIndex < 6);
        return angle > motors[motorIndex]->getMinAngle() && angle < motors[motorIndex]->getMaxAngle();
    }

	/**
     * Checks the path between two points.
     * 
     * @param begin The starting point.
     * @param end The end point.
     * 
     * @return if the path between two points is valid.
     **/
	bool StewartGough::checkPath(const StewartGoughLocation& begin, const StewartGoughLocation& end) {
		return sixAxisCalculations->checkPath(begin, end);
	}
			
	
	

    /**
     * Gets the acceleration in radians/s² for a motor rotation with a certain relative angle and time, which is half acceleration and half deceleration (there is no period of constant speed).
     * 
     * @param moveTime the move time.
     * @param relativeAngle The relative angle
     *
     * @return the acceleration in radians/s²
     **/
    double StewartGough::getAccelerationForRotation(double relativeAngle, double moveTime){
        return (4 * fabs(relativeAngle)) / (moveTime * moveTime);
    }

    /**
     * Gets the top speed in radians/s for a motor rotation with a certain relative angle, time and acceleration.
     * 
     * @param relativeAngle The relative angle
     * @param moveTime the move time.
     * @param acceleration the acceleration
     *
     * @return the speed in radians/s
     **/
    double StewartGough::getSpeedForRotation(double relativeAngle, double moveTime, double acceleration){
        return (acceleration/2) * (moveTime - sqrt((moveTime * moveTime) - (4 * fabs(relativeAngle) / acceleration)));
    }


	void StewartGough::moveTo(StewartGoughLocation point, double maxAcceleration){
		// check whether the motors are powered on.
		if(!motorManager->isPoweredOn()){
			throw rexos_motor::MotorException("motor drivers are not powered on");
		}

		//check if the requested location is the effectors current location,
		//if so the method can be cut short.
        if(effectorLocation == point) {
			return;
        }

        if(maxAcceleration > stepperMotorProperties.maxAcceleration) {
            // The acceleration is too high, putting it down to the maximum CRD514KD acceleration.
            maxAcceleration = stepperMotorProperties.maxAcceleration;
        } else if(maxAcceleration < stepperMotorProperties.minAcceleration) {
            // The acceleration is too low, throwing an exception.
            throw std::out_of_range("maxAcceleration too low");            
        }

        // Get the motor angles from the kinematics model
		SixAxisCalculations::EffectorMove effectorMove = sixAxisCalculations->getMotorAngles(point);
		std::vector<rexos_motor::MotorRotation> rotations;
		for(int i = 0; i < motors.size(); i++){
			rexos_motor::MotorRotation rotation;
			//Swap 4 and 5
			if(i == 4){
				rotation.angle = effectorMove.angles[5];
			} else if(i == 5){
				rotation.angle = effectorMove.angles[4];
			} else {
				rotation.angle = effectorMove.angles[i];
			}
			rotations.push_back(rotation);
		}

        // Check if the angles fit within the boundaries
		for(int i = 0; i < motors.size(); i++) {
			if(rotations[i].angle < stepperMotorProperties.motorMinAngle || rotations[i].angle < stepperMotorProperties.motorMaxAngle) {
				throw std::out_of_range("motion angles outside of valid range");
			}
		}
		
        // Check if the path fits within the boundaries
        if(!checkPath(effectorLocation, point)) {
            throw std::out_of_range("invalid path");
        }
		
		// An array to hold the relative angles for the motors
		std::vector<bool> relativeAngles;
		for(int i = 0; i < motors.size(); i++) {
			relativeAngles.push_back(0.0);
		}
		
		// An array that indicates for each motor whether it moves in this motion or not.
		std::vector<bool> motorIsMoved;
		for(int i = 0; i < motors.size(); i++) {
			motorIsMoved.push_back(true);
		}
		
		// Index for the motor with the biggest motion
		int motorWithBiggestMotion = 0;

		for(int i = 0; i < motors.size(); i++) {
			relativeAngles[i] = fabs(rotations[i].angle - motors[i]->getCurrentAngle());
			if (relativeAngles[i] > relativeAngles[motorWithBiggestMotion]){
				motorWithBiggestMotion = i;
			}

			if(relativeAngles[i] < stepperMotorProperties.microStepAngle){
				// motor does not have to move at all
				motorIsMoved[i] = false;
			}
		}
		
		// Check if any motor has to move at all
		bool anyMotorIsMoved = false;
		for(int i = 0; i < motors.size(); i++) {
			if(motorIsMoved[i] == true) {
				anyMotorIsMoved = true;
			}
		}
		if(anyMotorIsMoved == false) {
			// none of the motors have to move, method can be cut short
			return;
		}

		// switch currentMotionSlot
		/*currentMotionSlot++;
		if(currentMotionSlot > rexos_motor::CRD514KD::MOTION_SLOTS_USED){
			currentMotionSlot = 1;
		}*/

		// Set the acceleration of the motor with the biggest motion to the given maximum.
		rotations[motorWithBiggestMotion].acceleration = maxAcceleration;
		rotations[motorWithBiggestMotion].deceleration = maxAcceleration;

		// Calculate the time the motion will take, based on the assumption that the motion is two-phase (half acceleration and half deceleration).
		// TODO: Take the motor's maximum speed into account.
		double moveTime;

		if(sqrt(relativeAngles[motorWithBiggestMotion] * rotations[motorWithBiggestMotion].acceleration) > stepperMotorProperties.maxSpeed) {
			// In case of a two-phase motion, the top speed would come out above the motor's maximum, so a three-phase motion must be made.
			rotations[motorWithBiggestMotion].speed = stepperMotorProperties.maxSpeed;
			moveTime = (relativeAngles[motorWithBiggestMotion] / rotations[motorWithBiggestMotion].speed) + 
					(rotations[motorWithBiggestMotion].speed / rotations[motorWithBiggestMotion].acceleration);  
		} else {
			// The motion is fine as a two-phase motion.
			moveTime = 2 * sqrt(relativeAngles[motorWithBiggestMotion] / rotations[motorWithBiggestMotion].acceleration);
		}
		
		// Set speed, and also the acceleration for the smaller motion motors
		for(int i = 0; i < motors.size(); i++) {
			rotations[i].speed = stepperMotorProperties.maxSpeed;

			if(i != motorWithBiggestMotion) {
				if(motorIsMoved[i]) {
					rotations[i].acceleration = getAccelerationForRotation(relativeAngles[i], moveTime);
					rotations[i].deceleration = rotations[i].acceleration;  
					if(rotations[i].acceleration < stepperMotorProperties.minAcceleration) {
						// The acceleration comes out too low, this means the motion cannot be half acceleration and half deceleration (without a consant speed phase).
						// To make it comply with the move time, as well as the minimum acceleration requirements, we have to add a top speed.
						rotations[i].acceleration = stepperMotorProperties.minAcceleration;
						rotations[i].deceleration = stepperMotorProperties.minAcceleration;
						rotations[i].speed = getSpeedForRotation(relativeAngles[i], moveTime, rotations[i].acceleration);
					} else if(rotations[i].acceleration > stepperMotorProperties.maxAcceleration) {
						throw std::out_of_range("acceleration too high");
					}
				} else {
					rotations[i].acceleration = stepperMotorProperties.minAcceleration;
					rotations[i].deceleration = stepperMotorProperties.minAcceleration;
					rotations[i].angle = motors[i]->getCurrentAngle();
				}
			}
			motors[i]->writeRotationData(rotations[i]);
		}

		motorManager->startMovement();
		
        effectorLocation = point;
    }

    /**
    * Reads calibration sensor and returns whether it is hit.
    * 
    * @param sensorIndex Index of the sensor. This corresponds to the motor index.
    * 
    * @return true if sensor is hit, false otherwise.
    **/
    bool StewartGough::checkSensor(int sensorIndex){
		return sensors[sensorIndex]->isTriggered();
    }

    /**
     * Incrementally moves the motor until the sensor is of the value given in sensorValue. 
     * Calculates how many steps are made, and returns this as an int. 
     *
     * @param motorIndex The index of the motor
     * @param motorRotation The MotorRotation object that holds that the rotation data.
     * @param sensorValue value that the sensor needs to be for the motor to stop.
     *
     * @return The amount of motor steps the motor has moved.
     **/
    MotorGroup StewartGough::moveMotorUntilSensorIsOfValue(int motorIndex1, int motorIndex2, rexos_motor::MotorRotation motorRotation1 ,rexos_motor::MotorRotation motorRotation2, bool sensorValue){
        motors[motorIndex1]->writeRotationData(motorRotation1, false);
		motors[motorIndex2]->writeRotationData(motorRotation2, false);
		bool done1 = false;
		bool done2 = false;
        MotorGroup motorGroupSteps(0,0);
		
		while(1){
			done1 |= (checkSensor(motorMap[motorIndex1].sensor) == sensorValue);
			done2 |= (checkSensor(motorMap[motorIndex2].sensor) == sensorValue);
			
			if(!done1){
				motors[motorIndex1]->startMovement();
				motorGroupSteps.motorIndex1 += (motorRotation1.angle / stepperMotorProperties.microStepAngle); 
			}
			
			if(!done2) {
				motors[motorIndex2]->startMovement();
				motorGroupSteps.motorIndex2 += (motorRotation2.angle / stepperMotorProperties.microStepAngle);
			}
			
			if(done1 == true && done2 == true){
				return motorGroupSteps;
			}
		}
		return motorGroupSteps;
    }

    /**
    * Calibrates a single motor by:
    * -# Moving it to the sensor in big steps until the sensor is pushed
    * -# Moving it away from the sensor in big steps until the sensor is no longer pushed
    * -# Moving back to the sensor in small steps until the sensor is pushed.
    * -# Using the moved steps to calculate the deviation
    * 
    * @param motorIndex Index of the motor to be calibrated. When standing in front of the robot looking towards it, 0 is the right motor, 1 is the front motor and 2 is the left motor.
    **/
    void StewartGough::calibrateMotor(int motorIndex1, int motorIndex2){
        // Setup for incremental motion in big steps, to get to the sensor quickly.
        motors[motorIndex1]->setRelativeMode();
		motors[motorIndex2]->setRelativeMode();
        rexos_motor::MotorRotation motorRotation1;
		rexos_motor::MotorRotation motorRotation2;
        motorRotation1.angle = stepperMotorProperties.microStepAngle * calibrationBigStepFactor;
		motorRotation2.angle = stepperMotorProperties.microStepAngle * calibrationBigStepFactor;
        
        // Move to the sensor in large steps until it is pushed
        // actualAngleInSteps keeps track of how many motor steps the motor has moved. This is necessary to avoid accummulating errors.
        MotorGroup actualAnglesInTicks = moveMotorUntilSensorIsOfValue(motorIndex1, motorIndex2, motorRotation1,motorRotation2, true);
		
		int actualAngleInTicks1 = actualAnglesInTicks.motorIndex1;
		int actualAngleInTicks2 = actualAnglesInTicks.motorIndex2;
		
        // Move away from the sensor in big steps until it is no longer pushed.
        motorRotation1.angle = -motorRotation1.angle;
		motorRotation2.angle = -motorRotation2.angle;
        actualAnglesInTicks = moveMotorUntilSensorIsOfValue(motorIndex1,motorIndex2, motorRotation1,motorRotation2, false);
		actualAngleInTicks1 += actualAnglesInTicks.motorIndex1;
		actualAngleInTicks2 += actualAnglesInTicks.motorIndex2;

        // Move back to the sensor in small steps until it is pushed.
        motorRotation1.angle = stepperMotorProperties.microStepAngle;
		motorRotation2.angle = stepperMotorProperties.microStepAngle;
        actualAnglesInTicks = moveMotorUntilSensorIsOfValue(motorIndex1,motorIndex2, motorRotation1,motorRotation2, true);
		actualAngleInTicks1 += actualAnglesInTicks.motorIndex1;
		actualAngleInTicks2 += actualAnglesInTicks.motorIndex2;
		
		
		
		//std::cout << "actual steps: " << std::endl;
		//std::cout << actualAngleInSteps1 << std::endl;
		//calculate and set the deviation.
		//std::cout << stewartGoughMeasures->motorFromZeroToTopAngle << std::endl;
        double deviation1 = (actualAngleInTicks1 * stepperMotorProperties.microStepAngle) + stewartGoughMeasures.motorFromZeroToTopAngle;
        double deviation2 = (actualAngleInTicks2 * stepperMotorProperties.microStepAngle) + stewartGoughMeasures.motorFromZeroToTopAngle;
		
		//std::cout << "passed 1" << std::endl;
		motors[motorIndex1]->setDeviation(deviation1);
		motors[motorIndex2]->setDeviation(deviation2);
        // Move back to the new 0.
		
        motors[motorIndex1]->setAbsoluteMode();
		motors[motorIndex2]->setAbsoluteMode();
        motorRotation1.angle = 0;
		motorRotation2.angle = 0;
        motors[motorIndex1]->moveTo(motorRotation1);
		motors[motorIndex2]->moveTo(motorRotation2);
		
        motors[motorIndex1]->waitTillReady();
		motors[motorIndex2]->waitTillReady();
    }

    /**
    * Calibrates all six motors of the deltarobot by moving the motors upwards one by one.
    * After a motor is moved upwards, it is moved back to the 0 degrees state.
    * This function temporarily removes the limitations for the motorcontrollers.
    * 
    * @return true if the calibration was succesful. False otherwise (e.g. failure on sensors.)
    **/
    bool StewartGough::calibrateMotors(){
		REXOS_INFO("Start motor calibration");
		
		for(int i = 0; i < 6; i++){
			if(checkSensor(i)){
				REXOS_ERROR_STREAM("Sensor " << i << "failure (is the hardware connected?)");
				return false;
			}
		}

		for(int i = 0; i < 6; i = i + 2) {
			// Calibrate motors
			calibrateMotor(getMotorIndexByNumber(0 + i), getMotorIndexByNumber(1 + i));
			// Enable angle limitations
			getMotor(0 + i)->enableAngleLimitations();
			getMotor(1 + i)->enableAngleLimitations();
			
			REXOS_INFO_STREAM("Motors " << (0 + i) << " and " << (1 + i) << " calibrated");
		}
	
		
		
        effectorLocation.location.x = 0;
        effectorLocation.location.y = 0;
        effectorLocation.location.z = -270; // yet to be set 
		
		effectorLocation.rotationX = 0;
		effectorLocation.rotationY = 0;
		effectorLocation.rotationZ = 0;
        REXOS_DEBUG_STREAM("effector location z: " << effectorLocation.location.z << std::endl); 
		
        return true;
    }

    /**
     * Shuts down the stewart gough's hardware.
     **/
    void StewartGough::powerOff(void){
        if(motorManager->isPoweredOn()){
            motorManager->powerOff();
        }
    }

    /**
     * Turns on the stewart gough's hardware.
     **/
    void StewartGough::powerOn(void){
        if(!motorManager->isPoweredOn()){
            motorManager->powerOn();
        }
    }

    /**
     * Get the location of the midpoint of the effector.
     *
     * @return The coordinate for the midpoint of the effector.
     **/
    StewartGoughLocation StewartGough::getEffectorLocation(){
        return effectorLocation;
    }
	
	rexos_motor::MotorInterface* StewartGough::getMotor(int number){
		return motors[getMotorIndexByNumber(number)];
	}
	
	int StewartGough::getMotorIndexByNumber(int number){
		return motorMap[number].motor;
	}
	
	void StewartGough::deleteMotorRotationObjects(rexos_motor::MotorRotation* rotations[6]){
		for(int i = 0; i < 6; i++){
			delete rotations[i];
		}
	}
}
