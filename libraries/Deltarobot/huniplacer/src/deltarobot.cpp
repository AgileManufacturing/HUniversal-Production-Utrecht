//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        deltarobot.cpp
// File:           symbolizes an entire deltarobot
// Description:    Lukas Vermond & Kasper van Nieuwland
// Author:         -
// Notes:          
//
// License:        newBSD
//
// Copyright © 2012, HU University of Applied Sciences Utrecht
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of the HU University of Applied Sciences Utrecht nor the
// names of its contributors may be used to endorse or promote products
// derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************


#include <sstream>
#include <iostream>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <cmath>

#include <huniplacer/Point3D.h>
#include <huniplacer/imotor3.h>
#include <huniplacer/motor3_exception.h>
#include <huniplacer/effector_boundaries.h>
#include <huniplacer/InverseKinematicsException.h>
#include <huniplacer/deltarobot.h>
#define square(x) ((x)*(x))

/**
 * deltarobot.cpp -> This class symbolises an entire deltarobot
 */

namespace huniplacer
{
    /**
     * Constructor
     * @param kinematics kinematics model that will be used to convert points to motions
     * @param motors implementation of motor interface that will be used to communicate with the motors
     */
    deltarobot::deltarobot(InverseKinematicsModel& kinematics, steppermotor3& motors) :
        kinematics(kinematics),
        motors(motors),
        effector_location(Point3D(0, 0, -161.9)),
        boundaries_generated(false)
    {
    }

    deltarobot::~deltarobot(void)
    {
    	if(motors.is_powerd_on())
    	{
    		motors.stop();
    	}
    }
    
    void deltarobot::generate_boundaries(double voxel_size){
    	boundaries = effector_boundaries::generate_effector_boundaries(kinematics, motors, voxel_size);
    	boundaries_generated = true;
    }

    bool deltarobot::is_valid_angle(double angle)
    {
        return angle > motors.get_min_angle() && angle < motors.get_max_angle();
    }

    /**
     * Checks the path between two points
     * @param begin start point
     * @param end finish point
     */
    bool deltarobot::check_path(const Point3D& begin,const Point3D& end)
    {
    	return boundaries->check_path(begin, end);
    }

    /**
     * Makes the deltarobot move to a point
     * @param p 3-dimensional point to move to
     * @param speed movement speed in millimeters per second
     * @param async motions will be stored in a queue for later execution if true
     */
    void deltarobot::moveto(const Point3D& p, double speed, bool async)
    {
    	if(!motors.is_powerd_on())
    	{
    		throw motor3_exception("motor drivers are not powered on");
    	}
        motionf mf;
        try
        {
        	kinematics.pointToMotion(p, mf);
        }
        catch(InverseKinematicsException& ex)
        {
        	throw ex;
        }
        if(
            !is_valid_angle(mf.angles[0]) ||
            !is_valid_angle(mf.angles[1]) ||
            !is_valid_angle(mf.angles[2]))
        {
            throw InverseKinematicsException("motion angles outside of valid range", p);
        }

    	if(!boundaries->check_path(effector_location, p))
    	{
    		throw InverseKinematicsException("invalid path", p);
    	}
        double move_time = p.distance(effector_location) / speed;

        try
        {
        	motors.moveto_within(mf, move_time, async);
        }
        catch(std::out_of_range& ex) { throw ex; }


        effector_location = p;
    }

    /**
    * Reads calibration sensor and returns whether it is hit.
    * @param modbus The TCP modbus connection for IO controller.
    * @param sensorIndex index of the sensor. This corresponds to the motor index.
    * @returns True if sensor is hit, false otherwise.
    */
    bool deltarobot::checkSensor(modbus_t* modbus, int sensorIndex){
        // The modbus library only reads
        uint16_t sensorRegister;
        int result;

        // Read register 8000 -- this register contains the values of the input sensors.
        result = modbus_read_registers(modbus, 8000, 1, &sensorRegister);
        if (result == -1) {
            throw std::runtime_error(modbus_strerror(errno));
        }
        return (sensorRegister ^ 7) & 1 << sensorIndex;
    }

    /**
    * @brief Calibrates a single motor by moving the motor upwards till the calibration sensor is pushed.
    * @param modbus The TCP modbus connection for IO controller.
    * @param motors The StepperMotor class controlling the 3 deltarobot motors.
    * @param motorIndex Index of the motor to be calibrated. When standing in front of the robot looking towards it, 0 is the right motor, 1 is the front motor and 2 is the left motor.
    */
    void deltarobot::calibrateMotor(modbus_t* modbus, int motorIndex){
        std::cout << "Calibrating motor number " << motorIndex << std::endl;
        
        // Starting point of calibration
        double angle = 0;

        // Move motor upwards till the calibration sensor is pushed
        do {
            angle -= huniplacer::utils::rad(crd514_kd::MOTOR_STEP_IN_DEGREES);
            motors.moveSingleMotor(motorIndex, angle);
            
            usleep(25000);
        } while(!checkSensor(modbus, motorIndex));

        //
        angle += huniplacer::measures::MOTORS_DEVIATION;
        motors.moveSingleMotor(motorIndex, angle);
        motors.resetCounter(motorIndex);
        motors.moveSingleMotor(motorIndex, 0);
    }

    /**
    * @brief Calibrates all three motors of the deltarobot by moving the motors upwards one by one.
    * After a motor is moved upwards, it is moved back to the 0 degrees state.
    * This function temporarily removes the limitations for the motorcontrollers.
    * @param modbus The TCP modbus connection for IO controller.
    * @param motors The steppermotor3 class controlling the 3 deltarobot motors.
    */
    void deltarobot::calibrateMotors(modbus_t* modbus){
        // Disable limitations
        motors.disableControllerLimitations();
        
        // Calibrate motors
        calibrateMotor(modbus, 0);
        calibrateMotor(modbus, 1);
        calibrateMotor(modbus, 2);

        // Set limitations
        motors.setMotorLimits(huniplacer::measures::MOTOR_ROT_MIN, huniplacer::measures::MOTOR_ROT_MAX);
        
        // Set deviation to 0 for the 3 motors
        double deviation[3] = {0,0,0};
        motors.set_deviation(deviation);
        effector_location.x = 0;
        effector_location.y = 0;
        effector_location.z = -sqrt(square(huniplacer::measures::ANKLE) - square(huniplacer::measures::BASE+huniplacer::measures::HIP-huniplacer::measures::EFFECTOR));
        std::cout << "effector location z: " << effector_location.z << std::endl; 
    }

    /**
     * Stops the motors
     */    
    void deltarobot::stop(void)
    {
    	if(!motors.is_powerd_on())
		{
			throw motor3_exception("motor drivers are not powered on");
		}
        motors.stop();
    }
    
    /**
     * Wait for the deltarobot to become idle
     *
     * The deltarobot is idle when all it's motions are completed
     * and it has stopped moving
     *
     * @param timeout time in milliseconds for the wait to timeout. 0 means infinite
     */    
    bool deltarobot::wait_for_idle(long timeout)
    {
    	if(motors.is_powerd_on())
    	{
    		return motors.wait_for_idle(timeout);
    	}
    	return true;
    }

    /**
     * True if the deltarobot is idle, false otherwise
     **/    
    bool deltarobot::is_idle(void)
    {
        return motors.is_idle();
    }

    /**
     * Shuts down the deltarobot's hardware
     */
    void huniplacer::deltarobot::power_off(void)
    {
        if(motors.is_powerd_on())
        {
        	motors.power_off();
        }
    }

    /**
     * Turns on the deltarobot's hardware
     */
    void huniplacer::deltarobot::power_on(void)
    {
        if(!motors.is_powerd_on())
        {
        	motors.power_on();
        }
    }

    Point3D& deltarobot::getEffectorLocation() {
        return effector_location;
    }
}
#undef square
