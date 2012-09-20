//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        steppermotor3.cpp
// File:           steppermotor driver
// Description:    Lukas Vermond & Kasper van Nieuwland
// Author:         -
// Notes:          
//
// License:        GNU GPL v3
//
// This file is part of steppermotor3.cpp.
//
// steppermotor3.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// steppermotor3.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with steppermotor3.cpp.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#include <huniplacer/steppermotor3.h>

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <huniplacer/utils.h>
#include <huniplacer/CRD514_KD.h>
#include <huniplacer/crd514_kd_exception.h>
#include <huniplacer/motor3_exception.h>

namespace huniplacer
{
    steppermotor3::steppermotor3(modbus_t* context, double min_angle, double max_angle, motion_thread_exception_handler exhandler, const double* deviation) :
        imotor3(),
        motion_queue(),
        thread_running(true),
        idle(true),
        idle_mutex(), idle_cond(),
        queue_mutex(),
        modbus_mutex(),
        min_angle(min_angle), max_angle(max_angle),
        modbus(context),
        exhandler(exhandler),
        powered_on(false)
    {
    	//set deviation
    	this->deviation[0] = deviation[0];
    	this->deviation[1] = deviation[1];
    	this->deviation[2] = deviation[2];

        //start motion thread
        motion_thread = new boost::thread(motion_thread_func, this);
    }

    steppermotor3::~steppermotor3(void)
    {
        thread_running = false;
        motion_thread->interrupt();

        try
        {
        	stop();
        }
        catch(std::runtime_error& err)
        {
        }

        idle_cond.notify_all(); //its destructor will fail if threads are still waiting
        motion_thread->join();
        delete motion_thread;

        if(powered_on)
        {
        	wait_till_ready();
        	modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, 0);
        }
    }

    bool steppermotor3::is_idle(void)
    {
    	if(!powered_on && idle)
        {
        	boost::lock_guard<boost::mutex> lock(modbus_mutex);
        	return
				(modbus.read_u16(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::STATUS_1) & crd514_kd::status1_bits::READY) &&
				(modbus.read_u16(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::STATUS_1) & crd514_kd::status1_bits::READY) &&
				(modbus.read_u16(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::STATUS_1) & crd514_kd::status1_bits::READY);
        }
        return false;
    }

    void steppermotor3::motion_thread_func(steppermotor3* owner)
    {
    	using namespace utils;

        try
        {
            while(owner->thread_running)
            {
                owner->queue_mutex.lock();
                
                if(!owner->motion_queue.empty())
                {
                    //get motion, convert and pop
                    motionf& mf = owner->motion_queue.front();
                    printf("angles: %lf, %lf, %lf\n", mf.angles[0], mf.angles[1], mf.angles[2]);
                    fflush(stdout);
                    motioni mi;
                    owner->motion_float_to_int(mi, mf);
                    /*printf("%d, %d, %d\n, %d %d, %d\n %d, %d %d\n %d, %d, %d\n",
                    		mi.acceleration[0], mi.acceleration[1], mi.acceleration[2],
                    		mi.deceleration[0], mi.deceleration[1], mi.deceleration[2],
                    		mi.angles[0], mi.angles[1], mi.angles[2],
                    		mi.speed[0], mi.speed[1], mi.speed[2]
                    );*/
                    if(mi.speed[0] == 0)
                    	mi.speed[0] = 1;

                    if(mi.speed[1] == 0)
                    	mi.speed[1] = 1;

                    if(mi.speed[2] == 0)
                    	mi.speed[2] = 1;


                    owner->motion_queue.pop();
                    
                    owner->queue_mutex.unlock();
                    
                    if(owner->powered_on)
                    {

						//write motion
						boost::lock_guard<boost::mutex> lock(owner->modbus_mutex);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::OP_SPEED, mi.speed[0], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::OP_POS, mi.angles[0], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::OP_ACC, mi.acceleration[0], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::OP_DEC, mi.deceleration[0], true);

						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::OP_SPEED, mi.speed[1], false);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::OP_POS, mi.angles[1], false);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::OP_ACC, mi.acceleration[1], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::OP_DEC, mi.deceleration[1], true);

						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::OP_SPEED, mi.speed[2], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::OP_POS, mi.angles[2], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::OP_ACC, mi.acceleration[2], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::OP_DEC, mi.deceleration[2], true);

						//execute motion
						owner->wait_till_ready();

						owner->modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, crd514_kd::cmd1_bits::EXCITEMENT_ON);
						owner->modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, crd514_kd::cmd1_bits::EXCITEMENT_ON | crd514_kd::cmd1_bits::START);
						owner->modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, crd514_kd::cmd1_bits::EXCITEMENT_ON);
                    }
                }
                else //empty
                {
                    owner->queue_mutex.unlock();
                    
                    //set idle bool
					owner->idle_mutex.lock();
					owner->idle = true;
					owner->idle_mutex.unlock();
					owner->idle_cond.notify_all();

                    //wait until not idle
                    boost::unique_lock<boost::mutex> lock(owner->idle_mutex);
                    while(owner->idle)
                    {
                    	owner->idle_cond.wait(lock);
                    }
                }
            }
        }
        catch(boost::thread_interrupted& ex)
        {
        }
        catch(std::exception& ex)
        {
            if(owner->exhandler != NULL){
            	owner->exhandler(ex);
            }
        }
    }

    void steppermotor3::wait_till_ready(void)
    {
    	static const crd514_kd::slaves::t slaves[] =
    		{ crd514_kd::slaves::MOTOR_1, crd514_kd::slaves::MOTOR_2, crd514_kd::slaves::MOTOR_3 };

        for(int i = 0; i < 3; i++)
        {
        	uint16_t status_1;
        	while(!((status_1 = modbus.read_u16(slaves[i], crd514_kd::registers::STATUS_1)) & crd514_kd::status1_bits::READY))
        	{
        		if((status_1 & crd514_kd::status1_bits::ALARM) ||
        		   (status_1 & crd514_kd::status1_bits::WARNING))
        		{
        			throw crd514_kd_exception(
        				slaves[i], status_1 & crd514_kd::status1_bits::WARNING,
        				status_1 & crd514_kd::status1_bits::ALARM);
        		}
        	}
        }
    }

    void steppermotor3::moveto(const motionf& mf, bool async)
    {
        if(!powered_on)
        {
        	throw motor3_exception("motor drivers are not powered on");
        }

        if(mf.angles[0] <= min_angle || mf.angles[1] <= min_angle || mf.angles[2] <= min_angle ||
           mf.angles[0] >= max_angle || mf.angles[1] >= max_angle || mf.angles[2] >= max_angle)
        {
            throw std::out_of_range("one or more angles out of range");
        }

    	//push motion
        queue_mutex.lock();
        motion_queue.push(mf);
        queue_mutex.unlock();

        //unset idle bool
		idle_mutex.lock();
		idle = false;
		idle_mutex.unlock();
		idle_cond.notify_all();
        
        if(!async)
        {
            wait_for_idle();
        }

        current_angles[0] = mf.angles[0] + deviation[0];
        current_angles[1] = mf.angles[1] + deviation[1];
        current_angles[2] = mf.angles[2] + deviation[2];
    }

    void steppermotor3::stop(void)
    {
    	if(!powered_on)
		{
			throw motor3_exception("motor drivers are not powered on");
		}

    	boost::lock_guard<boost::mutex> queue_lock(queue_mutex);
        boost::lock_guard<boost::mutex> modbus_lock(modbus_mutex);
        
        try
        {
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, crd514_kd::cmd1_bits::STOP);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, 0);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, crd514_kd::cmd1_bits::EXCITEMENT_ON);
        }
        catch(modbus_exception& ex)
        {
        	fprintf(stderr, "steppermotor3::stop failed:\nwhat(): %s\n", ex.what());
        }
     
        //empty queue (there doesn't seem to be a more elegant way)
        while(!motion_queue.empty())
        {
            motion_queue.pop();
        }
    }

    bool steppermotor3::wait_for_idle(long timeout)
    {
    	if(!powered_on)
		{
			throw motor3_exception("motor drivers are not powered on");
		}

    	if(idle)
    	{
    		return true;
    	}
    	else if(timeout > 0)
    	{
    		long timeout_end = utils::time_now() + timeout;
    		boost::unique_lock<boost::mutex> lock(idle_mutex);
    		while(!idle)
    		{
    			timeout = timeout_end - utils::time_now();
    			if(timeout > 0)
    			{
    				if(!idle_cond.timed_wait(lock, boost::posix_time::milliseconds(timeout)))
    				{	//timeout
    					return false;
    				}
    			}
    			else
    			{	//timeout
    				return false;
    			}
    		}

    		boost::lock_guard<boost::mutex> modbus_lock(modbus_mutex);
    		wait_till_ready();
    		return true;
    	}

    	//wait indefinitely
    	boost::unique_lock<boost::mutex> lock(idle_mutex);
		while(!idle)
		{
			idle_cond.wait(lock);
		}

		boost::lock_guard<boost::mutex> modbus_lock(modbus_mutex);
		wait_till_ready();
		return true;
    }

    void steppermotor3::motion_float_to_int(motioni& mi, const motionf& mf)
    {
        for(int i = 0; i < 3; i++)
        {
            mi.angles[i] = (uint32_t)((mf.angles[i] + deviation[i]) / crd514_kd::MOTOR_STEP_ANGLE);
            mi.speed[i] = (uint32_t)(mf.speed[i] / crd514_kd::MOTOR_STEP_ANGLE);
            mi.acceleration[i] = (uint32_t)(crd514_kd::MOTOR_STEP_ANGLE * 1000000000.0 / mf.acceleration[i]);
            mi.deceleration[i] = (uint32_t)(crd514_kd::MOTOR_STEP_ANGLE * 1000000000.0 / mf.deceleration[i]);
        }
    }

    void steppermotor3::power_off(void)
    {
        if(powered_on){
            stop();
            boost::lock_guard<boost::mutex> lock(modbus_mutex);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, 0);
            powered_on = false;
        }
    }

    void steppermotor3::moveto_within(const motionf & mf, double time, bool async)
    {
        motionf newmf = mf;
        newmf.speed[0] = fabs(current_angles[0] - deviation[0] - mf.angles[0]) / time;
        newmf.speed[1] = fabs(current_angles[1] - deviation[1] - mf.angles[1]) / time;
        newmf.speed[2] = fabs(current_angles[2] - deviation[2] - mf.angles[2]) / time;

        //moveto(newmf, async);

        if(!powered_on)
        {
        	throw motor3_exception("motor drivers are not powered on");
        }

        if(newmf.angles[0] <= min_angle || newmf.angles[1] <= min_angle || newmf.angles[2] <= min_angle ||
           newmf.angles[0] >= max_angle || newmf.angles[1] >= max_angle || newmf.angles[2] >= max_angle)
        {
            throw std::out_of_range("one or more angles out of range");
        }

    	//push motion
        queue_mutex.lock();
        motion_queue.push(newmf);
        queue_mutex.unlock();

        //unset idle bool
		idle_mutex.lock();
		idle = false;
		idle_mutex.unlock();
		idle_cond.notify_all();

        current_angles[0] = newmf.angles[0] + deviation[0];
        current_angles[1] = newmf.angles[1] + deviation[1];
        current_angles[2] = newmf.angles[2] + deviation[2];

        if(!async)
        {
            wait_for_idle();
        }

    }

    void steppermotor3::power_on(void)
    {
        if(!powered_on){
            boost::lock_guard<boost::mutex> lock(modbus_mutex);
            //reset alarm
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::RESET_ALARM, 0);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::RESET_ALARM, 1);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::RESET_ALARM, 0);
            //set operating modes
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, 0);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::OP_POSMODE, 1);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::OP_OPMODE, 0);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::OP_SEQ_MODE + 0, 1);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::OP_SEQ_MODE + 1, /*1*/
            0);
            //modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::OP_SEQ_MODE+2, 0); //loopback @ 2
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, crd514_kd::cmd1_bits::EXCITEMENT_ON);
            //set motors limits
            modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((max_angle + deviation[0]) / crd514_kd::MOTOR_STEP_ANGLE));
            modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((min_angle + deviation[0]) / crd514_kd::MOTOR_STEP_ANGLE));
            modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((max_angle + deviation[1]) / crd514_kd::MOTOR_STEP_ANGLE));
            modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((min_angle + deviation[1]) / crd514_kd::MOTOR_STEP_ANGLE));
            modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((max_angle + deviation[2]) / crd514_kd::MOTOR_STEP_ANGLE));
            modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((min_angle + deviation[2]) / crd514_kd::MOTOR_STEP_ANGLE));
			modbus.write_u32(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CFG_START_SPEED, 1);
            //clear counter
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CLEAR_COUNTER, 1);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CLEAR_COUNTER, 0);
            current_angles[0] = current_angles[1] = current_angles[2] = 0;
            powered_on = true;
        }
    }

    void steppermotor3::override_current_angles(double * angles)
    {
    	this->deviation[0] = current_angles[0] - angles[0];
        this->deviation[1] = current_angles[1] - angles[1];
        this->deviation[2] = current_angles[2] - angles[2];
        modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((max_angle + deviation[0]) / crd514_kd::MOTOR_STEP_ANGLE));
        modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((min_angle + deviation[0]) / crd514_kd::MOTOR_STEP_ANGLE));
        modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((max_angle + deviation[1]) / crd514_kd::MOTOR_STEP_ANGLE));
        modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((min_angle + deviation[1]) / crd514_kd::MOTOR_STEP_ANGLE));
        modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((max_angle + deviation[2]) / crd514_kd::MOTOR_STEP_ANGLE));
        modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((min_angle + deviation[2]) / crd514_kd::MOTOR_STEP_ANGLE));

    }

    bool steppermotor3::is_powerd_on(void)
    {
    	return powered_on;
    }

    void steppermotor3::set_min_angle(double min_angle)
    {
    	this->min_angle = min_angle;
        modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((min_angle + deviation[0]) / crd514_kd::MOTOR_STEP_ANGLE));
        modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((min_angle + deviation[1]) / crd514_kd::MOTOR_STEP_ANGLE));
        modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((min_angle + deviation[2]) / crd514_kd::MOTOR_STEP_ANGLE));
    }

    void steppermotor3::set_max_angle(double max_angle)
    {
    	this->max_angle = max_angle;
        modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((max_angle + deviation[0]) / crd514_kd::MOTOR_STEP_ANGLE));
        modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((max_angle + deviation[1]) / crd514_kd::MOTOR_STEP_ANGLE));
        modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((max_angle + deviation[2]) / crd514_kd::MOTOR_STEP_ANGLE));
    }
}
