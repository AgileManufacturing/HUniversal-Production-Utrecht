//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        inverse_kinematics_impl.cpp
// File:           inverse kinematics implementation. based on work from Viacheslav Slavinsky
// Description:    Lukas Vermond & Kasper van Nieuwland
// Author:         -
// Notes:          
//
// License:        GNU GPL v3
//
// This file is part of inverse_kinematics_impl.cpp.
//
// inverse_kinematics_impl.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// inverse_kinematics_impl.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with inverse_kinematics_impl.cpp.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#include <huniplacer/inverse_kinematics_impl.h>

#include <cmath>
#include <cstdio>
#include <boost/math/special_functions/fpclassify.hpp>
#include <huniplacer/inverse_kinematics_exception.h>

#include <huniplacer/utils.h>

namespace huniplacer
{
    inverse_kinematics_impl::inverse_kinematics_impl(const double base, const double hip, const double effector, const double ankle, const double hip_ankle_angle_max) :
        inverse_kinematics_model(base, hip, effector, ankle, hip_ankle_angle_max)
    {
    }

    inverse_kinematics_impl::~inverse_kinematics_impl(void)
    {
    }

	#define SQR(x) ((x)*(x))
    double inverse_kinematics_impl::moveto(const point3& p, double motor_angle) const
    {
    	//ideas from Viacheslav Slavinsky are used
    	//conventions:
    	//	sitting in front of delta robot
    	//	x-axis goes from left to right
    	//	y-axis goes from front to back
    	//	z-axis goes from bottom to top
    	//	point (0,0,0) lies in the middle of all the motors at the motor's height

    	point3 p_fixed = p.rotate_z(-motor_angle);

    	p_fixed.y -= effector;
    	p_fixed.y += base;

    	//double c = sqrt(SQR(p_fixed.x) + SQR(p_fixed.y) + SQR(p_fixed.z));
    	double c = sqrt(SQR(p_fixed.y) + SQR(p_fixed.z));

    	if(c == 0)
    	{
    		throw inverse_kinematics_exception("point out of range", p);
    	}

    	double alpha_acos_input =
    			(-(SQR(ankle) - SQR(p_fixed.x)) + SQR(hip) + SQR(c))
    			/
    			(2*hip*c);
    	if(alpha_acos_input < -1 || alpha_acos_input > 1)
    	{
    		throw inverse_kinematics_exception("point out of range", p);
    	}

    	double alpha = acos(alpha_acos_input);

    	double beta = atan2(p_fixed.z, p_fixed.y);
    	double rho = beta - alpha;

    	double hip_ankle_angle = asin(abs(p_fixed.x)/ankle);

    	if(hip_ankle_angle > hip_ankle_angle_max)
    	{
    		throw inverse_kinematics_exception("angle between hip and ankle is out of range", p);
    	}

    	return rho;
    }
	#undef SQR
    
    void inverse_kinematics_impl::point_to_motion(const point3& p, motionf& mf) const
    {
        point3 goal = p;

		mf.angles[0] = utils::rad(-90) - moveto(p, utils::rad(0 * 120));
		mf.angles[1] = utils::rad(-90) - moveto(p, utils::rad(1 * 120));
		mf.angles[2] = utils::rad(-90) - moveto(p, utils::rad(2 * 120));

        mf.acceleration[0] = mf.acceleration[1] = mf.acceleration[2] = utils::rad(3600);
        mf.deceleration[0] = mf.deceleration[1] = mf.deceleration[2] = utils::rad(3600);
    }
}

