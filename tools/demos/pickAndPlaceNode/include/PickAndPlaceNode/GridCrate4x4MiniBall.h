/**
 * @file GridCrate4x4MiniBall.h
 * @brief Entity for a crate containing 4x4 mini balls
 * @date Created: 2012-11-19
 *
 * @author Koen Braham
 * @author Daan Veltman
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

#pragma once

#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <DataTypes/Crate.h>

class GridCrate4x4MiniBall {
public:
	GridCrate4x4MiniBall(std::string name) :
		name(name), x(0), y(0), angle(0) {
	}
	void setCrate(double x, double y, double angle){
		this->x = x;
		this->y = y;
		this->angle = angle;
	}
	void put(size_t index, MiniBall* crateContent);
	MiniBall* get(size_t index) const;
	const std::string& getName(void) const;
	void remove(size_t index);
	bool isEmpty( ) const;

	friend std::ostream & operator<<(std::ostream & os, const GridCrate4x4MiniBall & crate);

private:
	static const double DISTANCE_BETWEEN_CONTAINERS = 0.5;
	static const double RADIUS_OF_CONTAINER = 5.25;
	static const int ROWS = 4;
	static const int COLS = 4;
	//static const double BOTTOM_THICKNESS = 5.3; 

	double x, y, angle;

	DataTypes::Crate* crate;
	std::string name;
	std::vector<MiniBall*>& crateContents;
};
