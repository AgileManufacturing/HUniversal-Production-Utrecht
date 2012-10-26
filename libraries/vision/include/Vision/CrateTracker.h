/**
 * @file CrateTracker.h
 * @brief Keeps track of the crates and generates events like: new crate, crate removed or crate moved.
 * @date Created: 2011-11-11
 *
 * @author Kasper van Nieuwland
 * @author Zep Mouris
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
#include <DataTypes/Crate.h>
#include <map>
#include <vector>
#include <string>

class CrateEvent {
public:
	enum crate_event_type {
		type_in = 1, type_out = 2, type_moving = 3, type_moved = 4
	};
	/**
	 * the constructor
	 * @param type the type of crate event.
	 * @param name the name of the crate
	 * @param x the x coordinate
	 * @param y the y coordinate
	 * @param angle the angle of the crate
	 */
	CrateEvent(crate_event_type type = type_moving, std::string name = "", float x = 0, float y = 0, float angle = 0) :
			type(type), name(name), x(x), y(y), angle(angle) {
	}

	/**
	 * returns a string with the information about the event
	 * @return string with the information about the event
	 */
	std::string toString( ) {
		std::stringstream ss;
		std::string typeString;
		switch (type) {
		case type_in:
			typeString = "In";
			break;
		case type_out:
			typeString = "Out";
			break;
		case type_moving:
			typeString = "Moving";
			break;
		case type_moved:
			typeString = "Moved";
			break;
		}
		ss << "CrateEvent: \n\ttype: " << typeString << "\n\tName: " << name << "\n\tX: " << x << "\n\tY: " << y
		        << "\n\tAngle: " << angle;
		return ss.str();
	}

	int type;
	std::string name;
	float x, y, angle;
};

class exCrate: public DataTypes::Crate {
public:
	enum crate_state {
		state_stable = 1, state_moving = 2, state_non_existing = 3
	};
	/**
	 * empty constructor.
	 */
	exCrate( ) :
			Crate(), oldSituation(false), newSituation(false), exists(true), stable(false), framesLeft(0) {
	}

	/**
	 * constructor
	 * @param crate crate from fiducial project
	 * @param framesLeft the number of frames before a change is definite
	 */
	exCrate(const Crate& crate, int framesLeft = 0) :
			Crate(crate), oldSituation(false), newSituation(true), exists(true), stable(false), framesLeft(framesLeft) {
	}

	/**
	 * function determines the last known stable state
	 * @return the last stable state
	 */
	crate_state getState( );

	bool oldSituation, newSituation, exists, stable;
	int framesLeft;
};

class CrateTracker {
public:
	/**
	 * Constructor
	 * @param stableFrames framesLeft the number of frames before a change is definite
	 * @param movementThresshold the amount of mm a point has to move before it is marked as moving
	 */
	CrateTracker(int stableFrames, double movementThresshold);

	/**
	 * determines the current state of all crates from a list of seen crates and generates CrateEvent.
	 * @param crates list of seen crates
	 * @return list of events
	 */
	std::vector<CrateEvent> update(std::vector<DataTypes::Crate> crates);

	/**
	 * returns a list of crates with their last stable state
	 * @return list with crates with their last stable state
	 */
	std::vector<exCrate> getAllCrates( );

	/**
	 * returns the last stable state of a crate
	 * @param name the name of the crate
	 * @param result the last stable info of the crate
	 * @return true if crates exists, false otherwise
	 */
	bool getCrate(const std::string& name, exCrate& result);

	int stableFrames;
	double movementThresshold;
	double rotationThresshold;
private:
	/**
	 * determines whether a crate has moved or rotated
	 * @param newCrate the up to date values of the crate
	 * @param oldCrate the previous values of the crate
	 * @return true if moved
	 */
	bool hasChanged(const DataTypes::Crate& newCrate, const DataTypes::Crate& oldCrate);
	std::map<std::string, exCrate> knownCrates;

};
