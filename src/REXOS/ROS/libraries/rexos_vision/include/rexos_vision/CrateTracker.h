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
#include <rexos_datatypes/Crate.h>
#include <map>
#include <vector>
#include <string>

namespace rexos_vision{
	/**
	 * CrateEvent in which a crate changes state and gives new x and y coordinates and angle when possible.
	 **/
	class CrateEvent{
	public:
		/**
		 * Indicates the type of event received for a crate, namely found, moving, moved and removed.
		 **/
		enum crate_event_type{
			type_in = 1, type_out = 2, type_moving = 3, type_moved = 4
		};
		/**
		 * The constructor.
		 *
		 * @param type The type of crate event.
		 * @param name The name of the crate.
		 * @param x The x coordinate.
		 * @param y The y coordinate.
		 * @param angle The angle of the crate.
		 **/
		CrateEvent(crate_event_type type = type_moving, std::string name = "", float x = 0, float y = 0, float angle = 0) :
				type(type), name(name), x(x), y(y), angle(angle){
		}

		/**
		 * Returns a string with the information about the event.
		 *
		 * @return String with the information about the event, namely type, name, x and y coordinates and angle.
		 **/
		std::string toString(){
			std::stringstream ss;
			std::string typeString;
			switch (type){
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

		/**
		 * @var int type
		 * Event type, namely in, out, moving or moved.
		 **/
		int type;
		/**
		 * @var std::string name
		 * Name of the crate.
		 **/
		std::string name;
		/**
		 * @var float x
		 * x-coordinate
		 **/
		float x;
		/**
		 * @var float y
		 * y-coordinate
		 **/
		float y;
		/**
		 * @var float angle
		 * Angle of the crate, where 0 is up on the image of the camera.
		 **/
		float angle;
	};

	/**
	 * Contains the rexos_vision algorithems for tracking a crate
	 **/
	class CrateTracker{
	public:
		CrateTracker(int stableFrames, double movementThreshold);

		std::vector<CrateEvent> update(std::vector<rexos_datatypes::Crate> crates);
		std::vector<rexos_datatypes::Crate> getAllCrates();
		bool getCrate(const std::string& name, rexos_datatypes::Crate& result);

		/**
		 * @var int stableFrames
		 * Amount of frames a change has to be present for the crate to be counted as changed.
		 **/
		int stableFrames;
		/**
		 * @var double movementThreshold
		 * The amount of mm a point has to move on the camera image before it is marked as moving.
		 **/
		double movementThreshold;
	private:
		bool hasChanged(const rexos_datatypes::Crate& newCrate, const rexos_datatypes::Crate& oldCrate);
		void removeUntrackedCrates(std::vector<CrateEvent> &events);

		/**
		 * Map of known crates
		 **/
		std::map<std::string, rexos_datatypes::Crate> knownCrates;
	};
}
