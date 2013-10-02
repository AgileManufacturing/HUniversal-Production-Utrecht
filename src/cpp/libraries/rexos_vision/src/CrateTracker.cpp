/**
 * @file CrateTracker.cpp
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

#include <rexos_vision/CrateTracker.h>
#include <rexos_datatypes/Crate.h>
#include <map>

namespace rexos_vision{
	/**
	 * Constructor
	 *
	 * @param stableFrames The number of frames a change has to be observed before a change is definite.
	 * @param movementThreshold The amount of mm a point has to move on the camera image before it is marked as moving.
	 **/
	CrateTracker::CrateTracker(int stableFrames, double movementThreshold) :
			stableFrames(stableFrames), movementThreshold(movementThreshold){
	}

	/**
	 * Determines the current state of all crates from a list of seen crates. It generates appropriate CrateEvent messages.
	 *
	 * @param updatedCrates List of seen crates.
	 *
	 * @return Vector list of CrateEvent messages.
	 **/
	std::vector<CrateEvent> CrateTracker::update(std::vector<rexos_datatypes::Crate> updatedCrates){
		std::vector<CrateEvent> events;

		// Disable all crates (mark for removal).
		for(std::map<std::string, rexos_datatypes::Crate>::iterator it = knownCrates.begin(); it != knownCrates.end(); ++it){
			it->second.exists = false;
		}

		for(std::vector<rexos_datatypes::Crate>::iterator it = updatedCrates.begin(); it != updatedCrates.end(); ++it){
			if(knownCrates.find(it->name) == knownCrates.end()){
				// Crate does not exists in knownCrates yet, add the crate
				rexos_datatypes::Crate newCrate = rexos_datatypes::Crate(*it);
				newCrate.exists = true;
				newCrate.oldSituation = false;
				newCrate.newSituation = true;
				// TODO: hacked to true to determine behavior when framesLeft/stableFrames is irrelevant
				newCrate.stable = true;
				newCrate.framesLeft = stableFrames;

				knownCrates.insert(std::pair<std::string, rexos_datatypes::Crate>(it->name, newCrate));
			} else{
				// Crate already exists, update location
				rexos_datatypes::Crate& crate = knownCrates.find(it->name)->second;
				crate.exists = true;

				// Check for movement
				if(hasChanged(crate, (*it))){
					if(crate.stable){
						// Crate began to move as old state was stable. Push moving event
						cv::RotatedRect crateRect = it->rect();
						events.push_back(CrateEvent(CrateEvent::type_moving, crate.name, crateRect.center.x, crateRect.center.y, crateRect.angle));
					}

					// Reset timer
					crate.framesLeft = stableFrames;
					crate.stable = false;
					crate.newSituation = true;

					// Store new location in knownCrates
					std::vector<cv::Point2f> tempPoints = it->getPoints();
					crate.setPoints(tempPoints);

				} else if(!crate.stable){
					crate.framesLeft--;
					if(crate.framesLeft <= 0){
						crate.stable = true;

						// Add event
						if(crate.oldSituation){
							// Crate moved
							events.push_back(
									CrateEvent(CrateEvent::type_moved, crate.name, crate.rect().center.x,
											crate.rect().center.y, crate.rect().angle));
							// Store new location in knownCrates
							std::vector<cv::Point2f> tempPoints = it->getPoints();
							crate.setPoints(tempPoints);
							crate.newSituation = true;
						} else if(!crate.oldSituation && crate.newSituation){
							// Crate entered
							events.push_back(
									CrateEvent(CrateEvent::type_in, crate.name, crate.rect().center.x,
											crate.rect().center.y, crate.rect().angle));
							crate.oldSituation = true;
						}
					}
				}
			}
		}
		removeUntrackedCrates(events);
		return events;
	}

	/**
	 * Removes all crates that were not found in the update loop.
	 *
	 * @param events List of CrateEvent messages.
	 **/
	void CrateTracker::removeUntrackedCrates(std::vector<CrateEvent> &events){
		// Remove all crate that were not found in the update loop. These have been marked as non existing.
		std::vector<std::string> cratesToBeRemoved;
		for(std::map<std::string, rexos_datatypes::Crate>::iterator it = knownCrates.begin(); it != knownCrates.end(); ++it){
			if(!it->second.exists){
				rexos_datatypes::Crate& crate = it->second;
				if(crate.stable){
					events.push_back(CrateEvent(CrateEvent::type_moving, crate.name));
					// Reset timer
					crate.framesLeft = stableFrames;
					crate.stable = false;
				}

				crate.newSituation = false;

				crate.framesLeft--;
				if(crate.framesLeft <= 0){
					if(crate.oldSituation){
						// Add event crate left
						events.push_back(CrateEvent(CrateEvent::type_out, crate.name));
					}

					// Add to cratesToBeRemoved list
					cratesToBeRemoved.push_back(it->second.name);
				}
			}
		}

		//remove crates
		for(std::vector<std::string>::iterator it = cratesToBeRemoved.begin(); it != cratesToBeRemoved.end(); it++){
			knownCrates.erase(*it);
		}
	}

	/**
	 * Returns the last stable state of a crate.
	 *
	 * @param name The name of the crate, QR data.
	 * @param result The last stable info of the crate.
	 *
	 * @return True if crates exists, false otherwise.
	 **/
	bool CrateTracker::getCrate(const std::string& name, rexos_datatypes::Crate& result){
		std::map<std::string, rexos_datatypes::Crate>::iterator it = knownCrates.find(name);
		if(it != knownCrates.end() && it->second.getState() != rexos_datatypes::Crate::state_non_existing){
			result = it->second;
			return true;
		} else{
			return false;
		}
	}

	/**
	 * Returns a list of crates with their last stable state.
	 *
	 * @return Vector with crates with their last stable state.
	 **/
	std::vector<rexos_datatypes::Crate> CrateTracker::getAllCrates( ){
		std::vector<rexos_datatypes::Crate> allCrates;
		for(std::map<std::string, rexos_datatypes::Crate>::iterator it = knownCrates.begin(); it != knownCrates.end(); ++it){
			if(it->second.getState() != rexos_datatypes::Crate::state_non_existing){
				allCrates.push_back(it->second);
			}
		}
		return allCrates;
	}

	/**
	 * Determines whether a crate has moved or rotated.
	 *
	 * @param newCrate The up to date values of the crate.
	 * @param oldCrate The values of the crate for comparison.
	 *
	 * @return True if the crate has moved or rotated, false otherwise.
	 **/
	bool CrateTracker::hasChanged(const rexos_datatypes::Crate& newCrate, const rexos_datatypes::Crate& oldCrate){
		const std::vector<cv::Point2f>& oldPoints = oldCrate.getPoints();
		const std::vector<cv::Point2f>& newPoints = newCrate.getPoints();
		for(int point = 0; point < 3; point++){
			const float deltaX = newPoints[point].x - oldPoints[point].x;
			const float deltaY = newPoints[point].y - oldPoints[point].y;
			if(sqrt(deltaX * deltaX + deltaY * deltaY) > movementThreshold){
				return true;
			}
		}
		return false;
	}
}
