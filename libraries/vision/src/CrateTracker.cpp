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

#include <Vision/CrateTracker.h>
#include <DataTypes/Crate.h>
#include <map>

CrateTracker::CrateTracker(int stableFrames, double movementThresshold) :
		stableFrames(stableFrames), movementThresshold(movementThresshold) {
}

std::vector<CrateEvent> CrateTracker::update(std::vector<DataTypes::Crate> updatedCrates) {
	std::vector<CrateEvent> events;

	// Disable all crates (mark for removal).
	for (std::map<std::string, exCrate>::iterator it = knownCrates.begin(); it != knownCrates.end(); ++it) {
		it->second.exists = false;
	}

	for (std::vector<DataTypes::Crate>::iterator it = updatedCrates.begin(); it != updatedCrates.end(); ++it) {
		if (knownCrates.find(it->name) == knownCrates.end()) {
			// Crate does not exists in knownCrates yet, add the crate
			exCrate newCrate = exCrate(*it);
			newCrate.exists = true;
			newCrate.oldSituation = false;
			newCrate.newSituation = true;
			newCrate.stable = false;
			newCrate.framesLeft = stableFrames;

			knownCrates.insert(std::pair<std::string, exCrate>(it->name, newCrate));
		} else {
			// Crate already exists, update location
			exCrate& crate = knownCrates.find(it->name)->second;
			crate.exists = true;

			//check for movement
			if (hasChanged(crate, (*it))) {
				if (crate.stable) {
					//crate began to move as old state was stable. Push moving event
					events.push_back(CrateEvent(CrateEvent::type_moving, crate.name));
				}

				//reset timer
				crate.framesLeft = stableFrames;
				crate.stable = false;
				crate.newSituation = true;

				//store new location in knownCrates
				std::vector<cv::Point2f> tempPoints = it->getPoints();
				crate.setPoints(tempPoints);

			} else if (!crate.stable) {
				crate.framesLeft--;
				if (crate.framesLeft <= 0) {
					crate.stable = true;

					//add event
					if (crate.oldSituation) {
						//crate moved
						events.push_back(
						        CrateEvent(CrateEvent::type_moved, crate.name, crate.rect().center.x,
						                crate.rect().center.y, crate.rect().angle));
						//store new location in knownCrates
						std::vector<cv::Point2f> tempPoints = it->getPoints();
						crate.setPoints(tempPoints);
						crate.newSituation = true;
					} else if (!crate.oldSituation && crate.newSituation) {
						//crate entered
						events.push_back(
						        CrateEvent(CrateEvent::type_in, crate.name, crate.rect().center.x,
						                crate.rect().center.y, crate.rect().angle));
						crate.oldSituation = true;
					}
				}
			}
		}
	}

	// Remove all crate that were not found in the update loop. These have been marked as non existing.
	std::vector<std::string> cratesToBeRemoved;
	for (std::map<std::string, exCrate>::iterator it = knownCrates.begin(); it != knownCrates.end(); ++it) {
		if (!it->second.exists) {
			exCrate& crate = it->second;
			if (crate.stable) {
				events.push_back(CrateEvent(CrateEvent::type_moving, crate.name));
				//reset timer
				crate.framesLeft = stableFrames;
				crate.stable = false;
			}

			crate.newSituation = false;

			crate.framesLeft--;
			if (crate.framesLeft <= 0) {
				if (crate.oldSituation) {
					//add event crate left
					events.push_back(CrateEvent(CrateEvent::type_out, crate.name));
				}

				//add to cratesToBeRemoved list
				cratesToBeRemoved.push_back(it->second.name);

			}
		}
	}

	//remove crates
	for (std::vector<std::string>::iterator it = cratesToBeRemoved.begin(); it != cratesToBeRemoved.end(); it++) {
		knownCrates.erase(*it);
	}
	return events;
}

bool CrateTracker::getCrate(const std::string& name, exCrate& result) {
	std::map<std::string, exCrate>::iterator it = knownCrates.find(name);
	if (it != knownCrates.end() && it->second.getState() != exCrate::state_non_existing) {
		result = it->second;
		return true;
	} else {
		return false;
	}
}

std::vector<exCrate> CrateTracker::getAllCrates( ) {
	std::vector<exCrate> allCrates;
	for (std::map<std::string, exCrate>::iterator it = knownCrates.begin(); it != knownCrates.end(); ++it) {
		if (it->second.getState() != exCrate::state_non_existing) {
			allCrates.push_back(it->second);
		}
	}
	return allCrates;
}

bool CrateTracker::hasChanged(const DataTypes::Crate& newCrate, const DataTypes::Crate& oldCrate) {
	const std::vector<cv::Point2f>& oldp = oldCrate.getPoints();
	const std::vector<cv::Point2f>& newp = newCrate.getPoints();
	for (int i = 0; i < 3; i++) {
		const float dx = newp[i].x - oldp[i].x;
		const float dy = newp[i].y - oldp[i].y;
		if (sqrt(dx * dx + dy * dy) > movementThresshold) {
			return true;
		}
	}
	return false;
}

exCrate::crate_state exCrate::getState( ) {
	if (oldSituation) {
		if (stable) {
			return state_stable;
		} else {
			return state_moving;
		}
	}
	return state_non_existing;
}

