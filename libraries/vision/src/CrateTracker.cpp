//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        VisionNode
// File:           CrateTracker.cpp
// Description:    keeps track of the crates and generates events like: new crate, crate removed or crate moved.
// Author:         Kasper van Nieuwland en Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of VisionNode.
//
// VisionNode is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// VisionNode is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with VisionNode.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
#include <Vision/CrateTracker.h>
#include <Crate.h>
#include <map>

CrateTracker::CrateTracker(int stableFrames, double movementThresshold) :
		stableFrames(stableFrames), movementThresshold(movementThresshold) {
}

std::vector<CrateEvent> CrateTracker::update(std::vector<Crate> updatedCrates) {
	std::vector<CrateEvent> events;

	for (std::map<std::string, exCrate>::iterator it = knownCrates.begin();
		it != knownCrates.end(); ++it) {
		it->second.exists = false;
	}

	for (std::vector<Crate>::iterator it = updatedCrates.begin();
			it != updatedCrates.end(); ++it) {
		//ROS_INFO("%f %f", it->getPoints()[0].x, it->getPoints()[1].x);

		if (knownCrates.find(it->name) == knownCrates.end()) {
			//does not exists in knownCrates yet

			//add crate
			exCrate newCrate = exCrate(*it);
			newCrate.exists = true;
			newCrate.oldSituation = false;
			newCrate.newSituation = true;
			newCrate.stable = false;
			newCrate.framesLeft = stableFrames;

			knownCrates.insert(std::pair<std::string, exCrate>(it->name, newCrate));
		} else {
			//already exists
			exCrate& crate = knownCrates.find(it->name)->second;
			crate.exists = true;

			//check for movement
			if (hasChanged(crate, (*it))) {
				if (crate.stable) {
					//crate began to move
					//add moving event
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
						events.push_back(CrateEvent(CrateEvent::type_moved, crate.name, crate.rect().center.x, crate.rect().center.y, crate.rect().angle));
						//store new location in knownCrates
						std::vector<cv::Point2f> tempPoints = it->getPoints();
						crate.setPoints(tempPoints);
						crate.newSituation = true;
					} else if (!crate.oldSituation && crate.newSituation) {
						//crate entered
						events.push_back(CrateEvent(CrateEvent::type_in, crate.name, crate.rect().center.x, crate.rect().center.y, crate.rect().angle));
						crate.oldSituation = true;
					}
				}
			}
		}
	}

	//crates that were not updated
	std::vector<std::string> cratesToBeRemoved;
	for (std::map<std::string, exCrate>::iterator it = knownCrates.begin();
			it != knownCrates.end(); ++it) {
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
				if(crate.oldSituation)
				{
					//add event crate left
					events.push_back(CrateEvent(CrateEvent::type_out, crate.name));
				}

				//add to cratesToBeRemoved list
				cratesToBeRemoved.push_back(it->second.name);

			}
		}
	}

	//remove crates
	for(std::vector<std::string>::iterator it = cratesToBeRemoved.begin(); it != cratesToBeRemoved.end(); it++)
	{
		knownCrates.erase(*it);
	}
	return events;
}

bool CrateTracker::getCrate(const std::string& name, exCrate& result)
{
	std::map<std::string, exCrate>::iterator it = knownCrates.find(name);
	if(it != knownCrates.end() && it->second.getState() != exCrate::state_non_existing)
	{
		result = it->second;
		return true;
	}
	else
	{
		return false;
	}
}

std::vector<exCrate> CrateTracker::getAllCrates()
{
	std::vector<exCrate> allCrates;
	for (std::map<std::string, exCrate>::iterator it = knownCrates.begin();
		it != knownCrates.end(); ++it) {
		if(it->second.getState() != exCrate::state_non_existing){
			allCrates.push_back(it->second);
		}
	}
	return allCrates;
}

bool CrateTracker::hasChanged(const Crate& newCrate, const Crate& oldCrate) {
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

exCrate::crate_state exCrate::getState()
{
	if(oldSituation){
		if(stable){
			return state_stable;
		}
		else{
			return state_moving;
		}
	}
	return state_non_existing;
}



