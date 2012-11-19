/**
 * @file GridCrate4x4MiniBall.cpp
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

#include <PickAndPlaceNode/GridCrate4x4MiniBall.h>
#include <PickAndPlaceNode/MiniBall.h>
#include <PickAndPlaceNode/PickAndPlaceExceptions.h>

std::ostream & operator<<(std::ostream & os, const GridCrate4x4MiniBall & crate) {

	return os;
}

void GridCrate4x4MiniBall::put(size_t index, MiniBall* crateContent) {
	if (crateContents.at(index) != NULL) {
		throw PickAndPlaceExceptions::CrateLocationIsFullException();
	}
	crateContents[index] = crateContent;
}

MiniBall* GridCrate4x4MiniBall::get(size_t index) const {
	MiniBall* content = crateContents.at(index);
	if (content == NULL) {
		throw PickAndPlaceExceptions::CrateLocationIsEmptyException();
	}
	return content;
}

void GridCrate4x4MiniBall::remove(size_t index) {
	if (crateContents.at(index) == NULL) {
		throw PickAndPlaceExceptions::CrateLocationIsEmptyException();
	}
	crateContents[index] = NULL;
}

const std::string& GridCrate4x4MiniBall::getName(void) const {
	return name;
}

DataTypes::Point2D GridCrate4x4MiniBall::getLocation(int index) const {
	if(crate == NULL){
		throw std::exception("No crate in GridCrate4x4MiniBall");
	}

	// Calculate the offset in which 
	DataTypes::Point2D offset;
	offset.x = (index % COLS - (COLS / 2 - 0.5)) * (DISTANCE_BETWEEN_CONTAINERS + RADIUS_OF_CONTAINER);
	offset.y = (index / ROWS - (ROWS / 2 - 0.5)) * (DISTANCE_BETWEEN_CONTAINERS + RADIUS_OF_CONTAINER);

	DataTypes::Point2D result = offset.rotate(crate->getAngle());
	result += crate->getCenter();
	return result;
}

bool GridCrate4x4MiniBall::isEmpty( ) const {
	for (std::vector<MiniBall*>::iterator it = crateContents.begin(); it != crateContents.end(); ++it) {
		if (*it != NULL) {
			return false;
		}
	}
	return true;
}
