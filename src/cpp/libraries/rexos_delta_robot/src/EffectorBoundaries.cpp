/**
 * @file EffectorBoundaries.cpp
 * @brief Represents the effector's moving volume.
 *
 * @author 1.0 Lukas Vermond
 * @author 1.0 Kasper van Nieuwland
 * @author 1.1 Daan Veltman
 * @author 1.1 Koen Braham
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

#include <iostream>
#include <rexos_delta_robot/Measures.h>
#include <rexos_delta_robot/EffectorBoundaries.h>
#include <rexos_delta_robot/InverseKinematicsException.h>
#include <rexos_delta_robot/EffectorBoundariesException.h>
#include <stack>
#include <vector>
#include <set>
#include <cstring>

namespace rexos_delta_robot{
	/**
	 * Function to generate the boundaries and returns a pointer to the object.
	 * 
	 * @param model Used to calculate the boundaries.
	 * @param motorMinAngles An array holding the minimum angle of each of the three motors.
	 * @param motorMaxAngles An array holding the maximum angle of each of the three motors.
	 * @param voxelSize The size of the voxels in millimeters.
	 * 
	 * @return Pointer to the object.
	 **/
	EffectorBoundaries* EffectorBoundaries::generateEffectorBoundaries(const InverseKinematicsModel& model, double motorMinAngles[3], double motorMaxAngles[3], double voxelSize){
		EffectorBoundaries* boundaries = new EffectorBoundaries(model, motorMinAngles, motorMaxAngles, voxelSize);
		
		// Create boundaries variables in voxel space by dividing real space variables with the voxel size
		boundaries->width = (Measures::BOUNDARY_BOX_MAX_X  - Measures::BOUNDARY_BOX_MIN_X) / voxelSize;
        boundaries->height = (Measures::BOUNDARY_BOX_MAX_Z - Measures::BOUNDARY_BOX_MIN_Z) / voxelSize;
        boundaries->depth = (Measures::BOUNDARY_BOX_MAX_Y  - Measures::BOUNDARY_BOX_MIN_Y) / voxelSize;
        
        // Create bitmap with value false for all voxels
        boundaries->boundariesBitmap = new bool[boundaries->width * boundaries->height * boundaries->depth];
        for(int i = 0; i < boundaries->width * boundaries->height * boundaries->depth; i++){
        	boundaries->boundariesBitmap[i] = false;
        }
        boundaries->generateBoundariesBitmap();
        return boundaries;
    }

	/**
	 * Checks if the path from the starting- to the destination point is not going out of the
	 * robot's boundaries.
	 * 
	 * @param from The starting point.
	 * @param to The destination point.
	 *
	 * @return true if a straight path from parameter 'from' to parameter 'to' is valid.
	 **/
    bool EffectorBoundaries::checkPath(const rexos_datatypes::Point3D<double>& from, const rexos_datatypes::Point3D<double>& to) const{
    	double x_length = to.x - from.x;
    	double y_length = to.y - from.y;
    	double z_length = to.z - from.z;
    	double largest_length = (double)(fabs(x_length) > fabs(y_length) ?
    			(fabs(x_length) > fabs(z_length) ? fabs(x_length) : fabs(z_length)) :
    			(fabs(y_length) > fabs(z_length) ? fabs(y_length) : fabs(z_length)));

    	x_length = x_length / largest_length;
    	y_length = y_length / largest_length;
    	z_length = z_length / largest_length;
    	
		for(double i = 1; i <= largest_length; i++){
			int x = (from.x + x_length * i);
			int y = (from.y + y_length * i);
			int z = (from.z + z_length * i);
			BitmapCoordinate temp = fromRealCoordinate(rexos_datatypes::Point3D<double>(x, y, z));
			int index = temp.x + temp.y * width + temp.z * width * depth;

			if(temp.x < 0
				|| temp.x > width
				|| temp.y < 0
				|| temp.y > depth
				|| temp.z < 0
				|| temp.z > height
				|| !boundariesBitmap[index]){
				return false;
			}
		}
        return true;
    }

	/**
	 * Private constructor, it also initializes the voxel array.
	 * 
	 * @param model Used to calculate the boundaries.
	 * @param motorMinAngles An array holding the minimum angle of each of the three motors.
	 * @param motorMaxAngles An array holding the maximum angle of each of the three motors.
	 * @param voxelSize The size of the voxels.
	 **/
    EffectorBoundaries::EffectorBoundaries(const InverseKinematicsModel& model, double motorMinAngles[3], double motorMaxAngles[3], double voxelSize) : 
    	width(0), 
    	height(0), 
    	depth(0), 
    	boundariesBitmap(NULL), 
    	kinematics(model),  
    	voxelSize(voxelSize) {
    		for(int i = 0; i < 3; i++){
    			this->motorMinAngles[i] = motorMinAngles[i];
    			this->motorMaxAngles[i] = motorMaxAngles[i];
    		}
    	}

    EffectorBoundaries::~EffectorBoundaries(){
    	delete[] boundariesBitmap;
    }

	/**
	 * Checks if one of the neighbouring voxels can't be reached by the effector. This includes voxels outside of the MIN/BOUNDARY_BOX_MAX_X/Y/Z box as defined in measures.
	 * 
	 * @param coordinate The point in the bitmap that has to be checked.
	 * @param pointValidityCache Pointer to the cache where already checked values are stored, and unchecked points are unknown. This as opposed to the bitmap, which is defaulted to false instead of unknown.
	 *
	 * @return True if coordinate has unreachable neighbouring voxels.
	 **/
	bool EffectorBoundaries::hasInvalidNeighbours(const BitmapCoordinate& coordinate, char* pointValidityCache) const{
		//TODO: Change from has_invalid_neighbours to isOnTheEdgeOfValidArea due to functionality change.

    	// Check if the voxel is valid and on the edge of the box.
    	if(isValid(BitmapCoordinate(coordinate.x, coordinate.y, coordinate.z), pointValidityCache)){
    		// Voxel is on the edge of the box, automatically boardering invalid territory.
    		if(coordinate.x == 0 || coordinate.x == width - 1 || coordinate.y == 0 || coordinate.y == depth - 1 || coordinate.z == 0 || coordinate.z == height - 1){
    			return true;
    		}

	        // Check all voxels in the 3x3x3 box around voxel coordinate.
	        for(int y = coordinate.y - 1; y <= coordinate.y + 1; y++){
	            for(int x = coordinate.x - 1; x <= coordinate.x + 1; x++){
	                for(int z = coordinate.z - 1; z <= coordinate.z + 1; z++){
	                    // Check if one of the neighbours is not valid.
	                    if(!isValid(BitmapCoordinate(x, y, z), pointValidityCache)){
	                        return true;
	                    }
	                }
	            }
	        }
	    }
	    // Voxel is invalid OR inside of a box of 3x3x3 valid voxels.
        return false;
    }

	/**
	 * Checks if the point can be reached by the effector. Whether the point can be reached is determined by the kinematics, minimum and maximum angles of the motors and the MIN/BOUNDARY_BOX_MAX_X/Y/Z box determined in measures.
	 * @param coordinate The point that is checked if it can be reached by the effector.
	 * @param pointValidityCache Pointer to the cache where already checked values are stored, and unchecked points are unknown. This as opposed to the bitmap, which is defaulted to false instead of unknown.
	 * 
	 * @return true if coordinate is reachable by the effector.
	 **/
    bool EffectorBoundaries::isValid(const BitmapCoordinate& coordinate, char* pointValidityCache) const{
    	char* fromCache;
    	char dummy = UNKNOWN;
    	if(pointValidityCache == NULL){
    		fromCache = &dummy;
    	} else{
    		fromCache = &pointValidityCache[coordinate.x + coordinate.y * width + coordinate.z * width * depth];
    	}

    	if(*fromCache == UNKNOWN){
    		rexos_datatypes::MotorRotation* rotations[3];
    		rotations[0] = new rexos_datatypes::MotorRotation();
    		rotations[1] = new rexos_datatypes::MotorRotation();
    		rotations[2] = new rexos_datatypes::MotorRotation();

			try{
				kinematics.destinationPointToMotorRotations(fromBitmapCoordinate(coordinate), rotations);
			} catch(rexos_delta_robot::InverseKinematicsException & exception){
				*fromCache = INVALID;
				delete rotations[0];
				delete rotations[1];
				delete rotations[2];
				return false;
			}

			// Check motor angles.
			if(rotations[0]->angle <= motorMinAngles[0] || rotations[0]->angle >= motorMaxAngles[0] 
			|| rotations[1]->angle <= motorMinAngles[1] || rotations[1]->angle >= motorMaxAngles[1] 
			|| rotations[2]->angle <= motorMinAngles[2] || rotations[2]->angle >= motorMaxAngles[2]){
			  	*fromCache = INVALID;
				delete rotations[0];
				delete rotations[1];
				delete rotations[2];
			  	return false;
			}

			*fromCache = VALID;

			delete rotations[0];
			delete rotations[1];
			delete rotations[2];
			return true;
    	} else{
    		return *fromCache == VALID;
    	}
    }

	/**
	 * Generates boundaries bitmap for the robot. From the centre of the BOUNDARY_BOX voxels are checked and set to true in the bitmap if they are reachable. All members should be initialized before calling this function.
	 **/
    void EffectorBoundaries::generateBoundariesBitmap(void){
    	char* pointValidityCache = new char[width * depth * height];
    	memset(pointValidityCache, 0, width * depth * height * sizeof(char));
    	std::stack<BitmapCoordinate> cstack;

    	// Determine the center of the box.
    	rexos_datatypes::Point3D<double> point (0, 0, Measures::BOUNDARY_BOX_MIN_Z + (Measures::BOUNDARY_BOX_MAX_Z - Measures::BOUNDARY_BOX_MIN_Z) / 2);
    	
    	// If point pixel is not part of a valid voxel the box dimensions are incorrect.
    	if(!isValid(fromRealCoordinate(point), pointValidityCache)){
    		throw EffectorBoundariesException("starting point outside of valid area, please adjust BOUNDARY_BOX_MAX/BOUNDARY_BOX_MIN_X/Y/Z values to have a valid center");
    	}
    	
    	// Scan towards the right.
		for(; point.x < Measures::BOUNDARY_BOX_MAX_X; point.x += voxelSize){
			// If an invalid voxel is found:
			// - step back to the last valid voxel
			// - push the voxel on the empty stack
			// - set the voxel as true in the bitmap
			// - end the loop
			if(!isValid(fromRealCoordinate(point), pointValidityCache)){
				point.x -= voxelSize;
				BitmapCoordinate startingVoxel = fromRealCoordinate(point);
				cstack.push(startingVoxel);
				boundariesBitmap[startingVoxel.x + startingVoxel.y * width + startingVoxel.z * width * depth] = true;
				break;
			}
		}
		// If the right-most voxel is in reach and an invalid voxel is never found, the position of point.x will be outside of the box limits. Step back inside the box and add that voxel to the stack and set it as true in the bitmap.
		if(point.x >= Measures::BOUNDARY_BOX_MAX_X){
			point.x -= voxelSize;
			BitmapCoordinate startingVoxel = fromRealCoordinate(point);
			cstack.push(startingVoxel);
			boundariesBitmap[startingVoxel.x + startingVoxel.y * width + startingVoxel.z * width * depth] = true;
		}

		// Start with the last added voxel on the stack and add new voxels to the stack. Do this until the valid borders (all valid voxels bordering unvalid voxels or the BOUNDARY_BOX_MAX/BOUNDARY_BOX_MIN_X/Y/Z box) of the valid voxel area are known (stack = empty).
		while(!cstack.empty()){
			// Get last added voxel from the stack and remove it from the stack.
			BitmapCoordinate borderVoxel = cstack.top();
			cstack.pop();

			// Check all neighbours of the voxel.
			for(int y = borderVoxel.y - 1; y <= borderVoxel.y + 1; y++){
				for(int x = borderVoxel.x - 1; x <= borderVoxel.x + 1; x++){
					for(int z = borderVoxel.z - 1; z <= borderVoxel.z + 1; z++){
						// Don't do anything with voxels outside of the BOUNDARY_BOX_MAX/BOUNDARY_BOX_MIN_X/Y/Z box.
						if(z >= height || z < 0 || x >= width || x < 0 || y >= depth || y < 0){
							continue;
						} else{
							// New valid voxels on the valid border are added to the stack and set in the bitmap.
							int index = x + y * width + z * width * depth;
							if(isValid(BitmapCoordinate(x, y, z), pointValidityCache)
									&& !boundariesBitmap[index]
								    && hasInvalidNeighbours(BitmapCoordinate(x, y, z), pointValidityCache)){
								BitmapCoordinate bitmapCoordinate = BitmapCoordinate(x, y, z);
								cstack.push(bitmapCoordinate);
								boundariesBitmap[index] = true;
							}
						}
					}
				}
			}
		}

		delete[] pointValidityCache;
		pointValidityCache = NULL;
		
		// Adds all the points within the boundaries.
		cstack.push(fromRealCoordinate(rexos_datatypes::Point3D<double>(0, 0, Measures::BOUNDARY_BOX_MIN_Z + (Measures::BOUNDARY_BOX_MAX_Z - Measures::BOUNDARY_BOX_MIN_Z) / 2)));
		while(!cstack.empty()){
			BitmapCoordinate validVoxel = cstack.top();
			cstack.pop();
			
			if(validVoxel.x <= 0 || validVoxel.x >= width || validVoxel.y <= 0 || validVoxel.y >= depth || validVoxel.z <= 0 || validVoxel.z >= height){
				continue;
			}
			int index = validVoxel.x + validVoxel.y * width + validVoxel.z * width * depth;

			int indices[6] = {
				index - 1, 
				index + 1, 
				index - width, 
				index + width, 
				index - width * depth, 
				index + width * depth
			};

			for(unsigned int i = 0; i < ( sizeof(indices) / sizeof(indices[0]) ); i++){
				if(indices[i] < ((width*height*depth))){
					if(boundariesBitmap[indices[i]] == false){
						boundariesBitmap[indices[i]] = true;
						cstack.push(BitmapCoordinate(indices[i] % width, (indices[i] % (width * depth)) / width, indices[i] / (width * depth)));
					}
				}
			}	
		}
	}
}
