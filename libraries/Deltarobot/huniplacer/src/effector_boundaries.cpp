//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        effector_boundaries.cpp
// File:           represents the effector's moving volume
// Description:    Lukas Vermond & Kasper van Nieuwland
// Author:         -
// Notes:          
//
// License:        newBSD
//
// Copyright © 2012, HU University of Applied Sciences Utrecht
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of the HU University of Applied Sciences Utrecht nor the
// names of its contributors may be used to endorse or promote products
// derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************

#include <iostream>
#include <huniplacer/measures.h>
#include <huniplacer/effector_boundaries.h>
#include <huniplacer/InverseKinematicsException.h>
#include <stack>
#include <vector>
#include <set>
#include <cstring>

namespace huniplacer
{
	using namespace measures;

	effector_boundaries* effector_boundaries::generate_effector_boundaries(const InverseKinematicsModel& model, const imotor3& motors, double voxel_size)
	{
		effector_boundaries* boundaries = new effector_boundaries(model, motors, voxel_size);
		
		//create boundaries variables in voxel space by dividing real space variables with the voxel size
		boundaries->width = (MAX_X - MIN_X) / voxel_size;
        boundaries->height = (MAX_Z - MIN_Z) / voxel_size;
        boundaries->depth = (MAX_Y - MIN_Y) / voxel_size;
        //create bitmap with value false for all voxels
        boundaries->boundaries_bitmap = new bool[boundaries->width * boundaries->height * boundaries->depth];
        for(int i = 0; i < boundaries->width * boundaries->height * boundaries->depth; i++)
        {
        	boundaries->boundaries_bitmap[i] = false;
        }
        boundaries->generate_boundaries_bitmap();

        return boundaries;
    }

    bool effector_boundaries::check_path(const Point3D & from, const Point3D & to) const
    {
    	double x_length = to.x - from.x;
    	double y_length = to.y - from.y;
    	double z_length = to.z - from.z;
    	int largest_length = (fabs(x_length) > fabs(y_length) ?
    			(fabs(x_length) > fabs(z_length) ? fabs(x_length) : fabs(z_length)) :
    			(fabs(y_length) > fabs(z_length) ? fabs(y_length) : fabs(z_length)));

    	x_length = x_length / largest_length;
    	y_length = y_length / largest_length;
    	z_length = z_length / largest_length;

		for(double i = 1; i < largest_length; i++)
		{
			int x = (from.x + x_length * i);
			int y = (from.y + y_length * i);
			int z = (from.z + z_length * i);
			bitmap_coordinate temp = from_real_coordinate(Point3D(x, y, z));
			int index = temp.x + temp.y * width + temp.z * width * depth;

			if(index >= (width * height * depth) || !boundaries_bitmap[index])
			{
				return false;
			}
		}

        return true;
    }

    effector_boundaries::effector_boundaries(const InverseKinematicsModel& model, const imotor3& motors, double voxel_size)
    	: point_validity_cache(NULL), kinematics(model), motors(motors), voxel_size(voxel_size)
    {
    }

    effector_boundaries::~effector_boundaries()
    {
    	delete[] boundaries_bitmap;
    }

    //TODO: change from has_invalid_neighbours to isOnTheEdgeOfValidArea due to functionality change
    bool effector_boundaries::has_invalid_neighbours(const bitmap_coordinate & p) const
    {
    	//check if the voxel is valid and on the edge of the box
    	if(is_valid(bitmap_coordinate(p.x,p.y,p.z))){
    		//voxel is on the edge of the box, automatically boardering invalid territory
    		if(p.x == 0 
    			|| p.x == width 
    			|| p.y == 0 
    			|| p.y == depth 
    			|| p.z == 0 
    			|| p.z == height){
    			return true;
    		}

	        //check all voxels in the 3x3x3 box around voxel p
	        for(int y = p.y - 1; y <= p.y + 1; y++){
	            for(int x = p.x - 1; x <= p.x + 1; x++){
	                for(int z = p.z - 1; z <= p.z + 1; z++){
	                    //check if one of the neighbours is not valid
	                    if(!is_valid(bitmap_coordinate(x, y, z))){
	                        return true;
	                    }
	                }
	            }
	        }
	    }
	    /**
	     * voxel is invalid OR 
	     * inside a box of 3x3x3 valid voxels
	     */
        return false;
    }

    bool effector_boundaries::is_valid(const bitmap_coordinate & p) const
    {
    	char* from_cache;
    	char dummy = UNKNOWN;
    	if(point_validity_cache == NULL)
    	{
    		from_cache = &dummy;
    	}
    	else
    	{
    		from_cache = &point_validity_cache[p.x + p.y * width + p.z * width * depth];
    	}

    	if(*from_cache == UNKNOWN)
    	{
			motionf mf;
			try
			{
				kinematics.pointToMotion(from_bitmap_coordinate(p), mf);
			}
			catch(huniplacer::InverseKinematicsException & ex)
			{
				*from_cache = INVALID;
				return false;
			}
			for(int i = 0;i < 3;i++)
			{
				if(mf.angles[i] <= motors.get_min_angle() || mf.angles[i] >= motors.get_max_angle()){
					*from_cache = INVALID;
					return false;
				}
			}

			*from_cache = VALID;
			return true;
    	}
    	else
    	{
    		return *from_cache == VALID;
    	}
    }

    void effector_boundaries::generate_boundaries_bitmap()
    {
    	point_validity_cache = new char[width * depth * height];
    	memset(point_validity_cache, 0, width * depth * height * sizeof(char));
    	std::stack<bitmap_coordinate> cstack;

    	//determine the center of the box
    	Point3D begin (0, 0, MIN_Z + (MAX_Z - MIN_Z) / 2);
    	//if begin pixel is not part of a valid voxel the box dimensions are incorrect
    	if(!is_valid(from_real_coordinate(begin))){
    		//TODO: throw exception
    	}
    	
    	//scan towards the right
		for(; begin.x < MAX_X; begin.x += voxel_size)
		{
			/**
			 * If an invalid voxel is found:
			 * Step back to the last valid voxel
			 * push the voxel on the empty stack
			 * set the voxel as true in the bitmap
			 * end the loop
			 */
			if(!is_valid(from_real_coordinate(begin))){
				begin.x -= voxel_size;
				bitmap_coordinate startingVoxel = from_real_coordinate(begin);
				cstack.push(startingVoxel);
				boundaries_bitmap[startingVoxel.x + startingVoxel.y * width + startingVoxel.z * width * depth] = true;
				break;
			}
		}
		/**
		 * If the right-most voxel is in reach and an invalid voxel is never found:
		 * the position of begin.x will be outside of the box limits
		 * Step back inside the box and add that voxel to the stack and set it as true in the bitmap
		 */
		if(begin.x >= MAX_X){
			begin.x -= voxel_size;
			bitmap_coordinate startingVoxel = from_real_coordinate(begin);
			cstack.push(startingVoxel);
			boundaries_bitmap[startingVoxel.x 
				+ startingVoxel.y * width 
				+ startingVoxel.z * width * depth] = true;
		}

		/**
		 * start with the last added voxel on the stack and add new voxels to the stack
		 * do this until the valid borders (all valid voxels bordering unvalid voxels or 
		 * the MAX/MIN_X/Y/Z box) of the valid voxel area are known (stack = empty)
		 */
		while(!cstack.empty())
		{
			//get last added voxel from the stack and remove it from the stack
			bitmap_coordinate borderVoxel = cstack.top();
			cstack.pop();

			//check all neighbours of the voxel
			for(int y = borderVoxel.y-1; y <= borderVoxel.y+1; y++){
				for(int x = borderVoxel.x-1; x <= borderVoxel.x+1; x++){
					for(int z = borderVoxel.z-1; z <= borderVoxel.z+1; z++){
						//don't do anything with voxels outside of the MAX/MIN_X/Y/Z box
						if(z > height || z < 0 || x > width || x < 0 || y > depth || y < 0){
							continue;
						} else {
							/**
							 * new valid voxels on the valid border are 
							 * added to the stack and set in the bitmap
							 */
							int index = x + y * width + z * width * depth;
							if(is_valid(bitmap_coordinate(x, y, z))
									&& !boundaries_bitmap[index]
								    && has_invalid_neighbours(bitmap_coordinate(x, y, z))){
								cstack.push(bitmap_coordinate(x, y, z));
								boundaries_bitmap[index] = true;
							}
						}
					}
				}
			}
		}

		delete[] point_validity_cache;
		point_validity_cache = NULL;

		//adds all the points within the boundaries
		cstack.push(from_real_coordinate(Point3D(0, 0, MIN_Z + (MAX_Z - MIN_Z) / 2)));
		while(!cstack.empty())
		{
			bitmap_coordinate validVoxel = cstack.top();
			cstack.pop();

			int index = validVoxel.x + 
						validVoxel.y * width + 
						validVoxel.z * width * depth;
			int indices[6] = {index - 1, index + 1, index - width, index + width, index - width * depth, index + width * depth};

			for(unsigned int i = 0; i < ( sizeof(indices) / sizeof(indices[0]) ); i++)
			{
				if(boundaries_bitmap[indices[i]] == false){
					boundaries_bitmap[indices[i]] = true;
					cstack.push(bitmap_coordinate(indices[i] % width, (indices[i] % (width * depth)) / width, indices[i] / (width * depth)));
				}
			}	
		}
	}
}
