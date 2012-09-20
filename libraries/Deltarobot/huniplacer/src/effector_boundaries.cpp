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

		boundaries->width = (double)((MAX_X - MIN_X)) / voxel_size;
        boundaries->height = (double)((MAX_Z - MIN_Z)) / voxel_size;
        boundaries->depth = (double)((MAX_Y - MIN_Y)) / voxel_size;
        boundaries->boundaries_bitmap = new bool[boundaries->width * boundaries->height * boundaries->depth];

        for(int i = 0; i < boundaries->width * boundaries->height * boundaries->depth; i++)
        {
        	boundaries->boundaries_bitmap[i] = false;
        }
        char c;
        std::cin >> c;
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

    bool effector_boundaries::has_invalid_neighbours(const bitmap_coordinate & p) const
    {
        for(int y = p.y - 1; y <= p.y + 1; y++)
        {
            for(int x = p.x - 1; x <= p.x + 1; x++)
            {
                for(int z = p.z - 1; z <= p.z + 1; z++)
                {
                    if((x != p.x || y != p.y || z != p.z) && !is_valid(bitmap_coordinate(x, y, z)))
                    {
                        return true;
                    }
                }
            }
        }
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

    	//search from the center to the right side to the first point out of reach
    	Point3D begin (0, 0, MIN_Z + (MAX_Z - MIN_Z) / 2);
		for(; begin.x < MAX_X; begin.x += voxel_size)
		{
			if(!is_valid(from_real_coordinate(begin)))
			{
				begin.x -= voxel_size;
				bitmap_coordinate coordinate = from_real_coordinate(begin);
				cstack.push(coordinate);
				boundaries_bitmap[coordinate.x + coordinate.y * width + coordinate.z * width * depth] = true;
				break;
			}
		}

		//starts with the found point and adds all the point on the boundaries
		while(!cstack.empty())
		{
			bitmap_coordinate c = cstack.top();
			cstack.pop();

			for(int y = c.y-1; y <= c.y+1; y++)
			{
				for(int x = c.x-1; x <= c.x+1; x++)
				{
					for(int z = c.z-1; z <= c.z+1; z++)
					{
						if(z >= height-1 || z < 1 || x >= width-1 || x < 1 || y >= depth-1 || y < 1)
						{
							continue;
						}

						int index = x + y * width + z * width * depth;
						Point3D real_coordinate = from_bitmap_coordinate(bitmap_coordinate(x, y, z));
						if(is_valid(bitmap_coordinate(x, y, z))
								&& (real_coordinate.x < measures::MAX_X 
								&& real_coordinate.x >= measures::MIN_X 
								&& real_coordinate.y < measures::MAX_Y 
								&& real_coordinate.y >= measures::MIN_Y 
								&& real_coordinate.z < measures::MAX_Z 
								&& real_coordinate.z >= measures::MIN_Z)
								&& !boundaries_bitmap[index]
							    && has_invalid_neighbours(bitmap_coordinate(x, y, z)))
						{
							cstack.push(bitmap_coordinate(x, y, z));
							boundaries_bitmap[index] = true;
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
			bitmap_coordinate c = cstack.top();
			cstack.pop();
			//std::cout << "after cstack pop" << std::endl;
			int index = c.x + c.y * width + c.z * width * depth;
			int indices[6] = {index - 1, index + 1, index - width, index + width, index - width * depth, index + width * depth};
			



			for(unsigned int i = 0; i < ( sizeof(indices) / sizeof(indices[0]) ); i++)
			{
				if(indices[i] >= (width * height * depth))
				{
					std::cerr << "line=" << __LINE__ << " index=" << indices[i] << " out of bounds because maxsize/index = " << double(width*height*depth)/double(indices[1]) << " ";
				}
				else if(boundaries_bitmap[indices[i]] == false)
				{
					boundaries_bitmap[indices[i]] = true;
					cstack.push(bitmap_coordinate(indices[i] % width, (indices[i] % (width * depth)) / width, indices[i] / (width * depth)));
				    
				}
			}	
		}
	}
}
