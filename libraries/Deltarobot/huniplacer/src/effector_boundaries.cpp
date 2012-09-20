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
// License:        GNU GPL v3
//
// This file is part of effector_boundaries.cpp.
//
// effector_boundaries.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// effector_boundaries.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with effector_boundaries.cpp.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#include <huniplacer/measures.h>
#include <huniplacer/effector_boundaries.h>
#include <huniplacer/inverse_kinematics_exception.h>
#include <stack>
#include <vector>
#include <set>
#include <cstring>

namespace huniplacer
{
	using namespace measures;

	effector_boundaries* effector_boundaries::generate_effector_boundaries(const inverse_kinematics_model& model, const imotor3& motors, double voxel_size)
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

        boundaries->generate_boundaries_bitmap();

        return boundaries;
    }

    bool effector_boundaries::check_path(const point3 & from, const point3 & to) const
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
			bitmap_coordinate temp = from_real_coordinate(point3(x, y, z));

			int index = temp.x + temp.y * width + temp.z * width * depth;

			if(!boundaries_bitmap[index])
			{
				return false;
			}
		}

        return true;
    }

    effector_boundaries::effector_boundaries(const inverse_kinematics_model& model, const imotor3& motors, double voxel_size)
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
                    if(x != p.x && y != p.y && z != p.z && !is_valid(bitmap_coordinate(x, y, z)))
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
				kinematics.point_to_motion(from_bitmap_coordinate(p), mf);
			}
			catch(huniplacer::inverse_kinematics_exception & ex)
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
    	point3 begin (0, 0, MIN_Z + (MAX_Z - MIN_Z) / 2);
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
						point3 real_coordinate = from_bitmap_coordinate(bitmap_coordinate(x, y, z));

						if(is_valid(bitmap_coordinate(x, y, z))
								&& (real_coordinate.x < measures::MAX_X && real_coordinate.x >= measures::MIN_X && real_coordinate.y < measures::MAX_Y && real_coordinate.y >= measures::MIN_Y && real_coordinate.z < measures::MAX_Z && real_coordinate.z >= measures::MIN_Z)
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
		cstack.push(from_real_coordinate(point3(0, 0, MIN_Z + (MAX_Z - MIN_Z) / 2)));
		while(!cstack.empty())
		{
			bitmap_coordinate c = cstack.top();
			cstack.pop();

			int index = c.x + c.y * width + c.z * width * depth;
			int indices[6] = {index - 1, index + 1, index - width, index + width, index - width * depth, index + width * depth};

			for(unsigned int i = 0; i < sizeof(indices) / sizeof(indices[0]); i++)
			{
				if(boundaries_bitmap[indices[i]] == false)
				{
					boundaries_bitmap[indices[i]] = true;
					cstack.push(bitmap_coordinate(indices[i] % width, (indices[i] % (width * depth)) / width, indices[i] / (width * depth)));
				}
			}
		}
	}
}
