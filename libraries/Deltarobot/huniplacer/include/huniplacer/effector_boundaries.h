//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           effector_boundaries.h
// Description:    represents the effector's moving volume
// Author:         Lukas Vermond & Kasper van Nieuwland
// Notes:          -
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


#pragma once

#include <iostream>
#include <huniplacer/Point3D.h>
#include <huniplacer/imotor3.h>
#include <huniplacer/measures.h>
#include <huniplacer/InverseKinematicsModel.h>
#include <vector>

namespace huniplacer
{
	/*
	 * This class represents a delta robot's effector work field.
	 * This work field is stored as a 3D bitmap (bool array).
	 * Every "pixel" in this map is called a voxel.
	 */
	class effector_boundaries
	{
	public:
		~effector_boundaries();

		/*
		 * Function to generate the boundaries and returns a pointer to the object
		 * @param model used to calculate the boundaries
		 * @param motors used for the minimum and maximum angle of the motors
		 * @param voxel_size the size of the voxels
		 * @return pointer to the object
		 */
		static effector_boundaries* generate_effector_boundaries(const InverseKinematicsModel& model, const imotor3& motors, double voxel_size);
		/*
		 * Checks if the path from the starting to the destination point is not going out of the
		 * robots boundaries
		 * @param from the starting point
		 * @param to the destination point
		 * @return true if path is valid
		 */
		bool check_path(const Point3D& from, const Point3D& to) const;
		/*
		 * returns a const bool pointer to the boundaries bitmap
		 * @return const bool pointer to the boundaries bitmap
		 */
		inline const bool* get_bitmap() const;
		/*
		 * Gets the width of the boundary bitmap
		 * @return returns the width
		 */
		inline int get_width() const;
		/*
		 * Gets the height of the boundary bitmap
		 * @return returns the height
		 */
		inline int get_height() const;
		/*
		 * Gets the depth of the boundary bitmap
		 * @return returns the depth
		 */
		inline int get_depth() const;
		/*
		 * Gets the used voxel_size for this boundary bitmap
		 * @return returns the used voxel_size
		 */
		inline double get_voxel_size() const;

	private:
		/*
		 * private constructor
		 * also initializes the voxel array
		 * @param model used to calculate the boundaries
		 * @param motors used for the minimum and maximum angle of the motors
		 * @param voxel_size the size of the voxels
		 */
		effector_boundaries(const InverseKinematicsModel& model, const imotor3& motors, double voxel_size);

		/*
		 * represents a location in the 3D voxel array
		 */
		typedef struct bitmap_coordinate { int x, y, z; bitmap_coordinate(int x, int y, int z) : x(x), y(y), z(z) {} } bitmap_coordinate;

		/*
		 * Checks if one of the neighbor voxels can't be reached by the effector
		 * @param p the point in the bitmap
		 */
		bool has_invalid_neighbours(const bitmap_coordinate& p) const;
		/*
		 * Checks if the point is reachable by the effector
		 * @param the point to reach
		 */
		bool is_valid(const bitmap_coordinate& p) const;
		/*
		 * generates boundaries for the robot
		 * all members should be initialized before calling this function
		 */
		void generate_boundaries_bitmap();
		/*
		 * converts a bitmap coordinate to a real life coordinate
		 * @param coordinate the bitmap coordinate
		 */
		inline Point3D from_bitmap_coordinate(effector_boundaries::bitmap_coordinate coordinate) const;
		/*
		 * converts a real life coordinate to a bitmap coordinate
		 * @param coordinate the real life coordinate
		 */
		inline bitmap_coordinate from_real_coordinate(Point3D coordinate) const;

		enum cache_entry
		{
			UNKNOWN,
			VALID,
			INVALID
		};

		int width, height, depth;

		char* point_validity_cache;
		bool* boundaries_bitmap;
		//TODO std::vector<bool> boundaries_bitmap;
		const InverseKinematicsModel &kinematics;
		const imotor3 &motors;
		double voxel_size;


	};

	const bool* effector_boundaries::get_bitmap() const {return boundaries_bitmap;}
	int effector_boundaries::get_depth() const {return depth;}
	int effector_boundaries::get_height() const {return height;}
	int effector_boundaries::get_width() const {return width;}
	double effector_boundaries::get_voxel_size() const {return voxel_size;}

	Point3D effector_boundaries::from_bitmap_coordinate(effector_boundaries::bitmap_coordinate coordinate) const
	{
		return Point3D(
				(double) coordinate.x * voxel_size + measures::MIN_X,
				(double) coordinate.y * voxel_size + measures::MIN_Y,
				(double) coordinate.z * voxel_size + measures::MIN_Z);
	}

	effector_boundaries::bitmap_coordinate effector_boundaries::from_real_coordinate(Point3D coordinate) const
	{
		return effector_boundaries::bitmap_coordinate(
			(coordinate.x - measures::MIN_X) / voxel_size,
			(coordinate.y - measures::MIN_Y) / voxel_size,
			(coordinate.z - measures::MIN_Z) / voxel_size);
	}
}
