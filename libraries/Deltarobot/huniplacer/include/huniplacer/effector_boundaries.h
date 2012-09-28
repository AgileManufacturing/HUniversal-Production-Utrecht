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
	/**
	 * This class represents a delta robot's effector work field.
	 * This work field is stored as a 3D bitmap (bool array).
	 * Every "pixel" in this map is called a voxel.
	 **/
	class effector_boundaries
	{
	public:
		~effector_boundaries();

		static effector_boundaries* generate_effector_boundaries(const InverseKinematicsModel& model, const imotor3& motors, double voxel_size);
		
		bool check_path(const Point3D& from, const Point3D& to) const;
		
		/*
		 * Returns a const bool pointer to the boundaries bitmap.
		 * @return Const bool pointer to the boundaries bitmap.
		 */
		inline const bool* get_bitmap() const;
		
		/**
		 * Gets the width of the boundary bitmap.
		 * @return returns The width of the boundary bitmap.
		 **/
		inline int get_width() const;
		
		/**
		 * Gets the height of the boundary bitmap.
		 * @return returns The height of the boundary bitmap.
		 **/
		inline int get_height() const;
		
		/**
		 * Gets the depth of the boundary bitmap.
		 * @return returns The depth of the boundary bitmap.
		 **/
		inline int get_depth() const;
		
		/**
		 * Gets the used voxel_size for this boundary bitmap.
		 * @return returns The used voxel_size.
		 **/
		inline double get_voxel_size() const;

	private:
		
		effector_boundaries(const InverseKinematicsModel& model, const imotor3& motors, double voxel_size);

		/**
		 * Represents a 3-dimensional point in the 3D voxel array.
		 **/
		typedef struct bitmap_coordinate { int x, y, z; bitmap_coordinate(int x, int y, int z) : x(x), y(y), z(z) {} } bitmap_coordinate;

		bool has_invalid_neighbours(const bitmap_coordinate& p, char* point_validity_cache) const;
		

		bool is_valid(const bitmap_coordinate& p, char* point_validity_cache) const;
		
		void generate_boundaries_bitmap();
		
		/**
		 * Converts a bitmap coordinate to a real life coordinate.
		 * @param coordinate The bitmap coordinate.
		 **/
		inline Point3D from_bitmap_coordinate(effector_boundaries::bitmap_coordinate coordinate) const;
		
		/**
		 * Converts a real life coordinate to a bitmap coordinate.
		 * @param coordinate The real life coordinate.
		 **/
		inline bitmap_coordinate from_real_coordinate(Point3D coordinate) const;

		enum cache_entry
		{
			UNKNOWN,
			VALID,
			INVALID
		};

		int width, height, depth;

		bool* boundaries_bitmap;
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
