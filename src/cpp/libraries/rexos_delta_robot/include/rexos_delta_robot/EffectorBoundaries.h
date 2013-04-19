/**
 * @file EffectorBoundaries.h
 * @brief Represents the effector's moving volume.
 *
 * @author 1.0 Lukas Vermond
 * @author 1.0 Kasper van Nieuwland
 * @author 1.1 Koen Braham
 * @author 1.1 Daan Veltman
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

#pragma once

#include <iostream>
#include <rexos_datatypes/Point3D.h>
#include <rexos_motor/StepperMotor.h>
#include <rexos_delta_robot/Measures.h>
#include <rexos_delta_robot/InverseKinematicsModel.h>
#include <vector>

namespace rexos_delta_robot{
	/**
	 * This class represents a delta robot's effector work field.
	 * This work field is stored as a 3D bitmap (bool array).
	 * Every "pixel" in this map is called a voxel.
	 **/
	class EffectorBoundaries{
	public:
		~EffectorBoundaries();
		
		static EffectorBoundaries* generateEffectorBoundaries(const InverseKinematicsModel& model, double motorMinAngles[3], double motorMaxAngles[3], double voxelSize);

		bool checkPath(const rexos_datatypes::Point3D<double>& from, const rexos_datatypes::Point3D<double>& to) const;

	private:
		EffectorBoundaries(const InverseKinematicsModel& model, double motorMinAngles[3], double motormaxAngles[3], double voxelSize);

		/**
		 * Represents a 3-dimensional point in the 3D voxel array.
		 **/
		typedef struct BitmapCoordinate{
			/**
			 * @var int x
			 * The x coordinate of the point.
			 **/
			int x;

			/**
			 * @var int y
			 * The y coordinate of the point.
			 **/
			int y;

			/**
			 * @var int z 
			 * The z coordinate of the point.
			 **/ 
			int z;

			/**
			 * Constructor for the BitmapCoordinate.
			 * 
			 * @param x The x coordinate of the point.
			 * @param y The y coordinate of the point
			 * @param z The z coordinate of the point.
			 **/
			BitmapCoordinate(int x, int y, int z) : x(x), y(y), z(z){}
		} BitmapCoordinate;

		bool hasInvalidNeighbours(const BitmapCoordinate& coordinate, char* pointValidityCache) const;
		bool isValid(const BitmapCoordinate& coordinate, char* pointValidityCache) const;
		void generateBoundariesBitmap();

		/**
		 * Converts a bitmap coordinate to a real life coordinate.
		 * 
		 * @param coordinate The bitmap coordinate.
		 **/
		inline rexos_datatypes::Point3D<double> fromBitmapCoordinate(EffectorBoundaries::BitmapCoordinate coordinate) const{
			return rexos_datatypes::Point3D<double>(
					(double) coordinate.x * voxelSize + Measures::BOUNDARY_BOX_MIN_X,
					(double) coordinate.y * voxelSize + Measures::BOUNDARY_BOX_MIN_Y,
					(double) coordinate.z * voxelSize + Measures::BOUNDARY_BOX_MIN_Z);
		}

		/**
		 * Converts a real life coordinate to a bitmap coordinate.
		 * 
		 * @param coordinate The real life coordinate.
		 **/
		inline EffectorBoundaries::BitmapCoordinate fromRealCoordinate(rexos_datatypes::Point3D<double> coordinate) const{
			return EffectorBoundaries::BitmapCoordinate(
				(coordinate.x - Measures::BOUNDARY_BOX_MIN_X) / voxelSize,
				(coordinate.y - Measures::BOUNDARY_BOX_MIN_Y) / voxelSize,
				(coordinate.z - Measures::BOUNDARY_BOX_MIN_Z) / voxelSize);
		}

		/**
		 * An enum holding the values an entry in the pointValidityCache can have. They indicate whether a point fits within the boundaries or not, or if this has not yet been determined.
		 **/
		enum cacheEntry{
			UNKNOWN,
			VALID,
			INVALID
		};

		/**
		 * @var int width
		 * The width of the boundary bitmap.
		 **/
		int width;
		
		/**
		 * @var int height
		 * The height of the boundary bitmap.
		 **/
		int height; 

		/**
		 * @var int depth
		 * The depth of the boundary bitmap.
		 **/
		int depth;

		/**
		 * @var bool* boundariesBitmap
		 * A pointer to the boundaries bitmap. This bitmap is a one dimensional array of booleans, defaulted to false, where voxels are checked and set to true if they are reachable.
		 **/
		bool* boundariesBitmap;

		/**
		 * @var InverseKinematicsModel& kinematics
		 * A reference to the InverseKinematicsModel of the deltarobot, which is used to calculate the boundaries.
		 **/
		const InverseKinematicsModel &kinematics;

		/**
		 * @var double motorMinAngles 
		 * Holds the minimum angles of the motors.
		 **/
		double motorMinAngles[3];

		/**
		 * @var double motorMaxAngles 
		 * Holds the maximum angles of the motors.
		 **/
		double motorMaxAngles[3];

		/**
		 * @var double voxelSize
		 * The size of the voxels in the boundary bitmap.
		 **/
		double voxelSize;
	};
}
