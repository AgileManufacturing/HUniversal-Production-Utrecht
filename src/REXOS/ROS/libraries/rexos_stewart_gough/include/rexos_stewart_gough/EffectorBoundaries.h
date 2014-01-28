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
#include <rexos_motor/StepperMotorProperties.h>
#include <rexos_datatypes/DeltaRobotMeasures.h>
#include <rexos_stewart_gough/Measures.h>
#include <rexos_stewart_gough/InverseKinematicsModel.h>
#include <vector>

namespace rexos_stewart_gough{
	/**
	 * This class represents a delta robot's effector work field.
	 * This work field is stored as a 3D bitmap (bool array).
	 * Every "pixel" in this map is called a voxel.
	 **/
	class EffectorBoundaries{
	public:
		~EffectorBoundaries();
		
		static EffectorBoundaries* generateEffectorBoundaries(const InverseKinematicsModel& model, 
				const rexos_datatypes::DeltaRobotMeasures* deltaRobotMeasures, 
				const std::vector<rexos_motor::StepperMotor*> motors, double voxelSize);

		bool checkPath(const rexos_datatypes::Point3D<double>& from, const rexos_datatypes::Point3D<double>& to) const;

	private:
		EffectorBoundaries(const InverseKinematicsModel& model, const rexos_datatypes::DeltaRobotMeasures* deltaRobotMeasures, 
				const std::vector<rexos_motor::StepperMotor*> motors, double voxelSize);

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
					(double) coordinate.x * voxelSize + deltaRobotMeasures->boundaryBoxMinX,
					(double) coordinate.y * voxelSize + deltaRobotMeasures->boundaryBoxMinY,
					(double) coordinate.z * voxelSize + deltaRobotMeasures->boundaryBoxMinZ);
		}

		/**
		 * Converts a real life coordinate to a bitmap coordinate.
		 * 
		 * @param coordinate The real life coordinate.
		 **/
		inline EffectorBoundaries::BitmapCoordinate fromRealCoordinate(rexos_datatypes::Point3D<double> coordinate) const{
			return EffectorBoundaries::BitmapCoordinate(
				(coordinate.x - deltaRobotMeasures->boundaryBoxMinX) / voxelSize,
				(coordinate.y - deltaRobotMeasures->boundaryBoxMinY) / voxelSize,
				(coordinate.z - deltaRobotMeasures->boundaryBoxMinZ) / voxelSize);
		}

		/**
		 * An enum holding the values an entry in the pointValidityCache can have. They indicate whether a point fits within the boundaries or not, or if this has not yet been determined.
		 **/
		enum entry{
			UNKNOWN,
			VALID,
			INVALID,
			BORDER
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
		 * @var int8_t* boundariesBitmap
		 * A pointer to the boundaries bitmap. This bitmap is a one dimensional array of booleans, defaulted to false, where voxels are checked and set to true if they are reachable.
		 **/
		entry* boundariesBitmap;

		/**
		 * @var InverseKinematicsModel& kinematics
		 * A reference to the InverseKinematicsModel of the deltarobot, which is used to calculate the boundaries.
		 **/
		const InverseKinematicsModel &kinematics;
		/**
		 * @var rexos_datatypes::DeltaRobotMeasures& deltaRobotMeasures
		 * A reference to the InverseKinematicsModel of the deltarobot, which is used to calculate the boundaries.
		 **/
		const rexos_datatypes::DeltaRobotMeasures* deltaRobotMeasures;
		/**
		 * @var rexos_motor::StepperMotorProperties* stepperMotorProperties[3]
		 * A reference to the InverseKinematicsModel of the deltarobot, which is used to calculate the boundaries.
		 **/
		const std::vector<rexos_motor::StepperMotor*> motors;
		
		/**
		 * @var double voxelSize
		 * The size of the voxels in the boundary bitmap.
		 **/
		double voxelSize;
	};
}
