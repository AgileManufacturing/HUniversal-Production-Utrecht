//******************************************************************************
//
//                 REXOS
//
//******************************************************************************
// Project:        DeltaRobot
// File:           EffectorBoundaries.h
// Description:    represents the effector's moving volume
// Author:         1.0 Lukas Vermond & Kasper van Nieuwland
//				   1.1 Koen Braham		Daan Veltman
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
#include <DataTypes/Point3D.h>
#include <Motor/StepperMotor.h>
#include <DeltaRobot/Measures.h>
#include <DeltaRobot/InverseKinematicsModel.h>
#include <vector>

namespace DeltaRobot
{
	/**
	 * This class represents a delta robot's effector work field.
	 * This work field is stored as a 3D bitmap (bool array).
	 * Every "pixel" in this map is called a voxel.
	 **/
	class EffectorBoundaries
	{
	public:
		~EffectorBoundaries();

		// TODO: change Motor::StepperMotor to Motor::MotorInterface
		static EffectorBoundaries* generateEffectorBoundaries(const InverseKinematicsModel& model,  Motor::StepperMotor* (&motors)[3], double voxelSize);
		
		bool checkPath(const DataTypes::Point3D<double>& from, const DataTypes::Point3D<double>& to) const;
		
		/*
		 * Returns a const bool pointer to the boundaries bitmap.
		 * @return Const bool pointer to the boundaries bitmap.
		 */
		inline const bool* getBitmap() const;
		
		/**
		 * Gets the width of the boundary bitmap.
		 * @return returns The width of the boundary bitmap.
		 **/
		inline int getWidth() const;
		
		/**
		 * Gets the height of the boundary bitmap.
		 * @return returns The height of the boundary bitmap.
		 **/
		inline int getHeight() const;
		
		/**
		 * Gets the depth of the boundary bitmap.
		 * @return returns The depth of the boundary bitmap.
		 **/
		inline int getDepth() const;
		
		/**
		 * Gets the used voxel_size for this boundary bitmap.
		 * @return returns The used voxel_size.
		 **/
		inline double getVoxelSize() const;

	private:
		
		EffectorBoundaries(const InverseKinematicsModel& model,  Motor::StepperMotor* (&motors)[3], double voxelSize);

		/**
		 * Represents a 3-dimensional point in the 3D voxel array.
		 **/
		typedef struct BitmapCoordinate {
			int x, y, z;
			BitmapCoordinate(int x, int y, int z) : x(x), y(y), z(z) {}
		} BitmapCoordinate;

		bool hasInvalidNeighbours(const BitmapCoordinate& p, char* pointValidityCache) const;
		

		bool isValid(const BitmapCoordinate& p, char* pointValidityCache) const;
		
		void generateBoundariesBitmap();
		
		/**
		 * Converts a bitmap coordinate to a real life coordinate.
		 * @param coordinate The bitmap coordinate.
		 **/
		inline DataTypes::Point3D<double> fromBitmapCoordinate(EffectorBoundaries::BitmapCoordinate coordinate) const;
		
		/**
		 * Converts a real life coordinate to a bitmap coordinate.
		 * @param coordinate The real life coordinate.
		 **/
		inline BitmapCoordinate fromRealCoordinate(DataTypes::Point3D<double> coordinate) const;

		enum cacheEntry
		{
			UNKNOWN,
			VALID,
			INVALID
		};

		int width, height, depth;

		bool* boundariesBitmap;
		const InverseKinematicsModel &kinematics;
		Motor::StepperMotor* (&motors)[3];
		double voxelSize;


	};

	const bool* EffectorBoundaries::getBitmap() const {return boundariesBitmap;}
	int EffectorBoundaries::getDepth() const {return depth;}
	int EffectorBoundaries::getHeight() const {return height;}
	int EffectorBoundaries::getWidth() const {return width;}
	double EffectorBoundaries::getVoxelSize() const {return voxelSize;}

	DataTypes::Point3D<double> EffectorBoundaries::fromBitmapCoordinate(EffectorBoundaries::BitmapCoordinate coordinate) const
	{
		return DataTypes::Point3D<double>(
				(double) coordinate.x * voxelSize + Measures::MIN_X,
				(double) coordinate.y * voxelSize + Measures::MIN_Y,
				(double) coordinate.z * voxelSize + Measures::MIN_Z);
	}

	EffectorBoundaries::BitmapCoordinate EffectorBoundaries::fromRealCoordinate(DataTypes::Point3D<double> coordinate) const
	{
		return EffectorBoundaries::BitmapCoordinate(
			(coordinate.x - Measures::MIN_X) / voxelSize,
			(coordinate.y - Measures::MIN_Y) / voxelSize,
			(coordinate.z - Measures::MIN_Z) / voxelSize);
	}
}
