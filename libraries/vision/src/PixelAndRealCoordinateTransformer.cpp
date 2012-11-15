/**
 * @file PixelAndRealCoordinateTransformer.cpp
 * @brief Library to convert real-life coordinates to pixel coordinates and vice versa
 * @date Created: 2011-11-11
 *
 * @author Kasper van Nieuwland
 * @author Zep Mouris
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

#include <Vision/PixelAndRealCoordinateTransformer.h>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <float.h>
#include <list>
#include <iostream>

namespace Vision {
	/**
	 * constructor for a PixelAndRealCoordinateTransformer
	 *
	 * @param fiducialsRealCoordinates A vector with the real world coordinates of the fiducials in the same order as fiducialsPixelCoordinates.
	 * @param fiducialsPixelCoordinates A vector with the pixel coordinates of the fiducials in the same order as fiducialsRealCoordinates.
	 **/
	PixelAndRealCoordinateTransformer::PixelAndRealCoordinateTransformer(const std::vector<DataTypes::Point2D>& fiducialsRealCoordinates, const std::vector<DataTypes::Point2D>& fiducialsPixelCoordinates) :
			alpha(0), scale(0), offsetX(0), offsetY(0), mirrored(false), fiducialsRealCoordinates(fiducialsRealCoordinates), fiducialsPixelCoordinates(fiducialsPixelCoordinates) {
		updateTransformationParameters();
	}
	/**
	 * Destructor.
	 **/
	PixelAndRealCoordinateTransformer::~PixelAndRealCoordinateTransformer( ) {
	}
	/**
	 * Function for updating the pixel coordinates of the fiducials.
	 * 
	 * @param fiducialsPixelCoordinates The new pixel coordinates for the fiducials in a vector.
	 **/
	void PixelAndRealCoordinateTransformer::setFiducialPixelCoordinates(const std::vector<DataTypes::Point2D>& fiducialsRealCoordinates) {
		this->fiducialsPixelCoordinates = fiducialsRealCoordinates;
		updateTransformationParameters();
	}

	/**
	 * Converts pixel coordinates to real coordinates. The math is just a worked out version of a conversion matrix (translation, rotation, scaling and mirroring). Mirroring is available because one camera is on the bottom and one is on the top.
	 *
	 * @param pixelCoordinate The input coordinate that will be converted.
	 *
	 * @return The real coordinate.
	 **/
	DataTypes::Point2D PixelAndRealCoordinateTransformer::pixelToRealCoordinate(const DataTypes::Point2D & pixelCoordinate) const {
		int pixelCoordinateY = pixelCoordinate.y * -1;

		DataTypes::Point2D realCoordinate;

		realCoordinate.x = pixelToRealCoordinateScale * (cos(pixelToRealCoordinateAlpha) * (pixelCoordinate.x - pixelToRealCoordinateA) + sin(pixelToRealCoordinateAlpha) * (pixelCoordinateY - pixelToRealCoordinateB));
		realCoordinate.y = pixelToRealCoordinateScale * (-sin(pixelToRealCoordinateAlpha) * (pixelCoordinate.x - pixelToRealCoordinateA) + cos(pixelToRealCoordinateAlpha) * (pixelCoordinateY - pixelToRealCoordinateB));

		if (mirrored) {
			double temporaryX = fiducialsRealCoordinates[0].x - fiducialsRealCoordinates[0].x * cos(2 * realAlpha) + realCoordinate.x * cos(2 * realAlpha) - fiducialsRealCoordinates[0].y * sin(2 * realAlpha) + realCoordinate.y * sin(2 * realAlpha);
			realCoordinate.y = fiducialsRealCoordinates[0].y + fiducialsRealCoordinates[0].y * cos(2 * realAlpha) - realCoordinate.y * cos(2 * realAlpha) - fiducialsRealCoordinates[0].x * sin(2 * realAlpha) + realCoordinate.x * sin(2 * realAlpha);
			realCoordinate.x = temporaryX;
		}
		return realCoordinate;
	}

	/**
	 * Converts real coordinates to pixel coordinates. The math is just a worked out version of a conversion matrix (translation, rotation, scaling and mirroring). Mirroring is available because one camera is on the bottom and one is on the top.
	 *
	 * @param realCoordinate The input coordinate that will be converted.
	 *
	 * @return The pixel coordinate.
	 **/
	DataTypes::Point2D PixelAndRealCoordinateTransformer::realToPixelCoordinate(const DataTypes::Point2D& realCoordinate) const {
		DataTypes::Point2D pixelCoordinate;
		pixelCoordinate = realCoordinate;
		double temporaryX;

		if (mirrored) {
			temporaryX = fiducialsRealCoordinates[0].x - fiducialsRealCoordinates[0].x * cos(2 * realAlpha) + pixelCoordinate.x * cos(2 * realAlpha) - fiducialsRealCoordinates[0].y * sin(2 * realAlpha) + pixelCoordinate.y * sin(2 * realAlpha);
			pixelCoordinate.y = fiducialsRealCoordinates[0].y + fiducialsRealCoordinates[0].y * cos(2 * realAlpha) - pixelCoordinate.y * cos(2 * realAlpha) - fiducialsRealCoordinates[0].x * sin(2 * realAlpha) + pixelCoordinate.x * sin(2 * realAlpha);
			pixelCoordinate.x = temporaryX;
		}

		temporaryX = realToPixelCoordinateScale * (cos(realToPixelCoordinateAlpha) * (pixelCoordinate.x - realToPixelCoordinateA) + sin(realToPixelCoordinateAlpha) * (pixelCoordinate.y - realToPixelCoordinateB));
		pixelCoordinate.y = realToPixelCoordinateScale * (-sin(realToPixelCoordinateAlpha) * (pixelCoordinate.x - realToPixelCoordinateA) + cos(realToPixelCoordinateAlpha) * (pixelCoordinate.y - realToPixelCoordinateB));
		pixelCoordinate.x = temporaryX;
		pixelCoordinate.y *= -1;

		return pixelCoordinate;
	}

	/**
	 * Updates scale, mirrored, alpha.
	 * TODO update offsetX and offsetY.
	 **/
	void PixelAndRealCoordinateTransformer::updateTransformationParameters( ) {
		if (fiducialsRealCoordinates.size() != fiducialsPixelCoordinates.size()) {
			throw std::runtime_error("Number of real fiducial coordinates does not match number of pixel fiducials coordinates");
		}
		
		// Determine mirroring by rotating an outlying pixel point (0) around the midpoint (1) with 90 degrees both ways, then checking which rotation is closest to the other outlying pixel point (2).
		DataTypes::Point2D offset = fiducialsPixelCoordinates[0] - fiducialsPixelCoordinates[1];
		// Rotate 90 degrees clockwise
		DataTypes::Point2D mirroredPoint = offset.rotate(Utilities::degreesToRadians(-90));
		// Rotate 90 degrees counter clockwise
		DataTypes::Point2D nonmirroredPoint = offset.rotate(Utilities::degreesToRadians(+90));
		mirroredPoint += fiducialsPixelCoordinates[1];
		nonmirroredPoint += fiducialsPixelCoordinates[1];
		// Check if the image is mirrored by determining which rotation is closest to the actual point 2.
		if (mirroredPoint.distance(fiducialsPixelCoordinates[2]) < nonmirroredPoint.distance(fiducialsPixelCoordinates[2])) {
			mirrored = true;
		} else {
			mirrored = false;
		}

		// Find out the angle of the line between point 1 and 2 (this is the shortest line between the three fiducials)
		double pixelDeltaX = fiducialsPixelCoordinates[1].x - fiducialsPixelCoordinates[2].x;
		double pixelDeltaY = fiducialsPixelCoordinates[1].y - fiducialsPixelCoordinates[2].y;
		// Independent of mirroring, if the line between point 1 and 2 is at 90 degrees, the rotation of the image is 0 degrees
		alpha = atan2(pixelDeltaY, pixelDeltaX) + Utilities::degreesToRadians(-270);
		std::cout << "alpha: " << alpha << ", " << Utilities::radiansToDegrees(alpha) << std::endl;

		// Determine the scale.
		scale = 0;
		int distancesCount = 0;
		for (unsigned int n = 0; n < fiducialsRealCoordinates.size(); n++) {
			for (unsigned int m = n + 1; m < fiducialsRealCoordinates.size(); m++) {
				scale += fiducialsRealCoordinates[n].distance(fiducialsRealCoordinates[m]) / fiducialsPixelCoordinates[n].distance(fiducialsPixelCoordinates[m]);
				distancesCount++;
			}
		}
		scale /= distancesCount;

		offsetX = offsetY = 0;
		DataTypes::Point2D realpoint0 = pixelToRealCoordinate2(fiducialsPixelCoordinates[0]);
		DataTypes::Point2D realpoint1 = pixelToRealCoordinate2(fiducialsPixelCoordinates[1]);
		DataTypes::Point2D realpoint2 = pixelToRealCoordinate2(fiducialsPixelCoordinates[2]);
		offsetX = fiducialsRealCoordinates[0].x - realpoint0.x + fiducialsRealCoordinates[1].x - realpoint1.x + fiducialsRealCoordinates[2].x - realpoint2.x;
		offsetY = fiducialsRealCoordinates[0].y - realpoint0.y + fiducialsRealCoordinates[1].y - realpoint1.y + fiducialsRealCoordinates[2].y - realpoint2.y;
		offsetX /= 3;
		offsetY /= 3;
	}

	/**
	 * Applies a rotation matrix, potential mirroring on y axis, scaling and offsetting.
	 */
	DataTypes::Point2D PixelAndRealCoordinateTransformer::pixelToRealCoordinate2(const DataTypes::Point2D& pixelCoordinate) const {
		DataTypes::Point2D result = pixelCoordinate.rotate(alpha);
		// Worked out rotation matrix
		result.x = pixelCoordinate.x * cos(alpha) + pixelCoordinate.y * sin(alpha);
		result.y = pixelCoordinate.x * (-sin(alpha)) + pixelCoordinate.y * cos(alpha);
		// Mirror on the y axis, which after rotation is always the mirror axis.
		if (mirrored) {
			result.x *= -1;
		}
		//invert y axis because of 0,0 being topleft of a picture and 0,0 being bottomleft of the workspace
		result.y *= -1;
		result.x *= scale;
		result.y *= scale;
		result.x += offsetX;
		result.y += offsetY;

		return result;
	}

	/**
	 *  Inverse from pixelToRealCoordinate2
	 */
	DataTypes::Point2D PixelAndRealCoordinateTransformer::realToPixelCoordinate2(const DataTypes::Point2D& realCoordinate) const {
		DataTypes::Point2D result;
		result.x -= offsetX;
		result.y -= offsetY;
		result.x /= scale;
		result.y /= scale;
		if (mirrored) {
			result.x *= -1;
		}
		double inverseDeterminant = 1 / (cos(alpha) * cos(alpha) - sin(alpha) * -sin(alpha));
		result.x = realCoordinate.x * inverseDeterminant * cos(alpha) + realCoordinate.y * inverseDeterminant * -sin(alpha);
		result.y = realCoordinate.x * inverseDeterminant * sin(alpha) + realCoordinate.y * inverseDeterminant * cos(alpha);

		return result;
	}
}
