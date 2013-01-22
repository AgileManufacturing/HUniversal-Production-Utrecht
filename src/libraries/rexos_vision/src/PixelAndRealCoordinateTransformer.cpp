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

#include <rexos_vision/PixelAndRealCoordinateTransformer.h>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <float.h>
#include <list>
#include <iostream>

namespace rexos_vision{
	/**
	 * constructor for a PixelAndRealCoordinateTransformer
	 *
	 * @param fiducialsRealCoordinates A vector with the real world coordinates of the fiducials in the same order as fiducialsPixelCoordinates.
	 * @param fiducialsPixelCoordinates A vector with the pixel coordinates of the fiducials in the same order as fiducialsRealCoordinates.
	 **/
	PixelAndRealCoordinateTransformer::PixelAndRealCoordinateTransformer(const std::vector<rexos_datatypes::Point2D>& fiducialsRealCoordinates,
		const std::vector<rexos_datatypes::Point2D>& fiducialsPixelCoordinates) : fiducialsRealCoordinates(fiducialsRealCoordinates), fiducialsPixelCoordinates(fiducialsPixelCoordinates){
		updateTransformationParameters();
	}
	/**
	 * Destructor.
	 **/
	PixelAndRealCoordinateTransformer::~PixelAndRealCoordinateTransformer(){
	}
	/**
	 * Function for updating the pixel coordinates of the fiducials.
	 * 
	 * @param fiducialsRealCoordinates The new pixel coordinates for the fiducials in a vector.
	 **/
	void PixelAndRealCoordinateTransformer::setFiducialPixelCoordinates(const std::vector<rexos_datatypes::Point2D>& fiducialsRealCoordinates){
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
	rexos_datatypes::Point2D PixelAndRealCoordinateTransformer::pixelToRealCoordinate(const rexos_datatypes::Point2D & pixelCoordinate) const{
		int pixelCoordinateY = pixelCoordinate.y * -1;

		rexos_datatypes::Point2D realCoordinate;
		
		realCoordinate.x = pixelToRealCoordinateScale * (cos(pixelToRealCoordinateAlpha) * (pixelCoordinate.x - pixelToRealCoordinateA) + sin(pixelToRealCoordinateAlpha) * (pixelCoordinateY - pixelToRealCoordinateB));
		realCoordinate.y = pixelToRealCoordinateScale * (-sin(pixelToRealCoordinateAlpha) * (pixelCoordinate.x - pixelToRealCoordinateA) + cos(pixelToRealCoordinateAlpha) * (pixelCoordinateY - pixelToRealCoordinateB));
		
		if(mirrored){
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
	rexos_datatypes::Point2D PixelAndRealCoordinateTransformer::realToPixelCoordinate(const rexos_datatypes::Point2D& realCoordinate) const{
		rexos_datatypes::Point2D pixelCoordinate;
		pixelCoordinate = realCoordinate;
		double temporaryX;
		
		if(mirrored){
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
	 * Updates realToPixelCoordinateScale, pixelToRealCoordinateScale, realAlpha, pixelAlpha, realToPixelCoordinateAlpha, pixelToRealCoordinateAlpha and mirrored.
	 **/
	void PixelAndRealCoordinateTransformer::updateTransformationParameters(){
		if(fiducialsRealCoordinates.size() != fiducialsPixelCoordinates.size())
			throw std::runtime_error("Number of real fiducial coordinates does not match number of pixel fiducials coordinates");
		double scale = 0;
		int distancesCount = 0;
		for(unsigned int n = 0; n < fiducialsRealCoordinates.size(); n++)
		{
			for(unsigned int m = n + 1; m < fiducialsRealCoordinates.size(); m++)
			{
				scale += fiducialsRealCoordinates[n].distance(fiducialsRealCoordinates[m]) / fiducialsPixelCoordinates[n].distance(fiducialsPixelCoordinates[m]);
				distancesCount++;
			}
		}
		scale /= distancesCount;
		realToPixelCoordinateScale = 1 / scale;
		pixelToRealCoordinateScale = scale;
		
		double pixelX = fiducialsPixelCoordinates[0].x;
		double pixelY = -fiducialsPixelCoordinates[0].y;
		double realX = fiducialsRealCoordinates[0].x;
		double realY = fiducialsRealCoordinates[0].y;
		
		double pixelDeltaX = fiducialsPixelCoordinates[2].x - pixelX;
		double pixelDeltaY = -fiducialsPixelCoordinates[2].y - pixelY;
		
		double realDeltaX = fiducialsRealCoordinates[2].x - realX;
		double realDeltaY = fiducialsRealCoordinates[2].y - realY;
		
		realAlpha = atan2(realDeltaY, realDeltaX);
		pixelAlpha = atan2(pixelDeltaY, pixelDeltaX);
		realToPixelCoordinateAlpha = realAlpha - pixelAlpha;
		pixelToRealCoordinateAlpha = pixelAlpha - realAlpha;
		
		double rcos = cos(realToPixelCoordinateAlpha);
		double rcos2 = pow(rcos, 2);
		double rsin = sin(realToPixelCoordinateAlpha);
		double rsin2 = pow(rsin, 2);
		double rtan = tan(realToPixelCoordinateAlpha);
		
		realToPixelCoordinateA = realX - (rcos * pixelX - rsin * pixelY) / (realToPixelCoordinateScale * (rcos2 + rsin2));
		realToPixelCoordinateB = realY - pixelY / (realToPixelCoordinateScale * rcos) - rtan * (realX - realToPixelCoordinateA);
		
		rcos = cos(pixelToRealCoordinateAlpha);
		rcos2 = pow(rcos, 2);
		rsin = sin(pixelToRealCoordinateAlpha);
		rsin2 = pow(rsin, 2);
		rtan = tan(pixelToRealCoordinateAlpha);
		
		pixelToRealCoordinateA = pixelX - (rcos * realX - rsin * realY) / (pixelToRealCoordinateScale * (rcos2 + rsin2));
		pixelToRealCoordinateB = pixelY - realY / (pixelToRealCoordinateScale * rcos) - rtan * (pixelX - pixelToRealCoordinateA);
		
		mirrored = false;
		rexos_datatypes::Point2D test = pixelToRealCoordinate(fiducialsPixelCoordinates[1]);
		
		// Check to see if the 3rd fiducial is within 1 cm of the calculated point, if not so the image is mirrored
		if(!(test.x > fiducialsRealCoordinates[1].x - 10 && test.x < fiducialsRealCoordinates[1].x + 10)){
			mirrored = true;
		} else{
			if(!(test.y > fiducialsRealCoordinates[1].y - 10 && test.y < fiducialsRealCoordinates[1].y + 10)){
				mirrored = true;
			} else{
				mirrored = false;
			}
		}
	}
}
