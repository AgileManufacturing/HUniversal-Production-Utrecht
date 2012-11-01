/**
 * @file PixelAndRealCoordinateTransformer.h
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

#pragma once
#include <DataTypes/Point2D.h>
#include <vector>

namespace Vision {
	/**
	 * @brief object that transforms pixel coordinates to real life coordinates \
	   using a number of fiducials of which the real life and pixel positions are known.
	 **/
	class PixelAndRealCoordinateTransformer {
		public:
			PixelAndRealCoordinateTransformer(const std::vector<DataTypes::Point2D>& fiducialsRealCoordinates, const std::vector<DataTypes::Point2D>& fiducialsPixelCoordinates);
			virtual ~PixelAndRealCoordinateTransformer( );

			void setFiducialPixelCoordinates(const std::vector<DataTypes::Point2D>& fiducialsPixelCoordinates);

			//TODO pixelToRealCoordinate en realToPixelCoordinate zijn qua structuur hetzelfde, er zou een template functie moeten komen die gebruikt kan worden voor deze 2 functies

			//TODO bij pixelToRealCoordinate en realToPixelCoordinate mischien nog functies maken of aanpassen met out-parameters ipv een punt returnen over de stack. bijv: void pixelToRealCoordinate(const point2f& in, point2f& out)

			/**
			 * convert pixel coordinate to a real world coordinate.
			 *
			 * @param pixelCoordinate the pixel coordinate
			 *
			 * @return the real world location of the point.
			 **/
			DataTypes::Point2D pixelToRealCoordinate(const DataTypes::Point2D& pixelCoordinate) const;

			/**
			 * convert real world coordinate to a pixel coordinate.
			 *
			 * @param realCoordinate coordinate the real world coordinate
			 *
			 * @return the pixel location of the point.
			 **/
			DataTypes::Point2D realToPixelCoordinate(const DataTypes::Point2D& realCoordinate) const;

			double realToPixelCoordinateScale;
			double pixelToRealCoordinateScale;
			double scale;
			
			double realAlpha;
			double pixelAlpha;
			double realToPixelCoordinateAlpha;
			double pixelToRealCoordinateAlpha;

			//double realToPixelCoordinateA;
			double realToPixelCoordinateA;
			//double realToPixelCoordinateC;
			double pixelToRealCoordinateA;
			double pixelToRealCoordinateB;
			//double pixelToRealCoordinateC;

			bool mirrored;
		private:
			std::vector<DataTypes::Point2D> fiducialsRealCoordinates;
			std::vector<DataTypes::Point2D> fiducialsPixelCoordinates;

			void updateTransformationParameters( );
	};

}
