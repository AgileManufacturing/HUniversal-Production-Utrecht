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
#include <rexos_datatypes/Point2D.h>
#include <vector>

namespace rexos_vision{
	/**
	 * Object that transforms pixel coordinates to real life coordinates and vice versa using a number of fiducials of which the real life and pixel positions are known.
	 **/
	class PixelAndRealCoordinateTransformer{
		public:
			PixelAndRealCoordinateTransformer(const std::vector<rexos_datatypes::Point2D>& fiducialsRealCoordinates, const std::vector<rexos_datatypes::Point2D>& fiducialsPixelCoordinates);
			virtual ~PixelAndRealCoordinateTransformer();
			
			void setFiducialPixelCoordinates(const std::vector<rexos_datatypes::Point2D>& fiducialsPixelCoordinates);
			rexos_datatypes::Point2D pixelToRealCoordinate(const rexos_datatypes::Point2D& pixelCoordinate) const;
			rexos_datatypes::Point2D realToPixelCoordinate(const rexos_datatypes::Point2D& realCoordinate) const;
		private:
			/**
			 * @var double realToPixelCoordinateScale
			 * Scale to convert from a real coordinate to a pixel coordinate.
			 **/
			double realToPixelCoordinateScale;

			/**
			 * @var double pixelToRealCoordinateScale
			 * Scale to convert from a pixel coordinate to a real coordinate.
			 **/
			double pixelToRealCoordinateScale;

			/**
			 * @var double realAlpha
			 * Angle between two fiducial points real life coordinates. Should be the same two points as pixelAlpha.
			 **/
			double realAlpha;

			/**
			 * @var double pixelAlpha
			 * Angle between two fiducial points pixel coordinates. Should be the same two points as realAlpha.
			 **/
			double pixelAlpha;

			/**
			 * @var double realToPixelCoordinateAlpha
			 * Conversion in angle from real life coordinates to the pixel coordinate.
			 **/
			double realToPixelCoordinateAlpha;

			/**
			 * @var double pixelToRealCoordinateAlpha
			 * Conversion in angle from pixel coordinates to the real life coordinate.
			 **/
			double pixelToRealCoordinateAlpha;

			/**
			 * @var double realToPixelCoordinateA
			 * Variable calculated in updateTransformationParameters for use in conversions.
			 **/
			double realToPixelCoordinateA;

			/**
			 * @var double realToPixelCoordinateB
			 * Variable calculated in updateTransformationParameters for use in conversions.
			 **/
			double realToPixelCoordinateB;

			/**
			 * @var double pixelToRealCoordinateA
			 * Variable calculated in updateTransformationParameters for use in conversions.
			 **/
			double pixelToRealCoordinateA;

			/**
			 * @var double pixelToRealCoordinateB
			 * Variable calculated in updateTransformationParameters for use in conversions.
			 **/
			double pixelToRealCoordinateB;

			/**
			 * @var double mirrored
			 * Indicator whether the fiducials are seen from above or from below.
			 **/
			bool mirrored;

			/**
			 * @var std::vector<rexos_datatypes::Point2D> fiducialsRealCoordinates
			 * The real coordinates for the fiducial points. Should be in the same order as the fiducialsPixelCoordinates.
			 **/
			std::vector<rexos_datatypes::Point2D> fiducialsRealCoordinates;

			/**
			 * @var std::vector<rexos_datatypes::Point2D> fiducialsPixelCoordinates
			 * The pixel coordinates for the fiducial points. Should be in the same order as the fiducialsPixelCoordinates.
			 **/
			std::vector<rexos_datatypes::Point2D> fiducialsPixelCoordinates;

			void updateTransformationParameters();
	};
}
