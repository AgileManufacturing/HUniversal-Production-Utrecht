/**
 * @file PixelToRealCoordinateTransformer.h
 * @brief Library for convert reallife coordinates to pixel coordinates and vice versa
 * @date Created: 2012-01-??  TODO: Date
 *
 * @author Kasper van Nieuwland
 * @author Zep Mouris
 *
 * @section LICENSE
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
 *
 **/
#pragma once
#include <DataTypes/Point2D.h>
#include <vector>

namespace Vision {
	/**
	 * @brief object that transforms pixel coordinates to real life coordinates \
	   using a number of fiducials of which the real life and pixel positions are known.
	 **/
	class PixelToRealCoordinateTransformer {
	public:
		/**
		 * constructor for a pc_rc_transformer
		 * @param fiducials_real_coordinates a vector with the real world coordinates of the fiducials in the same order as fiducials_pixel_coordinates
		 * @param fiducials_pixel_coordinates a vector with the pixel coordinates of the fiducials in the same order as fiducials_real_coordinates
		 */
		PixelToRealCoordinateTransformer(const std::vector<DataTypes::Point2D>& fiducials_real_coordinates,
		        const std::vector<DataTypes::Point2D>& fiducials_pixel_coordinates);
		/**
		 * destructor
		 */
		virtual ~PixelToRealCoordinateTransformer( );
		/**
		 * function for updating the pixel coordinates of the fiducials
		 * @param fiducials_pixel_coordinates the new coordinates
		 */
		void set_fiducials_pixel_coordinates(const std::vector<DataTypes::Point2D>& fiducials_pixel_coordinates);

		//TODO to_rc en to_pc zijn qua structuur hetzelfde
		//TODO er zou een template functie moeten komen
		//TODO die gebruikt kan worden voor deze 2 functies

		//TODO bij to_rc en to_pc mischien nog functies maken
		//TODO of aanpassen met out-parameters ipv een punt returnen
		//TODO over de stack. bijv: void to_rc(const point2f& in, point2f& out)

		/**
		 * convert pixel coordinate to a real world coordinate.
		 * @param pixel_coordinate the pixel coordinate
		 * @return the real world location of the point.
		 */
		DataTypes::Point2D to_rc(const DataTypes::Point2D& pixel_coordinate) const;
		/**
		 * convert real world coordinate to a pixel coordinate.
		 * @param real_coordinate coordinate the real world coordinate
		 * @return the pixel location of the point.
		 */
		DataTypes::Point2D to_pc(const DataTypes::Point2D& real_coordinate) const;
		double to_pc_scale, to_rc_scale, scale;
		double alpha_real, alpha_pixel, to_pc_alpha;
		double to_rc_alpha;
		double to_pc_a, to_pc_b, to_pc_c;
		double to_rc_a, to_rc_b, to_rc_c;
		bool mirrored;
	private:
		std::vector<DataTypes::Point2D> fiducials_real_coordinates;
		std::vector<DataTypes::Point2D> fiducials_pixel_coordinates;

		void update_transformation_parameters( );
	};

}
