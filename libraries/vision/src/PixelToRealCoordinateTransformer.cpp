/**
 * @file PixelToRealCoordinateTransformer.cpp
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
#include <Vision/PixelToRealCoordinateTransformer.h>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <float.h>
#include <list>
#include <iostream>

namespace Vision {
	PixelToRealCoordinateTransformer::PixelToRealCoordinateTransformer(const std::vector<DataTypes::Point2D>& fiducials_real_cordinates,
			const std::vector<DataTypes::Point2D>& fiducials_pixel_cordinates )
		: scale(0), fiducials_real_coordinates(fiducials_real_cordinates), fiducials_pixel_coordinates(fiducials_pixel_cordinates)
	{
		update_transformation_parameters();
	}

	PixelToRealCoordinateTransformer::~PixelToRealCoordinateTransformer()
	{
	}

	void PixelToRealCoordinateTransformer::set_fiducials_pixel_coordinates(const std::vector<DataTypes::Point2D>& fiducials_real_cordinates)
	{
		this->fiducials_pixel_coordinates = fiducials_real_cordinates;
		update_transformation_parameters();
	}

	/**
	 * A function to convert pixel coordinates to robot coordinates
	 * The math is just a worked out version of a conversion matrix (translation, rotation, scaling and mirroring)
	 * Mirroring is available because one camera is on the bottom and 1 on the top
	 * @param pixel_coordinate is the input coordinate that will be converted
	 */
	DataTypes::Point2D PixelToRealCoordinateTransformer::to_rc(const DataTypes::Point2D & pixel_coordinate) const
	{
		int pix_y = pixel_coordinate.y*-1;		

		DataTypes::Point2D result;
		
		result.x = to_rc_scale * (cos(to_rc_alpha) * (pixel_coordinate.x - to_rc_a) + sin(to_rc_alpha) * (pix_y - to_rc_b));
		result.y = to_rc_scale * (-sin(to_rc_alpha) * (pixel_coordinate.x - to_rc_a) + cos(to_rc_alpha) * (pix_y - to_rc_b));
		
		if(mirrored){
			//std::cout << "result.x: " << result.x << ", " << result.y << std::endl;			
			double temp_x = fiducials_real_coordinates[0].x - fiducials_real_coordinates[0].x * cos(2* alpha_real) + result.x * cos(2 * alpha_real) - fiducials_real_coordinates[0].y * sin(2*alpha_real) + result.y * sin(2*alpha_real);
			
			result.y = fiducials_real_coordinates[0].y + fiducials_real_coordinates[0].y * cos(2*alpha_real) - result.y * cos(2*alpha_real) - fiducials_real_coordinates[0].x * sin(2*alpha_real) + result.x * sin(2*alpha_real);
		
			result.x = temp_x;
		
		}
		//std::cout << "result.x: " << result.x << ", " << result.y << std::endl;
		return result;
	}
	
	/**
	 * A function to convert robot coordinates to pixel coordinates
	 * This function does the exact opposite transformation as to_rc
	 * @see to_rc()
	 */
	DataTypes::Point2D PixelToRealCoordinateTransformer::to_pc(const DataTypes::Point2D& real_coordinate) const {
		DataTypes::Point2D result;
		
		result = real_coordinate;
		
		double temp_x;
		
		if(mirrored){
			//std::cout << "result.x: " << result.x << ", " << result.y << std::endl;			
			temp_x = fiducials_real_coordinates[0].x - fiducials_real_coordinates[0].x * cos(2* alpha_real) + result.x * cos(2 * alpha_real) - fiducials_real_coordinates[0].y * sin(2*alpha_real) + result.y * sin(2*alpha_real);
			
			result.y = fiducials_real_coordinates[0].y + fiducials_real_coordinates[0].y * cos(2*alpha_real) - result.y * cos(2*alpha_real) - fiducials_real_coordinates[0].x * sin(2*alpha_real) + result.x * sin(2*alpha_real);
		
			result.x = temp_x;
		
		}
		
		
		temp_x = to_pc_scale * (cos(to_pc_alpha) * (result.x - to_pc_a) + sin(to_pc_alpha) * (result.y - to_pc_b));
		result.y = to_pc_scale * (-sin(to_pc_alpha) * (result.x - to_pc_a) + cos(to_pc_alpha) * (result.y - to_pc_b));
		result.x = temp_x;
		
		result.y *= -1;
		
		return result;
	}
	
	/**
	 * This function is called by PixelToRealCoordinateTransformer::set_fiducials_pixel_coordinates() to calculate parameters for to_pc() and to_rc() transformations
	 * You can find handwritten math notes on sharepoint
	 * Scale and angles are stored in multiple variables because of readability (even if its just 1/{other scale})
	 */
	void PixelToRealCoordinateTransformer::update_transformation_parameters()
	{
		if(fiducials_real_coordinates.size() != fiducials_pixel_coordinates.size())
			throw std::runtime_error("Number of real fiducial coordinates does not match number of pixel fiducials coordinates");
		scale = 0;
		int distances_count = 0;
		for(unsigned int n = 0 ; n < fiducials_real_coordinates.size();n++)
		{
			for(unsigned int m = n+1 ; m < fiducials_real_coordinates.size();m++)
			{
				scale += fiducials_real_coordinates[n].distance(fiducials_real_coordinates[m])
						/ fiducials_pixel_coordinates[n].distance(fiducials_pixel_coordinates[m]);
				distances_count++;
			}
		}
		scale /= distances_count;
		to_pc_scale = 1/scale;
		to_rc_scale = scale;
		
		double x_pix = fiducials_pixel_coordinates[0].x;
		double y_pix = -fiducials_pixel_coordinates[0].y;
		double x_real = fiducials_real_coordinates[0].x;
		double y_real = fiducials_real_coordinates[0].y;
		
		double pix_delta_x = fiducials_pixel_coordinates[2].x - x_pix;
		double pix_delta_y = -fiducials_pixel_coordinates[2].y - y_pix;
		
		double real_delta_x = fiducials_real_coordinates[2].x - x_real;
		double real_delta_y = fiducials_real_coordinates[2].y - y_real;
		
		alpha_real = atan2(real_delta_y, real_delta_x);
		alpha_pixel = atan2(pix_delta_y, pix_delta_x);
		to_pc_alpha = alpha_real - alpha_pixel;
		to_rc_alpha = alpha_pixel - alpha_real;
		
		
		double rcos = cos(to_pc_alpha);
		double rcos2 = pow(rcos, 2);
		double rsin = sin(to_pc_alpha);
		double rsin2 = pow(rsin,2);
		double rtan = tan(to_pc_alpha);
		
		to_pc_a =  x_real - (rcos * x_pix - rsin*y_pix) / (to_pc_scale * (rcos2 + rsin2));	
		to_pc_b = y_real - y_pix / (to_pc_scale*rcos) - rtan*(x_real - to_pc_a);
		
		rcos = cos(to_rc_alpha);
		rcos2 = pow(rcos, 2);
		rsin = sin(to_rc_alpha);
		rsin2 = pow(rsin,2);
		rtan = tan(to_rc_alpha);
		
		to_rc_a =  x_pix - (rcos * x_real - rsin*y_real) / (to_rc_scale * (rcos2 + rsin2));	
		to_rc_b = y_pix - y_real / (to_rc_scale*rcos) - rtan*(x_pix - to_rc_a);
		
		mirrored = false;
		DataTypes::Point2D test = to_rc(fiducials_pixel_coordinates[1]);
		
		//check to see if the 3th fiducials is within 1 cm of the calculated point, if not so the image is mirrored
		if(!(test.x > fiducials_real_coordinates[1].x - 10 && test.x < fiducials_real_coordinates[1].x + 10)){
			mirrored = true;
		} else {
			if(!(test.y > fiducials_real_coordinates[1].y - 10 && test.y < fiducials_real_coordinates[1].y + 10)){
				mirrored = true;
			} else {
				mirrored = false;
			}
		}
		/*
		std::cout << "mirrored = " << mirrored << std::endl;
		
		std::cout << "to_pc_a: " << to_pc_a << " to_pc_b: " << to_pc_b << std::endl;
		std::cout << "to_rc_a: " << to_rc_a << " to_rc_b: " << to_rc_b << std::endl;
		std::cout << "alpha real: " << alpha_real << " alpha pixel: " << alpha_pixel << " to pc alpha: " << to_pc_alpha  << " to rc alpha: " << to_rc_alpha << std::endl;
		std::cout << "scale: " <<  scale << " to rc scale: " << to_rc_scale << " to pc scale" << to_pc_scale << std::endl << std::endl;		
		std::cout << "x pixel: " << x_pix << " y pixel: " << y_pix << " x real: " << x_real << " y real: " << y_real << std::endl;	
		
		for(int i = 0; i < 3; i++){
			std::cout << "real: " << fiducials_real_coordinates[i].x << ",  " << fiducials_real_coordinates[i].y << std::endl;
			std::cout << "pix : " << fiducials_pixel_coordinates[i].x << ", " << fiducials_pixel_coordinates[i].y << std::endl;
		}
		*/
		
	}
}
