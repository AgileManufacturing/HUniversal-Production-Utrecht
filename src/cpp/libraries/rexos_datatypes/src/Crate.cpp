/**
 * @file Crate.cpp
 * @brief Container class for a single product crate.
 * @date Created: 2011-11-11
 * @date Revisioned: 2012-10-22
 *
 * @author Jules Blok
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

#include "rexos_datatypes/Crate.h"
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>

namespace rexos_datatypes{
	/**
	 * Constructs a crate without specific location.
	 **/
	Crate::Crate() : points(3){}

	/**
	 * Constructs a crate with a specific location
	 *
	 * @param points The QR code points, ordering must be left-handed.
	 **/
	Crate::Crate(const std::vector<cv::Point2f>& points) : oldSituation(false), newSituation(true), exists(true), stable(false), framesLeft(0){
		this->points.assign(points.begin(), points.begin() + 3);
	}

	/**
	 * Constructs a crate with a specific location.
	 *
	 * @param name The crate identifier.
	 * @param points The QR code points, ordering must be left-handed.
	 **/
	Crate::Crate(std::string name, const std::vector<cv::Point2f>& points) : oldSituation(false), newSituation(true), exists(true), stable(false), framesLeft(0){
		this->points.assign(points.begin(), points.begin() + 3);
		this->name = name;
	}

	/**
	 * Copy constructor for a crate.
	 *
	 * @param crate The crate to be copied.
	 **/
	Crate::Crate(const Crate& crate) : name(crate.name), oldSituation(false), newSituation(true), exists(true), stable(false), framesLeft(0), bounds(crate.bounds), points(crate.points){}

	/**
	 * The crate destructor.
	 **/
	Crate::~Crate(){}

	/**
	 * Generate a rotated bounding rectangle, RotatedRect, that represents the crate. This rectangle is cached for subsequent calls to rect().
	 *
	 * @return The RotatedRect bounds.
	 **/
	cv::RotatedRect Crate::rect(){
		if(bounds.size.area() != 0.0f)
			return bounds;

		// Determine the distance between the fiducial points
		float distance1 = sqrt(pow(points[0].x - points[1].x, 2) + pow(points[0].y - points[1].y, 2));
		float distance2 = sqrt(pow(points[2].x - points[1].x, 2) + pow(points[2].y - points[1].y, 2));

		// Distance and angle between the diagonal points
		float length = sqrt(distance1 * distance1 + distance2 * distance2);
		float alpha = atan2(points[0].y - points[2].y, points[2].x - points[0].x);

		// Determine the center, size and angle
		bounds.center = cv::Point2f(points[0].x + (length / 2.0) * cos(-alpha),
		        points[0].y + (length / 2.0) * sin(-alpha));
		bounds.size = cv::Size(distance1, distance2);
		bounds.angle = alpha - M_PI / 4.0;
		if (bounds.angle < -M_PI)
			bounds.angle += 2 * M_PI;

		return bounds;
	}

	/**
	 * Get the fiducial points that represent the crate location.
	 *
	 * @return A copy of the fiducial points.
	 **/
	std::vector<cv::Point2f> Crate::getPoints( ) const{
		std::vector<cv::Point2f> copy;
		copy.assign(points.begin(), points.begin() + 3);
		return copy;
	}

	/**
	 * Sets the new QR code points and resets the bounding rectangle.
	 *
	 * @param newPoints The new QR code points.
	 **/
	void Crate::setPoints(std::vector<cv::Point2f>& newPoints){
		this->bounds.size = cv::Size(0, 0); // This is enough to force a regeneration
		this->points.assign(newPoints.begin(), newPoints.begin() + 3);
	}

	/**
	 * Draws the rectangle in the image including the QR code points, angle and bounding rectangle.
	 *
	 * @param image The image to draw on.
	 **/
	void Crate::draw(cv::Mat& image){
		// Draw the QR marker points
		cv::circle(image, points[0], 1, cv::Scalar(255, 0, 0), 2);
		cv::circle(image, points[1], 1, cv::Scalar(0, 255, 0), 2);
		cv::circle(image, points[2], 1, cv::Scalar(0, 0, 255), 2);

		cv::RotatedRect rect = this->rect();

		// Draw arrow
		{
			cv::Point pt1 = rect.center;
			cv::Point pt2(pt1.x - 50 * cos(-rect.angle + M_PI / 2.0), pt1.y - 50 * sin(-rect.angle + M_PI / 2.0));
			cv::line(image, pt1, pt2, cv::Scalar(0, 0, 0), 2);
			cv::line(image, pt2,
					cv::Point(pt2.x + 10 * cos(-rect.angle + 3 * M_PI / 4.0), pt2.y + 10 * sin(-rect.angle + 3 * M_PI / 4.0)),
					cv::Scalar(0, 0, 0), 2);
			cv::line(image, pt2,
					cv::Point(pt2.x + 10 * cos(-rect.angle + M_PI / 4.0), pt2.y + 10 * sin(-rect.angle + M_PI / 4.0)),
					cv::Scalar(0, 0, 0), 2);
			std::stringstream ss;
			ss << cv::saturate_cast<int>(rect.angle / (M_PI / 180.0));
			cv::putText(image, ss.str(), pt1 - cv::Point(15, 0), CV_FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255, 0, 0), 2);
		}

		if(!name.empty()){
			cv::putText(image, name, cv::Point(rect.center.x, rect.center.y - 20), CV_FONT_HERSHEY_COMPLEX, 1,
					cv::Scalar(0, 0, 255), 2);
		}
	}

	/**
	 * Function determines the last known state.
	 *
	 * @return The crate_state value.
	 **/
	Crate::crate_state Crate::getState(){
		if(oldSituation){
			if(stable){
				return state_stable;
			} else{
				return state_moving;
			}
		}
		return state_non_existing;
	}
}
