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

#include "DataTypes/Crate.h"
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <Utilities/Utilities.h>

namespace DataTypes {
	/**
	 * Constructs a crate without specific location.
	 **/
	Crate::Crate( ) :
			points(4) {
	}

	/**
	 * Constructs a crate with a specific location
	 *
	 * @param points The QR code points, ordering must be left-handed.
	 **/
	Crate::Crate(const std::vector<cv::Point2f>& points) :
			oldSituation(false), newSituation(true), exists(true), stable(false), framesLeft(0) {
		this->setPoints(points);
	}

	/**
	 * Constructs a crate with a specific location.
	 *
	 * @param name The crate identifier.
	 * @param points The QR code points, ordering must be left-handed.
	 **/
	Crate::Crate(std::string name, const std::vector<cv::Point2f>& points) :
			oldSituation(false), newSituation(true), exists(true), stable(false), framesLeft(0) {
		this->setPoints(points);
		this->name = name;
	}

	/**
	 * Copy constructor for a crate.
	 *
	 * @param crate The crate to be copied.
	 **/
	Crate::Crate(const Crate& crate) :
			name(crate.name), oldSituation(false), newSituation(true), exists(true), stable(false), framesLeft(0), points(crate.points), center(crate.center), alpha(crate.alpha) {
	}

	/**
	 * The crate destructor.
	 **/
	Crate::~Crate( ) {
	}

	/**
	 * Get the fiducial points that represent the crate location.
	 *
	 * @return A copy of the fiducial points.
	 **/
	std::vector<cv::Point2f> Crate::getPoints( ) const {
		std::vector<cv::Point2f> copy;
		copy.assign(points.begin(), points.begin() + 4);
		return copy;
	}

	/**
	 * Sets the new QR code points and resets the bounding rectangle.
	 *
	 * @param newPoints The new QR code points.
	 **/
	void Crate::setPoints(const std::vector<cv::Point2f>& newPoints) {
		points.assign(newPoints.begin(), newPoints.begin() + 4);
		center = DataTypes::Point2D(points[0]).mean(DataTypes::Point2D(points[2]));
		alpha = atan2(points[1].y - points[0].y, points[0].x - points[1].x);
	}

	/**
	 * Draws the rectangle in the image including the QR code points, angle and bounding rectangle.
	 *
	 * @param image The image to draw on.
	 **/
	void Crate::draw(cv::Mat& image) {
		// Draw the QR marker points
		cv::circle(image, points[0], 1, cv::Scalar(255, 0, 0), 2);
		cv::circle(image, points[1], 1, cv::Scalar(0, 255, 0), 2);
		cv::circle(image, points[2], 1, cv::Scalar(0, 0, 255), 2);
		cv::circle(image, points[3], 1, cv::Scalar(0, 255, 255), 2);

		// Calculate arrow endpoints
		cv::Point2f centerPoint = center.toCVPoint();
		cv::Point2f endPoint = (center + (DataTypes::Point2D(0,-30)).rotate(alpha + M_PI / 2)).toCVPoint();

		// Draw arrow
		cv::line(image, centerPoint, endPoint, cv::Scalar(255, 0, 0), 2);
		// Draw arrow head
		//cv::line(image, pt2,
		//        cv::Point(pt2.x + 10 * cos(angle + 3 * M_PI / 4.0),
		//                pt2.y + 10 * sin(-rect.angle + 3 * M_PI / 4.0)), cv::Scalar(0, 0, 0), 2);
		//cv::line(image, pt2,
		//        cv::Point(pt2.x + 10 * cos(-rect.angle + M_PI / 4.0), pt2.y + 10 * sin(-rect.angle + M_PI / 4.0)),
		//        cv::Scalar(0, 0, 0), 2);

		std::stringstream ss;
		ss << cv::saturate_cast<int>(Utilities::radiansToDegrees(alpha));
		cv::putText(image, ss.str(), centerPoint - cv::Point2f(15 * ss.str().length(), 0), CV_FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255, 0, 0), 2);

		if (!name.empty()) {
			cv::putText(image, name, cv::Point(centerPoint.x, centerPoint.y - 20), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);
		}
	}

	/**
	 * Function determines the last known state.
	 *
	 * @return The crate_state value.
	 **/
	Crate::crate_state Crate::getState( ) {
		if (oldSituation) {
			if (stable) {
				return state_stable;
			} else {
				return state_moving;
			}
		}
		return state_non_existing;
	}
}
