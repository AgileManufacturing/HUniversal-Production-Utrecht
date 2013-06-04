/**
 * @file Crate.h
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

#ifndef CRATE_H_
#define CRATE_H_

#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <rexos_datatypes/Point2D.h>

namespace rexos_datatypes{
	/**
	 * A crate, with name, location and status.
	 **/
	class Crate{
	public:
		/**
		 * @var std::string name
		 * The crate identifier.
		 **/
		std::string name;
		/**
		 * @var bool oldSituation
		 * Indicates whether the crate has been recognized in the previous cameraFrame.
		 **/
		bool oldSituation;
		/**
		 * @var bool newSituation
		 * Indicates whether the crate has been recognized in the current cameraFrame.
		 **/
		bool newSituation;
		/**
		 * @var bool exists
		 * Indicates whether the crate has already been found (and not removed since).
		 **/
		bool exists;
		/**
		 * @var bool stable
		 * Crate is not in motion if true, in motion if false.
		 **/
		bool stable;
		/**
		 * @var int framesLeft
		 * Number of frames left for a crate to reach a stable state.
		 **/
		int framesLeft;

		/**
		 * Indicates whether the crate is on the current cameraFrame, and if whether it is moving or stable.
		 **/
		enum crate_state{
			state_stable = 1, state_moving = 2, state_non_existing = 3
		};

		Crate();
		Crate(const std::vector<cv::Point2f>& points);
		Crate(std::string name, const std::vector<cv::Point2f>& points);
		Crate(const Crate& crate);

		virtual ~Crate();

		/**
		 * Calculate distance between two fiducial points.
		 *
		 * @param point1 A fiducial point.
		 * @param point2 A fiducial point.
		 * 
		 * @return The distance between the two points.
		 */
		static inline float distance(const cv::Point2f& point1, const cv::Point2f& point2){
			float deltaX = point1.x - point2.x;
			float deltaY = point1.y - point2.y;
			return sqrt(deltaX * deltaX + deltaY * deltaY);
		}

		cv::RotatedRect rect();
		std::vector<cv::Point2f> getPoints() const;
		void setPoints(std::vector<cv::Point2f>& points);
		void draw(cv::Mat& image);

		crate_state getState();

	private:
		/**
		 * @var cv::RotatedRect bounds
		 * Bounding rectangle saved after calling rect().
		 **/
		cv::RotatedRect bounds;
		/**
		 * @var std::vector<cv::Point2f> points
		 * Fiducial points belonging to the crate.
		 **/
		std::vector<cv::Point2f> points;
	};
}
#endif /* CRATE_H_ */
