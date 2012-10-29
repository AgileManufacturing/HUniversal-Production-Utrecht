/**
 * @file Crate.h
 * @brief Container class for a single productcrate
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
#include <DataTypes/Point2D.h>

namespace DataTypes {
	class Crate {
	public:
		//! The crate identifier
		std::string name;

		/*! \brief The Crate constructor
		 *
		 *  Constructs a crate without any fiducial points
		 */
		Crate( );

		/*! \brief The Crate constructor
		 *
		 *  Constructs a crate with the given three fiducial points.
		 *  The fiducial points ordering must be left-handed.
		 */
		Crate(const std::vector<cv::Point2f>& points);

		/*! \brief The Crate constructor
		 *
		 *  Constructs a crate with the given three fiducial points
		 *  and a name. The fiducial points ordering must be left-handed.
		 */
		Crate(std::string name, const std::vector<cv::Point2f>& points);

		/*! \brief The Crate copy-constructor
		 *
		 *  Constructs a crate with the same values as the
		 *  given crate, fiducial points are deep-copied.
		 */
		Crate(const Crate& crate);

		//! THe Crate deconstructor
		virtual ~Crate( );

		/*! \brief Calculate distance
		 *
		 *  Calculates the distance between two fiducial points
		 */
		static inline float distance(const cv::Point2f& pt1, const cv::Point2f& pt2) {
			float dx = pt1.x - pt2.x;
			float dy = pt1.y - pt2.y;
			return sqrt(dx * dx + dy * dy);
		}

		/*! \brief Order a list of fiducial points
		 *
		 *  Orders a vector with 3 fiducial points according
		 *  to the clockwise crate ordering.
		 */
		static void order(std::vector<cv::Point2f>& points);

		/*! \brief Generate a rotated bounding rectangle
		 *
		 *  Generates a RotatedRect that represents the
		 *  crate. This rectangle is cached for subsequent
		 *  calls to rect().
		 */
		cv::RotatedRect rect( );

		/*! \brief Get the fiducial points
		 *
		 * Gets a copy of the fiducial points
		 * that represent the crate.
		 */
		std::vector<cv::Point2f> getPoints( ) const;

		/*! \brief Set the fiducial points
		 *
		 *  Sets the new fiducial points and resets the
		 *  bounding rectangle.
		 */
		void setPoints(std::vector<cv::Point2f>& points);

		/*! \brief Draw the rectangle in the image
		 *
		 *  Draws the rectangle in the image including the
		 *  fiducial points, angle and bounding rectangle.
		 */
		void draw(cv::Mat& image);

		bool oldSituation, newSituation, exists, stable;
		int framesLeft;

		enum crate_state {
			state_stable = 1, state_moving = 2, state_non_existing = 3
		};

		/**
		 * function determines the last known stable state
		 * @return the last stable state
		 */
		crate_state getState( );

	private:
		//! Bounding rectangle saved after calling rect()
		cv::RotatedRect bounds;
		//! Fiducial points belonging to the crate
		std::vector<cv::Point2f> points;
	};
}
#endif /* CRATE_H_ */
