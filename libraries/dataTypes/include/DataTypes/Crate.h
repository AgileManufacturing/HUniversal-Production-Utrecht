//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        Fiducial
// File:           Crate.h
// Description:    Container class for a single productcrate
// Author:         Jules Blok
// Notes:          None
//
// License:        GNU GPL v3
//
// This file is part of Fiducial.
//
// Fiducial is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Fiducial is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Fiducial.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#ifndef CRATE_H_
#define CRATE_H_

#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <vector>

class Crate {
private:
	//! Bounding rectangle saved after calling rect()
	cv::RotatedRect bounds;
	//! Fiducial points belonging to the crate
	std::vector<cv::Point2f> points;
	
public:
	//! The crate identifier
	std::string name;

	/*! \brief The Crate constructor
	 *
	 *  Constructs a crate without any fiducial points
	 */
	Crate();

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
	virtual ~Crate();

	/*! \brief Calculate distance
	 *
	 *  Calculates the distance between two fiducial points
	 */
	static inline float distance(const cv::Point2f& pt1, const cv::Point2f& pt2) {
		float dx = pt1.x-pt2.x;
		float dy = pt1.y-pt2.y;
		return sqrt(dx*dx+dy*dy);
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
	cv::RotatedRect rect();

	/*! \brief Get the fiducial points
	 *
	 * Gets a copy of the fiducial points
	 * that represent the crate.
	 */
	std::vector<cv::Point2f> getPoints() const;

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
};

#endif /* CRATE_H_ */
