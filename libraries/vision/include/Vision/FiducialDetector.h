//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        Fiducial
// File:           FiducialDetector.h
// Description:    Detects fiduciary markers
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

#ifndef FIDUCIALDETECTOR_H_
#define FIDUCIALDETECTOR_H_

#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <vector>

namespace Vision {

	/*! \brief Detects fiducial markers.
	 *
	 *  This class allows you to locate the cross.png
	 *  and the crosshair.png fiducials in an image.
	 */
	class FiducialDetector {
	private:
		//! Draw a polar coordinate line
		void polarLine(cv::Mat& image, float rho, float theta, cv::Scalar color, int thickness);
		//! Determine the center line
		bool detectCenterLine(cv::Vec2f& centerLine, std::vector<cv::Vec2f> lines, cv::Mat* debugImage = NULL);
	public:
		//! Turn on console debug messages
		bool verbose;

		//! Gaussian blur size
		int blur;
		//! Gaussian blur sigma
		double sigma;
		//! Vote threshold for circles
		int circleVotes;
		//! Minimum distance between circles
		double distance;
		//! Minimum circle radius
		int minRad;
		//! Maximum circle radius
		int maxRad;
		//! High canny threshold for circle detection (low threshold is twice smaller)
		double circleThreshold;

		//! Starting vote threshold for lines, thinner lines get less votes.
		int lineVotes;
		//! Maximum amount of lines that can be found.
		unsigned int maxLines;
		//! Minimum distance between lines to use for the center line.
		float minDist;
		//! Maximum distance between lines to use for the center line.
		float maxDist;
		//! Low canny threshold for line detection
		double lowThreshold;
		//! High canny threshold for line detection
		double highThreshold;

		/*! \brief The FiducialDetector constructor
		 *
		 *  Constructs the fiducial detector with default properties.
		 *  The minimum and maximum radius can be changed in the constructor,
		 *  all other properties can be changed after construction.
		 */
		FiducialDetector(int minRad = 20, int maxRad = 40);

		//! The FiducialDetector deconstructor
		virtual ~FiducialDetector( );

		/*! \brief Detects all fiducials in an image
		 *
		 *  Detects all fiducials in the image and automatically
		 *  calls detectCrosshair for each fiducial adding the
		 *  center points to the points vector.
		 *
		 *  \param image Image with the fiducials
		 *  \param points Output vector that will contain the
		 *  center points
		 *  \param debugImage Output image where debug information
		 *  will be drawn on, set to NULL for no debug information
		 */
		void detect(cv::Mat& image, std::vector<cv::Point2f>& points, cv::Mat* debugImage = NULL);

		/*! \brief Detects the center point
		 *
		 *  Automatically called by detect(). After the fiducial has
		 *  been segmented this function will determine the center
		 *  point of the crosshair.
		 *
		 *  \sa detect()
		 *
		 *  \param image Image with the crosshair
		 *  \param center Output point that will be set to the
		 *  center point
		 *  \param mask Operation mask of the same size as image
		 *  \param debugImage Output image where debug information
		 *  will be drawn on, set to NULL for no debug information
		 *  \return <i>true</i> if center point was detected\n
		 *  <i>false</i> if detection failed
		 */
		bool detectCrosshair(cv::Mat& image, cv::Point2f& center, const cv::Mat& mask = cv::Mat(), cv::Mat* debugImage =
		        NULL);

		/*! \brief Order a list of fiducial points
		 *
		 *  Orders a vector with 3 fiducial points according
		 *  to the clockwise crate ordering.
		 */
		static void order(std::vector<cv::Point2f>& points);

		/*! \brief Calculate distance
		 *
		 *  Calculates the distance between two fiducial points
		 */
		static inline float fiducialDistance(const cv::Point2f& pt1, const cv::Point2f& pt2) {
			float dx = pt1.x-pt2.x;
			float dy = pt1.y-pt2.y;
			return sqrt(dx*dx+dy*dy);
		}
	};

}
#endif /* FIDUCIALDETECTOR_H_ */
