/**
 * @file FiducialDetector.h
 * @brief Detects fiduciary markers.
 * @date Created: 2011-11-11
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

#ifndef FIDUCIALDETECTOR_H_
#define FIDUCIALDETECTOR_H_

#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <vector>

namespace rexos_vision{

	/**
	 * Detects fiducial markers in an image.
	 **/
	class FiducialDetector{
	private:
		void drawPolarLine(cv::Mat& image, float rho, float theta, cv::Scalar color, int thickness);
		bool detectCenterLine(cv::Vec2f& centerLine, std::vector<cv::Vec2f> lines, cv::Mat* debugImage = NULL);

		/**
		 * @var int blur
		 * The size of a gaussian blur.
		 **/
		int blur;
		/**
		 * @var double sigma
		 * The sigma of a gaussian blur.
		 **/
		double sigma;
		
		/**
		 * @var int circleVotes
		 * The minimum number of votes needed for an object to be detected as a circle.
		 **/
		int circleVotes;
		/**
		 * @var double distance
		 * The minimum distance between circles in pixels.
		 **/
		double distance;
		/**
		 * @var int minRad
		 * The minimum radius in pixels of a circle.
		 **/
		int minRad;
		/**
		 * @var int maxRad
		 * The maximum radius in pixels of a circle.
		 **/
		int maxRad;
		/**
		 * @var double circleThreshold
		 * High canny threshold for circle detection (low threshold is twice smaller).
		 **/
		double circleThreshold;

		/**
		 * @var int lineVotes
		 * Starting vote threshold for lines, thinner lines get less votes.
		 **/
		int lineVotes;
		/**
		 * @var unsigned int maxLines
		 * Maximum amount of lines that can be found.
		 **/
		unsigned int maxLines;
		/**
		 * @var float minDist
		 * Minimum distance between lines to use for the center line.
		 **/
		float minDist;
		/**
		 * @var float maxDist
		 * Maximum distance between lines to use for the center line.
		 **/
		float maxDist;
		/**
		 * @var double lowThreshold
		 * Low canny threshold for line detection.
		 **/
		double lowThreshold;
		/**
		 * @var double highThreshold
		 * High canny threshold for line detection.
		 **/
		double highThreshold;
	public:
		/**
		 * @var bool verbose
		 * Turn on and off console messages.
		 **/
		bool verbose;

		FiducialDetector(int minRad = 20, int maxRad = 40);
		virtual ~FiducialDetector();

		void detect(cv::Mat& image, std::vector<cv::Point2f>& points, cv::Mat* debugImage = NULL);
		bool detectCrosshair(cv::Mat& image, cv::Point2f& center, const cv::Mat& mask = cv::Mat(), cv::Mat* debugImage = NULL);
		static void order(std::vector<cv::Point2f>& points);

		/**
		 * Set minRad.
		 *
		 * @param minRad The minimum radius in pixels of a circle.
		 **/
		void setMinRad(int minRad){ this->minRad = minRad; }

		/**
		 * Set maxRad.
		 *
		 * @param maxRad The maximum radius in pixels of a circle.
		 **/
		void setMaxRad(int maxRad){ this->maxRad = maxRad; }

		/**
		 * Set distance.
		 *
		 * @param distance The minimum distance between circles in pixels.
		 **/
		void setDistance(int distance){ this->distance = distance; }

		/**
		 * Set minDist.
		 *
		 * @param minDist Minimum distance between lines to use for the center line.
		 **/
		void setMinDist(int minDist){ this->minDist = minDist; }

		/**
		 * Set maxDist.
		 *
		 * @param maxDist Maximum distance between lines to use for the center line.
		 **/
		void setMaxDist(int maxDist){ this->maxDist = maxDist; }

		/**
		 * Set circleVotes.
		 *
		 * @param circleVotes The minimum number of votes needed for an object to be detected as a circle.
		 **/
		void setCircleVotes(int circleVotes){ this->circleVotes = circleVotes; }

		/**
		 * Calculate distance between two fiducial points
		 *
		 * @param pt1 Point 1.
		 * @param pt2 Point 2.
		 *
		 * @return The distance between the two points in pixels.
		 **/
		static inline float fiducialDistance(const cv::Point2f& pt1, const cv::Point2f& pt2){
			float dx = pt1.x-pt2.x;
			float dy = pt1.y-pt2.y;
			return sqrt(dx*dx+dy*dy);
		}
	};
}
#endif /* FIDUCIALDETECTOR_H_ */
