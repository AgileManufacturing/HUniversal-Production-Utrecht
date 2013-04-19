/**
 * @file FiducialDetector.cpp
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

#include "rexos_vision/FiducialDetector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <math.h>

namespace rexos_vision{
	/**
	 * Constructs the fiducial detector with default properties. The minimum and maximum radius can be changed in the constructor, all other properties can be changed after construction.
	 * 
	 * @param minRad The minimum radius in pixels of a circle.
	 * @param maxRad The maximum radius in pixels of a circle.
	 **/
	FiducialDetector::FiducialDetector(int minRad, int maxRad){
		this->verbose = false;
		// Canny high for circle detection
		this->circleThreshold = 300;
		// Blur size for circle detection
		this->blur = 3;
		// Sigma for the blur
		this->sigma = 2.0;
		this->minRad = minRad;
		this->maxRad = maxRad;
		// Distance between the recognized circles in pixels.
		this->distance = 70;
		// Quality of the circle needed for it to be detected, higher is more perfect circle.
		this->circleVotes = 100;

		// Center lines detection of fiducial min and max distance
		this->minDist = 1.5f;
		this->maxDist = 5.0f;
		// Quality of the line
		this->lineVotes = 10;
		// Maximum number of lines in the center line detection
		this->maxLines = 10;
		// Canny low threshold for line detection
		this->lowThreshold = 125;
		// Canny high threshold for line detection
		this->highThreshold = 300;
	}

	/**
	 * Virtual function to destruct the fiducial detector.
	 **/
	FiducialDetector::~FiducialDetector(){
	}

	/**
	 * Draw a polar coordinate line.
	 * 
	 * @param image The image to draw on.
	 * @param rho The distance from point to origin.
	 * @param theta The angle.
	 * @param color The color to draw the line with.
	 * @param thickness The thickness of the line.
	 **/
	inline void FiducialDetector::drawPolarLine(cv::Mat& image, float rho, float theta, cv::Scalar color, int thickness){
		if(theta < M_PI / 4. || theta > 3. * M_PI / 4.){ // ~vertical line
		// Point of intersection of the line with first row
			cv::Point pt1(rho / cos(theta), 0);
			// Point of intersection of the line with last row
			cv::Point pt2((rho - image.rows * sin(theta)) / cos(theta), image.rows);
			// Draw the line
			cv::line(image, pt1, pt2, color, thickness);
		} else{ // ~horizontal line
			// Point of intersection of the line with first column
			cv::Point pt1(0, rho / sin(theta));
			// Point of intersection of the line with last column
			cv::Point pt2(image.cols, (rho - image.cols * cos(theta)) / sin(theta));
			// Draw the line
			cv::line(image, pt1, pt2, color, thickness);
		}
	}

	/**
	 *  Detects all fiducials in the image and automatically calls detectCrosshair for each fiducial adding the center points to the points vector.
	 *
	 *  @param image Image with the fiducials
	 *  @param points Output vector that will contain the center points
	 *  @param debugImage Output image where debug information will be drawn on, set to NULL for no debug information
	 */
	void FiducialDetector::detect(cv::Mat& image, std::vector<cv::Point2f>& points, cv::Mat* debugImage){
		// Apply gaussian blur
		cv::Mat blur;

		cv::GaussianBlur(image, blur, cv::Size(this->blur, this->blur), this->sigma);

		// Detect circles
		std::vector<cv::Vec3f> circles;
		cv::HoughCircles(blur, circles, CV_HOUGH_GRADIENT, 2, // Accumulator resolution divisor
				distance, // Minimum distance between circles
				circleThreshold, // Canny high threshold
				circleVotes, // Minimum number of votes
				minRad, maxRad); // Min and max radius

		// Accurately detect the center for every circle with sub-pixel precision
		for(std::vector<cv::Vec3f>::const_iterator it = circles.begin(); it != circles.end(); it++){
			// Draw the detected circles
			if(debugImage != NULL){
				cv::circle(*debugImage, cv::Point((*it)[0], (*it)[1]), (*it)[2], cv::Scalar(0, 255, 0), 2);
				cv::Point center(cvRound((*it)[0]), cvRound((*it)[1]));
				// Circle center
				cv::circle(*debugImage, center, 3, cv::Scalar(0, 255, 255), -1, 8, 0);
			}

			// Set ROI to the circle
			cv::Point center((*it)[0], (*it)[1]);
			float rad = (*it)[2];
			cv::Rect bounds(MAX(center.x - rad, 0), MAX(center.y - rad, 0),
					center.x + rad < image.cols ? rad * 2 : (image.cols - center.x) * 2,
					center.y + rad < image.rows ? rad * 2 : (image.rows - center.y) * 2);
			cv::Mat roi = image(bounds);

			// Generate inverted circle mask
			cv::Mat roiMask = cv::Mat::zeros(roi.rows, roi.cols, CV_8U);
			cv::Point roiCenter(roi.cols / 2, roi.rows / 2);
			cv::circle(roiMask, roiCenter, rad - 1, cv::Scalar(255), -1);

			// If lines were found
			cv::Point2f roiPoint;
			bool ret = false;
			if(debugImage != NULL){
				cv::Mat roiDebug = (*debugImage)(bounds);
				ret = detectCrosshair(roi, roiPoint, cv::Mat(), &roiDebug);
			} else{
				ret = detectCrosshair(roi, roiPoint, cv::Mat());
			}

			if(ret){
				cv::Point2f point(bounds.x + roiPoint.x, bounds.y + roiPoint.y);
				if(bounds.contains(point))
					points.push_back(point);
				else if(verbose)
					std::cout << "Center: " << center << " outside ROI!" << std::endl;
			}

		}
	}

	/**
	 * Compares two OpenCV Vec2f's to see which has a bigger rho (distance).
	 * 
	 * @param i The first Vec2f.
	 * @param j The second Vec2f.
	 *
	 * @return True if first vector's rho is smaller than the second vector's rho, false otherwise.
	 **/
	bool rhoComp(cv::Vec2f i, cv::Vec2f j){
		return (i[0] < j[0]);
	}

	/**
	 * Calculate the median rho (distance) of the lines found.
	 *
	 * @param first First line found.
	 * @param last Last line found.
	 *
	 * @return The median line location.
	 **/
	inline cv::Vec2f medoidRho(std::vector<cv::Vec2f>::iterator first, std::vector<cv::Vec2f>::iterator last){
		std::vector<cv::Vec2f>::iterator n = first + std::distance(first, last) / 2;
		nth_element(first, n, last, rhoComp);
		return *n;
	}
	/**
	 * Compares two OpenCV Vec2f's to see which has a bigger theta (angle).
	 * 
	 * @param i The first Vec2f.
	 * @param j The second Vec2f.
	 *
	 * @return True if first vector's theta is smaller than the second vector's theta, false otherwise.
	 **/
	bool thetaComp(cv::Vec2f i, cv::Vec2f j){
		return (i[1] < j[1]);
	}

	/**
	 * Calculate the median theta (angle) of the lines found.
	 *
	 * @param first First line found.
	 * @param last Last line found.
	 *
	 * @return The median line location in the iterator.
	 **/
	inline cv::Vec2f medoidTheta(std::vector<cv::Vec2f>::iterator first, std::vector<cv::Vec2f>::iterator last){
		std::vector<cv::Vec2f>::iterator n = first + std::distance(first, last) / 2;
		nth_element(first, n, last, thetaComp);
		return *n;
	}

	/** 
	 * Automatically called by detect(). After the fiducial has been segmented this function will determine the center point of the crosshair.
	 *
	 * @param image Image with the crosshair
	 * @param center Output point that will be set to the center point
	 * @param mask Operation mask of the same size as image
	 * @param debugImage Output image where debug information will be drawn on, set to NULL for no debug information
	 *
	 * @return True if center point was detected, false if detection failed.
	 **/
	bool FiducialDetector::detectCrosshair(cv::Mat& image, cv::Point2f& center, const cv::Mat& mask, cv::Mat* debugImage){
		cv::Mat filtered;
		cv::bilateralFilter(image, filtered, 5, 50, 50);

		cv::Mat canny;
		cv::Canny(filtered, canny, lowThreshold, highThreshold);

		if(!mask.empty()){
			cv::Mat invMask;
			cv::threshold(mask, invMask, 128, 255, CV_THRESH_BINARY_INV);
			canny.setTo(cv::Scalar(0), invMask);
		}

		// Hough tranform for line detection
		int votes = lineVotes;
		std::vector<cv::Vec2f> lines;
		std::vector<cv::Vec2f> newLines;
		do{
			cv::HoughLines(canny, newLines, 1, M_PI / 180.0, // Step size
			votes); // Minimum number of votes
			if(newLines.size() > maxLines){
				lines = newLines;
				votes++;
			}
		} while(newLines.size() > maxLines);

		if(lines.empty())
			return false;

		// Segment perpendicular lines along the mean angle for center detection
		std::vector<cv::Vec2f> lines1;
		std::vector<cv::Vec2f> lines2;
		float refAngle = medoidTheta(lines.begin(), lines.end())[1];

		// Segment the lines
		for(std::vector<cv::Vec2f>::iterator it = lines.begin(); it != lines.end(); it++){
			float angle = (*it)[1];
			float dist = abs(refAngle - angle);
			if(dist < M_PI / 8.0){
				lines1.push_back(*it);

				// Draw a blue line
				if(debugImage != NULL)
					drawPolarLine(*debugImage, (*it)[0], (*it)[1], cv::Scalar(255, 0, 0), 1);
			} else{
				lines2.push_back(*it);

				// Draw a green line
				if(debugImage != NULL)
					drawPolarLine(*debugImage, (*it)[0], (*it)[1], cv::Scalar(0, 255, 0), 1);
			}
		}

		// Find two lines through the cross center
		cv::Vec2f center1;
		cv::Vec2f center2;

		bool found1 = detectCenterLine(center1, lines1, debugImage);
		bool found2 = detectCenterLine(center2, lines2, debugImage);

		if(verbose && !found1)
			std::cout << "Primary center line not found" << std::endl;
		if(verbose && !found2)
			std::cout << "Secondary center line not found" << std::endl;

		// If both have lines on opposing sides we can intersect them and find the center
		if(found1 && found2){
			float rho1 = center1[0];
			float rho2 = center2[0];
			float omega1 = center1[1];
			float omega2 = center2[1];

			if(rho1 != 0 && rho2 != 0){
				float theta = atan(
				        (cos(omega1) - (rho1 / rho2) * cos(omega2)) / -(sin(omega1) - (rho1 / rho2) * sin(omega2)));
				float r = rho1 / cos(theta - omega1);
				center = cv::Point2f(r * cos(theta), r * sin(theta));
			} else if(rho1 != 0){
				// Line 2 is through the origin
				omega2 -= M_PI / 2.0;
				if(omega1 < M_PI / 4. || omega1 > 3. * M_PI / 4.){
					float y = rho1 * (sin(omega2) / cos(omega1 - omega2));
					float x = (rho1 - y * sin(omega1)) / cos(omega1);
					center = cv::Point2f(x, y);
				} else{ // ~Horizontal line
					float x = rho1 * (cos(omega2) / cos(omega1 - omega2));
					float y = (rho1 - x * cos(omega1)) / sin(omega1);
					center = cv::Point2f(x, y);
				}
			} else if(rho2 != 0){
				// Line 1 is through the origin
				omega1 -= M_PI / 2.0;
				if(omega2 < M_PI / 4. || omega2 > 3. * M_PI / 4.){
					float y = rho2 * (sin(omega1) / cos(omega2 - omega1));
					float x = (rho2 - y * sin(omega2)) / cos(omega2);
					center = cv::Point2f(x, y);
				} else{ // ~Horizontal line
					float x = rho2 * (cos(omega1) / cos(omega2 - omega1));
					float y = (rho2 - x * cos(omega2)) / sin(omega2);
					center = cv::Point2f(x, y);
				}
			} else{
				// Both lines are through the origin
				center = cv::Point2f(0, 0);
			}

			if(debugImage != NULL)
				cv::circle(*debugImage, center, 1, cv::Scalar(0, 0, 255), 2);

			return true;
		}
		return false;
	}

	/**
	 * Detect the center line.
	 * 
	 * @param centerLine Pointer to the center line that will be detected.
	 * @param lines Lines as detected where the center line will lie between.
	 * @param debugImage Output image where debug information will be drawn on, set to NULL for no debug information
	 *
	 * @return True if a center line could be detected, false otherwise.
	 **/
	bool FiducialDetector::detectCenterLine(cv::Vec2f& centerLine, std::vector<cv::Vec2f> lines, cv::Mat* debugImage){
		if(lines.size() < 2){
			if(verbose)
				std::cout << "Not enough lines" << std::endl;
			return false;
		}

		std::vector<cv::Vec2f> lines1;
		std::vector<cv::Vec2f> lines2;

		// Ignore all lines too close or too far from the first line
		std::sort(lines.begin(), lines.end(), rhoComp);
		cv::Vec2f refLine = medoidTheta(lines.begin(), lines.end());
		for(std::vector<cv::Vec2f>::iterator it = lines.begin(); it != lines.end(); ++it){
			float dist = abs(refLine[0] - (*it)[0]);
			if(dist <= maxDist){
				if(dist >= minDist)
					lines2.push_back(*it);
				else
					lines1.push_back(*it);
			}
		}

		if(lines1.empty()){
			if(verbose)
				std::cout << "Primary line not found" << std::endl;
			return false;
		}

		// Select the first medoid line
		cv::Vec2f line1 = medoidRho(lines1.begin(), lines1.end());

		if(lines2.empty()){
			if(verbose)
				std::cout << "Secondary line not found" << std::endl;
			return false;
		}

		// Select the second medoid line
		cv::Vec2f line2 = medoidRho(lines2.begin(), lines2.end());

		// Determine the line between the two selected lines
		float rho = (line1[0] + line2[0]) / 2.0;
		float theta = (line1[1] + line2[1]) / 2.0;
		centerLine = cv::Vec2f(rho, theta);

		// Draw a red line on the debug image
		if(debugImage != NULL)
			drawPolarLine(*debugImage, rho, theta, cv::Scalar(0, 0, 255), 1);

		return true;
	}

	/**
	 * Order a list of three fiducial points. When you have three fiducials there will be three different distances, longest, shortest, and the middle distance.
	 * points[0] = point where the middle and longest lines meet
	 * points[1] = point where the shortest and middle lines meet
	 * points[2] = point where the shortest and longest lines meet
	 *
	 * @param points The points to be ordered.
	 **/
	void FiducialDetector::order(std::vector<cv::Point2f>& points){
		// Find the two diagonal opposite points
		float distance = 0;
		cv::Point2f maxLinePoint1;
		cv::Point2f maxLinePoint2;
		cv::Point2f extra;
		// Check fiducial 0 against 1 and 2 and fiducial 1 against 2
		for(int i = 0; i < 2; i++){
			for(int j = i + 1; j < 3; j++){
				// Find the maximum distance
				float dist = fiducialDistance(points[i], points[j]);
				if(dist > distance){
					distance = dist;
					maxLinePoint1 = points[i];
					maxLinePoint2 = points[j];
				}
			}
		}

		for(int i = 0; i < 3; i++){
			if(points[i] != maxLinePoint1 && points[i] != maxLinePoint2){
				extra = points[i];
				break;
			}
		}

		if(fiducialDistance(maxLinePoint1, extra) > fiducialDistance(maxLinePoint2, extra)){
			points[0] = maxLinePoint1;
			points[1] = extra;
			points[2] = maxLinePoint2;
		} else{
			points[0] = maxLinePoint2;
			points[1] = extra;
			points[2] = maxLinePoint1;
		}
	}
}
