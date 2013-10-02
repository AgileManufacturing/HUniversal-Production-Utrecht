/**
 * @file RectifyImage.h
 * @brief Interface class to correct lens distortion
 * @date Created: 2012-01-??  TODO: Date
 *
 * @author Glenn Meerstra
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

#ifndef RECTIFYIMAGE_H
#define RECTIFYIMAGE_H

#include <opencv2/core/core.hpp>
/**
 * @brief this class creates and uses a matrix in order to rectify an image \n
 * the matrix is created by loading a directory with checker board pattern images
 */
namespace Camera {
	class RectifyImage {
	public:
		cv::Mat distCoeffs;
		cv::Mat cameraMatrix;
	private:
		std::vector<std::vector<cv::Point3f> > objectPoints;
		std::vector<std::vector<cv::Point2f> > imagePoints;
		cv::Mat map1;
		cv::Mat map2;
		cv::Size imageSize;

		void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);
		double calibrate(cv::Size &imageSize);
	public:
		int createMatrices(const cv::Size &boardSize, const std::vector<cv::Mat*> images);
		/**
		 * Creates a matrix from all the images located in imageDir and stores it in the XMl file
		 *
		 * @param imageDir the directory which contains the image to rectify
		 * @param boardSize amount of squares horizontally -1, amount of squares vertically -1
		 * @param XMLName the name of the xml file were the matrix for rectification is written to
		 * @return the amount of images which are successfully processed
		 */
		void loadMatrices(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
		void setImageSize(const cv::Size &imageSize);
		bool createXML(const char* imageDir, const char* XMLName);
		bool loadXML(const char* XMLName);
		/**
		 * this function loads a matrix to rectify
		 *
		 * @param XMLName the name of the XML
		 * @param imageSize the size of the image
		 * @return <i>false</i> if XMLName is not available
		 */
		void initRectify();
		/**
		 * Returns the given image rectified
		 *
		 * @param input The image that needs to be rectified
		 * @param output The rectified image
		 */
		void rectify(const cv::Mat &input, cv::Mat &output);
	};
}
#endif /* RECTIFYIMAGE_H */
