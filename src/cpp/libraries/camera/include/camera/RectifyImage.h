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
		/**
		 * Creates the matrices from all the images in the vector
		 *
		 * @param boardSize amount of squares horizontally -1, amount of squares vertically -1
		 * @param images the vector containing all the cv:mat images. The images must be MONO8 format.
		 * @return the amount of images which are successfully processed
		 */
		int createMatrices(const cv::Size &boardSize, const std::vector<cv::Mat*> images);
		/**
		 * Loads the cameraMatrix and distCoeffs "matrix" from the params and initializes the rectifier
		 *
		 * @param cameraMatrix a 3x3 cv::mat matrix. This is a standard translation/rotation/scale matrix
		 * @param distCoeffs a 5x1 cv::mat matrix.
		 */
		void loadMatrices(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
		/**
		 * Sets the image size. Before the modification takes effect, you must initialize the rectifier
		 *
		 * @param imageSize the size of the input images
		 */
		void setImageSize(const cv::Size &imageSize);
		/**
		 * Creates the matrices from all the images in the vector and writes the xml file containing these matrices
		 * 
		 * @param boardSize amount of squares horizontally -1, amount of squares vertically -1
		 * @param images the vector containing all the cv:mat images. The images must be MONO8 format.
		 * @param XMLName the path to the xml file which will saved
		 * @return true if succesful
		 */
		bool createXML(const char* XMLName);
		/**
		 * Creates the matrices from the xml file containing these matrices
		 * 
		 * @param XMLName the path to the xml file which will loaded
		 * @return true if succesful
		 */
		bool loadXML(const char* XMLName);
		/**
		 * this function initializes the rectifier
		 *
		 */
		void initRectify();
		/**
		 * Returns the given image rectified
		 *
		 * @param input The image that needs to be rectified
		 */
		cv::Mat rectify(const cv::Mat &input);
	};
}
#endif /* RECTIFYIMAGE_H */
