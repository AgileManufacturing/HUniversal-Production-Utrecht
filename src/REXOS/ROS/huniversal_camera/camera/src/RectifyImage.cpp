/**
 * @file RectifyImage.cpp
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

#include <camera/RectifyImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/filesystem.hpp>
#include <sstream>
#include <iostream>

using namespace camera;

void RectifyImage::addPoints(const std::vector<cv::Point2f>& imageCorners,
		const std::vector<cv::Point3f>& objectCorners) {
	imagePoints.push_back(imageCorners);
	objectPoints.push_back(objectCorners);
}

double RectifyImage::calibrate(cv::Size &imageSize) {
	std::vector<cv::Mat> rvecs, tvecs;
	return calibrateCamera(objectPoints, imagePoints, // the image points
			imageSize, // image size
			cameraMatrix, // output camera matrix
			distCoeffs, // output distortion matrix
			rvecs, tvecs, // Rs, Ts
			0); // set options
}

int RectifyImage::createMatrices(const cv::Size &boardSize, const std::vector<cv::Mat*> images) {
	std::vector<cv::Point2f> imageCorners;
	std::vector<cv::Point3f> objectCorners;
	
	// openCV needs to know the locations of the points on the chessboard.
	for (int i = 0; i < boardSize.height; i++) {
		for (int j = 0; j < boardSize.width; j++) {
			objectCorners.push_back(cv::Point3f(i, j, 0.0f));
		}
	}

	int successes = 0;
	for (int i = 0; i < images.size(); i++) {
		cv::Mat image = *images[i];

		if (image.data) {
			bool found = findChessboardCorners(image, boardSize, imageCorners);

			if (found) {
				// find a more precise location using subpixel information (color transition)
				cv::cornerSubPix(image, imageCorners, cv::Size(4, 4), cv::Size(-1, -1),
						cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));

				if (imageCorners.size() == (uint) boardSize.area()) {
					addPoints(imageCorners, objectCorners);
					successes++;
				}
			}
		}
	}

	if (successes == 0) {
		return successes;
	}

	cv::Size imageSize(images[0]->cols, images[0]->rows);
	
	calibrate(imageSize);
	
	return successes;
}

void RectifyImage::loadMatrices(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
	this->cameraMatrix = cameraMatrix;
	this->distCoeffs = distCoeffs;
	initRectify();
}

void RectifyImage::setImageSize(const cv::Size &imageSize) {
	this->imageSize = imageSize;
}


bool RectifyImage::createXML(const char* XMLName) {
	cv::FileStorage fs(XMLName, cv::FileStorage::WRITE);
	fs << "cameraMatrix" << cameraMatrix;
	fs << "distCoeffs" << distCoeffs;
	fs.release();
	return true;
}

bool RectifyImage::loadXML(const char* XMLName) {
	if (!boost::filesystem::is_regular_file(XMLName)) {
		return false;
	}

	cv::FileStorage fs(XMLName, cv::FileStorage::READ);
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	initRectify();
	return true;
}

void RectifyImage::initRectify() {
	initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cv::Mat(), imageSize, CV_32FC1, map1, map2);
}

cv::Mat RectifyImage::rectify(const cv::Mat &input) {
	cv::Mat output;
	cv::remap(input, output, map1, map2, cv::INTER_LINEAR);
	return output;
}
