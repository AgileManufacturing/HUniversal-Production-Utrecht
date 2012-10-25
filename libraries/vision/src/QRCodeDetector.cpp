//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        DetectQRCode
// File:           BarcodeDetector.cpp
// Description:    Detects barcodes and extract values
// Author:         Glenn Meerstra & Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of DetectQRCode.
//
// DetectQRCode is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DetectQRCode is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DetectQRCode.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <sstream>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include "Vision/QRCodeDetector.h"

namespace Vision{
	QRCodeDetector::QRCodeDetector() {
		scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
	}

	QRCodeDetector::~QRCodeDetector() {}

	bool QRCodeDetector::detect(cv::Mat& image, std::string &result) {
		try {
		/**
		 * create an image in zbar with:
		 * width
		 * height
		 * fourcc format "Y800" (simple, single Y plane for monchrome images)
		 * pointer to image.data
		 * area of the image
		**/
		 zbar::Image zbarImage(image.cols, image.rows, "Y800", (void*)image.data, image.cols * image.rows);

		// scan for symbols
		 int amountOfScannedResults = scanner.scan(zbarImage);

		 if (amountOfScannedResults) {
		 	zbar::Image::SymbolIterator symbol = zbarImage.symbol_begin();

		 	result = symbol->get_data();

		 } else {
		 	return false;
		 }
		} catch (std::exception &e) {
			return false;
		}
		return true;
	}

	void QRCodeDetector::detectCrates(cv::Mat& image, std::vector<DataTypes::Crate> &crates, cv::TermCriteria criteria) {
		try {
		/**
		 * create an image in zbar with:
		 * width
		 * height
		 * fourcc format "Y800" (simple, single Y plane for monchrome images)
		 * pointer to image.data
		 * area of the image
		**/
		 zbar::Image zbarImage(image.cols, image.rows, "Y800", (void*)image.data, image.cols * image.rows);

		 int amountOfScannedResults = scanner.scan(zbarImage);

		 if (amountOfScannedResults > 0) {

		 	zbar::Image::SymbolIterator it = zbarImage.symbol_begin();
		 	for(; it!=zbarImage.symbol_end(); ++it) {
				// add all "position" corners of a qr code to a vector
		 		std::vector<cv::Point2f> corners;
		 		corners.push_back(cv::Point2f(it->get_location_x(1), it->get_location_y(1)));
		 		corners.push_back(cv::Point2f(it->get_location_x(0), it->get_location_y(0)));
		 		corners.push_back(cv::Point2f(it->get_location_x(3), it->get_location_y(3)));

				/**
				 * windowsSize is half of the sidelength of the window around every coordinate to check by cornerSubPix.
				 * No idea why the distance between two corners is divided by 65.0 (effectively), but the result is:
				 *  65 px distance = (1 x 1) windowsSize > ( 3 x  3) window
				 * 130 px distance = (2 x 2) windowsSize > ( 5 x  5) window
				 * 195 px distance = (3 x 3) windowsSize > ( 7 x  7) window
				 * 260 px distance = (4 x 4) windowsSize > ( 9 x  9) window
				 * 325 px distance = (5 x 5) windowsSize > (11 x 11) window
				 * 390 px distance = (6 x 6) windowsSize > (13 x 13) window
				 * 455 px distance = (7 x 7) windowsSize > (15 x 15) window
				 * 520 px distance = (8 x 8) windowsSize > (17 x 17) window
				 * etc...
				 **/
				 float windowsSize = 2.0 * (DataTypes::Crate::distance(corners[0], corners[2]) / 130.0);

				/**
				 * The cornerSubPix function iterates to find the sub-pixel accurate location of corners or radial saddle points
				 * corners is now updated!
				 **/
				 cv::cornerSubPix(image, corners, cv::Size(windowsSize,windowsSize), cv::Size(-1,-1), criteria);

				 crates.push_back(DataTypes::Crate(it->get_data(), corners));
				}
			}
		} catch (std::exception &e) {
			return;
		}
	}
}