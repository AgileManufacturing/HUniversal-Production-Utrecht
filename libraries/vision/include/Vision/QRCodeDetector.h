//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        DetectQRCode
// File:           BarcodeDetector.h
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
#ifndef DETECTBARCODE_H_
#define DETECTBARCODE_H_

#include <stdlib.h>
#include <zbar.h>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include "DataTypes/Crate.h"

namespace Vision{
/**
 * @brief This class can detect barcodes from a Mat object
 */

    class QRCodeDetector{
    private:
	///@brief the scanner which scans the code from an image
        zbar::ImageScanner scanner;
    public:
        ///@brief constructor sets the values for the scanner
        QRCodeDetector();
        ///@brief deconstructor
        ~QRCodeDetector();
    
        /**
         * @fn bool detect(cv::Mat image, std::string &result)
         * @brief detects QR codes on the image
         * @param image the image to detect the code on
         * @param result the string to write the result to
         * @return true if we have a result
         **/
        bool detect(cv::Mat& image, std::string& result);
    
        /**
         * @fn bool detect(cv::Mat image, std::string &result)
         * @brief detects crates using the QR code
         * @param image the image to detect the code on
         * @param crates list of crates
         */
        void detectCrates(cv::Mat& image, std::vector<DataTypes::Crate>& crates, cv::TermCriteria criteria =
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 15, 0.1));
    };
}


#endif /* DETECTBARCODE_H_ */
