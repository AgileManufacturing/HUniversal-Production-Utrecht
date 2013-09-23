/**
 * @file QRCodeDetector.h
 * @brief Detects QR codes and extract their values. If the QR code belongs to a crate, also give positionary data.
 * @date Created: 2012-10-02
 *
 * @author Glenn Meerstra
 * @author Zep Mouris
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

#ifndef QRCodeDetector_h
#define QRCodeDetector_h

#include <stdlib.h>
#include <zbar.h>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include "rexos_datatypes/Crate.h"

namespace rexos_vision{
	/**
	 * This class can detect QR-/barcodes from a Mat object
	 **/
	class QRCodeDetector{
	private:
		/**
		 * @var zbar::ImageScanner scanner
		 * The QR-/barcode detector from the zbar library
		 **/
		zbar::ImageScanner scanner;

	public:
		QRCodeDetector();

		void detectQRCodes(cv::Mat& image, std::vector<rexos_datatypes::Crate>& crates, cv::TermCriteria criteria =
				cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 15, 0.1));
		void detectQRCodes(cv::Mat& image, std::vector<std::string>& reconfigureCommands);
	};
}
#endif /* QRCodeDetector_h */
