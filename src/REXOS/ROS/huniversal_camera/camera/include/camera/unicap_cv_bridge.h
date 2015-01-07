/**
 * @file unicap_cv_bridge.h
 * @brief Symbolizes a unicap camera. frames can be grabbed and put into matrices
 * @date Created: 2012-01-??  TODO: Date
 *
 * @author Lukas Vermond
 * @author Kasper van Nieuwland
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

#pragma once

#define private reserved
#include <unicap.h>
#include <unicap_status.h>
#undef private
#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <iostream>
#include "rexos_logger/rexos_logger.h"

namespace unicap_cv_bridge {
	/**
	 * @brief obtain a list of device names
	 * @return devices will contain device names
	 **/
	std::vector<std::string> listDevices();

	/**
	 * @brief obtain a list of formats
	 * @param deviceIndex the device number
	 * @return formats will contain format desciptions
	 **/
	std::vector<std::string> listFormats(int deviceIndex);

	/**
	 * @brief obtain a list of properties
	 * @param deviceIndex the device number
	 * @return properties will contain properties
	 **/
	std::vector<std::string> listProperties(int deviceIndex);
	/**
	 * @brief print a list of all connected devices with it's formats
	 * @param out the output stream. Defaults to standard out
	 **/
	void printDevicesInventory(std::ostream& out = std::cout);

	/**
	 * @brief unicap exception
	 **/
	class UnicapCvException: public std::runtime_error {
	public:
		UnicapCvException(const std::string& msg) :
				std::runtime_error(msg) {
		}
		virtual ~UnicapCvException(void) throw () {
		}
	};

	/**
	 * @brief handle to unicap camera
	 **/
	class UnicapCvCamera {
	private:
		typedef enum frameCapState {
			FCSTATE_DONT_COPY, FCSTATE_COPY, FCSTATE_SUCCESS, FCSTATE_FAILURE
		} frameCapState;

		bool tripmode;

		unicap_handle_t handle;
		unicap_device_t device;
		unicap_format_t format;

		boost::mutex mut;
		boost::condition_variable cond;
		frameCapState state;
		cv::Mat* mat;

	public:
		/**
		 * @brief constructs a unicap camera
		 * @param dev device number
		 * @param formatIndex format number
		 **/
		UnicapCvCamera(int dev, int formatIndex);

		~UnicapCvCamera(void);

		void setTripMode(bool tripmode);

		/**
		 * @brief get white balance settings
		 * @param blue out parameter
		 * @param red out parameter
		 **/
		void getWhiteBalance(double& blue, double& red);

		/**
		 * @brief set white balance settings
		 * @param blue blue
		 * @param red red
		 **/
		void setWhiteBalance(double blue, double red);

		/**
		 * @brief set exposure
		 * @param exposure exposure
		 **/
		void setExposure(double exposure);

		/**
		 * @brief set auto white balance
		 * @param automatic true for on, false otherwise
		 **/
		void setAutoWhiteBalance(bool automatic);

		/**
		 * @brief get outputed image width
		 * @return image width
		 **/
		int getImgWidth(void);

		/**
		 * @brief get outputed image height
		 * @return image height
		 **/
		int getImgHeight(void);

		/**
		 * @brief get outputed image format
		 * @return image format
		 **/
		int getImgFormat(void);

		/**
		 * @brief callback function which gets called when a new frame is captured
		 * @note should not be called by user
		 * @param buffer buffer which holds the frame
		 **/
		void newFrameCallback(unicap_data_buffer_t* buffer);

		/**
		 * @brief get a frame
		 *
		 * this function blocks until a new frame is captured
		 * @param mat the frame is copied into this matrix
		 **/
		void getFrame(cv::Mat* mat);
	};
}
