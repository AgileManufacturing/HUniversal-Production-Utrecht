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

#include <camera/unicap_cv_bridge.h>

#include <cstdio>
#include <stdint.h>

#include <boost/interprocess/sync/scoped_lock.hpp>

namespace camera {

namespace unicap_cv_bridge {
	std::vector<std::string> listDevices() {
		std::vector<std::string> devices;
		
		unicap_device_t device;
		for (int i = 0; SUCCESS(unicap_enumerate_devices(NULL, &device, i)); i++) {
			devices.push_back(std::string(device.identifier));
		}
		
		return devices;
	}

	std::vector<std::string> listFormats(int deviceIndex) {
		std::vector<std::string> formats;
		
		unicap_handle_t handle;
		unicap_device_t device;
		unicap_format_t formatSpec;
		unicap_format_t format;

		if (!SUCCESS(unicap_enumerate_devices(NULL, &device, deviceIndex))) {
			throw UnicapCvException("Failed to get device info");
		}

		if (!SUCCESS(unicap_open(&handle, &device))) {
			throw UnicapCvException("Failed to open device");
		}

		unicap_void_format(&formatSpec);

		char str[256];

		for (int i = 0; SUCCESS(unicap_enumerate_formats(handle, &formatSpec, &format, i)); i++) {
			sprintf(str, "%d: %s [%dx%d]", i, format.identifier, format.size.width, format.size.height);
			formats.push_back(std::string(str));
		}

		if (!SUCCESS(unicap_close(handle))) {
			throw UnicapCvException("Failed to close the device");
		}
		return formats;
	}

	std::vector<std::string> listProperties(int deviceIndex) {
		std::vector<std::string> properties;
	
		unicap_handle_t handle;
		unicap_device_t device;
		unicap_property_t propertySpec;
		unicap_property_t property;
		if (!SUCCESS(unicap_enumerate_devices(NULL, &device, deviceIndex))) {
			throw UnicapCvException("Failed to get device info");
		}

		if (!SUCCESS(unicap_open(&handle, &device))) {
			throw UnicapCvException("Failed to open device");
		}

		unicap_void_property(&propertySpec);

		for (int i = 0; SUCCESS(unicap_enumerate_properties(handle, &propertySpec, &property, i)); i++) {
			std::stringstream stringStream;
			stringStream << "[" << property.identifier << "] catagory:" << property.category;
			if (std::string(property.unit) != "")
				stringStream << property.unit;
			bool first = true;
			{
#define PRINT(f, s) if(property.flags_mask & (f)) stringStream << (first ? ((first = false), "") : ", ") << (s) << ":" << (property.flags & (f) ? "1" : "0")
				stringStream << " flags:[";
				PRINT(UNICAP_FLAGS_MANUAL, "manual");
				PRINT(UNICAP_FLAGS_AUTO, "auto");
				PRINT(UNICAP_FLAGS_ONE_PUSH, "onepush");
				PRINT(UNICAP_FLAGS_READ_OUT, "readout");
				PRINT(UNICAP_FLAGS_ON_OFF, "on");
				PRINT(UNICAP_FLAGS_READ_ONLY, "readonly");
				PRINT(UNICAP_FLAGS_FORMAT_CHANGE, "formatchange");
				PRINT(UNICAP_FLAGS_WRITE_ONLY, "writeonly");
				PRINT(UNICAP_FLAGS_CHECK_STEPPING, "checkstepping");
				PRINT(UNICAP_FLAGS_DUMMY_VALUE, "dummyvalue");
#undef PRINT
				stringStream << "] ";
			}
			switch (property.type) {
			case UNICAP_PROPERTY_TYPE_RANGE:
				stringStream << "type:range stepping:" << property.stepping << " min:" << property.range.min << " max:"
				        << property.range.max << " value:" << property.value;
				break;
			case UNICAP_PROPERTY_TYPE_VALUE_LIST:
				stringStream << "type:value list value:" << property.value << " list:{ ";
				if (property.value_list.value_count > 0) {
					stringStream << property.value_list.values[0];
				}
				for (int i = 1; i < property.value_list.value_count; i++) {
					stringStream << ", " << property.value_list.values[i];
				}
				stringStream << " }";
				break;
			case UNICAP_PROPERTY_TYPE_MENU:
				stringStream << "type:menu value:" << property.menu_item << "list:{ ";
				if (property.menu.menu_item_count > 0) {
					stringStream << property.menu.menu_items[0];
				}
				for (int i = 1; i < property.menu.menu_item_count; i++) {
					stringStream << ", " << property.menu.menu_items[i];
				}
				stringStream << " }";
				break;
			case UNICAP_PROPERTY_TYPE_DATA:
				stringStream << std::hex << "type:data {";
				if (property.property_data_size > 0) {
					stringStream << (unsigned) (((char*) property.property_data)[0]);
				}
				for (unsigned i = 1; i < property.property_data_size; i++) {
					stringStream << ", " << (unsigned) (((char*) property.property_data)[i]);
				}
				stringStream << " }";
				break;
			case UNICAP_PROPERTY_TYPE_FLAGS:

				break;
			default:
				stringStream << "type:UNKNOWN";
				break;
			}

			properties.push_back(stringStream.str());
		}

		if (!SUCCESS(unicap_close(handle))) {
			throw UnicapCvException("Failed to close the device");
		}
		
		return properties;
	}

	void printDevicesInventory(std::ostream& out) {
		std::vector<std::string> devices;
		devices = listDevices();
		for (unsigned deviceIndex = 0; deviceIndex < devices.size(); deviceIndex++) {
			out << "[" << deviceIndex << "] " << devices[deviceIndex] << std::endl << "\tformats:" << std::endl;
			std::vector<std::string> formats;
			formats = listFormats(deviceIndex);
			for (unsigned formatIndex = 0; formatIndex < formats.size(); formatIndex++) {
				out << "\t\t[" << formatIndex << "] " << formats[formatIndex] << std::endl;
			}

			out << "\tproperties:" << std::endl;
			std::vector<std::string> properties;
			properties = listProperties(deviceIndex);
			for (std::vector<std::string>::iterator it = properties.begin(); it != properties.end(); ++it) {
				out << "\t\t" << *it << std::endl;
			}
		}
	}

	static void newFrameCallback(unicap_event_t event, unicap_handle_t handle, unicap_data_buffer_t* buffer, void *owner) {
		((UnicapCvCamera*) owner)->handleFrame(buffer);
	}

	UnicapCvCamera::UnicapCvCamera(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, CameraListener* listener, double fps, 
			int deviceIndex, int formatIndex) :
			Camera(equipletName, identifier, listener, fps, deviceIndex) {
		if (!SUCCESS(unicap_enumerate_devices(NULL, &device, deviceIndex))) {
			throw UnicapCvException("Failed to get device info");
		}

		if (!SUCCESS(unicap_open(&handle, &device))) {
			throw UnicapCvException("Failed to open device");
		}

		unicap_format_t formatSpec;
		unicap_void_format(&formatSpec);

		if (!SUCCESS(unicap_enumerate_formats(handle, &formatSpec, &format, formatIndex))) {
			throw UnicapCvException("Failed to get video format");
		}

		if (format.size_count) {
			format.size.width = format.sizes[0].width;
			format.size.height = format.sizes[0].height;
		}

		format.buffer_type = UNICAP_BUFFER_TYPE_SYSTEM;

		if (!SUCCESS(unicap_set_format( handle, &format))) {
			throw UnicapCvException("Failed setting video format");
		}

		if (format.bpp != 24) {
			throw UnicapCvException("Invalid bits per pixel");
		}

		// set these values for the camFrame, enabeling direct copying the frame into the buffer
		camFrame = cv::Mat(format.size.height, format.size.width, getFrameFormat());

		unicap_register_callback(handle, UNICAP_EVENT_NEW_FRAME, (unicap_callback_t) (unicap_cv_bridge::newFrameCallback),
		        this);
		if (!SUCCESS(unicap_start_capture(handle))) {
			throw UnicapCvException("Failed to start capture");
		}

	}

	UnicapCvCamera::~UnicapCvCamera(void) {
		if (!SUCCESS( unicap_stop_capture(handle))) {
			throw UnicapCvException("Failed to stop capture");
		}

		if (!SUCCESS(unicap_close(handle))) {
			throw UnicapCvException("Failed to close the device");
		}
	}

	
	void UnicapCvCamera::getWhiteBalance(double& blue, double& red) {
		if (!SUCCESS(unicap_get_property_value(handle, "White Balance Blue", &blue))) {
			throw UnicapCvException("Failed to get property \"White Balance Blue\"");
		}

		if (!SUCCESS(unicap_get_property_value(handle, "White Balance Red", &red))) {
			throw UnicapCvException("Failed to get property \"White Balance Red\"");
		}
	}

	void UnicapCvCamera::setWhiteBalance(double blue, double red) {
		if (!SUCCESS(unicap_set_property_value(handle, (char*)"White Balance Blue", blue))) {
			throw UnicapCvException("Failed to set property \"White Balance Blue\"");
		}

		if (!SUCCESS(unicap_set_property_value(handle, (char*)"White Balance Red", red))) {
			throw UnicapCvException("Failed to set property \"White Balance Red\"");
		}
	}

	void UnicapCvCamera::setAutoWhiteBalance(bool automatic) {
		if (automatic) {
			if (!SUCCESS(unicap_set_property_auto(handle, (char*)"White Balance Mode"))) {
				throw UnicapCvException("Failed to set white balance mode to auto");
			}
		} else {
			if (!SUCCESS(unicap_set_property_manual(handle, (char*)"White Balance Mode"))) {
				throw UnicapCvException("Failed to set white balance mode to auto");
			}
		}
	}

	void UnicapCvCamera::setExposure(double exposure) {
		if (!SUCCESS(unicap_set_property_value(handle, (char*)"Shutter", exposure))) {
			throw UnicapCvException("Failed to set exposure");
		}
	}

	void UnicapCvCamera::handleFrame(unicap_data_buffer_t* buffer) {
		if(isNewFrameRequired() == true) {
			if (camFrame.cols != format.size.width) {
				REXOS_ERROR_STREAM("Got frame with incorrect width, expecting " << format.size.width << ", got " << camFrame.cols);
				return;
			} else if (camFrame.rows != format.size.height) {
				REXOS_ERROR_STREAM("Got frame with incorrect height, expecting " << format.size.height << ", got " << camFrame.rows);
				return;
			} else if (camFrame.type() != getFrameFormat()) {
				REXOS_ERROR_STREAM("Got frame with incorrect format, expecting " << getFrameFormat() << ", got " << camFrame.type());
				return;
			} else {
				uint8_t* dest = camFrame.ptr();
				uint8_t* source = buffer->data;
				//memcpy(dest, source, 3 * (mat->cols * mat->rows));
				for (int n = 0; n < format.size.width * format.size.height * 3; n += 3) {
					dest[n] = source[n + 2];
					dest[n + 1] = source[n + 1];
					dest[n + 2] = source[n];
				}
			}
			onNewFrame();
		}
		unicap_set_property_auto(handle, (char*) "Shutter");
	}
	cv::Size UnicapCvCamera::getFrameSize() {
		return cv::Size(format.size.width, format.size.height);
	}
	int UnicapCvCamera::getFrameFormat() {
		return CV_8UC3;
	}
	void UnicapCvCamera::enableCamera(bool enabled) {
		Camera::enableCamera(enabled);
		if(enabled == true) {
			if (!SUCCESS(unicap_start_capture(handle))) {
				throw UnicapCvException("Failed to start capture");
			}
		} else {
			if (!SUCCESS( unicap_stop_capture(handle))) {
				throw UnicapCvException("Failed to stop capture");
			}
		}
	}
}
}
