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

namespace unicap_cv_bridge {
	void list_devices(std::vector<std::string>& devices) {
		unicap_device_t device;

		for (int i = 0; SUCCESS(unicap_enumerate_devices(NULL, &device, i)); i++) {
			devices.push_back(std::string(device.identifier));
		}
	}

	void list_formats(int dev, std::vector<std::string>& formats) {
		unicap_handle_t handle;
		unicap_device_t device;
		unicap_format_t format_spec;
		unicap_format_t format;

		if (!SUCCESS(unicap_enumerate_devices(NULL, &device, dev))) {
			throw unicap_cv_exception("Failed to get device info");
		}

		if (!SUCCESS(unicap_open(&handle, &device))) {
			throw unicap_cv_exception("Failed to open device");
		}

		unicap_void_format(&format_spec);

		char str[256];

		for (int i = 0; SUCCESS(unicap_enumerate_formats(handle, &format_spec, &format, i)); i++) {
			sprintf(str, "%d: %s [%dx%d]", i, format.identifier, format.size.width, format.size.height);
			formats.push_back(std::string(str));
		}

		if (!SUCCESS(unicap_close(handle))) {
			throw unicap_cv_exception("Failed to close the device");
		}
	}

	void list_properties(int dev, std::vector<std::string>& properties) {
		unicap_handle_t handle;
		unicap_device_t device;
		unicap_property_t property_spec;
		unicap_property_t property;
		if (!SUCCESS(unicap_enumerate_devices(NULL, &device, dev))) {
			throw unicap_cv_exception("Failed to get device info");
		}

		if (!SUCCESS(unicap_open(&handle, &device))) {
			throw unicap_cv_exception("Failed to open device");
		}

		unicap_void_property(&property_spec);

		for (int i = 0; SUCCESS(unicap_enumerate_properties(handle, &property_spec, &property, i)); i++) {
			std::stringstream ss;
			ss << "[" << property.identifier << "] catagory:" << property.category;
			if (std::string(property.unit) != "")
				ss << property.unit;
			bool first = true;
			{
#define PRINT(f, s) if(property.flags_mask & (f)) ss << (first ? ((first = false), "") : ", ") << (s) << ":" << (property.flags & (f) ? "1" : "0")
				ss << " flags:[";
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
				ss << "] ";
			}
			switch (property.type) {
			case UNICAP_PROPERTY_TYPE_RANGE:
				ss << "type:range stepping:" << property.stepping << " min:" << property.range.min << " max:"
				        << property.range.max << " value:" << property.value;
				break;
			case UNICAP_PROPERTY_TYPE_VALUE_LIST:
				ss << "type:value list value:" << property.value << " list:{ ";
				if (property.value_list.value_count > 0) {
					ss << property.value_list.values[0];
				}
				for (int i = 1; i < property.value_list.value_count; i++) {
					ss << ", " << property.value_list.values[i];
				}
				ss << " }";
				break;
			case UNICAP_PROPERTY_TYPE_MENU:
				ss << "type:menu value:" << property.menu_item << "list:{ ";
				if (property.menu.menu_item_count > 0) {
					ss << property.menu.menu_items[0];
				}
				for (int i = 1; i < property.menu.menu_item_count; i++) {
					ss << ", " << property.menu.menu_items[i];
				}
				ss << " }";
				break;
			case UNICAP_PROPERTY_TYPE_DATA:
				ss << std::hex << "type:data {";
				if (property.property_data_size > 0) {
					ss << (unsigned) (((char*) property.property_data)[0]);
				}
				for (unsigned i = 1; i < property.property_data_size; i++) {
					ss << ", " << (unsigned) (((char*) property.property_data)[i]);
				}
				ss << " }";
				break;
			case UNICAP_PROPERTY_TYPE_FLAGS:

				break;
			default:
				ss << "type:UNKNOWN";
				break;
			}

			properties.push_back(ss.str());
		}

		if (!SUCCESS(unicap_close(handle))) {
			throw unicap_cv_exception("Failed to close the device");
		}
	}

	void print_devices_inventory(std::ostream& out) {
		std::vector<std::string> devices;
		list_devices(devices);
		for (unsigned dev = 0; dev < devices.size(); dev++) {
			out << "[" << dev << "] " << devices[dev] << std::endl << "\tformats:" << std::endl;
			std::vector<std::string> formats;
			list_formats(dev, formats);
			for (unsigned fmt = 0; fmt < formats.size(); fmt++) {
				out << "\t\t[" << fmt << "] " << formats[fmt] << std::endl;
			}

			out << "\tproperties:" << std::endl;
			std::vector<std::string> properties;
			list_properties(dev, properties);
			for (std::vector<std::string>::iterator it = properties.begin(); it != properties.end(); ++it) {
				out << "\t\t" << *it << std::endl;
			}
		}
	}

	static void new_frame_cb(unicap_event_t event, unicap_handle_t handle, unicap_data_buffer_t* buffer, void *owner) {
		((unicap_cv_camera*) owner)->new_frame_cb(buffer);
	}

	unicap_cv_camera::unicap_cv_camera(int dev, int fmt) :
			tripmode(false) {
		if (!SUCCESS(unicap_enumerate_devices(NULL, &device, dev))) {
			throw unicap_cv_exception("Failed to get device info");
		}

		if (!SUCCESS(unicap_open(&handle, &device))) {
			throw unicap_cv_exception("Failed to open device");
		}

		unicap_format_t format_spec;
		unicap_void_format(&format_spec);

		if (!SUCCESS(unicap_enumerate_formats(handle, &format_spec, &format, fmt))) {
			throw unicap_cv_exception("Failed to get video format");
		}

		if (format.size_count) {
			format.size.width = format.sizes[0].width;
			format.size.height = format.sizes[0].height;
		}

		format.buffer_type = UNICAP_BUFFER_TYPE_SYSTEM;

		if (!SUCCESS(unicap_set_format( handle, &format))) {
			throw unicap_cv_exception("Failed setting video format");
		}

		if (format.bpp != 24) {
			std::cout << "bits per pixel: " << format.bpp << std::endl;
			throw unicap_cv_exception("Invalid bits per pixel");
		}

		unicap_register_callback(handle, UNICAP_EVENT_NEW_FRAME, (unicap_callback_t) (unicap_cv_bridge::new_frame_cb),
		        this);
		if (!SUCCESS(unicap_start_capture(handle))) {
			throw unicap_cv_exception("Failed to start capture");
		}

	}

	unicap_cv_camera::~unicap_cv_camera(void) {
		if (!SUCCESS( unicap_stop_capture(handle))) {
			throw unicap_cv_exception("Failed to stop capture");
		}

		if (!SUCCESS(unicap_close(handle))) {
			throw unicap_cv_exception("Failed to close the device");
		}
	}

	void unicap_cv_camera::set_trip_mode(bool tripmode) {
		this->tripmode = tripmode;
	}

	void unicap_cv_camera::get_white_balance(double& blue, double& red) {
		if (!SUCCESS(unicap_get_property_value(handle, "White Balance Blue", &blue))) {
			throw unicap_cv_exception("Failed to get property \"White Balance Blue\"");
		}

		if (!SUCCESS(unicap_get_property_value(handle, "White Balance Red", &red))) {
			throw unicap_cv_exception("Failed to get property \"White Balance Red\"");
		}
	}

	void unicap_cv_camera::set_white_balance(double blue, double red) {
		if (!SUCCESS(unicap_set_property_value(handle, (char*)"White Balance Blue", blue))) {
			throw unicap_cv_exception("Failed to set property \"White Balance Blue\"");
		}

		if (!SUCCESS(unicap_set_property_value(handle, (char*)"White Balance Red", red))) {
			throw unicap_cv_exception("Failed to set property \"White Balance Red\"");
		}
	}

	void unicap_cv_camera::set_auto_white_balance(bool automatic) {
		if (automatic) {
			if (!SUCCESS(unicap_set_property_auto(handle, (char*)"White Balance Mode"))) {
				throw unicap_cv_exception("Failed to set white balance mode to auto");
			}
		} else {
			if (!SUCCESS(unicap_set_property_manual(handle, (char*)"White Balance Mode"))) {
				throw unicap_cv_exception("Failed to set white balance mode to auto");
			}
		}
	}

	void unicap_cv_camera::set_exposure(double exposure) {
		if (!SUCCESS(unicap_set_property_value(handle, (char*)"Shutter", exposure))) {
			throw unicap_cv_exception("Failed to set exposure");
		}
	}

	int unicap_cv_camera::get_img_width(void) {
		return format.size.width;
	}

	int unicap_cv_camera::get_img_height(void) {
		return format.size.height;
	}

	int unicap_cv_camera::get_img_format(void) {
		return CV_8UC3;
	}

	void unicap_cv_camera::new_frame_cb(unicap_data_buffer_t* buffer) {
		boost::unique_lock<boost::mutex> lock(mut);
		if (state == FCSTATE_COPY) {
			if (mat->cols == format.size.width && mat->rows == format.size.height && mat->type() == CV_8UC3) {
				uint8_t* dest = mat->ptr();
				uint8_t* source = buffer->data;
				//memcpy(dest, source, 3 * (mat->cols * mat->rows));
				for (int n = 0; n < format.size.width * format.size.height * 3; n += 3) {
					if (tripmode) {
						dest[n] = (source[n] % 33) * (255 / 33);
						dest[n + 1] = (source[n + 1] % 40) * (255 / 40);
						dest[n + 2] = (source[n + 2] % 18) * (255 / 18);
					} else {
						dest[n] = source[n + 2];
						dest[n + 1] = source[n + 1];
						dest[n + 2] = source[n];
					}
				}

				state = FCSTATE_SUCCESS;
			} else {
				state = FCSTATE_FAILURE;
			}
			cond.notify_all();
		}
		unicap_set_property_auto(handle, (char*) "Shutter");
	}

	void unicap_cv_camera::get_frame(cv::Mat* mat) {
		boost::unique_lock<boost::mutex> lock(mut);
		state = FCSTATE_COPY;
		this->mat = mat;
		cond.wait(lock);
		frame_cap_state _state = state;
		state = FCSTATE_DONT_COPY;
		if (_state == FCSTATE_FAILURE) {
			throw unicap_cv_exception("Failed to get frame");
		}
	}
}

