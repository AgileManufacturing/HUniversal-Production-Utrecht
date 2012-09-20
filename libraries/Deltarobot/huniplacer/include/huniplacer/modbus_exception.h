//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           modbus_exception.h
// Description:    exception thrown if an error ocures during modbus actions
// Author:         Lukas Vermond & Kasper van Nieuwland
// Notes:          -
//
// License:        GNU GPL v3
//
// This file is part of huniplacer.
//
// huniplacer is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// huniplacer is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with huniplacer.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#pragma once

#include <stdexcept>
#include <string>
#include <sstream>
#include <cerrno>

#include <modbus/modbus.h>

namespace huniplacer
{
	/**
	 * @brief exception to indicate modbus errors
	 *
	 * modbus_ctrl can throw this exception whenever a modbus related error occurs
	 **/
    class modbus_exception : public std::runtime_error
    {
        private:
    		/// @brief modbus error code
            const int error_code;
            /// @brief modbus error string (obtained using modbus_strerror)
            std::string msg;
            
        public:
            modbus_exception(void) :
                std::runtime_error(""),
                error_code(errno)
            {
                std::stringstream ss;
                ss << "modbus error[" << error_code << "]: " << modbus_strerror(error_code);
                msg = ss.str();
            }
            
            virtual ~modbus_exception(void) throw()
            {
            }
            
            virtual const char* what() const throw()
			{
				return msg.c_str();
			}
            
            int get_error_code(void)
            {
                return error_code;
            }
    };
}
