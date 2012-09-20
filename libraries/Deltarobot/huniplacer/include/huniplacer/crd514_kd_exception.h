//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           crd514_kd_exception.h
// Description:    exception thrown if the motorcontroller alarm flag is set
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

#include <huniplacer/CRD514_KD.h>

#include <stdexcept>
#include <string>
#include <sstream>

namespace huniplacer
{
	/**
	 * @brief raised if the alarm flag is set
	 **/
	class crd514_kd_exception : public std::runtime_error
	{
		private:
			const crd514_kd::slaves::t slave;
			const bool warning, alarm;
			std::string message;

		public:
			crd514_kd_exception(const crd514_kd::slaves::t slave, const bool warning, const bool alarm) :
				std::runtime_error(""),
				slave(slave),
				warning(warning),
				alarm(alarm)
			{
				std::stringstream ss;
				ss << "slave: " << (int)slave << ": warning=" << (int)warning << " alarm=" << (int)alarm;
				message = ss.str();
			}

			virtual ~crd514_kd_exception() throw()
			{ }

			virtual const char* what() const throw()
			{
				return message.c_str();
			}

			crd514_kd::slaves::t get_slave(void)
			{
				return slave;
			}

			bool get_warning(void)
			{
				return warning;
			}

			bool get_alarm(void)
			{
				return alarm;
			}
	};
}
