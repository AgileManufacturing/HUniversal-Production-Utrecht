/**
 * @file CRD514KDException.h
 * @brief Exception thrown if the motorcontroller alarm flag is set.
 * @date Created: 2012-10-01
 *
 * @author 1.0 Lukas Vermond
 * @author 1.0 Kasper van Nieuwland
 * @author 1.1 Koen Braham
 * @author 1.1 Dennis Koole
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


#pragma once

#include <Motor/CRD514KD.h>

#include <stdexcept>
#include <string>
#include <sstream>

namespace Motor
{
	/**
	 * CRD514KDException -> An exception that is raied when the CRD514KD alarm flag is set
	 **/
	class CRD514KDException : public std::runtime_error
	{
		private:
			const CRD514KD::Slaves::t slave;
			const bool warning, alarm;
			std::string message;

		public:
			CRD514KDException(const CRD514KD::Slaves::t slave, const bool warning, const bool alarm) :
				std::runtime_error(""),
				slave(slave),
				warning(warning),
				alarm(alarm)
			{
				std::stringstream ss;
				ss << "slave: " << (int)slave << ": warning=" << (int)warning << " alarm=" << (int)alarm;
				message = ss.str();
			}

			virtual ~CRD514KDException() throw()
			{ }

			virtual const char* what() const throw()
			{
				return message.c_str();
			}

			CRD514KD::Slaves::t getSlave(void)
			{
				return slave;
			}

			/**
			 * Check if there is a warning
			 * @return warning True if there is a warning
			 **/
			bool isWarning(void)
			{
				return warning;
			}

			/**
			 * Check if there is an alarm
			 * @return alarm is true when there is an alarm
			 **/
			bool isAlarm(void)
			{
				return alarm;
			}
	};
}
