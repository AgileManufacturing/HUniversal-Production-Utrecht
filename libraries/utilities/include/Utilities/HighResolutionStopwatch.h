/**
 * @file HighResolutionStopwatch.h
 * @brief High resolution stopwatch
 *
 * @author Koen Braham
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

#include <sys/types.h>
#include <sys/time.h>
#include <iostream>

namespace Utilities {

	/**
	 * Class used for high resolution timings. Measures in nanoseconds.
	 **/
	class HighResolutionStopwatch {
	public:

		HighResolutionStopwatch(clockid_t clk_id);
		virtual ~HighResolutionStopwatch( );
		void start( );
		void stop( );
		timespec getTime( );
		timespec divideTime(int divider);
		static std::ostream & print(std::ostream & os, const timespec & time);
		static void print(const timespec & time);

	private:

		/**
		 * @var clockid_t clk_id
		 * The id of the clock to use.
		 * The following settings are available:
		 *  - CLOCK_REALTIME
		 *		System-wide realtime clock. Setting this clock requires appropriate privileges.
		 *	- CLOCK_MONOTONIC
		 *		Clock that cannot be set and represents monotonic time since some unspecified starting point.
		 *	- CLOCK_PROCESS_CPUTIME_ID
		 *		High-resolution per-process timer from the CPU.
		 *	- CLOCK_THREAD_CPUTIME_ID
		 *		Thread-specific CPU-time clock.
		 **/
		clockid_t clk_id;

		/**
		 * @var timespec timeStart
		 * Starting time of the stopwatch
		 **/
		timespec timeStart;

		/**
		 * @var timespec timeEnd
		 * End time of the stopwatch
		 **/
		timespec timeEnd;
	};

} /* namespace Utilities */
