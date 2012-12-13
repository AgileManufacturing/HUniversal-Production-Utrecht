/**
 * @file HighResolutionStopwatch.cpp
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

#include <Utilities/HighResolutionStopwatch.h>

namespace Utilities {
	/**
	 * Constructor of the high resolution stopwatch
	 *
	 * @param clk_id The id of the clock to use.
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
	HighResolutionStopwatch::HighResolutionStopwatch(clockid_t clk_id) :
			clk_id(clk_id) {
	}

	/**
	 * Destructor of the high resolution stopwatch
	 **/
	HighResolutionStopwatch::~HighResolutionStopwatch( ) {
	}

	/**
	 * Starts the stopwatch
	 **/
	void HighResolutionStopwatch::start( ) {
		clock_gettime(clk_id, &timeStart);
	}

	/**
	 * Stops the stopwatch
	 **/
	void HighResolutionStopwatch::stop( ) {
		clock_gettime(clk_id, &timeEnd);
	}

	/**
	 * Get elapsed time.
	 * Calculated by getting the difference between start and end timestamps.
	 **/
	timespec HighResolutionStopwatch::getTime( ) {
		timespec temp;
		if ((timeEnd.tv_nsec - timeStart.tv_nsec) < 0) {
			temp.tv_sec = timeEnd.tv_sec - timeStart.tv_sec - 1;
			temp.tv_nsec = 1000000000 + timeEnd.tv_nsec - timeStart.tv_nsec;
		} else {
			temp.tv_sec = timeEnd.tv_sec - timeStart.tv_sec;
			temp.tv_nsec = timeEnd.tv_nsec - timeStart.tv_nsec;
		}
		return temp;
	}

	/**
	 * Divides the time by a given divider.
	 * Useful for computing averages.
	 *
	 * For example:
	 * stopwatch.start
	 * for(int i =0; i < 1000; ++i){
	 * 	// do something intensive
	 * }
	 * stopwatch.stop
	 * time = stopwatch.divide(1000)
	 * stopwatch.print(time)
	 *
	 * @param divider The divider used on the time
	 **/
	timespec HighResolutionStopwatch::divideTime(int divider) {
		timespec temp = getTime();

		long long nanoseconds = temp.tv_sec * 1000000000 + temp.tv_nsec;
		nanoseconds /= divider;

		temp.tv_sec = nanoseconds / 1000000000;
		temp.tv_nsec = nanoseconds % 1000000000;
		return temp;
	}

	/**
	 * Prints the given time to a std::ostream
	 **/
	std::ostream & HighResolutionStopwatch::print(std::ostream & os, const timespec & time) {
		os << time.tv_sec << "." << time.tv_nsec << std::endl;
		return os;
	}

	/**
	 * Prints the given time to std::cout
	 **/
	void HighResolutionStopwatch::print(const timespec & time) {
		print(std::cout, time);
	}

} /* namespace Utilities */
