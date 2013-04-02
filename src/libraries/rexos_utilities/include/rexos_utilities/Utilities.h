/**
 * @file Utilities.h
 * @brief Miscellaneous utilities.
 *
 * @author Lukas Vermond
 * @author Kasper van Nieuwland
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

#include <boost/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <cstdio>
#include <algorithm>
#include <vector>
#include <sstream>

namespace rexos_utilities{
    long timeNow(void);
    void sleep(long milliseconds);
    double radiansToDegrees(double radians);
    double degreesToRadians(double degrees);
    int stringToInt(int &i, char const *s, int base = 0);
    double stringToDouble(const std::string& s);

    /**
     * Utility class to time stuff.
     **/
    class StopWatch{
	private:
		/**
		 * @var const char* name
		 * Stopwatch name.
		 **/
		const char* name;

		/**
		 * @var long timeOfStart
		 * Starting time of the stopwatch.
		 **/
		long timeOfStart;

		/**
		 * @var long timeOfStop
		 * Time at which the stopwatch is stopped.
		 **/
		long timeOfStop;

	public:
		/**
		 * Constructor of a stopwatch, keeps start and stop time to measure the elapsed time in between.
		 *
		 * @param name Name of the stopwatch, useful for clarity when printing the stopwatch.
		 * @param autoStart Optional flag (defaults to false) to start the stopwatch when it is created.
		 **/
		StopWatch(const char* name, bool autoStart = false) : name(name), timeOfStart(0), timeOfStop(0){
			if(autoStart){ start(); }
		}

		/**
		 * Start method of the stopwatch, sets the starting time to the current time.
		 **/
		void start(void){ timeOfStart = timeNow(); }

		/**
		 * Stop method of the stopwatch, sets the stop time to the current time.
		 **/
		void stop(void){ timeOfStop = timeNow(); }
		
		/**
		 * Prints name of the stopwatch and the time elapsed between start and stop time to the stream.
	 	 *
		 * @param stream Pointer to the stream (file or cout) to print to.
		 **/
        void print(FILE* stream){
			fprintf(stream, "%s: %ld ms\n", name, timeOfStop - timeOfStart);
		}

		/**
		 * Stops the stopwatch and prints the name of the stopwatch and the time elapsed between start and stop (now) time to the stream.
		 *
		 * @param stream Pointer to the stream (file or cout) to print to.
		 **/
		void stopAndPrint(FILE* stream){
			stop();
			print(stream);
		}
    };

    template<typename T>
    /**
     * Checks if the vector contains the element.
     *
     * @param vector The vector of elements.
     * @param element The element to check for in the vector.
     *
     * @return If the element is found return true, else return false.
     **/
    bool vectorContains(const std::vector<T>& vector, const T& element){
    	return std::find(vector.begin(), vector.end(), element) != vector.end();
    }
}
