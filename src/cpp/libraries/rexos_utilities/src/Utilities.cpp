/**
 * @file Utilities.cpp
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

#include <rexos_utilities/Utilities.h>

namespace rexos_utilities{
    /**
     * Get the current time in milliseconds.
     *
     * @return time in milliseconds.
     **/
    long timeNow(void){
        boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration duration(time.time_of_day());
        return duration.total_milliseconds();
    }
    
    /**
     * Sleep the current thread for a specific time. Then resume the thread.
     *
     * @param milliseconds time in milliseconds.
     **/
    void sleep(long milliseconds){
        boost::this_thread::sleep(boost::posix_time::milliseconds(milliseconds));
    }

    /**
     * Converts radians to degrees.
     *
     * @param rad Amount of radians.
     *
     * @return An Amount of degrees.
     **/
    double radiansToDegrees(double radians){
		return (radians / M_PI) * 180;
	}

    /**
     * Converts degrees to radians.
     *
     * @param deg Amount of degrees.
     *
     * @return An amount of radians.
     **/
	double degreesToRadians(double degrees){
		return (degrees / 180) * M_PI;
	}

    /**
     * Converts a string to integer
     * @param i Reference to integer to be assigned
     * @param s String that contains the value
     * @param base A value between 2 and 36 inclusive, which determines the base of the value in the string. Special value is 0, which takes the value as base 10 unless a prefix of 0x (hexadecimal) or 0 (octal).
     *
     * @return error code
     *  - 0 is normal
     *  - 1 is overflow
     *  - 2 is underflow
     *  - 3 is inconvertible
     **/
    int stringToInt(int &i, char const *s, int base) {
        char *end;
        long  l;
        errno = 0;
        l = strtol(s, &end, base);
        if ((errno == ERANGE && l == LONG_MAX) || l > INT_MAX) {
            return 1;
        }
        if ((errno == ERANGE && l == LONG_MIN) || l < INT_MIN) {
            return 2;
        }
        if (*s == '\0' || *end != '\0') {
            return 3;
        }
        i = l;

        return 0;
    }

    /**
     * Converts a string to a double.
     * @param s the string to be converted
     * 
     * @return The converted double. Returns a 0 if the given string is not convertable.
     **/
    double stringToDouble(const std::string& s){
        std::istringstream i(s);
        double x;
        if (!(i >> x))
            return 0;
        return x;
    }
}
