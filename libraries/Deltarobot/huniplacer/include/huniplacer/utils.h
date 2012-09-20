//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           utils.h
// Description:    miscellaneous utilities
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

#include <boost/thread.hpp>
#include <cstdio>
#include <algorithm>
#include <vector>

namespace huniplacer
{
	/**
	 * @brief various utilities
	 **/
    namespace utils
    {
    	/**
    	 * @brief get the current time in milliseconds
    	 * @return time in milliseconds
    	 **/
        long time_now(void);

        /**
         * @brief sleep for X milliseconds
         * @param ms time in milliseconds
         **/
        void sleep(long ms);

        /**
         * @brief convert radians to degrees
         * @param rad radians
         * @return degrees
         **/
        double deg(double rad);

        /**
         * @brief convert degrees to radians
         * @param deg degrees
         * @return radians
         **/
        double rad(double deg);

        /**
         * @brief utility class to time stuff
         * @note TEMPORARY
         **/
        class stopwatch
        {
			private:
        		const char* name;
				long t0, t1;

			public:
				stopwatch(const char* name, bool s = false) : name(name)
				{
					if(s) { start(); }
				}

				~stopwatch(void) { }
				void start(void) { t0 = time_now(); }
				void stop(void) { t1 = time_now(); }
				void print(FILE* stream)
				{
					fprintf(stream, "%s: %ld ms\n", name, t1 - t0);
				}

				void stop_and_print(FILE* stream)
				{
					stop();
					print(stream);
				}
        };

        template<typename T>
        bool vector_contains(const std::vector<T>& vec, const T& elem)
        {
        	return std::find(vec.begin(), vec.end(), elem) != vec.end();
        }
    }
}
