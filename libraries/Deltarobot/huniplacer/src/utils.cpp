//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        utils.cpp
// File:           miscellaneous utilities
// Description:    Lukas Vermond & Kasper van Nieuwland
// Author:         -
// Notes:          
//
// License:        GNU GPL v3
//
// This file is part of utils.cpp.
//
// utils.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// utils.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with utils.cpp.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#include <huniplacer/utils.h>

namespace huniplacer
{
    namespace utils
    {
        long time_now(void)
        {
            boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration duration(time.time_of_day());
            return duration.total_milliseconds();
        }
        
        void sleep(long ms)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
        }

        double deg(double rad)
		{
			return (rad / M_PI) * 180;
		}

		double rad(double deg)
		{
			return (deg / 180) * M_PI;
		}
    }
}
