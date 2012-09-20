//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        datatypes
// File:           size3.h
// Description:    3 dimensional size class
// Author:         Wouter Langerak & Lukas Vermond
// Notes:          -
//
// License:        GNU GPL v3
//
// This file is part of datatypes.
//
// datatypes is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// datatypes is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with datatypes.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#pragma once

namespace datatypes
{
	/// @brief 3 dimensional size class
	template<typename T>
    class size3
    {
        public:
            T width , depth , height;

			size3(void) { }
            size3(T width, T depth, T height) : width(width), depth(depth), height(height) { }
            ~size3() { }
            

    };
	
	typedef size3<float> size3f;
	typedef size3<double> size3lf;
	typedef size3<int> size3i;
}
