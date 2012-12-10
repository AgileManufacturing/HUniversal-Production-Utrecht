/**
 * @file DotMatrixPrinterNodeSettings.h
 * @brief Settings for the environment for the DotMatrixPrinterNode.
 * @date Created: 2012-11-06
 *
 * @author Koen Braham
 * @author Daan Veltman
 * @author Arjen van Zanten
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

 namespace DotMatrixPrinterNodeSettings {
	/**
	 * @var static const double DRAW_FIELD_DOTS_PER_MM
	 * Number of dots per millimeter on the drawing field.
	 */
	static const double DRAW_FIELD_DOTS_PER_MM = 1;
	/**
	 * @var static const double DRAW_FIELD_MM_PER_DOT
	 * Number of millimeters per dot on the drawing field.
	 **/
	static const double DRAW_FIELD_MM_PER_DOT = 1 / DRAW_FIELD_DOTS_PER_MM;

	/**
	 * @var static const double DRAW_FIELD_WIDTH
	 * Width of the field in millimeters that is drawable by the deltarobot.
	 */
	static const double DRAW_FIELD_WIDTH = 90;

	/**
	 * @var static const double DRAW_FIELD_HEIGHT
	 * Height of the field in millimeters that is drawable by the deltarobot.
	 */
	static const double DRAW_FIELD_HEIGHT = 155;

	/**
	 * @var static const double DRAW_FIELD_WIDTH_DOTS
	 * Width of the field in dots
	 **/
	static const double DRAW_FIELD_WIDTH_DOTS = DRAW_FIELD_WIDTH * DRAW_FIELD_DOTS_PER_MM;

	/**
	 * @var static const double DRAW_FIELD_HEIGHT_DOTS
	 * Height of the field in dots
	 **/
	static const double DRAW_FIELD_HEIGHT_DOTS = DRAW_FIELD_HEIGHT * DRAW_FIELD_DOTS_PER_MM;

	/**
	 * @var static const double MOVEMENT_THRESHOLD
	 * The threshold for the distance between two consecutive drawing points, below which the effector will be raised a small amount and above which the effector will be moved a large amount.
	 **/
	static const double MOVEMENT_THRESHOLD = 20;

	/**
	 * @var static const double ELEVATION_SMALL
	 * The distance (in millimeters) that the effector will be raised above the drawing surface when the distance between the drawn dots is below the movement threshold.  
	 **/
	static const double ELEVATION_SMALL = 5;

	/**
	 * @var static const double ELEVATION_BIG
	 * The distance (in millimeters) the effector will be raised above the drawing surface when the distance between the drawn dots is above the movement threshold.  
	 **/
	static const double ELEVATION_BIG = 20;

	/**
	 * @var static const double ACCELERATION
	 * The acceleration for the delta robot.
	 **/
	static const double ACCELERATION = 25;

	/**
	 * @var static const double CALIBRATION_ACCELERATION
	 * The acceleration for the delta robot during the calibration.
	 **/
	static const double CALIBRATION_ACCELERATION = 25;
	

}
