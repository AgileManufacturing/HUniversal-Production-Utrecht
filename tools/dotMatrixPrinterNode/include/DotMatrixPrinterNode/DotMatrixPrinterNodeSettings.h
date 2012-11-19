/**
 * @file DotMatrixNodeSettings.h
 * @brief Settings for the environment for the DotMatrixNode.
 * @date 2012-11-06
 *
 * @author Koen Braham
 * @author Daan Veltman
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
	 * @var static const unsigned int DRAW_FIELD_HEIGHT
	 * Height of the field (in millimeters) that is drawable by the deltarobot.
	 */
	static const unsigned int DRAW_FIELD_HEIGHT = 155;

	/**
	 * @var static const unsigned int DRAW_FIELD_WIDTH
	 * Width of the field (in millimeters) that is drawable by the deltarobot.
	 */
	static const unsigned int DRAW_FIELD_WIDTH = 90;

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
	 * @var static const double DRAW_FIELD_Z_LOW
	 * The z position where the pen reaches the paper.
	 **/
	static const double DRAW_FIELD_Z_LOW = 300;//TODO: ?
	/**
	 * @var static const double DRAW_FIELD_Z_HIGH
	 * The z position where the pen doesn't reach the paper, and is just safe for movement.
	 **/
	static const double DRAW_FIELD_Z_HIGH = 280;//TODO: ?

	/**
	 * @var static const double ACCELERATION
	 * The acceleration for the delta robot.
	 **/
	static const double ACCELERATION = 100;

}
