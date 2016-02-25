/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	templater.h
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This class is responsible for generating and applying a template based on a gripper.
 *       ..MMM#      ,MMr  .MMMMMMMMr
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Edwin Koek
 *   .dMMMMMF           7Y=d9  dMMMMMr
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		@section LICENSE
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	License:	newBSD
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	Copyright © 2013, HU University of Applied Sciences Utrecht.
 *  JMMMMMMMm.    MMMMMM#!.MMMMMMMMM'.		All rights reserved.
 *   WMMMMMMMMNNN,.TMMM@ .MMMMMMMM#`.M
 *    JMMMMMMMMMMMN,?MD  TYYYYYYY= dM
 *
 *	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *	- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
#ifndef TEMPLATER_H
#define TEMPLATER_H

#include <vector>
#include <featurefactory.h>
/**
 * @brief The Templater class
 *
 * This class is responsible for generating and applying a template based on a gripper.
 */
class Templater{
public:
    /**
     * @brief generateGripperTemp
     *
     * This function is responsible for generating the gripper template. It does
     * this based on the number of arms that the gripper has and the dimensions of
     * their tips. It also requires the maximum expansion distance. This minimum
     * distance is calculated based on the number of arms and their size.
     * The template is organised as follows:
     * [0-1] = distance
     * [0-1][0-35] = rotation
     * [0-1][0-35][0...n] = pixel info
     * @param arms The number of arms that the gripper has.
     * @param armHeight The height of the tip of each arm.
     * @param armWidth The width of the tip of each arm.
     * @param maxDist The maximum expansion distance.
     * @return Returns a gripper template.
     */
    static std::vector<std::vector<std::vector<int>>> generateGripperTemp
    (int arms,float armHeight,float armWidth,float maxDist);
    /**
     * @brief getGripPoint
     *
     * This function applies a generated template to an image of an object.
     * It does this by examining each pixel of the object with the template. It will
     * first attempt to fit the gripper at its minimal expansion distance templates. Only if this does
     * not fit it will attempt to fit the maximum expansion distance templates. In case the maximum
     * distance template does fit the pixel wil be deemed as a possible location to pick up the part.
     * @param image The image of an object.
     * @param gTemplate The gripper template.
     * @return Returns a pair of the best coördinate and rotation for picking up the part.
     */
    static pair<Point,int> getGripPoint(Mat &image, vector<vector<vector<int> > >& gTemplate);
};

#endif // TEMPLATER_H
