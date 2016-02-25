/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	templater.cpp
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This class is responsible for generating and applying a template based on a gripper.
 *       ..MMM#      ,MMr  .MMMMMMMMr
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Edwin Koek
 *   .dMMMMMF           7Y=d9  dMMMMMr
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		@section LICENSE
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	License:	newBSD
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	Copyright © 2016, HU University of Applied Sciences Utrecht.
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
#include "templater.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

vector<vector<vector<int>>> Templater::generateGripperTemp(int arms,float armHeight,float armWidth,float maxDist){
    vector<vector<vector<int>>> templates;
    //currently only supports min distance and max distance as the gripper
    //can only fully grip or fully expand
    templates.resize(2);

    float angleBetweenArms =(360/arms)/(180/M_PI);
    //Possible orientations before the arm goes back to a similar one
    int orientations = 36 / arms;
    //Calculate minimal distance
    int minDist = ceil(cos((((arms-2)*180)/arms)/2) * armHeight);

    int tempHeight = (maxDist + armHeight) * 2;
    int tempWidth = (maxDist + armWidth) * 2;
    vector<int> firstArm(tempWidth * tempHeight);
    vector<int> gripperTemplate(firstArm.size());
    if(minDist == 0){
        minDist = armWidth;
    }

    int centerX = tempWidth / 2;
    int centerY = tempHeight / 2;
    double currentAngle = 0;

    int checkX, checkY;
    int currentDist;
    for(int distance = 0; distance < 2; ++distance){
        fill(firstArm.begin(),firstArm.end(),0);

        currentDist = (distance == 0)? minDist : maxDist;

        for(int i = 0; i < orientations;++i){
            currentAngle = (M_PI/18)*i;
            //clear vector
            fill(gripperTemplate.begin(),gripperTemplate.end(),0);
            //Add first arm first rotation to template
            if(i == 0){
                for(int y = centerY + (armHeight/2); y >= centerY - (armHeight/2);--y){
                    for(int x = centerX + currentDist; x < centerX + currentDist + armWidth;++x){
                        firstArm[(y * tempWidth) + x] = 1;
                    }
                    cout << endl;
                }
            }
            for(int y = 0; y < tempHeight;++y){
                for(int x = 0; x < tempWidth;++x){
                    for(int a = 0; a < arms;++a){
                        checkX = centerX + ((x - centerX) * cos((a*angleBetweenArms) + currentAngle))
                                - ((y - centerY) * sin((a*angleBetweenArms) + currentAngle));
                        checkY = centerY + ((x - centerX) * sin((a*angleBetweenArms) + currentAngle))
                                + ((y - centerY) * cos((a*angleBetweenArms) + currentAngle));

                        if(checkX > 0 && checkX < tempWidth && checkY > 0
                                && checkY < tempHeight){
                            if(firstArm[(checkY * tempWidth) + checkX] == 1){
                                gripperTemplate[(y * tempWidth) + x] = 1;
                            }
                        }
                    }
                }
            }
            templates[distance].push_back(gripperTemplate);
        }
    }
    return templates;
}

pair<Point, int> Templater::getGripPoint(cv::Mat& image, vector<vector<vector<int>>>& gTemplate){
    Point center = FeatureFactory::findCenter(FeatureFactory::findConnectedComponents(image)[0]);
    threshold(image,image,100,255,CV_THRESH_BINARY);

    pair<Point,int> bestPoint = make_pair(Point(0,0),0);
    int offset = -sqrt(gTemplate[0][0].size())/2;

    int sumPos =0, sumNeg= 0;
    //IMAGE
    for(int y = 0; y < image.size().height;++y){
        for(int x = 0; x < image.size().width;++x){
            if(static_cast<int>(image.data[(y * image.size().width)+x]) != 0){
                //KERNEL
                for(int a = 0; a < gTemplate[0].size();++a){
                    sumNeg = 0;
                    sumPos = 0;
                    for(int ky = 0; ky < sqrt(gTemplate[0][a].size());++ky){
                        for(int kx = 0;kx < sqrt(gTemplate[0][a].size());++kx){
                            if(gTemplate[0][a][(ky*sqrt(gTemplate[0][a].size()))+kx] == 1){
                                if(y +(ky + offset) >= 0 && y +(ky + offset) < image.size().height &&
                                        x +(kx + offset) >= 0 && x +(kx + offset) < image.size().width){
                                    if(static_cast<int>(image.data[((y +(ky + offset)) * image.size().width) + (x + (kx + offset))]) !=0){
                                        ++sumNeg;
                                    }else{
                                        ++sumPos;
                                    }
                                }else{
                                    ++sumPos;
                                }
                            }
                        }
                    }
                    if(sumNeg/(double)(sumNeg+sumPos) >= 1){
                        sumNeg = 0;
                        sumPos = 0;
                        for(int ky = 0; ky < sqrt(gTemplate[1][a].size());++ky){
                            for(int kx = 0;kx < sqrt(gTemplate[1][a].size());++kx){
                                if(gTemplate[1][a][(ky*sqrt(gTemplate[1][a].size()))+kx] == 1){
                                    if(y +(ky + offset) >= 0 && y +(ky + offset) < image.size().height &&
                                            x +(kx + offset) >= 0 && x +(kx + offset) < image.size().width){
                                        if(static_cast<int>(image.data[((y +(ky + offset)) * image.size().width) + (x + (kx + offset))]) != 0){
                                            ++sumNeg;
                                        }else{
                                            ++sumPos;
                                        }
                                    }else{
                                        ++sumPos;
                                    }
                                }
                            }
                        }
                        if(sumPos/(double)(sumNeg+sumPos) == 1){
                            if(FeatureFactory::getEuclideanDistance(Point(x,y),center) < FeatureFactory::getEuclideanDistance(bestPoint.first,center)){
                                bestPoint.first = Point(x,y);
                                bestPoint.second = a * 10;
                            }

                        }
                    }
                }
            }
        }
    }
    return make_pair(bestPoint.first-center,bestPoint.second);
}


