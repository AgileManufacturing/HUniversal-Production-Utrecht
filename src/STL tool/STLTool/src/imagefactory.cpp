/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	imagefactory.cpp
 *         .MN.      MMMMM;  ?^ ,THM		@brief  This class is responsible for modifying and saving images.
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
#include <iostream>
#include "imagefactory.h"
#include "QDir"
#include "QColor"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>

void ImageFactory::save(QImage& image,string imageName){
    string savePath = "/images/" + imageName + ".jpg";
    if(QDir(QDir::currentPath() + "/images").exists()){
        if(image.save(QDir::currentPath() + QString::fromStdString(savePath))){
            cout << "Image saved succesfully" << endl << QDir::currentPath().toStdString() << savePath << endl;
        }else{
            cout << "Image save failed :(" << endl;
        }
    }else{
        QDir().mkdir(QDir::currentPath() + "/images");
        if(image.save(QDir::currentPath() + QString::fromStdString(savePath))){
            cout << "Image saved succesfully" << endl << QDir::currentPath().toStdString() << savePath << endl;
        }else{
            cout << "Image save failed :(" << endl;
        }
    }
}

void ImageFactory::save(Mat &image, string imageName){
    string savePath = QDir::currentPath().toStdString() + "/images/" + imageName + ".jpg";
    if(QDir(QDir::currentPath() + "/images").exists()){
        imwrite(savePath,image);
    }else{
        QDir().mkdir(QDir::currentPath() + "/images");
        imwrite(savePath,image);
    }
}

Mat ImageFactory::applyOtsuThreshold(const Mat &image){
    Mat thresholdImage;
    threshold(image,thresholdImage,0,255,THRESH_BINARY | THRESH_OTSU);
    return thresholdImage;
}

Mat ImageFactory::normalizeImageScale(const Mat& image, const vector<float> &modelData){
    int minX = INFINITY,maxX = -INFINITY;
    int minY = INFINITY,maxY = -INFINITY;
    int minZ = INFINITY,maxZ = -INFINITY;
    for(int i = 0; i < modelData.size();i+=6){
        minX = (modelData[i] < minX)? modelData[i] : minX;
        maxX = (modelData[i] > maxX)? modelData[i] : maxX;
        minY = (modelData[i+1] < minY)? modelData[i+1] : minY;
        maxY = (modelData[i+1] > maxY)? modelData[i+1] : maxY;
        minZ = (modelData[i+2] < minZ)? modelData[i+2] : minZ;
        maxZ = (modelData[i+2] > maxZ)? modelData[i+2] : maxZ;
    }
    double scale = (maxX-minX)/(double)(image.size().width);
    Mat scaledImage;
    Size s = Size(image.size().width*scale,
                  image.size().height*scale);
    resize(image,scaledImage,s);
    return scaledImage.clone();
}

vector<VisionObject> ImageFactory::filterObjects(vector<vector<Point>>& objects,Mat& image){
    vector<VisionObject> visionObjects;
    int x,y;
    int minx = 1080,maxx = 0;
    int miny = 1920,maxy = 0;
    for(int i = 0; i < objects.size();++i){
        if(objects[i].size() < 100000 && objects[i].size() > 500){
            minx = 1080,maxx = 0;
            miny = 1920,maxy = 0;
            for(int j = 0; j < objects[i].size();j++){
                x = objects[i][j].x;
                y = objects[i][j].y;
                if(x > maxx){
                    maxx = x;
                }
                if(x < minx){
                    minx = x;
                }
                if(y > maxy){
                    maxy = y;
                }
                if( y < miny){
                    miny = y;
                }
            }
//            //create a small border for some extra space around the object in the objectimage
//            maxx+=10;
//            maxy+=10;
//            minx-=10;
//            miny-=10;
            Mat objectImage = Mat::zeros(maxy - miny,maxx-minx,CV_8U);
            for(int j = 0; j < objects[i].size();++j){
                //TODO(Edwin): zoek uit waarom deze check nodig is...
                if(objects[i][j].x - minx < objectImage.size().width && objects[i][j].y - miny < objectImage.size().height){
                    objectImage.at<uchar>(objects[i][j].y - miny,objects[i][j].x - minx) =
                            image.at<uchar>(objects[i][j].y,objects[i][j].x);
                }
            }
            VisionObject filteredObject;
            filteredObject.data = objects[i];
            filteredObject.objectImage = objectImage;
            visionObjects.push_back(filteredObject);
        }
    }
    return visionObjects;
}


QImage ImageFactory::setGrayscale(QImage& image){
    int gray, pixelSize = 4;
    unsigned char* pixelLine;
    for(int i = 0; i < image.height(); ++i){
        pixelLine = image.scanLine(i);
        for(int j = 0; j < image.width(); ++j){
            QRgb* rgbPixel = reinterpret_cast<QRgb*>(pixelLine + (j * pixelSize));
            gray = qGray(*rgbPixel);
            *rgbPixel = QColor(gray,gray,gray).rgba();
        }
    }
    return image;
}

Mat ImageFactory::setGrayscale(Mat &image){
    Mat greyMat;
    cv::cvtColor(image, greyMat, cv::COLOR_BGR2GRAY);
    return greyMat;
}

bool ImageFactory::checkGrayscale(QImage& image){
    return image.allGray();
}

