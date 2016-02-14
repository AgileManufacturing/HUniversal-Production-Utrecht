
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

