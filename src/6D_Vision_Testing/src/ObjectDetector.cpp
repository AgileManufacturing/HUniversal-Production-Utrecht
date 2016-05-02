#define _USE_MATH_DEFINES
#include <math.h>
#include "ObjectDetector.h"
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat ObjectDetector::applyOtsuThreshold(const cv::Mat& image){
	cv::Mat thresholdImage;
    threshold(image,thresholdImage,0,255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    return thresholdImage;
}

std::vector<std::vector<cv::Point>> ObjectDetector::findConnectedComponents(const cv::Mat& image){
	std::vector<std::vector<cv::Point>> blobs;
	cv::Mat workingImage = applyOtsuThreshold(image);
    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    threshold(workingImage,workingImage,0.0,1.0, cv::THRESH_BINARY);
    workingImage.convertTo(workingImage, CV_32SC1);

    //TODO: Do i really need this?

    int labelCount = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y < workingImage.rows; y++) {
        int *row = (int*)workingImage.ptr(y);
        for(int x=0; x < workingImage.cols; x++) {
            if(row[x] != 1) {
                continue;
            }
            cv::Rect rect;
            cv::floodFill(workingImage, cv::Point(x,y), labelCount, &rect, 0, 0, 4);
            std::vector <cv::Point> blob;
            for( int i=rect.y; i < (rect.y+rect.height); i++) {
                int *row2 = (int*)workingImage.ptr(i);
                for( int j=rect.x; j < (rect.x+rect.width); j++) {
                    if(row2[j] != labelCount) {
                        continue;
                    }

                    blob.push_back(cv::Point(j,i));
                }
            }
            if(blob.size() > 200){
                blobs.push_back(blob);
            }
            labelCount++;
        }
    }
    return blobs;
}

std::vector<std::vector<cv::Point>> ObjectDetector::getContours(const cv::Mat &image){
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
    findContours(image,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    return contours;
}

std::pair<std::vector<std::vector<cv::Point>>, std::vector<cv::Vec4i>> ObjectDetector::getContoursHierarchy(const cv::Mat &image){
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
    findContours(image,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    return make_pair(contours,hierarchy);
}

std::vector<std::vector<cv::Point>> ObjectDetector::getHoles(const std::pair<std::vector<std::vector<cv::Point>>, std::vector<cv::Vec4i> >& contours){
	std::vector<std::vector<cv::Point>> holes;
    for(unsigned int i = 0; i < contours.first.size(); ++i){
        // The size check is to filter out the smallest insignificant holes
        if(contours.second[i][3] != -1 && contours.first[i].size() > 10 * M_PI){
            holes.push_back(contours.first[i]);
        }
    }
    return holes;
}

std::vector<std::vector<cv::Point>> ObjectDetector::getHoles(const cv::Mat &image){
    return getHoles(getContoursHierarchy(image));
}

std::vector<VisionObject> ObjectDetector::filterObjects(std::vector<std::vector<cv::Point>>& objects, cv::Mat& image){
	std::vector<VisionObject> visionObjects;
    int x,y;
    int minx = 1080,maxx = 0;
    int miny = 1920,maxy = 0;
    for(unsigned int i = 0; i < objects.size();++i){
        if(objects[i].size() < 100000 && objects[i].size() > 500){
            minx = 1080,maxx = 0;
            miny = 1920,maxy = 0;
            for(unsigned int j = 0; j < objects[i].size();j++){
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

	        cv::Mat objectImage = cv::Mat::zeros(maxy - miny,maxx-minx,CV_8U);
            for(unsigned int j = 0; j < objects[i].size();++j){
                //TODO: zoek uit waarom deze check nodig is...
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
