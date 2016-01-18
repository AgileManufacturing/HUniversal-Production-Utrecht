#include "vision_node/StlNode.h"
#include <stdio.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include <unistd.h>

#include <dirent.h>
#include <map>

#include "vision_node/ObjectDetector.h"
#include "vision_node/PartMatcher.h"

using namespace cv;
using namespace std;

struct VisionObject{
    Mat objectImage;
    vector<Point> data;
};

struct Part{
    string name;
    map<string,double> parameters;
};



StlNode::StlNode(image_transport::ImageTransport& imageTransport){
    debugImagePublisher = imageTransport.advertise("camera/stl_debug_image", 1);
}


Mat applyOtsuThreshold(const Mat &image){
    Mat thresholdImage;
    threshold(image,thresholdImage,0,255,THRESH_BINARY | THRESH_OTSU);
    return thresholdImage;
}

vector<vector<Point>> findConnectedComponents(const Mat& image){
    vector<vector<Point>> blobs;
    Mat workingImage = applyOtsuThreshold(image);
    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    threshold(workingImage,workingImage,0.0,1.0,THRESH_BINARY);
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

                    blob.push_back(Point(j,i));
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

vector<vector<Point> > getContours(const Mat &image){
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(image,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    return contours;
}

pair<vector<vector<Point>>, vector<Vec4i>> getContoursHierarchy(const Mat &image){
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(image,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    return make_pair(contours,hierarchy);
}

vector<vector<Point>> getHoles(const pair<vector<vector<Point> >, vector<Vec4i> >& contours){
    vector<vector<Point>> holes;
    for(unsigned int i = 0; i < contours.first.size(); ++i){
        // The size check is to filter out the smallest insignificant holes
        if(contours.second[i][3] != -1 && contours.first[i].size() > 10*M_PI){
            holes.push_back(contours.first[i]);
        }
    }
    return holes;
}

vector<vector<Point> > getHoles(const Mat &image){
    return getHoles(getContoursHierarchy(image));
}

vector<VisionObject> filterObjects(vector<vector<Point>>& objects,Mat& image,Mat& debugImage){
    vector<VisionObject> visionObjects;
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
            //create a small border for some extra space around the object in the objectimage
            maxx+=10;
            maxy+=10;
            minx-=10;
            miny-=10;

            line(debugImage,Point(minx,maxy),Point(maxx,maxy),Scalar(0,200,0),5);
            line(debugImage,Point(minx,miny),Point(maxx,miny),Scalar(0,200,0),5);
            line(debugImage,Point(minx,miny),Point(minx,maxy),Scalar(0,200,0),5);
            line(debugImage,Point(maxx,miny),Point(maxx,maxy),Scalar(0,200,0),5);
            Mat objectImage = Mat::zeros(maxy - miny,maxx-minx,CV_8U);
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

vector<string> getPartList(){
    char currentPath[FILENAME_MAX];
    string directory;
    if (getcwd(currentPath, sizeof(currentPath))){
        directory = currentPath;
        directory += "/parts";
    }
    DIR* dir;
    struct dirent *ent;
    vector<string> fileList;
    if((dir = opendir(directory.c_str())) != NULL){
        while((ent = readdir(dir)) != NULL){
            string currentFile = ent->d_name;
            if(currentFile.substr(currentFile.find("."),100) == ".config"){
                fileList.push_back(currentFile);
            }
        }
        closedir(dir);
    }else{
        REXOS_INFO("COULD NOT OPEN DIRECTORY");
    }
    return fileList;
}

Part parsePart(string partName){
    char currentPath[FILENAME_MAX];
    Part part;
    string directory;
    if (getcwd(currentPath, sizeof(currentPath))){
        directory = currentPath;
        directory += "/parts/" + partName;
    }
    ifstream file;
    file.open(directory);
    char buffer[256];
    string currentLine;
    while(!file.eof()){
        file.getline(buffer,256);
        currentLine = buffer;
        if(currentLine.substr(0,currentLine.find(":=")) == "Partname"){
            part.name = currentLine.substr(currentLine.find(":=")+2);
        }else if(currentLine == "/parameters"){
            while(!file.eof()){
                file.getline(buffer,256);
                currentLine = buffer;
                if(currentLine[0] != '/' && !file.eof()){
                    part.parameters.insert(pair<string,double>(
                                               currentLine.substr(0,currentLine.find(":=")),
                                               atof(currentLine.substr(currentLine.find(":=")+2).c_str())));
                }else{
                    break;
                }
            }
        }
    }
    return part;
}

vector<Part> parseAllParts(){
    vector<Part> parts;
    vector<string> partList = getPartList();
    parts.resize(partList.size());
    for(int i = 0; i < partList.size(); ++i){
        parts[i] = parsePart(partList[i]);
    }
    return parts;
}

pair<Part, double> matchPart(map<string,double>& partFeatures,string partName){
    Part matchPart = parsePart(partName);
    map<string,double>::iterator it;
    for(it = partFeatures.begin(); it != partFeatures.end();++it){
        if(matchPart.parameters.find(it->first) == matchPart.parameters.end()){
            return make_pair(matchPart,0);
        }
    }
    double matchSum = 0;
    for(it = partFeatures.begin(); it != partFeatures.end();++it){
        //Matching is based of of the percentage that the current value
        //deviates from the "known value"
        if(it->first == "SurfacePercentage"){
            matchSum += 100 - abs(1 - (it->second / matchPart.parameters.find(it->first)->second)) * 100;
        }
        if(it->first == "Height" || it->first == "Width"){
            matchSum += 100 - abs(1 -(pow(it->second,2) / pow(matchPart.parameters.find(it->first)->second,2))) * 100;
        }
        if(it->first == "NumberOfHoles"){
            int numberOfHoles = matchPart.parameters.find(it->first)->second;
            if(numberOfHoles == it->second){
                matchSum += 100;
            }else if(numberOfHoles == 0){
                matchSum += 0;
            }else{
                matchSum += 100 - abs(1 -(it->second / numberOfHoles)) * 100;
            }
        }
    }
    return make_pair(matchPart,matchSum / partFeatures.size());
}

double matchPart(map<string,double>& partFeatures,map<string,double>& matchFeatures){
    map<string,double>::iterator it;
    for(it = partFeatures.begin(); it != partFeatures.end();it++){
        if(matchFeatures.find(it->first) == matchFeatures.end()){
            return 0;
        }
    }
    double matchSum = 0;
    int averageWeight = 0;
    for(it = partFeatures.begin(); it != partFeatures.end();it++){
        //Matching is based of of the percentage that the current value
        //deviates from the "known value"
        if(it->first == "SurfacePercentage"){
            matchSum += 100 - abs(1 - (it->second / matchFeatures.find(it->first)->second)) * 100;
            averageWeight +=1;
        }
        if(it->first == "Height" || it->first == "Width"){
            matchSum += 100 - abs(1 -(pow(it->second,2) / pow(matchFeatures.find(it->first)->second,2))) * 100;
            averageWeight +=1;
        }
        if(it->first == "NumberOfHoles"){
            int holesWeight = 3;
            int numberOfHoles = matchFeatures.find(it->first)->second;
            REXOS_WARN_STREAM("Number of holes: " + to_string(it->second) +  " Found: " + to_string(numberOfHoles));
            if(numberOfHoles == it->second){
                matchSum += 100 * holesWeight;
                averageWeight +=holesWeight;
            }else if(numberOfHoles == 0){
                matchSum += 0 * holesWeight;
                averageWeight +=holesWeight;
            }else{
                matchSum += (100 - abs(1 -(it->second / numberOfHoles)) * 100) * holesWeight;
                averageWeight +=holesWeight;
            }
        }
    }
    return matchSum /averageWeight;
}

pair<Part, double> matchPart(map<string, double>& partFeatures, Part referencePart){
    map<string,double>::iterator it;
    for(it = partFeatures.begin(); it != partFeatures.end();it++){
        if(referencePart.parameters.find(it->first) == referencePart.parameters.end()){
            return make_pair(referencePart,0);
        }
    }
    double matchSum = 0;
    int averageWeight = 0;
    for(it = partFeatures.begin(); it != partFeatures.end();it++){
        //Matching is based of of the percentage that the current value
        //deviates from the "known value"
        if(it->first == "SurfacePercentage"){
            matchSum += 100 - abs(1 - (it->second / referencePart.parameters.find(it->first)->second)) * 100;
            averageWeight +=1;
        }
        if(it->first == "Height" || it->first == "Width"){
            matchSum += 100 -
                    abs(1 -(pow(it->second,2) / pow(referencePart.parameters.find(it->first)->second,2))) * 100;
            averageWeight +=1;
        }
        //Number of holes has a stronger weight
        if(it->first == "NumberOfHoles"){
            int holesWeight = 3;
            int numberOfHoles = referencePart.parameters.find(it->first)->second;
            if(numberOfHoles == it->second){
                matchSum += 100 * holesWeight;
                averageWeight +=holesWeight;
            }else if(numberOfHoles == 0){
                matchSum += 0 * holesWeight;
                averageWeight +=holesWeight;
            }else{
                matchSum += (100 - abs(1 -(it->second / numberOfHoles)) * 100) * holesWeight;
                averageWeight +=holesWeight;
            }
        }
    }
    return make_pair(referencePart,matchSum / averageWeight);
}

pair<Part,double> matchPart(map<string, double> partFeatures){
    vector<Part> parts = parseAllParts();
    vector<pair<Part,double> > matchPercentages;
    matchPercentages.resize(parts.size());
    for(int i = 0; i < parts.size(); ++i){
        matchPercentages[i] = make_pair(parts[i],matchPart(partFeatures,parts[i].parameters));
        //        cout <<  parts[i].name << " - " << matchPercentages[i] << endl;
        REXOS_WARN_STREAM(matchPercentages[i].first.name + " - " + to_string(matchPercentages[i].second));
    }
    pair<Part,double> bestMatch;
    bestMatch.second = 0;

    for(int i = 0; i < matchPercentages.size(); ++i){
        if(matchPercentages[i].second > bestMatch.second){
            bestMatch = matchPercentages[i];
        }
    }

    return bestMatch;
}

map<string, double> createParameterMap(const VisionObject& object){
    map<string,double> parameters;
    // - 20 to remove the 10 pixel wide border around the edges
    double height = object.objectImage.size().height - 20;
    double width = object.objectImage.size().width - 20;
    double area = height * width;
    double surfacePercentage = (object.data.size()/area) * 100;
    vector<vector<Point>> holes = getHoles(getContoursHierarchy(object.objectImage));
    double numberOfHoles = holes.size();

    //Add parameters to map

    parameters.insert(make_pair("Height",height));
    parameters.insert(make_pair("Width",width));
    //parameters.insert(make_pair("Area",area));
    //surfacepercentage
    parameters.insert(make_pair("SurfacePercentage",surfacePercentage));
    //number of holes
    parameters.insert(make_pair("NumberOfHoles",numberOfHoles));
    return parameters;
}

void StlNode::handleFrame(cv::Mat& frame){
    //    vector<string> partList = getPartList();
    //    for(unsigned i = 0; i < partList.size();++i){
    //        REXOS_WARN_STREAM("STL VISION: " + partList[i]);
    //    }
    //    Part p = parsePart(partList[0]);
    //    REXOS_WARN_STREAM(p.name);
    //    map<string,double>::iterator it;
    //    for(it = p.parameters.begin(); it != p.parameters.end();++it){
    //        REXOS_WARN_STREAM(it->first + to_string(it->second));
    //    }
    cv_bridge::CvImage cvi;
    Mat grayFrame;
    cvtColor(frame,grayFrame,CV_RGB2GRAY);
    vector<vector<Point>> blobs = ObjectDetector::findConnectedComponents(grayFrame);
    vector<VisionObject> objects = ObjectDetector::filterObjects(blobs,grayFrame,frame);
    for(int p = 0; p < objects.size();++p){
        pair<Part,double> match = PartMatcher::matchPart(PartMatcher::createParameterMap(objects[p]));
        if(match.second > 80){
            string matchResult = match.first.name + " - " + to_string(match.second);
            REXOS_WARN_STREAM(matchResult);
        }
    }
    if(!frame.empty()){
        ros::Time time = ros::Time::now();
        cvi.header.stamp = time;
        cvi.header.frame_id = "stl_debug_image";
        cvi.encoding = sensor_msgs::image_encodings::BGR8;
        cvi.image = frame;
        debugImagePublisher.publish(cvi.toImageMsg());
    }else{
        REXOS_INFO("STL VISION: Given clone frame was empty.");
    }
    //    parseAllParts();


}
