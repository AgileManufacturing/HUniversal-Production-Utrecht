#include "featurefactory.h"
#include "imagefactory.h"
#include "templater.h"
#include "QDir"
#include "QFile"
#include "QTextStream"
#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>

void FeatureFactory::savePartConfig(const map<string, double>& parameterMap,const QString partName){
    QString path = QDir::currentPath() + "/parts/" + partName + ".config";
    QFile configFile;
    configFile.setFileName(path);
    configFile.open(QIODevice::WriteOnly);
    QTextStream outputStream(&configFile);
    outputStream << "Product agent configuration file" << "\n"
                 << "Version:=2.0" << "\n"
                 << "Partname:=" << partName << "\n"                  
                 << "/parameters" << "\n";
    for(auto i : parameterMap){
        QString parameterName = QString(i.first.data());
        if(parameterName[0] == 'p'){
            outputStream << QString(i.first.data()) << ":=" << i.second << "\n";
        }
    }
    outputStream << "parameters/" << "\n";

    outputStream << "/gripper" << "\n";
    for(auto i : parameterMap){
        QString parameterName = QString(i.first.data());
        if(parameterName[0] == 'g'){
            outputStream << QString(i.first.data()) << ":=" << i.second << "\n";
        }
    }
    outputStream << "gripper/" << "\n";

    configFile.close();
}

void FeatureFactory::savePartConfig(const Part& part){
    QString path = QDir::currentPath() + "/parts/" + QString(part.name.data()) + ".config";
    QFile configFile;
    configFile.setFileName(path);
    configFile.open(QIODevice::WriteOnly);
    QTextStream outputStream(&configFile);
    outputStream << "Product agent configuration file" << "\n"
                 << "Version:=2.0" << "\n"
                 << "Partname:=" << QString(part.name.data()) << "\n"
                 << "/parameters" << "\n";
    for(auto i : part.parameters){
        QString parameterName = QString(i.first.data());
        if(parameterName[0] == 'p'){
            outputStream << QString(i.first.data()) << ":=" << i.second << "\n";
        }
    }
    outputStream << "/gripper" << "\n";
    for(auto i : part.parameters){
        QString parameterName = QString(i.first.data());
        if(parameterName[0] == 'g'){
            outputStream << QString(i.first.data()) << ":=" << i.second << "\n";
        }
    }
    outputStream << "gripper/" << "\n";
    configFile.close();
}

map<string, double> FeatureFactory::createParameterMap(const VisionObject& object){
    map<string,double> parameters;
    // calculating width and height of the object, the broadest side is always set
    // as width
    RotatedRect rect = minAreaRect(object.data);
    Point2f vertices[4];
    rect.points(vertices);
    double maxLength = -INFINITY, minLength = INFINITY;
    for(int i = 0; i < 4;++i){
        double dX,dY;
        dX = vertices[i].x - vertices[(i+1)%4].x;
        dY = vertices[i].y - vertices[(i+1)%4].y;
        double length = sqrt((dX * dX) + (dY * dY));
        (length <= minLength)? minLength = length : minLength = minLength;
        (length >= maxLength)? maxLength = length : maxLength = maxLength;
    }
    double height = minLength;
    double width = maxLength;
    double area = height * width;

    double surfacePercentage = (object.data.size()/area) * 100;
    vector<vector<Point>> holes = getHoles(getContoursHierarchy(object.objectImage));
    double numberOfHoles = holes.size();
    //Add parameters to map
    parameters.insert(make_pair("pAspect",width/height));
    parameters.insert(make_pair("pHeight",height));
    parameters.insert(make_pair("pWidth",width));
    //parameters.insert(make_pair("pArea",area));
    parameters.insert(make_pair("pSurfacePercentage",surfacePercentage));
    parameters.insert(make_pair("pNumberOfHoles",numberOfHoles));
    return parameters;
}

map<string, double> FeatureFactory::createParameterMap(const VisionObject& object,vector<float>& modelData,vector<vector<vector<int>>>& gTemplate){
    map<string,double> parameters;
    // calculating width and height of the object, the broadest side is always set
    // as width
    RotatedRect rect = minAreaRect(object.data);
    Point2f vertices[4];
    rect.points(vertices);
    double maxLength = -INFINITY, minLength = INFINITY;
    for(int i = 0; i < 4;++i){
        double dX,dY;
        dX = vertices[i].x - vertices[(i+1)%4].x;
        dY = vertices[i].y - vertices[(i+1)%4].y;
        double length = sqrt((dX * dX) + (dY * dY));
        (length <= minLength)? minLength = length : minLength = minLength;
        (length >= maxLength)? maxLength = length : maxLength = maxLength;
    }
    double height = minLength;
    double width = maxLength;
    double area = height * width;

    double surfacePercentage = (object.data.size()/area) * 100;

    vector<vector<Point>> holes = getHoles(getContoursHierarchy(object.objectImage));
    double numberOfHoles = holes.size();

    pair<Point,int> gripPoint = Templater::getGripPoint(object.objectImage.clone(),gTemplate);


    //Add parameters to map
    parameters.insert(make_pair("pAspect",width/height));
    parameters.insert(make_pair("pHeight",height));
    parameters.insert(make_pair("pWidth",width));
    //parameters.insert(make_pair("pArea",area));
    parameters.insert(make_pair("pSurfacePercentage",surfacePercentage));
    parameters.insert(make_pair("pNumberOfHoles",numberOfHoles));
    //Gripper parameters
    parameters.insert(make_pair("gXOffset",gripPoint.first.x));
    parameters.insert(make_pair("gYOffset",gripPoint.first.y));
    parameters.insert(make_pair("gAngle",gripPoint.second));
    return parameters;
}

Point FeatureFactory::findCenter(const vector<Point> blob){
    int x,y;
    int minx = 1080,maxx = 0;
    int miny = 1920,maxy = 0;
    for(int i = 0; i < blob.size(); ++i){
        x = blob[i].x;
        y = blob[i].y;
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
    int centerX,centerY;
    centerX = ((maxx - minx)/2) + minx;
    centerY = ((maxy - miny)/2) + miny;
    return Point(centerX,centerY);
}

Point FeatureFactory::findCenterOfMass(const vector<Point> blob){
    Moments blobMoments = moments(blob);
    return Point(blobMoments.m10/blobMoments.m00,blobMoments.m01/blobMoments.m00);
}

vector<vector<Point> > FeatureFactory::extractCurves(vector<Point>& curve){
    double sum =0;
    double average = 0;

    vector<vector<Point>> curves;
    for(int i = 0; i < curve.size();++i){
        sum += pow(getEuclideanDistance(curve[i],curve[(i+1)%curve.size()]),3);
    }
    average = sum / curve.size();

    vector<Point> tempCurv;
    for(int i = 0, vectorOffset = 0; i < (curve.size() + vectorOffset);++i){
        if(pow(getEuclideanDistance(curve[i],curve[(i+1)%curve.size()]),3) > average){
            for(++i; i < (curve.size() + vectorOffset);++i){
                if(pow(getEuclideanDistance(curve[i%curve.size()],curve[(i+1)%curve.size()]),3) <= average){
                    if(vectorOffset == 0){
                        vectorOffset = i;
                    }
                    tempCurv.push_back(curve[i%curve.size()]);
                }else{
                    if(tempCurv.size() != 0){
                        tempCurv.push_back(curve[i%curve.size()]);
                        curves.push_back(tempCurv);
                        tempCurv.clear();
                    }
                }
            }
        }
    }
    return curves;
}

double FeatureFactory::getEuclideanDistance(Point &p1, Point &p2){
    double dX,dY;
    dX = p1.x - p2.x;
    dY = p1.y - p2.y;
    return sqrt((dX * dX) + (dY * dY));
}

vector<vector<Point>> FeatureFactory::findConnectedComponents(const Mat& image){
    vector<vector<Point>> blobs;
    Mat workingImage = ImageFactory::applyOtsuThreshold(image);
    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    threshold(workingImage,workingImage,0.0,1.0,THRESH_BINARY);
    workingImage.convertTo(workingImage, CV_32SC1);

    //TODO(Edwin): Do i really need this?

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
            for(int i=rect.y; i < (rect.y+rect.height); i++) {
                int *row2 = (int*)workingImage.ptr(i);
                for(int j=rect.x; j < (rect.x+rect.width); j++) {
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

vector<vector<Point> > FeatureFactory::getContours(const Mat &image){
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(image,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    return contours;
}

pair<vector<vector<Point>>, vector<Vec4i>> FeatureFactory::getContoursHierarchy(const Mat &image){
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(image.clone(),contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    return make_pair(contours,hierarchy);
}

vector<vector<Point>> FeatureFactory::getHoles(const pair<vector<vector<Point> >, vector<Vec4i> >& contours){
    vector<vector<Point>> holes;
    for(int i = 0; i < contours.first.size(); ++i){
        // The size check is to filter out the smallest insignificant holes
        if(contours.second[i][3] != -1 && contours.first[i].size() > 10*M_PI){
            holes.push_back(contours.first[i]);
        }
    }
    return holes;
}

vector<vector<Point> > FeatureFactory::getHoles(const Mat &image){
    return getHoles(getContoursHierarchy(image));
}
