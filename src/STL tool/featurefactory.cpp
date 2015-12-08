#include "featurefactory.h"
#include "imagefactory.h"
#include "QDir"
#include "QFile"
#include "QTextStream"



void FeatureFactory::savePartConfig(const map<string, double>& parameterMap,const QString partName){
    QString path = QDir::currentPath() + "/parts/" + partName + ".config";
    QFile configFile;
    configFile.setFileName(path);
    configFile.open(QIODevice::WriteOnly);
    QTextStream outputStream(&configFile);
    outputStream << "Product agent configuration file" << "\n"
                 << "Version:=1.0" << "\n"
                 << "Partname:=" << partName << "\n"
                 << "/parameters" << "\n";
    for(auto i : parameterMap){
        outputStream << QString(i.first.data()) << ":=" << i.second << "\n";
    }
    configFile.close();
}

void FeatureFactory::savePartConfig(const Part& part){
    QString path = QDir::currentPath() + "/parts/" + QString(part.name.data()) + ".config";
    QFile configFile;
    configFile.setFileName(path);
    configFile.open(QIODevice::WriteOnly);
    QTextStream outputStream(&configFile);
    outputStream << "Product agent configuration file" << "\n"
                 << "Version:=1.0" << "\n"
                 << "Partname:=" << QString(part.name.data()) << "\n"
                 << "/parameters" << "\n";
    for(auto i : part.parameters){
        outputStream << QString(i.first.data()) << ":=" << i.second << "\n";
    }
    configFile.close();
}

map<string, double> FeatureFactory::createParameterMap(const VisionObject& object){
    map<string,double> parameters;
    double height = object.objectImage.size().height;
    double width = object.objectImage.size().width;
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
    findContours(image,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);
    return contours;
}

pair<vector<vector<Point>>, vector<Vec4i>> FeatureFactory::getContoursHierarchy(const Mat &image){
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(image,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);
    return make_pair(contours,hierarchy);
}

vector<vector<Point>> FeatureFactory::getHoles(const pair<vector<vector<Point> >, vector<Vec4i> >& contours){
    vector<vector<Point>> holes;
    for(int i = 0; i < contours.first .size(); ++i){
        // The size check is to filter out the smallest insignificant holes
        if(contours.second[i][3] != -1 && contours.first[i].size() > 100){
            holes.push_back(contours.first[i]);
        }
    }
    return holes;
}

vector<vector<Point> > FeatureFactory::getHoles(const Mat &image){
    return getHoles(getContoursHierarchy(image));
}
