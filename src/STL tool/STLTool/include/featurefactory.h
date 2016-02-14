#ifndef FEATUREFACTORY_H
#define FEATUREFACTORY_H

#include "imagefactory.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "part.h"

using namespace std;
using namespace cv;

//TODO: REDESIGN AND RENAME
class FeatureFactory{
public:
    //Maybe move to imagefactory
    static vector<vector<Point>> findConnectedComponents(const Mat& image);

    static void savePartConfig(const map<string, double> &parameterMap, const QString partName);
    static void savePartConfig(const Part& part);

    static map<string,double> createParameterMap(const VisionObject &object);
    static map<string, double> createParameterMap(const VisionObject &object,
                                                  vector<float> &modelData, vector<vector<vector<int> > > &gTemplate);
    static vector<vector<Point>> getContours(const Mat& image);
    static pair<vector<vector<Point>>, vector<Vec4i>> getContoursHierarchy(const Mat& image);
    static vector<vector<Point>> getHoles(const pair<vector<vector<Point>>, vector<Vec4i>>& contours);
    static vector<vector<Point>> getHoles(const Mat& image);
    static Point findCenter(const vector<Point> blob);
    static Point findCenterOfMass(const vector<Point> blob);

    //test function that isent used
    static vector<vector<Point>> extractCurves(vector<Point>& curve);
    //Helper function
    static double getEuclideanDistance(Point& p1,Point& p2);

};


#endif // FEATUREFACTORY_H
