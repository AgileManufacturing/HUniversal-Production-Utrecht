#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

#include "ros/ros.h"
#include "rexos_logger/rexos_logger.h"

#include <vector>
#include <opencv2/core/core.hpp>
#include "vision_node/VisionObject.h"

using namespace std;
using namespace cv;

class ObjectDetector{
public:
   static Mat applyOtsuThreshold(const Mat &image);
   static vector<vector<Point>> findConnectedComponents(const Mat& image);
   static vector<vector<Point>> getContours(const Mat &image);
   static pair<vector<vector<Point>>, vector<Vec4i>> getContoursHierarchy(const Mat &image);
   static vector<vector<Point>> getHoles(const pair<vector<vector<Point>>, vector<Vec4i>>& contours);
   static vector<vector<Point>> getHoles(const Mat &image);
   static vector<VisionObject> filterObjects(vector<vector<Point>>& objects,Mat& image,Mat& debugImage);


private:
};

#endif // OBJECTDETECTOR_H
