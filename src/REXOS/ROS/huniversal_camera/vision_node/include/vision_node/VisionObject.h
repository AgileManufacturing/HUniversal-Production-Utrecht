#ifndef VISIONOBJECT_H
#define VISIONOBJECT_H

#include <vector>
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

struct VisionObject{
    Mat objectImage;
    vector<Point> data;
};

#endif // VISIONOBJECT_H
