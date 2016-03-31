#ifndef VISIONOBJECT_H
#define VISIONOBJECT_H

#include <vector>
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

/**
 * @brief The VisionObject struct
 *
 * Contains the information of and object that is extracted
 * fromt a camera frame.
 */
struct VisionObject{
    Mat objectImage;
    vector<Point> data;
};

#endif // VISIONOBJECT_H
