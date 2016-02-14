#ifndef IMAGEFACTORY_H
#define IMAGEFACTORY_H

#include "QImage"
#include <string>

#include <opencv2/core/core.hpp>


#include "visionobject.h"

using namespace std;
using namespace cv;

//TODO: REDESIGN AND RENAME
class ImageFactory{
public:

    static void save(QImage& image, string imageName);
    static void save(Mat& image, string imageName);

    static Mat applyOtsuThreshold(const Mat& image);

    static Mat normalizeImageScale(const Mat& image, const vector<float>& modelData);
    //Maybe move to feature factory
    static vector<VisionObject> filterObjects(vector<vector<Point> > &objects, Mat &image);

    static QImage setGrayscale(QImage &image);
    static Mat setGrayscale(Mat& image);

    static bool checkGrayscale(QImage &image);

};

#endif // IMAGEFACTORY_H
