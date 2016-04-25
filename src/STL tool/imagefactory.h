#ifndef IMAGEFACTORY_H
#define IMAGEFACTORY_H

#include "QImage"
#include <string>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "visionobject.h"

using namespace std;
using namespace cv;


class ImageFactory{
public:

    static void save(QImage& image, string imageName);
    static void save(Mat& image, string imageName);

    static Mat applyOtsuThreshold(const Mat& image);
    static vector<VisionObject> filterObjects(vector<vector<Point> > &objects, Mat &image);

    static QImage setGrayscale(QImage &image);
    static Mat setGrayscale(Mat& image);

    static bool checkGrayscale(QImage &image);

};

#endif // IMAGEFACTORY_H
