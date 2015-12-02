#include <iostream>
#include <QApplication>
#include "mainwindow.h"
#include "stlparser.h"
#include "workplane.h"
#include <QWidget>
#include <QFileDialog>
#include "glm/glm.hpp"
#include "QDir"
#include "QFile"

#include "QElapsedTimer"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <cmath>

#include "featureparser.h"
#include "featurematcher.h"
#include "featurefactory.h"

#include "imagefactory.h"

#include "visionobject.h"

#define _USE_MATH_DEFINES
#include <math.h>


using namespace std;
using namespace cv;

bool tool = false;
bool testing = true;

bool actually_find_chessboard_corners(const cv::Mat& frame, cv::Mat& corners, const cv::Size& size, int flags) {
    int count = size.area();
    corners.create(count, 1, CV_32FC2);
    CvMat _image = frame;
    bool ok = cvFindChessboardCorners(&_image, size,
                                      reinterpret_cast<CvPoint2D32f*>(corners.data),
                                      &count, flags ) > 0;
    return ok;
}

int main(int argc, char *argv[])
{
    QElapsedTimer timer;
    if(testing){

        vector<vector<Point>> blobs;
        vector<VisionObject> objects;
        Mat testImage = imread(QDir::currentPath().toStdString() + "/positives/testImage15.bmp",CV_LOAD_IMAGE_GRAYSCALE);
//        Mat testImage = imread(QDir::currentPath().toStdString() + "/images/image1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
        timer.start();
        testImage = ImageFactory::applyOtsuThreshold(testImage);
        cout << timer.elapsed() << endl;
        blobs = FeatureFactory::findConnectedComponents(testImage);

        objects = ImageFactory::filterObjects(blobs,testImage);
//        FeatureFactory::savePartConfig(FeatureFactory::createParameterMap(objects[1]),"EdwinVinger");
//        vector<vector<Point>> hull;
//        hull.resize(1);
//        vector<Vec4i> convexityDefects;
        cout << objects.size() << endl;
        for(int i = 0; i < objects.size(); ++i){

//            int size = (objects[i].objectImage.size().height + objects[i].objectImage.size().width) / 2;
//            int height = objects[i].objectImage.size().height;
//            int width = objects[i].objectImage.size().width;
//            imshow("Object1" + to_string(i),objects[i].objectImage);
//            resize(objects[i].objectImage,objects[i].objectImage,Size(size,size));
            vector<vector<Point>> contours = FeatureFactory::getContours(objects[i].objectImage);
            vector<RotatedRect> rects(contours.size());
            vector<Moments> mu(contours.size());
            int biggestContour = 0;
            for(int j = 0; j < contours.size(); ++j){
                if (arcLength(contours[j],false) > biggestContour){
                    biggestContour = j;
                }
                mu[j] = moments(contours[j],false);
            }
            vector<Point> mc(contours.size());
            for(int j = 0; j < contours.size(); ++j){
                if(contours[j].size() > (M_PI * 10)){
                  mc[j] = Point(mu[j].m10/mu[j].m00,mu[j].m01/mu[j].m00);
                }
            }
            for(int j = 0; j < mc.size(); ++j){
                cv::line(objects[i].objectImage,mc[j],mc[j],Scalar(255,255,255),5);
            }

            vector<vector<Point>> hull(2);
            convexHull(contours[0],hull[0]);
            std::vector< float > vecCurvature;//( hull[0].size() );
            cv::Point2f posOld, posOlder;
            cv::Point2f f1stDerivative, f2ndDerivative;

            for (size_t j = 0; j < hull[0].size(); j++ ){
                const cv::Point2f& pos = hull[0][j];

                if ( j == 0 ){ posOld = posOlder = pos; }

                f1stDerivative.x =   pos.x -        posOld.x;
                f1stDerivative.y =   pos.y -        posOld.y;
                f2ndDerivative.x = - pos.x + 2.0f * posOld.x - posOlder.x;
                f2ndDerivative.y = - pos.y + 2.0f * posOld.y - posOlder.y;

                float curvature2D = 0.0f;
                if ( std::abs(f2ndDerivative.x) > 10e-4 && std::abs(f2ndDerivative.y) > 10e-4 )
                {
                    curvature2D = sqrt( std::abs(
                        pow( f2ndDerivative.y*f1stDerivative.x - f2ndDerivative.x*f1stDerivative.y, 2.0f ) /
                        pow( f2ndDerivative.x + f2ndDerivative.y, 3.0 ) ) );
                }
                if(curvature2D > 0 && curvature2D < INFINITE){
//                    vecCurvature[j] = curvature2D;
                    vecCurvature.push_back(curvature2D);
                    hull[1].push_back(hull[0][j]);
                } else {
//                    vecCurvature[j] = 0.0f;
                    vecCurvature.push_back(0.0f);
                    hull[1].push_back(hull[0][j]);
                }

                posOlder = posOld;
                posOld = pos;
            }
            for(auto i : vecCurvature){
                cout << i << endl;
            }
            for(int j = 0; j < hull[1].size();++j){

                line(objects[i].objectImage,hull[1][j],hull[1][j+1],Scalar(vecCurvature[j]*255,vecCurvature[j]*255,vecCurvature[j]*255));

            }
//            drawContours(objects[i].objectImage,hull,0,Scalar(255,255,255));
//            cout << contours.size() << " - " << biggestContour << endl;
//            rects[0] = minAreaRect(contours[biggestContour]);
//            Point2f rect_points[4];
//            rects[0].points(rect_points);
//            for(int k = 0; k < 4; ++k){
//                line( objects[i].objectImage, rect_points[0], rect_points[(k+1)%4],Scalar(255,255,255), 1, 8 );
//            }



            cout << "Object" + to_string(i) + " - " << contours.size() << endl;
//            convexHull(objects[i].data, hull[0],false,false);
//            drawContours(testImage,hull,0,Scalar(255,255,255));
//            cv::convexityDefects(objects[i].data,hull[0],convexityDefects);
//            cout << convexityDefects.size() << endl;


            imshow("Object" + to_string(i),objects[i].objectImage);
//            pair<Part,double> match =FeatureMatcher::matchPart(FeatureFactory::createParameterMap(objects[i]));
//            cout << match.first.name << " - " << match.second << endl;
        }

        imshow("Testing",testImage);
        waitKey(0);
////ParseTest
//        vector<string> test;
//        test = FeatureParser::getPartList();
//        Part testpart;
//        testpart = FeatureParser::parsePart(test[1]);
//        testpart.name = "savetest";
//        FeatureFactory::savePartConfig(testpart);

    }
    if(tool){
        QApplication app(argc,argv);
        MainWindow window;
        window.show();
        return app.exec();
    }

    return 0;
}

