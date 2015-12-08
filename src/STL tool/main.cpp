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

#include "QInputDialog"

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
bool testing = false;
bool imagetaker = false;
bool camerafeed = true;

vector<vector<Point>> filterCurves(vector<Point>& convex, Mat& image){
    vector<vector<Point>> curves;
    double arcS =  arcLength(convex,true)/10;
    cout << "Arclength: " << arcLength(convex,true) << endl;
    int dx,dy;
    double distance;
    double sumDist = 0;
    vector<Point> newConvex; //= convex;
    cv::approxPolyDP(convex,newConvex,1,true);
    vector<pair<double,double>> lines(newConvex.size());
    cout << convex.size() << " - " << newConvex.size() << endl;
    for(int i = 0;  i < newConvex.size();++i){
        if( i != newConvex.size()-1){
            dx = newConvex[i].x - newConvex[i+1].x;
            dy = newConvex[i].y - newConvex[i+1].y;
            distance = sqrt((dx * dx) + (dy * dy));
            sumDist += distance;
            if(dx == 0){
                if(dy > 0){
                    lines[i] = make_pair(distance,0.5*M_PI);
                }
                if(dy < 0){
                    lines[i] = make_pair(distance,-0.5*M_PI);
                }
            }else{
                lines[i] = make_pair(distance,atan2(dy,dx));
            }
            cout << "Distance: " << lines[i].first << " - Angle: " << lines[i].second * (180/M_PI) << " (" << dx << "," << dy << ")" <<endl;

        }else{
            dx = abs(newConvex[i].x - newConvex[0].x);
            dy = abs(newConvex[i].y - newConvex[0].y);
            distance = sqrt((dx * dx) + (dy * dy));
            sumDist += distance;
            if(dx == 0){
                if(dy > 0){
                    lines[i] = make_pair(distance,0.5*M_PI);
                }
                if(dy < 0){
                    lines[i] = make_pair(distance,-0.5*M_PI);
                }
            }else{
                lines[i] = make_pair(distance,atan2(dy,dx));
            }
            cout << "Distance: " << lines[i].first << " - Angle: " << lines[i].second * (180/M_PI) << " (" << dx << "," << dy << ")" <<endl;

        }
    }


    cout << "Average line length: " << sumDist/lines.size() << "Arcss: " << arcS <<endl;
    cout << "Line count: " << lines.size() << endl;
    int curvess = 0;
    for(int i = 0; i < lines.size();++i){
        if(i != lines.size() - 1){
            cout << "Line size: " << lines[i].first << " -  10% Arc: " << arcS << endl;
            if(lines[i].first < arcS){
                line(image,newConvex[i],newConvex[i+1],Scalar(255,255,255),3);
                cout << "Good line" << endl;
            }
            if(lines[i].first > arcS){
                ++curvess;
                cout << "Bad line" << i <<  endl;

            }
        } else {
            if(lines[i].first < arcS){
                line(image,newConvex[i],newConvex[0],Scalar(255,255,255),3);
                cout << "Goodline" << endl;
            }
            if(lines[i].first > arcS){
                cout << "Bad line" << i <<  endl;
                ++curvess;
            }
        }

    }
    cout << "Curves: " << curvess << endl;

    return curves;
}

bool actually_find_chessboard_corners(const cv::Mat& frame, cv::Mat& corners, const cv::Size& size, int flags) {
    int count = size.area();
    corners.create(count, 1, CV_32FC2);
    CvMat _image = frame;
    bool ok = cvFindChessboardCorners(&_image, size,
                                      reinterpret_cast<CvPoint2D32f*>(corners.data),
                                      &count, flags ) > 0;
    return ok;
}

void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2)
{
    double angle;
    double hypotenuse;
    angle = atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
    hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
    //    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
    //    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
    // Here we lengthen the arrow by a factor of scale
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    line(img, p, q, colour, 1, CV_AA);
    // create the arrow hooks
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
}

double getOrientation(const vector<Point> &pts, Mat &img)
{
    //Construct a buffer used by the pca analysis
    int sz = static_cast<int>(pts.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    //Store the center of the object
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                       static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    // Draw the principal components
    circle(img, cntr, 3, Scalar(255, 0, 255), 2);
    Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    drawAxis(img, cntr, p1, Scalar(125, 125, 125), 1);
    drawAxis(img, cntr, p2, Scalar(125, 125, 125), 5);
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
    return angle;
}

int main(int argc, char *argv[])
{

    QElapsedTimer timer;
    timer.start();
    if(imagetaker){
        QApplication app(argc,argv);
        QWidget inputwidget;
        string testDir = "/namedimages/";
        string imageName;

        bool succes;
        cv::namedWindow("Test",cv::WINDOW_AUTOSIZE);
        cv::VideoCapture cap(1);
        cv::Mat frame;
        for(int i = 0;; ++i){
            cv::waitKey(0);
            cout << i << endl;
            for(int j = 0; j < 6; ++j){
                cap >> frame;
            }
            cv::imshow("Test",frame);

            imageName = QInputDialog::getText(&inputwidget,"Save","Image name:",QLineEdit::Normal,"",&succes).toStdString() + ".bmp";
            if(succes){
                cv::imwrite(QDir::currentPath().toStdString() + testDir + imageName,frame);
            }
        }
    }

    if(camerafeed){
        VideoCapture cap(1);
        Mat cameraFrame;
        vector<vector<Point>> blobs;
        vector<VisionObject> objects;
        vector<vector<Point>> contours;
//        kkORB orb;
        for(int i = 0; i < 6; ++i){
            cap >> cameraFrame;
        }
        cvtColor(cameraFrame,cameraFrame,cv::COLOR_RGB2GRAY);
        cameraFrame = ImageFactory::applyOtsuThreshold(cameraFrame);
        blobs = FeatureFactory::findConnectedComponents(cameraFrame);
        objects = ImageFactory::filterObjects(blobs,cameraFrame);
        contours = FeatureFactory::getContours(objects[0].objectImage);
        vector<Point> testContour = contours[0];
        while(true){
            timer.restart();
            for(int i = 0; i < 6; ++i){
                cap >> cameraFrame;
            }
            cvtColor(cameraFrame,cameraFrame,cv::COLOR_RGB2GRAY);
            cameraFrame = ImageFactory::applyOtsuThreshold(cameraFrame);
            blobs = FeatureFactory::findConnectedComponents(cameraFrame);
            objects = ImageFactory::filterObjects(blobs,cameraFrame);
            if(objects.size() > 0){
                int height = objects[0].objectImage.size().height;
                int width = objects[0].objectImage.size().width;
                int fixedSize = (height + width) / 2;
                //           d resize(objects[0].objectImage,objects[0].objectImage,Size(fixedSize,fixedSize));
                contours = FeatureFactory::getContours(objects[0].objectImage);
                vector<vector<Point>> hull(1);
                convexHull(contours[0],hull[0]);

//                orb.detectAndCompute(Mat(hull[0]),);
                //                for(int i = 0; i < hull[0].size();++i){
                //                    line(objects[0].objectImage,hull[0][i],hull[0][i],Scalar(255,255,255),3);
                //                }
                //                cout << hull[0].size() << endl;
                //            cout << getOrientation(contours[0],cameraFrame) << endl;
                //            drawContours(objects[0].objectImage,hull,0,Scalar(255,255,255));
                //            cout << timer.elapsed() << endl;
                //            Moments moment = moments(contours[0],false);
                //            Point center = Point(moment.m10/moment.m00,moment.m01/moment.m00);
                //            line(objects[0].objectImage,center,center,Scalar(125,125,125),10);
                ////            cout << "Match: " << matchShapes(testContour,contours[0],CV_CONTOURS_MATCH_I1,0) << endl;
                //            double theta = 0.5 * atan((2 * moment.mu11)/(moment.mu20 - moment.mu02));
                //            double thetaGrade = theta * (180/M_PI);
                //            cout << thetaGrade << endl;
                //            line(objects[0].objectImage,Point((center.x - 200) * cos(theta),(center.y - 200) * cos(theta)),Point((center.x + 200) * cos(theta),(center.y + 200) * cos(theta))
                //                    ,Scalar(255,255,255),3);

//                filterCurves(hull[0],objects[0].objectImage);
                imshow("Floe",objects[0].objectImage);
            }
            imshow("camerafeed",cameraFrame);
            waitKey(0);
            if(waitKey(30) >= 0){
                break;
            }
        }
    }
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
            if(mu[0].m20 != mu[0].m02){
                double theta = 0.5 * atan((2 * mu[0].mu11)/(mu[0].mu20 - mu[0].mu02));
                double thetaGrade = theta * (180/M_PI);
                line(objects[0].objectImage,Point((mc[0].x - 200) * cos(theta),(mc[0].y - 200 )* cos(theta)),Point((mc[0].x + 200 )* cos(theta),(mc[0].y + 200 )* cos(theta))
                        ,Scalar(255,255,255),3);
                cout << "Axis theta: " << thetaGrade << endl;
                Mat rot_mat = cv::getRotationMatrix2D(mc[0],thetaGrade,1.0);
                //            Mat dst;
                //            cv::warpAffine(objects[0].objectImage,dst,rot_mat,objects[0].objectImage.size());
                //            imshow("warped",dst);
                //            drawContours(objects[i].objectImage,hull,0,Scalar(255,255,255));
                //            cout << contours.size() << " - " << biggestContour << endl;
                //            rects[0] = minAreaRect(contours[biggestContour]);
                //            Point2f rect_points[4];
                //            rects[0].points(rect_points);
                //            for(int k = 0; k < 4; ++k){
                //                line( objects[i].objectImage, rect_points[0], rect_points[(k+1)%4],Scalar(255,255,255), 1, 8 );
                //            }
            }


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

