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

using namespace std;
using namespace cv;

bool chestboardCalibrate = false;
bool tool = false;
bool visionTest = false;
bool featureDetect = false;
bool imageTaker = false;
bool pointGrid = true;
bool cameraFeed = false;
bool segmentationTest = true;
bool circleTest = false;


bool actually_find_chessboard_corners(const cv::Mat& frame, cv::Mat& corners, const cv::Size& size, int flags) {
    int count = size.area();
    corners.create(count, 1, CV_32FC2);
    CvMat _image = frame;
    bool ok = cvFindChessboardCorners(&_image, size,
        reinterpret_cast<CvPoint2D32f*>(corners.data),
        &count, flags ) > 0;
    return ok;
}
void FindBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs){
    blobs.clear();

    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    cv::Mat label_image;
    binary.convertTo(label_image, CV_32SC1);

    int label_count = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y < label_image.rows; y++) {
        int *row = (int*)label_image.ptr(y);
        for(int x=0; x < label_image.cols; x++) {
            if(row[x] != 1) {
                continue;
            }

            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 4);

            std::vector <cv::Point2i> blob;

            for(int i=rect.y; i < (rect.y+rect.height); i++) {
                int *row2 = (int*)label_image.ptr(i);
                for(int j=rect.x; j < (rect.x+rect.width); j++) {
                    if(row2[j] != label_count) {
                        continue;
                    }

                    blob.push_back(cv::Point2i(j,i));
                }
            }

            blobs.push_back(blob);

            label_count++;
        }
    }
}

int main(int argc, char *argv[])
{
    QElapsedTimer timer;
    QFile configFile;
    configFile.setFileName(QDir::currentPath() + QString("test.config"));
    configFile.open(QIODevice::ReadWrite);

    Mat resultImage;
//    Mat resultImage = imread(QDir::currentPath().toStdString() + "/positives/testImage21.bmp",CV_LOAD_IMAGE_GRAYSCALE);

    if(chestboardCalibrate){
//        cv::VideoCapture cap(1);
//        Mat frame;
//        for(int i = 0; i < 6; ++i){
//            cap >> frame;
//       }
//        Mat chessboard = imread(QDir::currentPath().toStdString() + "/chessboard/testImage.bmp",CV_LOAD_IMAGE_GRAYSCALE);
//        vector<Point3f> corners;
//       Mat corner;
//       bool found = actually_find_chessboard_corners(chessboard,corner,Size(8,5),CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
//       if(found){
//           corners.assign((Point2f*)corner.datastart, (Point2f*)corner.dataend);
//           cv::cornerSubPix(chessboard, corner, cv::Size(4, 4), cv::Size(-1, -1),
//                                   cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.1));
//           drawChessboardCorners(chessboard, Size(7,5), corner, found);
//            vector<Point3f> objectCorners;

//            for (int i = 0; i < 5; i++) {
//                for (int j = 0; j < 8; j++) {
//                    objectCorners.push_back(cv::Point3f(i, j, 0.0f));
//                }
//            }

//            vector<Mat> rvecs,tvecs;
//            Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
//            Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
//            double rms = calibrateCamera(objectCorners,corners,Size(640,480),cameraMatrix,distCoeffs,rvecs,tvecs,0);
//        fisheye::calibrate()



//       }
////       for(int i = 0; i < corners.size()
////       for(int i = 0 ; i < corners.size();++i){
////           cout << corners[i].x << " - " << corners[i].y << endl;

////       }

////       cornerSubPix(chessboard, corners, Size(11, 11), Size(-1, -1),
////         TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
////       cout << corners.size() << endl;

////        int found = findChessboardCorners(chessboard,Size(8,5),corners,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
////        if(found){

//        imshow("Test", chessboard);
////}
//        waitKey(0);


  }
    if(cameraFeed){
        cv::VideoCapture cap(1);
        cv::Mat frame;
        while(true){
            cap >> frame;

            cv::imshow("Crash niet",frame);
            if(cv::waitKey(27) >= 0) break;
        }
    }
    if(imageTaker){
        string testDir = "/negatives/";
        string imageName;
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

            imageName = "testImage" + to_string(i) + ".bmp";
            cv::imwrite(QDir::currentPath().toStdString() + testDir + imageName,frame);
        }
    }
    if(pointGrid){
        cv::Mat CI;
        cv::Mat TI;
        TI = cv::imread(QDir::currentPath().toStdString() + "/positives/testImage7.bmp",CV_LOAD_IMAGE_GRAYSCALE);
        CI = cv::imread(QDir::currentPath().toStdString() + "/negatives/testImage0.bmp",CV_LOAD_IMAGE_GRAYSCALE);

        cv::Mat RI;
        int gridSize = 20;
        int imageW = CI.size().width;
        int imageH = CI.size().height;
        RI = cv::Mat::zeros(imageH/gridSize,imageW/gridSize,CV_8U);
        cout << RI.size().width << " " << RI.size().height << endl;
        cout << "Width: " << imageW << " Height: " << imageH << endl;
        cout << (imageH/gridSize) *(imageW/gridSize) << endl << endl;

        vector<int> ciCheckables,tiCheckables;
        ciCheckables.resize((imageH/gridSize) *(imageW/gridSize));
        tiCheckables.resize((imageH/gridSize) *(imageW/gridSize));

        for(int y = 0; y < imageH; y+=gridSize){
            for(int x = 0; x < imageW; x+=gridSize){
                ciCheckables[((y/gridSize) * (imageW/gridSize)) + x/gridSize] = CI.data[(y * imageW) + x];
                tiCheckables[((y/gridSize) * (imageW/gridSize)) + x/gridSize] = TI.data[(y * imageW) + x];
            }
        }
        cout << "Calibration image data" << endl;
        for(int y = 0; y < imageH/gridSize;++y){
            for(int x = 0; x < imageW/gridSize;++x){
                cout << ciCheckables[y * (imageW/gridSize) + x] << " ";
            }
            cout << endl;
        }
        cout << endl << "Object test image data" << endl << endl;
        for(int y = 0; y < imageH/gridSize;++y){
            for(int x = 0; x < imageW/gridSize;++x){
                cout << tiCheckables[y * (imageW/gridSize) + x] << " ";
            }
            cout << endl;
        }
        cout << endl << "Substracted Data" << endl << endl;

        for(int y = 0; y < imageH/gridSize;++y){
            for(int x = 0; x < imageW/gridSize;++x){
                cout << tiCheckables[y * (imageW/gridSize) + x] - ciCheckables[y * (imageW/gridSize) + x]<< " ";
            }
            cout << endl;
        }
        // Filling downscaled image
        int newValue;
        vector<int> newData;
        newData.resize(32 * 24);
        cout << endl << "downscaled Data" << endl << endl;
        for(int y = 0; y < RI.size().height;++y){
            for(int x = 0; x < RI.size().width;++x){
//                RI.data[y * RI.size().width + x] =
                newValue = TI.data[((y * gridSize) * imageW) + (x * gridSize)] - CI.data[((y * gridSize) * imageW) + (x * gridSize)];
                if(newValue < -100|| newValue > 100){
                    cout << newValue << " ";
                }else{
                    newValue = 0;
                    cout << 0 << " ";
                }
                RI.at<uchar>(y,x) = newValue;
                newData[y * RI.size().width + x] = newValue;
            }
            cout << endl;
        }
        int average= 0;
        for(int i : newData){
            average += i;
        }
        average = round(average / newData.size());
        cout << "Average: " <<(int) average << endl;
        cv::Mat DI = cv::Mat::zeros(RI.size().height,RI.size().width,CV_8U);
        cv::Mat EI = cv::Mat::zeros(RI.size().height,RI.size().width,CV_8U);

        int checkVal;

        cout << "Added erosion and dilation" << endl;
        for(int y = 1; y < RI.size().height - 1;++y){
            for(int x = 1; x < RI.size().width - 1;++x){
                //DILATION
                for(int ky = -1; ky <= 1; ++ky){
                    for(int kx = -1; kx <=1; ++kx){
                        checkVal = RI.data[((y + ky) * RI.size().width) + (x + kx)];
                        if(checkVal!= 0){
                            DI.at<uchar>(y,x) = checkVal;
//                            DI.data[(y * DI.size().width) + x] = checkVal;
                        }
                    }
                }
            }
        }
        for(int y = 1; y < DI.size().height - 1;++y){
            for(int x = 1; x < DI.size().width - 1;++x){
                //erosion
                EI.at<uchar>(y,x) = DI.at<uchar>(y,x);

                for(int ky = -1; ky <= 1; ++ky){
                    for(int kx = -1; kx <=1; ++kx){
                        checkVal = DI.data[((y + ky) * DI.size().width) + (x + kx)];
                        if(checkVal == 0){
                            EI.at<uchar>(y,x) = 0;
                        }
                    }
                }
            }
        }
        cout << endl << "Closed image" << endl << endl;
        for(int y = 0; y < EI.size().height;++y){
            for(int x = 0; x < EI.size().width;++x){
                newValue = DI.at<uchar>(y,x);
                cout << newValue << " ";

            }
            cout << endl;
        }

        int minX = EI.size().width,minY = EI.size().height;
        int maxX = -1,maxY =-1;
        for(int y = 0; y < EI.size().height;++y){
            for(int x = 0; x < EI.size().width;++x){
                newValue = DI.at<uchar>(y,x);
                if (newValue != 0){
                    if(x < minX){
                        minX = x;
                    }
                    if(x > maxX){
                        maxX = x;
                    }
                    if(y < minY){
                        minY = y;
                    }
                    if(y > maxY){
                        maxY = y;
                    }
                }

            }
            cout << endl;
        }
        cout << endl << "min: (" << minX << "," << minY << ")" << endl;
        cout << endl << "max: (" << maxX << "," << maxY << ")" << endl;

        cv::Size size((maxX - minX) * gridSize,(maxY - minY) * gridSize);
        cv::resize(TI,resultImage,size);
        for(int y = 0; y < resultImage.size().height;++y){
            for(int x = 0; x < resultImage.size().width;++x){
                resultImage.at<uchar>(y,x) = TI.at<uchar>(y +(minY * gridSize),x + (minX * gridSize));

            }

        }
        line(TI,Point(minX * gridSize, minY * gridSize),Point(minX * gridSize,maxY * gridSize),Scalar(0,255,0),3);
        line(TI,Point(maxX * gridSize, minY * gridSize),Point(maxX * gridSize,maxY * gridSize),Scalar(0,255,0),3);
        line(TI,Point(minX * gridSize, minY * gridSize),Point(maxX * gridSize,minY * gridSize),Scalar(0,255,0),3);
        line(TI,Point(minX * gridSize, maxY * gridSize),Point(maxX * gridSize,maxY * gridSize),Scalar(0,255,0),3);
        imshow("Location", TI);
        cv::imshow("Result",resultImage);
        cv::waitKey(0);
    }
    if(visionTest){
////OpenCV testing
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

//    cv::Mat img_1 = cv::imread(QDir::currentPath().toStdString() + "/images/STL tool 29-10-2015.JPG", CV_LOAD_IMAGE_GRAYSCALE);
//    cv::Mat img_2 = cv::imread(QDir::currentPath().toStdString() + "/images/TestImage.jpg", CV_LOAD_IMAGE_GRAYSCALE);

//    if(!img_1.data || !img_2.data){
//        cout << "Reading image failed :(" << endl;
//        return -1;
//    }

    std::vector<cv::KeyPoint> keypoints1,keypoints2;

//    orb->detect(img_2,keypoints);

    cv::Mat keypointImage1;
    cv::Mat keypointImage2;

//    cv::drawKeypoints(img_2,keypoints,keypointImage);

//    //cv::imshow("Clean",img_1);
//    cv::imshow("Keypoints",keypointImage);

    //cv::waitKey(0);

    //cv::ORB(img_1,cv::Mat(),keypoints,cv::noArray());

//    cv::Mat image = cv::imread(QDir::currentPath().toStdString() + "/images/TestImage.jpg",CV_LOAD_IMAGE_UNCHANGED);
//    if(!image.data){
//        cout << "Reading image failed :(" << endl;
//        return -1;
//    }
//    orb.

//    cv::namedWindow("Test", CV_WINDOW_AUTOSIZE);
//    cv::imshow( "Test", image );

//    GLint sliderValue = 50;
//    cv::createTrackbar("SliderTest","Test",&sliderValue,100);



//    cv::waitKey(0);
//    cv::destroyWindow("Test");


//WEBCAM
//    cv::VideoCapture cap(1);
//    cv::namedWindow("Test", cv::WINDOW_AUTOSIZE);
//    cv::namedWindow("Test2", cv::WINDOW_AUTOSIZE);
//    cv::namedWindow("Test3", cv::WINDOW_AUTOSIZE);

    orb->setNLevels(1);
//    cv::Ptr<cv::BFMatcher> BFmatcher = cv::BFMatcher::create("BruteForce-Hamming");
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

//    if(!cap.isOpened()){
//        cout << "Cannot open the camera" << endl;
//    }

    cv::Mat frame;
    cv::Mat testImage = cv::imread(QDir::currentPath().toStdString() + "/images/image1.jpg",CV_LOAD_IMAGE_GRAYSCALE);

    cv::Mat match_img1,match_img2;

    cv::FlannBasedMatcher flannMatcher;
    cv::Mat descriptor1, descriptor2;
    vector<cv::DMatch> matches,filtered;
    vector<vector<cv::DMatch>> knnmatches;
//    BFmatcher->
    orb->setMaxFeatures(10);


//    for(int i = 0; i < 6; ++i){
//        cap >> frame;
//    }
    frame = cv::imread(QDir::currentPath().toStdString() + "/histogramtest/testImage5.bmp",CV_LOAD_IMAGE_UNCHANGED);
    cv::cvtColor(frame,frame,cv::COLOR_RGB2GRAY);
//    vector<uchar> imageData(frame.rows * frame.cols);
//    if(frame.isContinuous()){
//        imageData = &frame.data[0];
//    }
    vector<int> xHistogram1,yHistogram1,xHistogram1E,yHistogram1E,histoResultY;
    vector<int> xHistogram2,yHistogram2,xHistogram2E,yHistogram2E,histoResultX;

    int binSize = 10;

    xHistogram1.resize(frame.size().width / binSize);
    yHistogram1.resize(frame.size().height / binSize);

    xHistogram1E.resize(frame.size().width / binSize);
    yHistogram1E.resize(frame.size().height / binSize);

    histoResultX.resize(frame.size().width / binSize);
    histoResultY.resize(frame.size().height / binSize);

    for(int y = 0; y < frame.size().height / binSize; ++y){

        for(int x = 0; x < frame.size().width / binSize; ++x){
            yHistogram1[y] += frame.data[((y * binSize) * frame.size().width) + (x * binSize)];
            xHistogram1[x] += frame.data[((y * binSize) * frame.size().width) + (x * binSize)];
        }
    }
//    for(int y : yHistogram){
//        cout << y  << endl;
//    }
//    for(int x : xHistogram){
//        cout << x << " ";
//    }
    cout << endl;
    cv::imshow("Test1",frame);
    cv::waitKey(0);

//    for(int i = 0; i < 6; ++i){
//        cap >> frame;
//    }
    frame = cv::imread(QDir::currentPath().toStdString() + "/histogramtest/testImage28.bmp",CV_LOAD_IMAGE_UNCHANGED);
    cv::cvtColor(frame,frame,cv::COLOR_RGB2GRAY);
    xHistogram2.resize(frame.size().width / binSize);
    yHistogram2.resize(frame.size().height / binSize);

    xHistogram2E.resize(frame.size().width / binSize);
    yHistogram2E.resize(frame.size().height / binSize);

    for(int y = 0; y < frame.size().height / binSize; ++y){
        for(int x = 0; x < frame.size().width / binSize; ++x){
            yHistogram2[y] += frame.data[((y * binSize) * frame.size().width) + (x * binSize)];
            xHistogram2[x] += frame.data[((y * binSize) * frame.size().width) + (x * binSize)];
        }
    }
    yHistogram1E[1] = yHistogram1[1]; yHistogram1E[yHistogram1E.size()-1] = yHistogram1[yHistogram1.size()-1];
    yHistogram2E[1] = yHistogram2[1]; yHistogram2E[yHistogram2E.size()-1] = yHistogram2[yHistogram2.size()-1];
    for(int i = 1;i < yHistogram1.size()-1;++i){
        yHistogram2E[i] = ((2*yHistogram2[i-1]) + yHistogram2[i] + (2*yHistogram2[i+1]))/5;
        yHistogram1E[i] = ((2*yHistogram1[i-1]) + yHistogram1[i] + (2*yHistogram1[i+1]))/5;
    }
    xHistogram1E[1] = xHistogram1[1]; xHistogram1E[xHistogram1E.size()-1] = xHistogram1[xHistogram1.size()-1];
    xHistogram2E[1] = xHistogram2[1]; xHistogram2E[xHistogram2E.size()-1] = xHistogram2[xHistogram2.size()-1];
    for(int i = 1; i < xHistogram1.size()-1;++i){
        xHistogram2E[i] = ((2*xHistogram2[i-1]) + xHistogram2[i] + (2*xHistogram2[i+1]))/5;
        xHistogram1E[i] = ((2*xHistogram1[i-1]) + xHistogram1[i] + (2*xHistogram1[i+1]))/5;
    }
    cv::imshow("Test2",frame);
    int averageX = 0,averageY = 0;
    int newValue;

    //Cali frame
    for(int i = 0; i < yHistogram1.size(); ++i){
        newValue = yHistogram1[i];
        cout << newValue << endl;
    }
    for(int i = 0; i < xHistogram1.size(); ++i){
        newValue = xHistogram1[i];
        cout << newValue << " ";
    }
    //Object frame
    for(int i = 0; i < yHistogram1.size(); ++i){
        newValue = yHistogram2[i];
        cout << newValue << endl;
    }
    for(int i = 0; i < xHistogram1.size(); ++i){
        newValue = xHistogram2[i];
        cout << newValue << " ";
    }
    //Substracted frame;
    for(int i = 0; i < yHistogram1.size(); ++i){
        newValue = pow(yHistogram2E[i] - yHistogram1E[i],2);
        histoResultY[i] = newValue;
        averageY += newValue;
        cout << newValue << endl;
    }
    for(int i = 0; i < xHistogram1.size(); ++i){
        newValue = pow(xHistogram2E[i] - xHistogram1E[i],2);
        histoResultX[i] = newValue;
        averageX += newValue;
        cout << newValue << " ";
    }
    averageX /= xHistogram1.size();
    averageY /= xHistogram2.size();
    cout << averageX  << " - " << averageY << endl;
//    for(int i = 0; i < yHistogram1.size(); ++i){
//        newValue = yHistogram2[i] - yHistogram1[i] - averageY;
//        if(newValue > 0){
//            newValue = 0;
//        }
//        cout << newValue << endl;
//    }
//    for(int i = 0; i < xHistogram1.size(); ++i){
//        newValue = xHistogram2[i] - xHistogram1[i] - averageX;
//        if(newValue > 0){
//            newValue = 0;
//        }
//        cout << newValue << " ";
//    }
//    cout << endl;
//    cout << averageX << " - " << averageY << endl;
//    int minY = -1,maxY = -1;
//    int minX = -1,maxX = -1;
//    for(int i = 0; i < yHistogram1.size(); ++i){
//        newValue = yHistogram2[i] - yHistogram1[i] - averageY;
//        if(newValue < 0){
//            cout << yHistogram2[i] - yHistogram1[i] << " ";
//            if(minY = -1){
//                minY = i * binSize;
//            }
//            while(newValue < 0){
//                ++i;
//                newValue = yHistogram2[i] - yHistogram1[i] - averageY;
//                if(newValue >= 0){
//                    maxY = i * binSize;
//                } else{
//                    cout << yHistogram2[i] - yHistogram1[i] << endl;
//                }
//            }
//            if(minY != -1 && maxY != -1){
//                break;
//            }
//        }
//    }
//    for(int i = 0; i < xHistogram1.size(); ++i){
//        newValue = xHistogram2[i] - xHistogram1[i] - averageX;
//        if(newValue < 0){
//            cout << xHistogram2[i] - xHistogram1[i] << " ";
//            if(minX = -1){
//                minX = i * binSize;
//            }
//            while(newValue < 0){
//                ++i;
//                newValue = xHistogram2[i] - xHistogram1[i] - averageX;
//                if(newValue >= 0){
//                    maxX = i * binSize;
//                } else{
//                    cout << xHistogram2[i] - xHistogram1[i] << " ";
//                }
//            }
//            if(minX != -1 && maxX != -1){
//                break;
//            }
//        }
//    }
//    minX -= 25;
//    maxX += 25;
//    minY -= 25;
//    maxY += 25;
//    cout << endl;
//    cout << minX << "," << minY << endl;
//    cout << maxX << "," << maxY << endl;
//    cv::waitKey(0);

//    cv::Mat result;

//    cv::Size size(maxX - minX,maxY - minY);
////    cout << size.width << "," << size.height << endl;
//    cv::resize(frame,result,size);
//    for(int y = minY; y < maxY; ++y){

//        for(int x = minX; x < maxX; ++x){
//            result.data[((y - minY) * result.size().width) + x - minX] = frame.data[(y * frame.size().width) + x];
//        }
//    }
////    cv::imshow("Test3",result);
////    cv::waitKey(0);
////    for(;;){
////        cap >> frame;
//        orb->detect(result,keypoints1);
//        orb->detect(testImage,keypoints2);

//        orb->compute(result,keypoints1,descriptor1);
//        orb->compute(testImage,keypoints2,descriptor2);
////        matcher->knnMatch(descriptor1,descriptor2,knnmatches,2);
//        matcher->match(descriptor1,descriptor2,matches);
////        for(vector<cv::DMatch> m : knnmatches){
////            if(m[0].distance < 0.8 * m[1].distance){
////                matches.push_back(m[0]);
////            }
////        }
////        for(cv::DMatch m : matches){
////            if(m.distance < 100){
////                filtered.push_back(m);
////            }
////        }

////        for(cv::KeyPoint k : keypoints1){
////            cout << k.angle << " ";
////        }
////        cout << endl;
////        cv::drawMatches(frame,keypoints1,testImage,keypoints2,matches,match_img1);
////        cv::drawMatches(frame,keypoints1,testImage,keypoints2,filtered,match_img2);

////        cv::drawKeypoints(frame,keypoints1,keypointImage1);
////        cv::drawKeypoints(testImage,keypoints2,keypointImage2);
////        //cv::drawMatches()
////        cv::imshow("Camera test",keypointImage1);
////        cv::imshow("Test image",keypointImage2);
////        cv::imshow("Test",match_img1);
////        cv::imshow("Test2",match_img2);
////        cv::drawKeypoints(frame,keypoints1,keypointImage1);
////        imshow("Test", keypointImage1);
//        cv::drawMatches(result,keypoints1,testImage,keypoints2,matches,match_img1);
//        cv::imshow("Test",match_img1);
////        cout << filtered.size() << " - " << matches.size() << endl;
////        if(cv::waitKey(27) >= 0) break;
//        cv::waitKey(0);
//    }
    }   
    if(segmentationTest){




//        Mat object_image= cv::imread(QDir::currentPath().toStdString() + "/DetectionTest/testImage0.bmp",CV_LOAD_IMAGE_GRAYSCALE);

        Mat object_image= cv::imread(QDir::currentPath().toStdString() + "/positives/testImage15.bmp",CV_LOAD_IMAGE_GRAYSCALE);
//        object_image.convertTo(object_image,CV_32F,1.0/255);
//        Mat segmentedImage;
//        kmeans(object_image,8,segmentedImage,TermCriteria( CV_TERMCRIT_EPS|CV_TERMCRIT_ITER, 10000, 0.0001),5,KMEANS_RANDOM_CENTERS);
//        imshow("Test",segmentedImage);
        timer.start();
//        Canny(object_image,object_image,100,200,3,true);
//        imshow("Canny",object_image);
//        Mat connected;
//        Mat labeled;
          cv::threshold(resultImage,resultImage,100,255,cv::THRESH_TOZERO);
          imshow("wat",resultImage);
//        imshow("Test",labeled);
//        waitKey(0);
//        connectedComponents(object_image,labeled,4);
//        imshow("Test",labeled);
//        waitKey(0);
           cv::namedWindow("binary");
           cv::namedWindow("labelled");

           cv::Mat output = cv::Mat::zeros(resultImage.size(), CV_8UC3);

           cv::Mat binary;
           Mat test;
           std::vector < std::vector<cv::Point2i > > blobs;

           cv::threshold(resultImage, binary, 0.0, 1.0, cv::THRESH_BINARY);

           FindBlobs(binary, blobs);
           cout << "Points: " <<blobs[1].size() << endl;
           int minx = resultImage.size().width,maxx = 0;
           int miny = resultImage.size().height ,maxy = 0;

           int x,y;
           for(int i = 0; i < blobs[1].size(); ++i){
               x = blobs[1][i].x;
               y = blobs[1][i].y;
               if(x <  minx){
                   minx = x;
               }
               if(x > maxx){
                   maxx = x;
               }
               if(y < miny){
                   miny = y;
               }
               if(y > maxy){
                   maxy = y;
               }
           }
           cout << "start point: (" << minx << "," << maxy << ")" << endl;
           cout << "eind point: (" << maxx << "," << miny << ")" << endl;
           int surface = (maxx - minx) * (maxy-miny);
           cout << "Surface percentage: " << ((float)blobs[1].size()/(float)(surface))*100 << endl;
           Mat check = Mat::zeros(maxy-miny,maxx-minx,CV_8U);
           for(int y = miny; y < maxy;y++){
               for(int x = minx; x < maxx;x++){
                   check.at<uchar>(y-miny,x-minx) = resultImage.at<uchar>(y,x);
               }
           }
//           for(int i = 0; i < blobs[1].size();++i){
//               x = blobs[1][i].x;
//               y = blobs[1][i].y;

//               check.at<uchar>(y - miny,x - minx) = resultImage.at<uchar>(y,x);
//           }
           waitKey(0);
           // Randomy color the blobs
           for(size_t i=0; i < blobs.size(); i++) {
               if(blobs[i].size() > 900){
                   unsigned char r = 255 * (rand()/(1.0 + RAND_MAX));
                   unsigned char g = 255 * (rand()/(1.0 + RAND_MAX));
                   unsigned char b = 255 * (rand()/(1.0 + RAND_MAX));

                   for(size_t j=0; j < blobs[i].size(); j++) {
                       int x = blobs[i][j].x;
                       int y = blobs[i][j].y;

                       output.at<cv::Vec3b>(y,x)[0] = b;
                       output.at<cv::Vec3b>(y,x)[1] = g;
                       output.at<cv::Vec3b>(y,x)[2] = r;
                   }
               }
           }
           vector<vector<Point>> regions,contours;
           vector<Vec4i> hierarchy;
           vector<Rect> boxes;
           Ptr<MSER> ms = MSER::create();
           ms->detectRegions(output,regions,boxes);
           for(int i = 0; i < regions.size(); ++i){
               ellipse(output,fitEllipse(regions[i]),Scalar(255),1);
               cout << regions[i].size() << endl;
           }

           findContours(check,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
           int holeCount = 0;
           for(int i = 0; i < contours.size();++i){
               if(hierarchy[i][3] != -1){
                   Scalar colour(255,255,255);
                   drawContours(check,contours,i,colour);
                   ++holeCount;
               }
           }
           cout << holeCount << endl;
           imshow("check",check);

           imshow("MSER", resultImage);
           cv::imshow("binary", resultImage);
           cv::imshow("labelled", output);
           cv::waitKey(0);

    }
    if(featureDetect){


//        Canny(resultImage,resultImage,100,200);
//        cv::VideoCapture cap(1);
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

        Mat object_image= cv::imread(QDir::currentPath().toStdString() + "/DetectionTest/testImage0.bmp",CV_LOAD_IMAGE_GRAYSCALE);;
        Mat scene_image = cv::imread(QDir::currentPath().toStdString() + "/images/image1.jpg",CV_LOAD_IMAGE_GRAYSCALE);

        resize(scene_image,scene_image,Size(640,480));
//        Canny(referenceImage,referenceImage,100,200);
//        imshow("edges",resultImage);
//        imshow("edges2",referenceImage);

        cv::Mat object_descriptor, scene_descriptor;

        vector<cv::KeyPoint> scene_points;
        vector<cv::KeyPoint> object_points;

        cv::Mat rKeypointImage, dKeypointImage;
        cv::Mat matchImage;
        vector<cv::DMatch> matches;
        vector<DMatch> goodmatches,bestmatches;
        vector<vector<DMatch>> knnmatches;

        vector<Point2f> obj,scene;

        cv::Ptr<cv::ORB> orb = cv::ORB::create();
//        orb->setScaleFactor(1.05);
        orb->setNLevels(2);
        orb->setMaxFeatures(100);
//        orb->setEdgeThreshold(8);
//        orb->setPatchSize(8);
//        orb->setWTA_K(3);
//        orb->setScoreType(ORB::FAST_SCORE);
//        for(;;){
//            matches.clear();
//            for(int i = 0; i < 6; ++i){
//            cap >> object_image;
//            }
//            imshow("Wtf camera",resultImage);
//            orb->detect(scene_image,scene_points);
//            orb->detect(object_image,object_points);

//            orb->compute(scene_image,scene_points,scene_descriptor);
//            orb->compute(object_image,object_points,object_descriptor);


////            matcher->match(descriptor1,descriptor2,matches);
//            matcher->knnMatch(scene_descriptor,object_descriptor,knnmatches,2);
//            drawKeypoints(object_image,object_points,dKeypointImage);
//            imshow("echt geen keypoints?",dKeypointImage);


            double max_dist = 0;
            double min_dist = 100;

//            for(int i = 0; i < matches.size();++i){
//                double distance = matches[i].distance;
//                if(distance < min_dist){
//                    min_dist = distance;
//                }
//                if(distance > max_dist){
//                    max_dist = distance;
//                }
//            }

//            for(int i = 0; i < knnmatches.size(); ++i){
//                if(knnmatches[i][0].distance < 0.8 * knnmatches[i][1].distance){
//                    goodmatches.push_back(knnmatches[i][0]);
//                }
//            }
//            cout << goodmatches.size() << endl;
//            for(int i = 0; i < goodmatches.size(); ++i){
//                obj.push_back(scene_points[goodmatches[i].queryIdx].pt);
//                cout << goodmatches[i].queryIdx << " - ";
//                scene.push_back(object_points[goodmatches[i].trainIdx].pt);
//                cout << goodmatches[i].trainIdx << endl;

//            }

//            cout << obj.size() << endl;

//            Mat H = findHomography(obj,scene,CV_RANSAC);
//            cv::drawMatches(scene_image,scene_points,object_image,object_points,goodmatches,matchImage);

//            vector<Point2f> obj_corners(4);

//            obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(scene_image.cols,0);
//            obj_corners[2] = cvPoint(scene_image.cols,scene_image.rows); obj_corners[3] = cvPoint(0,scene_image.rows);

//            vector<Point2f> scene_corners(4);
//            cv::perspectiveTransform(obj_corners,scene_corners,H);

//            line( matchImage, scene_corners[0] + Point2f( scene_image.cols, 0), scene_corners[1] + Point2f( scene_image.cols, 0), Scalar(0, 255, 0), 4 );
//            line( matchImage, scene_corners[1] + Point2f( scene_image.cols, 0), scene_corners[2] + Point2f( scene_image.cols, 0), Scalar( 0, 255, 0), 4 );
//            line( matchImage, scene_corners[2] + Point2f( scene_image.cols, 0), scene_corners[3] + Point2f( scene_image.cols, 0), Scalar( 0, 255, 0), 4 );
//            line( matchImage, scene_corners[3] + Point2f( scene_image.cols, 0), scene_corners[0] + Point2f( scene_image.cols, 0), Scalar( 0, 255, 0), 4 );
//            cout << object_points.size() << " - " << scene_points.size() << endl;

//            for(int i = 0; i < goodmatches.size(); ++i){
//                double distance = goodmatches[i].distance;
//                if(distance < min_dist){
//                    min_dist = distance;
//                }
//                if(distance > max_dist){
//                    max_dist = distance;
//                }
//            }
//            for(int i = 0; i < goodmatches.size(); ++i){
//                if(goodmatches[i].distance < ((min_dist + max_dist) / 2)){
//                    bestmatches.push_back(goodmatches[i]);
//                }
//            }


//            for(int i = 0; i < matches.size();++i){
//                if(matches[i].distance < ((min_dist + max_dist) / 2) *0.8){
//                    goodmatches.push_back(matches[i]);
//                }
//            }

//            cout << min_dist << " - " << max_dist << endl;

//            cv::drawKeypoints(referenceImage,referencePoints,rKeypointImage);
//            cv::drawKeypoints(resultImage,detectedPoints,dKeypointImage);

//            cv::imshow("ReferenceImage",rKeypointImage);
//            cv::imshow("DetectedImage",dKeypointImage);

//                referencePoints.resize(5250);
//                detectedPoints.resize(5250);

            cout << goodmatches.size() << endl;
//            cout << H.size().width << "," << H.size().height << endl;
//            cv::imshow("Matches",matchImage);
//            }
            matches.clear();
            goodmatches.clear();
            knnmatches.clear();
            bestmatches.clear();
            scene_points.clear();
            object_points.clear();

            waitKey(0);

//            if(cv::waitKey(27) >= 0) break;
//        }

    }
    if(circleTest){

        GaussianBlur(resultImage,resultImage,Size(9,9),2,2);
//        imshow("Blur",resultImage);
        vector<Vec3f> circles;
        cv::HoughCircles(resultImage,circles,CV_HOUGH_GRADIENT,2,20,40,40,5,20);

        cout << circles.size() << endl;
        for(size_t i = 0; i < circles.size();++i){
            Point center = (round(circles[i][0]),round(circles[i][1]));
            int radius = round(circles[i][2]);
            circle( resultImage, center, 3, Scalar(0,255,0), -1, 8, 0 );
            circle( resultImage, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }

//        vector<KeyPoint> keyPoints;
//        Ptr<MSER> detector = MSER.create();
//        detector->detect(resultImage,keyPoints);


        imshow("Circles",resultImage);
        waitKey(0);
    }
    if(tool){
        QApplication app(argc,argv);
        MainWindow window;
        window.show();
        return app.exec();
    }


    return 0;
}

