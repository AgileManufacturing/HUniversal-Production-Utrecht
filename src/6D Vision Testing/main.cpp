#include <iostream>
#include <QApplication>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/video.hpp>

#include "QInputDialog"
#include "QDir"

//#define CAMERAFEED
#define IMAGETAKER

using namespace std;

bool camerafeed = true;

int main(int argc, char *argv[]){
#ifdef CAMERAFEED
    QApplication app(argc,argv);
    cv::VideoCapture cap(1);
    cv::Mat frame;
    while(true){

    cap >> frame;

    cv::imshow("Test",frame);
    if(cv::waitKey(30) >= 0) break;
    }
    return 0;
#endif
#ifdef IMAGETAKER
    QApplication app(argc,argv);
    QWidget inputwidget;
    string testDir = "/testimages/";
    string imageName;

    bool succes;
    cv::namedWindow("Test",cv::WINDOW_AUTOSIZE);
    //Camera is chosen here
    cv::VideoCapture cap(1);
    cv::Mat frame;
    bool takeImage = false;
    while(true){
        while(!takeImage){
            cap >> frame;
            cv::imshow("Test",frame);
            
            switch(cv::waitKey(30)){
            case 116:
                takeImage = true;
                break;
            case 27:
                return 0;
            }
        }
        imageName = QInputDialog::getText(&inputwidget,"Save","Image name:",QLineEdit::Normal,"",&succes).toStdString() + ".bmp";
        if(succes){
            cv::imwrite(QDir::currentPath().toStdString() + testDir + imageName,frame);
        }
        takeImage = false;
    }

#endif
}

