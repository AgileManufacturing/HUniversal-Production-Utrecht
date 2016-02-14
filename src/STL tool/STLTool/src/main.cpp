#include "glm/glm.hpp"

#include <QApplication>

#include "mainwindow.h"
#include "glsurface.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/video.hpp>

using namespace std;
using namespace cv;

bool tool = true;
//Enabling this wil enable part of the tool that takes images from the specified camera
//bool imagetaker = true;

int main(int argc, char *argv[]){
//    if(imagetaker){
//        QApplication app(argc,argv);
//        QWidget inputwidget;
//        string testDir = "/namedimages/";
//        string imageName;

//        bool succes;
//        cv::namedWindow("Test",cv::WINDOW_AUTOSIZE);
//        //Camera is chosen here
//        cv::VideoCapture cap(1);
//        cv::Mat frame;
//        for(int i = 0;; ++i){
//            cv::waitKey(0);
//            cout << i << endl;
//            for(int j = 0; j < 6; ++j){
//                cap >> frame;
//            }
//            cv::imshow("Test",frame);

//            imageName = QInputDialog::getText(&inputwidget,"Save","Image name:",QLineEdit::Normal,"",&succes).toStdString() + ".bmp";
//            if(succes){
//                cv::imwrite(QDir::currentPath().toStdString() + testDir + imageName,frame);
//            }
//        }
//    }
    if(argc > 3){
        QApplication app(argc,argv);
        GLSurface offscreenSurface;
        offscreenSurface.executeTool(argc,argv);
        return 0;
        return app.exec();
    }else if(tool){
        QApplication app(argc,argv);
        MainWindow window;
        window.show();
        return app.exec();
    }
    return 0;
}

