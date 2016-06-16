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

#include <iostream>

using namespace std;
using namespace cv;

bool tool = true;

int main(int argc, char *argv[]){
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

