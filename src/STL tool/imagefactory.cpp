#include <iostream>
#include "imagefactory.h"
#include "QDir"
#include "QColor"

ImageFactory::ImageFactory()
{}

void ImageFactory::save(QImage& image,string imageName){
    string savePath = "/images/" + imageName + ".jpg";
    if(QDir(QDir::currentPath() + "/images").exists()){
        if(image.save(QDir::currentPath() + QString::fromStdString(savePath))){
            cout << "Image saved succesfully" << endl << QDir::currentPath().toStdString() << savePath << endl;
        }else{
            cout << "Image save failed :(" << endl;
        }
    }else{
        QDir().mkdir(QDir::currentPath() + "/images");
        if(image.save(QDir::currentPath() + QString::fromStdString(savePath))){
            cout << "Image saved succesfully" << endl << QDir::currentPath().toStdString() << savePath << endl;
        }else{
            cout << "Image save failed :(" << endl;
        }
    }
}

QImage ImageFactory::setGrayscale(QImage& image){
    int gray, pixelSize = 4;
    unsigned char* pixelLine;
    for(int i = 0; i < image.height(); ++i){
        pixelLine = image.scanLine(i);
        for(int j = 0; j < image.width(); ++j){
            QRgb* rgbPixel = reinterpret_cast<QRgb*>(pixelLine + (j * pixelSize));
            gray = qGray(*rgbPixel);
            *rgbPixel = QColor(gray,gray,gray).rgba();
        }
    }
    return image;
}

bool ImageFactory::checkGrayscale(QImage& image){
    return image.allGray();
}

