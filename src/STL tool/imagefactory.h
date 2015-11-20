#ifndef IMAGEFACTORY_H
#define IMAGEFACTORY_H

#include "QImage"
#include <string>

using namespace std;


class ImageFactory
{
public:
    ImageFactory();

    void save(QImage& image, string imageName);

    QImage setGrayscale(QImage &image);
    bool checkGrayscale(QImage &image);
private:

};

#endif // IMAGEFACTORY_H
