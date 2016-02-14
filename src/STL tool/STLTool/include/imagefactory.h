#ifndef IMAGEFACTORY_H
#define IMAGEFACTORY_H

#include "QImage"
#include <string>

#include <opencv2/core/core.hpp>


#include "visionobject.h"

using namespace std;
using namespace cv;

//TODO: RENAME
/**
 * @brief The ImageFactory class
 *
 * This class is responsible for modifying and saving images.
 */
class ImageFactory{
public:
    /**
     * @brief save
     *
     * This function saves a QImage under the specified name.
     * @param image The QImage.
     * @param imageName The name.
     */
    static void save(QImage& image, string imageName);
    /**
     * @brief save
     *
     * This function saves an openCV image under the specified name.
     * @param image The openCV image.
     * @param imageName The name.
     */
    static void save(Mat& image, string imageName);

    static Mat applyOtsuThreshold(const Mat& image);
    /**
     * @brief normalizeImageScale
     *
     * This function rescales the image according to the model data to a point where
     * 1 pixel equals 1 mm^2
     * @param image The image that is rescaled.
     * @param modelData The model that the scaling is based on.
     * @return Returns a scaled iamge.
     */
    static Mat normalizeImageScale(const Mat& image, const vector<float>& modelData);

    /**
     * @brief filterObjects
     *
     * This function filters objects that are detected from an image.
     * (this function might better belong in the feature factory).
     * @param objects The possible VisionObjects.
     * @param image The image that contains the objects.
     * @return
     */
    static vector<VisionObject> filterObjects(vector<vector<Point> > &objects, Mat &image);
    /**
     * @brief setGrayscale
     *
     * This function grayscales a QImage.
     * @param image The QImage.
     * @return Returns a graysclaed version of the image.
     */
    static QImage setGrayscale(QImage &image);
    /**
     * @brief setGrayscale
     *
     * This function grayscaled an openCV image.
     * @param image The openCV image.
     * @return Returns a grayscaled version of the image.
     */
    static Mat setGrayscale(Mat& image);
    /**
     * @brief checkGrayscale
     *
     * This function checks whether a QImage is grayscaled.
     * @param image The image that is checked.
     * @return Returns whether the image is grayscaled or not.
     */
    static bool checkGrayscale(QImage &image);

};

#endif // IMAGEFACTORY_H
