#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

#include <vector>

#include <opencv2/core/core.hpp>

#include "VisionObject.h"

using namespace std;
using namespace cv;
/**
 * @brief The ObjectDetector class
 *
 * This class provides static functions that are used to extract usefull information
 * from a camera frame.
 */
class ObjectDetector{
public:
    /**
    * @brief applyOtsuThreshold
    *
    * This function uses the openCV threshold function to apply the Otsu thresholding
    * technique to an image.
    * @param image The image that the tresholding is applied to.
    * @return Returns a thresholded image.
    */
   static Mat applyOtsuThreshold(const Mat &image);
   /**
    * @brief findConnectedComponents
    *
    * This function uses the openCV floodFill() function to attempt to extract
    * connected components from an image.
    * @param image The image that the connected components are extracted from.
    * @return Returns a vector that contains the pixel information of each object.
    */
   static vector<vector<Point>> findConnectedComponents(const Mat& image);
   /**
    * @brief getContours
    *
    * This function uses openCV to extract that are located within an image and stores
    * them in a vector.
    * @param image The image that contours are extracted from.
    * @return Returns a vector that contains pixel data of each detected contour.
    */
   static vector<vector<Point>> getContours(const Mat &image);
   /**
    * @brief getContoursHierarchy
    *
    * This function uses openCV to extract the contours that are located within an image and stores
    * them in a vector. The contours also come with aditional information which specifies whether
    * a contour has a parent or a child contour.
    * @param image The image that contours are extracted from.
    * @return Returns a pair of contours and their Heirarchy information.
    */
   static pair<vector<vector<Point>>, vector<Vec4i>> getContoursHierarchy(const Mat &image);
   /**
    * @brief getHoles
    *
    * This function extracts the contours that are to be considered holes based on contour heirarchy.
    * @param contours The contours and their corresponding heirarchy.
    * @return Returns a vector of contours that are holes.
    */
   static vector<vector<Point>> getHoles(const pair<vector<vector<Point>>, vector<Vec4i>>& contours);
   /**
    * @brief getHoles
    *
    * This function call the getContoursHierarchy() function and extracts the contours that are to be
    * considered holes based on contour heirarchy.
    * @param image The image that the contour heirarchy is extracted from.
    * @return Returns a vector of contours that are holes.
    */
   static vector<vector<Point>> getHoles(const Mat &image);
   /**
    * @brief filterObjects
    *
    * This function creates VisionObjects. This is done based on the pixel data that is extracted by the
    * findConnectedComponents() function.
    * @param objects The pixel data of al the connected components found by the findConnectedComponents() function.
    * @param image The UNALTERED image that the connected components were detected from.
    * @param debugImage An extra image that can be used to draw debug data on (CURRENTLY NOT USED).
    * @return
    */
   static vector<VisionObject> filterObjects(vector<vector<Point>>& objects,Mat& image,Mat& debugImage);


private:
};

#endif // OBJECTDETECTOR_H
