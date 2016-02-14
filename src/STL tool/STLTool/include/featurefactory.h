#ifndef FEATUREFACTORY_H
#define FEATUREFACTORY_H

#include "imagefactory.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "part.h"

using namespace std;
using namespace cv;

//TODO: RENAME
/**
 * @brief The FeatureFactory class
 */
class FeatureFactory{
public:
    //Maybe move to imagefactory
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
     * @brief savePartConfig
     *
     * This function creates and saves a part configuration file.
     * @param parameterMap The parameter map that is made into a configuration file.
     * @param partName The part name.
     */
    static void savePartConfig(const map<string, double> &parameterMap, const QString partName);
    /**
     * @brief savePartConfig
     *
     * This function creates and saves a part configuration file.
     * @param part The part that is made into a configuration file.
     */
    static void savePartConfig(const Part& part);
    /**
     * @brief createParameterMap
     *
     * This function extracts features from a VisionObject and puts them
     * into a parameterMap
     * @param object The VisionObject.
     * @return Returns a parameter map based on the VisionObject.
     */
    static map<string,double> createParameterMap(const VisionObject &object);
    /**
     * @brief createParameterMap
     *
     * Creates a parameter based on a VisionObject. This function also includes
     * the gripper templating that determines where a part can be picked up with a
     * gripper.
     * @param object The VisionObject.
     * @param modelData The vertices from a 3D model.
     * @param gTemplate The template that is used for the gripper.
     * @return Returns a parameter map with grip location based on the VisionObject.
     */
    static map<string, double> createParameterMap(const VisionObject &object,
                                                  vector<float> &modelData,
                                                  vector<vector<vector<int> > > &gTemplate);
    /**
     * @brief getContours
     *
     * This function uses openCV to extract that are located within an image and stores
     * them in a vector.
     * @param image The image that contours are extracted from.
     * @return Returns a vector that contains pixel data of each detected contour.
     */
    static vector<vector<Point>> getContours(const Mat& image);
    /**
     * @brief getContoursHierarchy
     *
     * This function uses openCV to extract the contours that are located within an image and stores
     * them in a vector. The contours also come with aditional information which specifies whether
     * a contour has a parent or a child contour.
     * @param image The image that contours are extracted from.
     * @return Returns a pair of contours and their Heirarchy information
     */
    static pair<vector<vector<Point>>, vector<Vec4i>> getContoursHierarchy(const Mat& image);
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
    static vector<vector<Point>> getHoles(const Mat& image);
    /**
     * @brief findCenter
     *
     * This function determines the center of a connected component based on pixel coördinates.
     * @param blob The connected component.
     * @return The coördinates of the center.
     */
    static Point findCenter(const vector<Point> blob);
    /**
     * @brief findCenterOfMass
     *
     * This function determines the center of a connected component based on pixel coördinates.
     * @param blob The connected component.
     * @return The coördinates of the center.
     */
    static Point findCenterOfMass(const vector<Point> blob);
    /**
     * @brief extractCurves
     *
     * Test function that isent used. It extracts the curves of a bigger
     * curve.
     * @param curve The curve that the curves are extracted from.
     * @return Returns the extracted curves.
     */
    static vector<vector<Point>> extractCurves(vector<Point>& curve);
    /**
     * @brief getEuclideanDistance
     *
     * This function determines the euclidean distance between two points.
     * @param p1 The first point.
     * @param p2 The second point.
     * @return Returns the euclidean distance between p1 and p2.
     */
    static double getEuclideanDistance(Point& p1,Point& p2);

};


#endif // FEATUREFACTORY_H
