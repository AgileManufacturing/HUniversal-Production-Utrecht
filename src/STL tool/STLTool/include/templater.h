#ifndef TEMPLATER_H
#define TEMPLATER_H

#include <vector>
#include <featurefactory.h>
/**
 * @brief The Templater class
 *
 * This class is responsible for generating and applying a template
 * based on a gripper.
 */
class Templater{
public:
    /**
     * @brief generateGripperTemp
     *
     * This function is responsible for generating the gripper template. It does
     * this based on the number of arms that the gripper has and the dimensions of
     * their tips. It also requires the maximum expansion distance. This minimum
     * distance is calculated based on the number of arms and their size.
     * The template is organised as follows:
     * [0-1] = distance
     * [0-1][0-35] = rotation
     * [0-1][0-35][0...n] = pixel info
     * @param arms The number of arms that the gripper has.
     * @param armHeight The height of the tip of each arm.
     * @param armWidth The width of the tip of each arm.
     * @param maxDist The maximum expansion distance.
     * @return Returns a gripper template.
     */
    static std::vector<std::vector<std::vector<int>>> generateGripperTemp
    (int arms,float armHeight,float armWidth,float maxDist);
    /**
     * @brief getGripPoint
     *
     * This function applies a generated template to an image of an object.
     * It does this by examining each pixel of the object with the template. It will
     * first attempt to fit the gripper at its minimal expansion distance templates. Only if this does
     * not fit it will attempt to fit the maximum expansion distance templates. In case the maximum
     * distance template does fit the pixel wil be deemed as a possible location to pick up the part.
     * @param image The image of an object.
     * @param gTemplate The gripper template.
     * @return Returns a pair of the best coördinate and rotation for picking up the part.
     */
    static pair<Point,int> getGripPoint(Mat &image, vector<vector<vector<int> > >& gTemplate);
};

#endif // TEMPLATER_H
