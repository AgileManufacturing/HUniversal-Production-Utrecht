#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>

/**
 * @brief The Camera class
 *
 * This class is responsible for keeping and setting the camera
 * position and orientation.
 */
class Camera{
public:
    Camera();

    glm::vec3 position;
    glm::vec3 facing;
    glm::vec3 up;
    glm::mat4 projection;
    glm::vec3 partOffset;

    /**
     * @brief getView
     *
     * This function returns the view matrix that is used by the 3D objects
     * in the rendering process. This matrix is caclulated based on the current
     * position and orientation.
     * @return Returns the view matrix.
     */
    glm::mat4 getView();
    /**
     * @brief setPosition
     *
     * This function sets the position of the camera.
     * @param x The x coördinate.
     * @param y The y coördinate.
     * @param z The z coördinate.
     */
    void setPosition(float x, float y, float z);
    /**
     * @brief setDirection
     *
     * This function sets the direction that the camera is facing based
     * on new pitch and yaw values. Facing downwards with the bottom of the
     * camera towards the equiplet is considered to be the base orientation.
     * @param yaw The new yaw orientation.
     * @param pitch The new pitch orientation.
     */
    void setDirection(float yaw, float pitch);

};

#endif // CAMERA_H
