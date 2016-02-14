#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>


class Camera{
public:
    Camera();

    glm::vec3 position;
    glm::vec3 facing;
    glm::vec3 up;
    glm::mat4 projection;
    glm::vec3 partOffset;

    glm::mat4 getView();
    void setPosition(float x, float y, float z);
    void setDirection(float yaw, float pitch);

};

#endif // CAMERA_H
