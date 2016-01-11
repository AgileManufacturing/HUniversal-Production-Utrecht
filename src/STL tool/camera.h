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

    glm::mat4 getView();

};

#endif // CAMERA_H
