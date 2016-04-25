#include "camera.h"

#include "glm/gtc/matrix_transform.hpp"

Camera::Camera():
    position{glm::vec3(0.0f,0.0f,400.0f)},
    facing{glm::vec3(0.0f,0.0f,-1.0f)},
    up{glm::vec3(0.0f,1.0f,0.0f)},
    projection{glm::perspective(45.0f,640.0f/480.0f,0.1f,500.0f)}
{}

glm::mat4 Camera::getView(){
    return glm::lookAt(position, position + facing, up);
}

