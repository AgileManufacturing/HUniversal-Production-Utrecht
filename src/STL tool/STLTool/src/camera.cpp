#include "camera.h"

#include "glm/gtc/matrix_transform.hpp"
#include "glm/ext.hpp"

Camera::Camera():
    position{glm::vec3(0.0f,0.0f,200.0f)},
    facing{glm::vec3(0.0f,0.0f,-1.0f)},
    up{glm::vec3(0.0f,1.0f,0.0f)},
    projection{glm::perspective(45.0f,640.0f/480.0f,0.1f,1000.0f)},
    partOffset{glm::vec3(0.0f,0.0f,0.0f)}
{}

glm::mat4 Camera::getView(){
    return glm::lookAt(position + partOffset, (position + partOffset) + facing, up);
}

void Camera::setPosition(float x, float y, float z){
    position = glm::vec3(x,y,z);
}

void Camera::setDirection(float yaw, float pitch){
    glm::vec3 cameraDirection = glm::vec3{0.0f,0.0f,-1.0f};
    cameraDirection = glm::rotateY(cameraDirection,glm::radians(yaw));
    cameraDirection = glm::rotateX(cameraDirection,glm::radians(pitch));
    cameraDirection = glm::normalize(cameraDirection);
    facing = cameraDirection;
}

