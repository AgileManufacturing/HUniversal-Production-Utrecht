#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "OpenGL/opengl.h"

#include <chrono>
#include <QOpenGLWidget>
#include <QOpenGLContext>
#include <QTimer>
#include "workplane.h"
#include "stlobject.h"



class GLWidget : public QOpenGLWidget{
public:
    GLWidget(float width, float height);
    ~GLWidget();

    void setCamera(glm::vec3 position, glm::vec3 direction);

    void initializeGL() override;
    void paintGL() override;
    void keyPressEvent(QKeyEvent* e) override;

    QSize sizeHint() const;

    void loadSTLObject(std::vector<float>& data);
    void enableWorkplane(bool newWorkplaneState);
private:
    Camera* camera;
    Workplane* workplane;
    STLObject* m_stlObject;

    QTimer drawTimer;


    std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();

    float pitch = 0,yaw = -90;
    bool m_activeWorkplane;

    //Test attributes

    glm::mat4 trans;
};

#endif
