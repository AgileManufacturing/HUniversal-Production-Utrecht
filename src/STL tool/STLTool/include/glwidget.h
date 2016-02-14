#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "opengl.h"

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

    void setCamera(float x, float y, float z, float yaw, float pitch);

    void initializeGL() override;
    void paintGL() override;

    QSize sizeHint() const;

    void loadSTLObject(std::vector<float>& data);

    void enableWorkplane(bool newWorkplaneState);
    void switchShading();
    void save(QString imageName);

//    void keyPressEvent(QKeyEvent* e) override;
private:
    Camera* camera;
    Workplane* workplane;
    STLObject* m_stlObject;

    QTimer drawTimer;

    bool m_activeWorkplane;

    //Test attributes
    //glm::mat4 trans;
};

#endif
