#include "glwidget.h"
#include "QPainter"
#include "GL/glew.h"
#include <iostream>
#include <string>
#include "QOpenGLContext"
#include <QKeyEvent>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/ext.hpp"

#include <QLayout>

using namespace std;

GLWidget::GLWidget(float width,float height):
//    fragmentShader{"ObjectFragmentShader"},
//    vertexShader{"ObjectVertexShader"},
    camera{new Camera},
    workplane{new Workplane(camera)},
    m_stlObject{new STLObject(camera)},
    m_activeWorkplane{false}
    //TODO: Aspect ratio fixen
{

    resize(width,height);
    QSurfaceFormat fmt = format();
    fmt.setDepthBufferSize(32);
    fmt.setVersion(3,3);
    setFormat(fmt);
    
    connect(&drawTimer,SIGNAL(timeout()),this,SLOT(update()));
    drawTimer.start(static_cast<float>(1000/60));
}

GLWidget::~GLWidget(){
    delete workplane;
    delete m_stlObject;
    delete camera;
}

void GLWidget::setCamera(glm::vec3 position, glm::vec3 direction){
    camera->position = position;
    camera->facing = direction;
}

void GLWidget::initializeGL(){
    GLenum error = glewInit();
    if (error != GLEW_OK) {
        qFatal("Failed to initialize OpenGL: "
               "failed to initialize GLEW (%s)", glewGetErrorString(error));
    }
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1, 0.1, 0.1, 1);

    workplane->initialize();
    m_stlObject->initialize();
}

void GLWidget::paintGL(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(m_activeWorkplane){
        workplane->draw();
    }
    if(m_stlObject->isCreated()){
        m_stlObject->draw();
    }
}

void GLWidget::loadSTLObject(std::vector<float> &data){
    m_stlObject->setData(data);
}

void GLWidget::enableWorkplane(bool newWorkplaneState){
    m_activeWorkplane = newWorkplaneState;
}

QSize GLWidget::sizeHint() const{
    return QSize(width(),height());
}
void GLWidget::keyPressEvent(QKeyEvent* e){
    switch(e->key()) {
        case Qt::Key_W :
            cout << "Pressing W" << endl;
            camera->position += glm::vec3(0.0f,0.0f,2.0f);
            camera->facing += glm::vec3(0.0f,0.0f,2.0f);
            break;
        case Qt::Key_Up:
            cout << "Pressing up arrow" << endl;
            m_stlObject->m_trans = glm::rotate(trans,glm::radians(3.0f),glm::vec3(1.0f,0.0f,0.0f));
            break;
        case Qt::Key_A:
            cout << "Pressing A" << endl;
            camera->position += glm::vec3(-2.0f,0.0f,0.0f);
            camera->facing += glm::vec3(-2.0f,0.0f,0.0f);
            break;
        case Qt::Key_Left:
            cout << "Pressing left arrow" << endl;
            m_stlObject->m_trans  = glm::rotate(trans,glm::radians(3.0f),glm::vec3(0.0f,1.0f,0.0f));
            break;
        case Qt::Key_S:
            cout << "Pressing S" << endl;
            camera->position += glm::vec3(0.0f,0.0f,-2.0f);
            camera->facing += glm::vec3(0.0f,0.0f,-2.0f);
            break;
        case Qt::Key_Down:
            cout << "Pressing down arrow" << endl;
            m_stlObject->m_trans  = glm::rotate(trans,glm::radians(-3.0f),glm::vec3(1.0f,0.0f,0.0f));
            break;
        case Qt::Key_D:
            cout << "Pressing D" << endl;
            camera->position += glm::vec3(2.0f,0.0f,0.0f);
            camera->facing += glm::vec3(2.0f,0.0f,0.0f);
            break;
        case Qt::Key_Right:
            cout << "Pressing right arrow" << endl;
            m_stlObject->m_trans  = glm::rotate(trans,glm::radians(-3.0f),glm::vec3(0.0f,1.0f,0.0f));
            break;
        case Qt::Key_X:
            cout << "Pressing X" << endl;
            break;
    }
}
