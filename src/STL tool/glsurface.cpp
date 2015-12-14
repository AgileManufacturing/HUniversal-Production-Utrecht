#include "glsurface.h"
#include <iostream>
#include "QOpenGLFunctions_3_3_Core"
#include <QDebug>
GLSurface::GLSurface():
    m_surface{new QOffscreenSurface},
    m_context{new QOpenGLContext},
    m_camera{new Camera},
    m_workplane{new Workplane(m_camera)},
    m_stlObject{new STLObject(m_camera)},
    m_workplaneState{false},
    m_rbo{640,480}
{
    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(32);
    fmt.setVersion(3,3);

    m_context->setFormat(fmt);
    m_context->create();

    m_surface->setFormat(fmt);
    m_surface->create();

    m_context->makeCurrent(m_surface);
    initializeGL();
}

void GLSurface::renderGL(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_fbo.bind(GL_DRAW_FRAMEBUFFER);

    
    if(m_workplaneState){
        m_workplane->draw();
    }
    if(m_stlObject->isCreated()){
        m_stlObject->draw();
    }
    vector<int> data(m_rbo.getBufferSize());
    m_fbo.read(data,m_rbo);
    cout <<"Data?: " << data.size() << endl;
    for(int i = 0; i < 100;++i){
        cout << data[i] << endl;
    }
    m_fbo.release();
}

void GLSurface::initializeGL(){
    GLenum error = glewInit();
    if (error != GLEW_OK) {
        qFatal("Failed to initialize OpenGL: "
               "failed to initialize GLEW (%s)", glewGetErrorString(error));
    }
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1, 0.1, 0.1, 1);

    m_fbo.create();
    m_fbo.bind(GL_DRAW_FRAMEBUFFER);
    m_rbo.create();
    m_rbo.bind();
    m_rbo.setStorage();
    m_fbo.bindRenderBuffer(m_rbo);
    m_workplane->initialize();
    m_stlObject->initialize();
}

void GLSurface::loadSTLObject(std::vector<float> &data){
    m_stlObject->setData(data);
}

bool GLSurface::workplaneState() const{
    return m_workplaneState;
}

void GLSurface::setWorkplaneState(bool workplaneState){
    m_workplaneState = workplaneState;
}

