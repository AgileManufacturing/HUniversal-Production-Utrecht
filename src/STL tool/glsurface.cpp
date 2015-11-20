#include "glsurface.h"

GLSurface::GLSurface():
    m_surface{new QOffscreenSurface},
    m_context{new QOpenGLContext},
    m_camera{new Camera},
    m_workplane{new Workplane(m_camera)},
    m_stlObject{new STLObject(m_camera)},
    m_workplaneState{false}
{
    m_surface->create();
    m_context->makeCurrent(m_surface);
    initializeGL();

    
}

void GLSurface::renderGL(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    if(m_workplaneState){
        m_workplane->draw();
    }
    if(m_stlObject->isCreated()){
        m_stlObject->draw();
    }
}

void GLSurface::initializeGL(){
    GLenum error = glewInit();
    if (error != GLEW_OK) {
        qFatal("Failed to initialize OpenGL: "
               "failed to initialize GLEW (%s)", glewGetErrorString(error));
    }
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1, 0.1, 0.1, 1);

    m_workplane->initialize();
    m_stlObject->initialize();
}

bool GLSurface::workplaneState() const{
    return m_workplaneState;
}

void GLSurface::setWorkplaneState(bool workplaneState){
    m_workplaneState = workplaneState;
}

