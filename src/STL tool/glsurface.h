#ifndef GLSURFACE_H
#define GLSURFACE_H

#include "OpenGL/opengl.h"

#include "QOffscreenSurface"
#include "QOpenGLContext"

#include "camera.h"
#include "stlobject.h"
#include "workplane.h"

using namespace OpenGL;

class GLSurface{
public:
    GLSurface();

    void renderGL();
    
    bool workplaneState() const;
    void setWorkplaneState(bool workplaneState);
    
    void loadSTLObject(std::vector<float> &data);
    void initializeGL();
private:

    QOffscreenSurface* m_surface;
    QOpenGLContext* m_context;
    
    Camera* m_camera;
    STLObject* m_stlObject;
    Workplane* m_workplane;
    
    bool m_workplaneState;
    
    Framebuffer m_fbo;
    RenderBuffer m_rbo;
    
};

#endif // GLSURFACE_H
