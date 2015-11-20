#ifndef GLSURFACE_H
#define GLSURFACE_H

#include "GL/glew.h"

#include "QOffscreenSurface"
#include "QOpenGLContext"

#include "camera.h"
#include "stlobject.h"
#include "workplane.h"

class GLSurface{
public:
    GLSurface();

    void renderGL();
    
    bool workplaneState() const;
    void setWorkplaneState(bool workplaneState);
    
private:
    QOffscreenSurface* m_surface;
    QOpenGLContext* m_context;
    
    Camera* m_camera;
    STLObject* m_stlObject;
    Workplane* m_workplane;
    
    void initializeGL();
    
    bool m_workplaneState;
    
};

#endif // GLSURFACE_H
