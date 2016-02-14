#ifndef GLSURFACE_H
#define GLSURFACE_H

#include "opengl.h"

#include "QOffscreenSurface"
#include "QOpenGLContext"

#include "stlparser.h"

#include "camera.h"
#include "stlobject.h"
#include "workplane.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace OpenGL;
using namespace cv;

class GLSurface{
public:
    GLSurface();

    Mat renderGL();
    
    bool workplaneState() const;
    
    void loadSTLObject(std::vector<float> &data);
    void initializeGL();

    void setWorkplaneState(bool workplaneState);
    void switchShadeState();

    void executeTool(int argc, char *argv[]);

private:
    STLParser parser;

    QOffscreenSurface* m_surface;
    QOpenGLContext* m_context;
    
    Camera* m_camera;
    STLObject* m_stlObject;
    Workplane* m_workplane;
    
    bool m_workplaneState;
    
    texture2D texture;
    Framebuffer m_fbo;
    RenderBuffer m_rbo;

    Mat m_image;
    
};

#endif // GLSURFACE_H
