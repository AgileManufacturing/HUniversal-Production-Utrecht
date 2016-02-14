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
/**
 * @brief The GLSurface class
 *
 * This class is responsible for the offscreen rendering process.
 */
class GLSurface{
public:
    /**
     * @brief GLSurface constructor
     *
     * The constructor sets up the context and the surface that OpenGL
     * requires to draw.
     */
    GLSurface();
    /**
     * @brief renderGL
     *
     * This function calls the draw functions of the 3D objects. Before the draws
     * are called the framebuffer has to be bound otherwise it wont be drawn to the
     * framebuffer.
     * @return Returns an openCV image that is created from the rendered data.
     */
    Mat renderGL();
    /**
     * @brief workplaneState
     * @return Returns the active state of the workplane.
     */
    bool workplaneState() const;
    /**
     * @brief loadSTLObject
     *
     * This function loads al the vertice data into the STL object.
     * @param data The vertice data.
     */
    void loadSTLObject(std::vector<float> &data);

    /**
     * @brief initializeGL
     *
     * Initialises al the openGL buffers.
     */
    void initializeGL();
    /**
     * @brief setWorkplaneState
     *
     * This function changes the state of the workplane. This determines
     * whether or not it is drawn.
     * @param workplaneState The new state of the workplane.
     */
    void setWorkplaneState(bool workplaneState);
    /**
     * @brief switchShadeState
     *
     * Calling this function will switch between including or not including
     * diffuse lighting in the rendering.
     */
    void switchShadeState();
    /**
     * @brief executeTool
     *
     * This function executes the rendering process based on the arguments that
     * are given to the function. In order to set the resolution the argument after
     * the name of the STL file has to be -r followed by the width and height in pixels.
     * for example: -r 1920 1080
     * After the name and (optionally) resolution have been specified the function
     * requires camera setups based on x y z coördinates and pitch and yaw rotations. This
     * function does not have a limit regarding how many cameras are specified.
     * @param argc the number of arguments.
     * @param argv the array of arguments.
     */
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
