#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "opengl.h"

#include <chrono>
#include <QOpenGLWidget>
#include <QOpenGLContext>
#include <QTimer>
#include "workplane.h"
#include "stlobject.h"
/**
 * @brief The GLWidget class
 *
 * This class is responsible for the main widget of the STL tool. OpenGL
 * will draw onto this widget when the tool is used with a GUI.
 */
class GLWidget : public QOpenGLWidget{
public:
    /**
     * @brief GLWidget
     *
     * The constructor of the openGL widget.
     * @param width Defines the width of the widget.
     * @param height Defines the height of the widget.
     */
    GLWidget(float width, float height);
    ~GLWidget();
    /**
     * @brief setCamera
     *
     * This function call the setPosition and setDirection functions of the camera.
     * @param x The new x coördinate of the camera.
     * @param y The new y coördinate of the camera.
     * @param z The new z coördinate of the camera.
     * @param yaw The new yar rotation of the camera.
     * @param pitch The new pitch rotation of the camera.
     */
    void setCamera(float x, float y, float z, float yaw, float pitch);
    /**
     * @brief initializeGL
     *
     * This function initialises OpenGL.
     */
    void initializeGL() override;
    /**
     * @brief paintGL
     *
     * This function calls al the draw functions.
     */
    void paintGL() override;
    /**
     * @brief sizeHint
     *
     * Qt requires this to be defined in order to set the
     * widget dimensions.
     * @return The size of the widget.
     */
    QSize sizeHint() const;

    /**
     * @brief loadSTLObject
     *
     * This function passes the vertice data to the STL object.
     * @param data The vertice data.
     */
    void loadSTLObject(std::vector<float>& data);
    /**
     * @brief enableWorkplane
     *
     * This function sets the active state of the workplane. This determines
     * whether it is drawn or not.
     * @param newWorkplaneState The new workplane state.
     */
    void enableWorkplane(bool newWorkplaneState);
    /**
     * @brief switchShading
     *
     * Switches between including the including or not including the diffuse lighting
     * in the rendering.
     */
    void switchShading();
    /**
     * @brief save
     *
     * This function saves the rendered image and saves a part config file based on the part
     * that was rendered.
     * @param imageName The name that both the file and image are saved as.
     */
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
