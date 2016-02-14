#ifndef STLOBJECT_H
#define STLOBJECT_H

#include <vector>

#include "opengl.h"

#include "camera.h"

using namespace std;
/**
 * @brief The STLObject class
 *
 * This class represents the 3D object that is drawn
 * from data that is parsed from STL files.
 */
class STLObject{
public:
    /**
     * @brief STLObject constructor
     * @param camera Contains information regarding the current camera setup.
     */
    STLObject(Camera* camera);
    /**
     * @brief initialize
     *
     * This function initialises al the buffers that are needed by
     * openGL in order to draw the STL object.
     */
    void initialize();
    /**
     * @brief draw
     *
     * This function updates the positional values of the object and
     * then calls the opengl draw function.
     */
    void draw();
    /**
     * @brief setData
     *
     * This updates the model data with new vertices.
     * @param data The new vertices.
     */
    void setData(vector<float>& data);

    /**
     * @brief getData
     * @return Returns the model data
     */
    vector<float> getData();
    /**
     * @brief isCreated
     * @return Returns whether the object is initialised and has valid model data.
     */
    bool isCreated();
    /**
     * @brief switchShading
     *
     * This function switched between including and not including the diffuse lighting in
     * the rendering.
     */
    void switchShading();


//TODO: i dont think trans should be public
    glm::mat4 m_trans;
private:
    vector<float> modelData;
    bool m_shadeState;

    Camera* m_camera;

    GLuint m_verticeCount = 0;

    GLuint m_uniFragColor,m_uniView,m_uniProj,m_uniTrans,m_shadeBoolean;

    OpenGL::Shader m_vertexShader;
    OpenGL::Shader m_fragmentShader;
    OpenGL::VertexArray m_vao;
    OpenGL::VertexBuffer m_vbo;
    OpenGL::ShaderProgram m_objectProgram;
};

#endif // STLOBJECT_H
