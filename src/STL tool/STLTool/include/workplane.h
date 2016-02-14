#ifndef WORKPLANE_H
#define WORKPLANE_H
#include "opengl.h"

#include <string>
#include <vector>


#include "camera.h"

using namespace std;
/**
 * @brief The Workplane class
 *
 * This class represents the simulated workplane in the rendering.
 */
class Workplane{
public:
    /**
     * @brief Workplane constructor
     * @param camera Camera Contains information regarding the current camera setup.
     */
    Workplane(Camera* camera);
    /**
     * @brief create
     *
     * This function generates the vertices that are required to draw
     * the workplane with openGL.
     * @param gridSize The size of the grid.
     * @param cellSize The size of each cell.
     */
    void create(glm::vec2 gridSize, glm::vec2 cellSize);
    /**
     * @brief setGridColors
     *
     * This function adjusts the colour of the cells of the checkerboard like
     * workplane.
     * @param lightColor The light colour of the checkerboard
     * @param darkColor The dark colour of the checkerboard.
     */
    void setGridColors(glm::vec3 lightColor, glm::vec3 darkColor);
    /**
     * @brief getWorkplaneData
     * @return Returns the vertice data of the work plane.
     */
    vector<float> getWorkplaneData();
    /**
     * @brief isCreated
     * @return Returns whether the workplane has been initialised and created.
     */
    bool isCreated();
    /**
     * @brief draw
     *
     * This function is responsible for calling the openGL draw function.
     */
    void draw();
    /**
     * @brief initialize
     *
     * This function is responsible for setting up the openGL buffers that are
     * needed in order to draw the workplane.
     */
    void initialize();

private:
    Camera* m_camera;
    vector<float> generateGridCell(glm::vec3 startPos, glm::vec2 cellSize, glm::vec4 color);

    glm::vec4 m_lightGridColor, m_darkGridColor;

    vector<float> m_workplaneData;

    GLint m_uniProj, m_uniView;

    OpenGL::Shader m_vertexShader;
    OpenGL::Shader m_fragmentShader;
    OpenGL::VertexArray m_vao;
    OpenGL::VertexBuffer m_vbo;
    OpenGL::ShaderProgram m_workplaneProgram;
};

#endif // WORKPLANE_H
