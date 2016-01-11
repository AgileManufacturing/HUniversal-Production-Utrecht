#ifndef WORKPLANE_H
#define WORKPLANE_H
#include "OpenGL/opengl.h"

#include <string>
#include <vector>


#include "camera.h"

using namespace std;

class Workplane{
public:
    Workplane(Camera* camera);

    void create(glm::vec2 gridSize, glm::vec2 cellSize);

    void setGridColors(glm::vec3 lightColor, glm::vec3 darkColor);

    string getVertexSource();
    string getFragmentSource();

    vector<float> getWorkplaneData();

    GLint getWorkplaneProgram();

    void draw();
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
