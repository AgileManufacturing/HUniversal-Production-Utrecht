#ifndef STLOBJECT_H
#define STLOBJECT_H

#include <vector>

#include "OpenGL/opengl.h"

#include "camera.h"

using namespace std;

class STLObject{
public:
    STLObject(Camera* camera);

    void initialize();

    void draw();

    void setData(vector<float>& data);

    string getVertexSource();
    string getFragmentSource();

    bool isCreated();

    glm::mat4 m_trans;
private:
    Camera* m_camera;

    string m_vertexSource, m_fragmentSource;
    GLuint m_verticeCount = 0;

    GLuint m_uniFragColor,m_uniView,m_uniProj,m_uniTrans;

    OpenGL::Shader m_vertexShader;
    OpenGL::Shader m_fragmentShader;
    OpenGL::VertexArray m_vao;
    OpenGL::VertexBuffer m_vbo;
    OpenGL::ShaderProgram m_objectProgram;
};

#endif // STLOBJECT_H
