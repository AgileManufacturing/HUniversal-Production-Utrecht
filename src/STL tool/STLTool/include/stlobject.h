#ifndef STLOBJECT_H
#define STLOBJECT_H

#include <vector>

#include "opengl.h"

#include "camera.h"

using namespace std;

class STLObject{
public:
    STLObject(Camera* camera);

    void initialize();

    void draw();
    //TODO: Automaticly center
    void setData(vector<float>& data);
    vector<float> getData();

    bool isCreated();

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
