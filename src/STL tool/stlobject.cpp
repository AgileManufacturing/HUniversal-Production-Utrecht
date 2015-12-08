#include "stlobject.h"

#include "glm/ext.hpp"

#include <iostream>

STLObject::STLObject(Camera* camera):
    m_vertexShader{"STLObject vertex shader"},
    m_fragmentShader{"STLObject fragment shader"},
    m_camera(camera)
{}

void STLObject::initialize(){
    m_vao.create();
    m_vao.bind();

    m_vbo.create(GL_ARRAY_BUFFER);
    m_vbo.bind();

    m_vertexShader.create(GL_VERTEX_SHADER);
    m_vertexShader.addSourceFile("STLObject.vert");
    m_vertexShader.compile();

    m_fragmentShader.create(GL_FRAGMENT_SHADER);
    m_fragmentShader.addSourceFile("STLObject.frag");
    m_fragmentShader.compile();

    m_objectProgram.create();
    m_objectProgram.attachShader(m_vertexShader.getHandle());
    m_objectProgram.attachShader(m_fragmentShader.getHandle());
    m_objectProgram.bindFragData(0,"outColor");
    m_objectProgram.link();
    m_objectProgram.use();

    m_vao.enableVertexAttrib(m_objectProgram,"position",
                           3,GL_FLOAT,GL_FALSE,6 * sizeof(GLfloat), nullptr);

    m_vao.enableVertexAttrib(m_objectProgram,"normal",
                           3,GL_FLOAT,GL_FALSE,6 * sizeof(GLfloat), (void*)(3 * sizeof(float)));

    m_uniFragColor = m_objectProgram.getUniform("fragColor");
    m_uniProj = m_objectProgram.getUniform("proj");
    m_uniView = m_objectProgram.getUniform("view");
    m_uniTrans = m_objectProgram.getUniform("trans");

    glUniform3f(m_uniFragColor, 1.0,1.0,1.0);

    m_vao.release();
}

void STLObject::draw(){
    m_objectProgram.use();
    m_vao.bind();

    glUniformMatrix4fv(m_uniTrans,1,GL_FALSE,glm::value_ptr(m_trans));
    glUniformMatrix4fv(m_uniProj,1,GL_FALSE,glm::value_ptr(m_camera->projection));
    glm::mat4 view = m_camera->getView();
    glUniformMatrix4fv(m_uniView,1,GL_FALSE,glm::value_ptr(view));

    glDrawArrays(GL_TRIANGLES,0, m_verticeCount);
    m_vao.release();
    m_objectProgram.release();
}

void STLObject::setData(vector<float> &data){
    m_verticeCount = data.size() / 6;
    m_vbo.bind();
    m_vbo.bindData(data,GL_STATIC_DRAW);
}

bool STLObject::isCreated(){
    return m_verticeCount != 0;
}
