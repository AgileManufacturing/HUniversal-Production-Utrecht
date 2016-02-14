#include "stlobject.h"
#include <cmath>

#include "glm/ext.hpp"

#include <iostream>

STLObject::STLObject(Camera* camera):
    m_vertexShader{"STLObject vertex shader"},
    m_fragmentShader{"STLObject fragment shader"},
    m_camera(camera),
    m_shadeState{true}
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
    m_shadeBoolean = m_objectProgram.getUniform("shading");
    m_uniProj = m_objectProgram.getUniform("proj");
    m_uniView = m_objectProgram.getUniform("view");
    m_uniTrans = m_objectProgram.getUniform("trans");
    glUniform1i(m_shadeBoolean,1);
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

void STLObject::switchShading(){
    m_objectProgram.use();
    if(!m_shadeState){
        glUniform1i(m_shadeBoolean,1);
        m_shadeState = true;
    }else{
        glUniform1i(m_shadeBoolean,0);
        m_shadeState = false;
    }
    m_objectProgram.release();
}

void STLObject::setData(vector<float> &data){
    modelData = data;
    m_verticeCount = modelData.size() / 6;
    m_vbo.bind();
    m_vbo.bindData(data,GL_STATIC_DRAW);
    //TODO: AUTOMATICLY CENTER OBJECT
    int minX = INFINITY,maxX = -INFINITY;
    int minY = INFINITY,maxY = -INFINITY;
    for(int i = 0; i < modelData.size();i+=6){
        minX = (modelData[i] < minX)? modelData[i] : minX;
        maxX = (modelData[i] > maxX)? modelData[i] : maxX;
        minY = (modelData[i+1] < minY)? modelData[i+1] : minY;
        maxY = (modelData[i+1] > maxY)? modelData[i+1] : maxY;
    }
    m_camera->partOffset = glm::vec3((maxX-minX)/2,(maxY-minY)/2,0);
}

vector<float> STLObject::getData(){
    return modelData;
}

bool STLObject::isCreated(){
    return m_verticeCount != 0;
}
