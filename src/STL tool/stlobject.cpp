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

    m_vertexSource =
            "#version 330\n"
            "uniform mat4 trans;\n"
            "uniform mat4 view;\n"
            "uniform mat4 proj;\n"
            "in vec3 position;\n"
            "in vec3 normal;\n"
            "out vec3 outNormal;\n"
            "void main() {\n"
            "   gl_Position = proj * view * (trans * vec4(position,1.0));\n"
            "   outNormal = ( view * trans * vec4(normal,0)).xyz;\n"
            "}\n";

    m_fragmentSource =
            "#version 330\n"
            "uniform vec3 fragColor;\n"
            "in vec3 outNormal;\n"
            "in vec3 testColor;\n"
            "out vec4 outColor;\n"
            "void main() {\n"
            "   vec3 lightDirection = vec3(0,0,-1);\n"
            "   float diffuseFactor = max(dot(-lightDirection,outNormal), 0);\n"
            "   outColor = vec4(fragColor,1.0) * diffuseFactor;\n"
            "}\n";

    m_vertexShader.create(GL_VERTEX_SHADER);
    m_vertexShader.addSource(m_vertexSource.data());
    m_vertexShader.compile();


    m_fragmentShader.create(GL_FRAGMENT_SHADER);
    m_fragmentShader.addSource(m_fragmentSource.data());
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

string STLObject::getVertexSource(){
    return m_vertexSource;
}

string STLObject::getFragmentSource(){
    return m_fragmentSource;
}

bool STLObject::isCreated(){
    return m_verticeCount != 0;
}
