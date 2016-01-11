#include "vertexarray.h"
#include <iostream>

using namespace std;

namespace OpenGL{

VertexArray::VertexArray():
    m_isCreated{false},
    m_isBound{false}
{}

bool VertexArray::create(){
    //TODO check context

    glGenVertexArrays(1,&m_vaoHandle);
    m_isCreated = true;
    return m_isCreated;
}

void VertexArray::destroy(){
    glDeleteVertexArrays(1,&m_vaoHandle);
}

bool VertexArray::bind(){
    if(m_isCreated){
        glBindVertexArray(m_vaoHandle);
        m_isBound = true;
        return m_isBound;
    }
    else{
        cout << "Create VAO before binding" << endl;
        return m_isBound;
    }
}

void VertexArray::release(){
    glBindVertexArray(0);
    m_isBound = false;
}

void VertexArray::enableVertexAttrib(ShaderProgram& program,
                                     const GLchar* attributeName,
                                     GLint components,
                                     GLenum type,
                                     GLboolean normalised,
                                     GLsizei stride,
                                     const GLvoid* offset)
{
    if(m_isBound){
        GLint attribPosition = glGetAttribLocation(program.getHandle(),attributeName);
        glEnableVertexAttribArray(attribPosition);
        glVertexAttribPointer(attribPosition,components,type,normalised,stride, offset);
    }else{
        cout << "Bind VAO before enabling attributes" << endl;
    }
}

GLuint VertexArray::getHandle(){
    return m_vaoHandle;
}
} //namespace OpenGL
