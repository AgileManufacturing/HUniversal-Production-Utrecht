#include "shaderprogram.h"

namespace OpenGL{

ShaderProgram::ShaderProgram()
{}

void ShaderProgram::create(){
    m_programHandle = glCreateProgram();
}

void ShaderProgram::destroy(){
    glDeleteProgram(m_programHandle);
}

void ShaderProgram::link(){
    glLinkProgram(m_programHandle);
}

void ShaderProgram::use(){
    glUseProgram(m_programHandle);
}

void ShaderProgram::release(){
    glUseProgram(0);
}

void ShaderProgram::attachShader(GLuint shader){
    glAttachShader(m_programHandle,shader);
}

void ShaderProgram::bindFragData(GLuint index, const GLchar *name){
    glBindFragDataLocation(m_programHandle,index,name);
}

GLuint ShaderProgram::getUniform(const GLchar *name){
    return glGetUniformLocation(m_programHandle,name);
}

GLuint ShaderProgram::getHandle(){
    return m_programHandle;
}

} //namespace OpenGL

