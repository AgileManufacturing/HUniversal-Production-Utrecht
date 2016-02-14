#include "shader.h"
#include "QDir"
#include "QTextstream"
#include <iostream>

using namespace std;

namespace OpenGL{

Shader::Shader(string shaderName)
{
    m_shaderName = shaderName;
}

void Shader::create(GLenum type){
    m_shaderHandle = glCreateShader(type);
}

void Shader::destroy(){
    glDeleteShader(m_shaderHandle);
}

void Shader::addSource(const GLchar* shaderSource){
    glShaderSource(m_shaderHandle,1,&shaderSource,nullptr);
}

void Shader::addSource(string shaderSource){
    const GLchar* shaderData = shaderSource.data();
    glShaderSource(m_shaderHandle,1,&shaderData,nullptr);
}

void Shader::addSourceFile(string filename){
    QString filePath = QDir::currentPath() + "/Shaders/" + QString(filename.data());
    cout << filePath.toStdString() << endl;
    QFile shaderFile;
    shaderFile.setFileName(filePath);
    shaderFile.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream inStream(&shaderFile);
    QString data = inStream.readAll();
    string stringData  = data.toStdString();
    const GLchar* shaderData = stringData.data();
    glShaderSource(m_shaderHandle,1,&shaderData,nullptr);
    shaderFile.close();

}

void Shader::compile(){
    glCompileShader(m_shaderHandle);
    glGetShaderiv(m_shaderHandle, GL_COMPILE_STATUS, &m_status);

    if(m_status != GL_TRUE){
        cout << "Shader: " << m_shaderName << " compilation failed" << endl;
        glGetShaderiv(m_shaderHandle,GL_INFO_LOG_LENGTH, &m_logLength);
        m_log.resize(m_logLength);
        glGetShaderInfoLog(m_shaderHandle,m_logLength,&m_logLength,&m_log[0]);
        cout << m_log << endl;
    } else{
        cout << "Shader: " << m_shaderName << " compilation succesful" << endl;
    }
}

std::string Shader::getLog(){
    glGetShaderiv(m_shaderHandle,GL_INFO_LOG_LENGTH, &m_logLength);
    m_log.resize(m_logLength);
    glGetShaderInfoLog(m_shaderHandle,m_logLength, &m_logLength,&m_log[0]);
    return m_log;
}

GLuint Shader::getHandle(){
    return m_shaderHandle;
}

} //namespace OpenGL
