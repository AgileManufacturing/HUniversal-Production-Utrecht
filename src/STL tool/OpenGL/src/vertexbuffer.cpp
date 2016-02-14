#include "vertexbuffer.h"

namespace OpenGL{

VertexBuffer::VertexBuffer()
{}

void VertexBuffer::create(GLenum target){
    m_target = target;
    glGenBuffers(1,&m_vbo);
}

void VertexBuffer::destroy(){
    glDeleteBuffers(1,&m_vbo);
}

void VertexBuffer::bind(){
    glBindBuffer(m_target,m_vbo);
}


void VertexBuffer::release(){
    glBindBuffer(m_target,0);
}

GLuint VertexBuffer::getHandle(){
    return m_vbo;
}

void VertexBuffer::bindData(vector<float> data, GLenum usage){
    glBufferData(m_target,data.size() * sizeof(float), data.data(), usage);
}



} //namespace OpenGL
