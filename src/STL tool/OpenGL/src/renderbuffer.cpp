#include "renderbuffer.h"
#include <iostream>

namespace OpenGL{

RenderBuffer::RenderBuffer(GLuint width = 0, GLuint height = 0):
    m_width{width},
    m_height{height}
{
//    if(!setStorage()){
//        cout << "Renderbuffer storage size has not yet been set" << endl;
//    }
}

void OpenGL::RenderBuffer::create(){
    glGenRenderbuffers(1,&m_rboHandle);
}

void OpenGL::RenderBuffer::destroy(){
    glDeleteRenderbuffers(1,&m_rboHandle);
}

void OpenGL::RenderBuffer::bind(){
    glBindRenderbuffer(GL_RENDERBUFFER,m_rboHandle);
}

void OpenGL::RenderBuffer::release(){
    glBindRenderbuffer(GL_RENDERBUFFER,0);

}

void RenderBuffer::setBufferSize(GLuint width, GLuint height){
    m_width = width;
    m_height = height;
}

int RenderBuffer::getBufferSize(){
    return m_width * m_height * 4;
}

bool RenderBuffer::setBufferStorage(GLenum target){
    if(m_width <= 0 && m_height <= 0){
        cout << "Buffer size has not been set" << endl;
        return false;
    }else{
        glRenderbufferStorage(GL_RENDERBUFFER,target,m_width,m_height);
        return true;
    }
}

GLuint RenderBuffer::rboHandle() const{
    return m_rboHandle;
}

GLuint RenderBuffer::width() const{
    return m_width;
}

void RenderBuffer::setWidth(const GLuint &width){
    m_width = width;
}

GLuint RenderBuffer::height() const{
    return m_height;
}

void RenderBuffer::setHeight(const GLuint &height){
    m_height = height;
}


} //namespace OpenGL
