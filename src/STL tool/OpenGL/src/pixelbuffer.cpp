#include "pixelbuffer.h"

namespace OpenGL{

PixelBuffer::PixelBuffer()
{}

void PixelBuffer::create(GLenum target){
    m_target = target;
    glGenBuffers(1,&m_pboHandle);
}

void PixelBuffer::destroy(){
    glDeleteBuffers(1,&m_pboHandle);
}

void PixelBuffer::bind(){
    glBindBuffer(m_target,m_pboHandle);
}

void PixelBuffer::release(){
    glBindBuffer(m_target,m_pboHandle);
}

}// namespace OpenGL
