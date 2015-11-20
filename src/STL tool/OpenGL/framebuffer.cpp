#include "framebuffer.h"

namespace OpenGL{

Framebuffer::Framebuffer()
{}

void Framebuffer::create(){
    glGenFramebuffers(1,&m_fboHandle);
}

void Framebuffer::destroy(){
    glDeleteFramebuffers(1,&m_fboHandle);
}

void Framebuffer::bind(GLenum target){
    m_target = target;
    glBindFramebuffer(target,m_fboHandle);
}

void Framebuffer::release(){
    glBindFramebuffer(m_target,0);
}

void Framebuffer::bindRenderBuffer(RenderBuffer& renderbuffer){
    glFramebufferRenderbuffer(m_target,GL_COLOR_ATTACHMENT0,GL_RENDERBUFFER, renderbuffer.rboHandle());
}

vector<unsigned int> Framebuffer::read(vector<unsigned int>& data,RenderBuffer& renderbuffer){
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0,0,renderbuffer.width(),renderbuffer.height(),GL_RGB,GL_UNSIGNED_BYTE,&data[0]);
    return data;
}


}// namespace OpenGL
