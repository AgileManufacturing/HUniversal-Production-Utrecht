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

void Framebuffer::bindRenderBuffer(GLenum target,RenderBuffer& renderbuffer){
    glFramebufferRenderbuffer(m_target,target,GL_RENDERBUFFER, renderbuffer.rboHandle());
}

void Framebuffer::bindTexture2D(GLuint handle){
    glFramebufferTexture2D(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D,handle,0);
}

}// namespace OpenGL
