#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

#include <vector>
#include <iostream>
#include "GL/glew.h"
#include "renderbuffer.h"

using namespace std;

namespace OpenGL{

class Framebuffer
{
public:
    Framebuffer();

    void create();
    void destroy();

    void bind(GLenum target);
    void release();

    void bindRenderBuffer(GLenum target, RenderBuffer& renderbuffer);
    void bindTexture2D(GLuint handle);

private:
    GLenum m_target;
    GLuint m_fboHandle;

};

}// namespace OpenGL

#endif // FRAMEBUFFER_H
