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

    void bindRenderBuffer(RenderBuffer& renderbuffer);

    void read(vector<int> &data, RenderBuffer &renderbuffer);
private:
    GLenum m_target;
    GLuint m_fboHandle;

};

}// namespace OpenGL

#endif // FRAMEBUFFER_H