#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

#include <vector>

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

    vector<unsigned int> read(vector<unsigned int>& data, RenderBuffer &renderbuffer);
private:
    GLenum m_target;
    GLuint m_fboHandle;

};

}// namespace OpenGL

#endif // FRAMEBUFFER_H
