#ifndef PIXELBUFFER_H
#define PIXELBUFFER_H

#include "GL/glew.h"

using namespace std;

namespace OpenGL{

////Pixel Buffer
/// Reading pixel data without a pixel buffer blocks the CPU untill
/// the action is complete.(need to test the speed with and without it.

class PixelBuffer
{
public:
    PixelBuffer();

    void create(GLenum target);
    void destroy();

    void bind();
    void release();

private:
    GLuint m_pboHandle;
    GLenum m_target;
};

}//namespace OpenGL
#endif // PIXELBUFFER_H
