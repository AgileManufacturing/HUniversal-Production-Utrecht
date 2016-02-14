#ifndef RENDERBUFFER_H
#define RENDERBUFFER_H

#include "GL/glew.h"

using namespace std;

namespace OpenGL{

class RenderBuffer
{
public:
    RenderBuffer(GLuint width, GLuint height);

    void create();
    void destroy();

    void bind();
    void release();

    void setBufferSize(GLuint width,GLuint height);

    bool setBufferStorage(GLenum target);

    GLuint rboHandle() const;
    GLuint width() const;
    GLuint height() const;

    void setWidth(const GLuint &width);
    void setHeight(const GLuint &height);

    int getBufferSize();
private:
    GLuint m_rboHandle;
    GLuint m_width,m_height;

};
} //namespace OpenGL
#endif // RENDERBUFFER_H
