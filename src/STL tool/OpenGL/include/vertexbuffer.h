#ifndef VERTEXBUFFER_H
#define VERTEXBUFFER_H

#include <vector>
#include "GL/glew.h"

using namespace std;

namespace OpenGL{

class VertexBuffer{
public:
    VertexBuffer();

    void create(GLenum target);
    void destroy();

    void bind();
    void release();

    GLuint getHandle();
    void bindData(vector<float> data, GLenum usage);

private:
    bool m_isCreated;
    bool m_isBound;

    GLuint m_vbo;
    GLenum m_target;
};

} //namespace OpenGL

#endif // VERTEXBUFFER_H
