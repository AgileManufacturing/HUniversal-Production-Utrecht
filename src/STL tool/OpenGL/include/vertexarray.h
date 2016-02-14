#ifndef VERTEXARRAY_H
#define VERTEXARRAY_H

#include "GL/glew.h"
#include "shaderprogram.h"
#include <string>

using namespace std;

namespace OpenGL{

class VertexArray{
public:
    VertexArray();

    bool create();
    void destroy();

    bool bind();
    void release();


    // Stride represents the amount of bytes between each attribute in the array
    void enableVertexAttrib(ShaderProgram& program,
                            const GLchar* attributeName,
                            GLint components,
                            GLenum type,
                            GLboolean normalised,
                            GLsizei stride,
                            const GLvoid* offset);

    GLuint getHandle();

private:
    GLuint m_vaoHandle;

    bool m_isCreated;
    bool m_isBound;

};

} //namespace OpenGL
#endif // VERTEXARRAY_H
