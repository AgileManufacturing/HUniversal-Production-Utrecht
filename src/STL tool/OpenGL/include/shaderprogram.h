#ifndef SHADERPROGRAM_H
#define SHADERPROGRAM_H
#include "GL/glew.h"

#include "shader.h"

namespace OpenGL{

class ShaderProgram{
public:
    ShaderProgram();

    void create();
    void destroy();

    void link();

    void use();
    void release();

    void attachShader(GLuint shader);
    void bindFragData(GLuint index, const GLchar* name);

    GLuint getUniform(const GLchar* name);

    GLuint getHandle();

private:
    GLuint m_programHandle;

};

} //namespace OpenGL

#endif // SHADERPROGRAM_H
