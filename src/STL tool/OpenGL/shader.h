#ifndef SHADER_H
#define SHADER_H

#include "GL/glew.h"
#include <string>

using namespace std;

namespace OpenGL{

class Shader{
public:
    Shader(string shaderName);

    void create(GLenum type);
    void destroy();

    void addSource(const GLchar* shaderSource);
    void addSource(string shaderSource);

    void compile();

    std::string getLog();

    GLuint getHandle();
private:
    string m_shaderName;
    GLuint m_shaderHandle;

    string m_log;
    GLint m_logLength;

    GLint m_status;
};

} // namespace OpenGL

#endif // SHADER_H
