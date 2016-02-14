#ifndef TEXTURE2D_H
#define TEXTURE2D_H

#include "gl/glew.h"
#include <vector>

using namespace std;

class texture2D{
public:
    texture2D();
    void create();

    void bind();
    void release();

    void setSize(GLuint width,GLuint height);
    void setParameter(GLenum pName,GLenum param);
    void initialise(GLsizei width,GLsizei height);

    GLuint getHandle();

    vector<unsigned char> getData();
private:
    vector<unsigned char> data;
    GLuint textureHandle;
    GLuint textureWidth;
    GLuint textureHeight;
};



#endif // TEXTURE2D_H
