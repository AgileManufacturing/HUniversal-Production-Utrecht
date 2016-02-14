#include "texture2d.h"
#include <iostream>

texture2D::texture2D()
{}

void texture2D::create(){
    glGenTextures(1,&textureHandle);
}

void texture2D::bind(){
    glBindTexture(GL_TEXTURE_2D,textureHandle);
}

void texture2D::release(){
    glBindTexture(GL_TEXTURE_2D,0);
}

void texture2D::setSize(GLuint width, GLuint height){
    textureWidth = width;
    textureHeight = height;
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,width,height,0,GL_RGBA,GL_UNSIGNED_BYTE,NULL);
}

void texture2D::setParameter(GLenum pName, GLenum param){
    glTexParameterf(GL_TEXTURE_2D,pName,param);
}

void texture2D::initialise(GLsizei width, GLsizei height){
    textureWidth = width;
    textureHeight = height;
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,width,height,0,GL_RGBA,GL_UNSIGNED_BYTE,NULL);
}

GLuint texture2D::getHandle(){
    return textureHandle;
}

vector<unsigned char> texture2D::getData(){
    //Working with RGBA so each pixel is 4 bytes
    data.resize(textureWidth * textureHeight * 4);
    glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_UNSIGNED_BYTE,&data[0]);
    return data;
}

