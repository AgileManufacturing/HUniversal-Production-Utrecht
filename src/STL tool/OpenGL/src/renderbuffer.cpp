/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	renderbuffer.cpp
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	The openGL renderbuffer.
 *       ..MMM#      ,MMr  .MMMMMMMMr
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Edwin Koek
 *   .dMMMMMF           7Y=d9  dMMMMMr
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		@section LICENSE
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	License:	newBSD
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	Copyright © 2016, HU University of Applied Sciences Utrecht.
 *  JMMMMMMMm.    MMMMMM#!.MMMMMMMMM'.		All rights reserved.
 *   WMMMMMMMMNNN,.TMMM@ .MMMMMMMM#`.M
 *    JMMMMMMMMMMMN,?MD  TYYYYYYY= dM
 *
 *	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *	- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
#include "renderbuffer.h"
#include <iostream>

namespace OpenGL{

RenderBuffer::RenderBuffer(GLuint width = 0, GLuint height = 0):
    m_width{width},
    m_height{height}
{
//    if(!setStorage()){
//        cout << "Renderbuffer storage size has not yet been set" << endl;
//    }
}

void OpenGL::RenderBuffer::create(){
    glGenRenderbuffers(1,&m_rboHandle);
}

void OpenGL::RenderBuffer::destroy(){
    glDeleteRenderbuffers(1,&m_rboHandle);
}

void OpenGL::RenderBuffer::bind(){
    glBindRenderbuffer(GL_RENDERBUFFER,m_rboHandle);
}

void OpenGL::RenderBuffer::release(){
    glBindRenderbuffer(GL_RENDERBUFFER,0);

}

void RenderBuffer::setBufferSize(GLuint width, GLuint height){
    m_width = width;
    m_height = height;
}

int RenderBuffer::getBufferSize(){
    return m_width * m_height * 4;
}

bool RenderBuffer::setBufferStorage(GLenum target){
    if(m_width <= 0 && m_height <= 0){
        cout << "Buffer size has not been set" << endl;
        return false;
    }else{
        glRenderbufferStorage(GL_RENDERBUFFER,target,m_width,m_height);
        return true;
    }
}

GLuint RenderBuffer::rboHandle() const{
    return m_rboHandle;
}

GLuint RenderBuffer::width() const{
    return m_width;
}

void RenderBuffer::setWidth(const GLuint &width){
    m_width = width;
}

GLuint RenderBuffer::height() const{
    return m_height;
}

void RenderBuffer::setHeight(const GLuint &height){
    m_height = height;
}


} //namespace OpenGL
