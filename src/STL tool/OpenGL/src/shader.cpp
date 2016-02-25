/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	shader.cpp
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This class represents an openGL shader.
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
#include "shader.h"
#include "QDir"
#include "QTextstream"
#include <iostream>

using namespace std;

namespace OpenGL{

Shader::Shader(string shaderName)
{
    m_shaderName = shaderName;
}

void Shader::create(GLenum type){
    m_shaderHandle = glCreateShader(type);
}

void Shader::destroy(){
    glDeleteShader(m_shaderHandle);
}

void Shader::addSource(const GLchar* shaderSource){
    glShaderSource(m_shaderHandle,1,&shaderSource,nullptr);
}

void Shader::addSource(string shaderSource){
    const GLchar* shaderData = shaderSource.data();
    glShaderSource(m_shaderHandle,1,&shaderData,nullptr);
}

void Shader::addSourceFile(string filename){
    QString filePath = QDir::currentPath() + "/Shaders/" + QString(filename.data());
    cout << filePath.toStdString() << endl;
    QFile shaderFile;
    shaderFile.setFileName(filePath);
    shaderFile.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream inStream(&shaderFile);
    QString data = inStream.readAll();
    string stringData  = data.toStdString();
    const GLchar* shaderData = stringData.data();
    glShaderSource(m_shaderHandle,1,&shaderData,nullptr);
    shaderFile.close();

}

void Shader::compile(){
    glCompileShader(m_shaderHandle);
    glGetShaderiv(m_shaderHandle, GL_COMPILE_STATUS, &m_status);

    if(m_status != GL_TRUE){
        cout << "Shader: " << m_shaderName << " compilation failed" << endl;
        glGetShaderiv(m_shaderHandle,GL_INFO_LOG_LENGTH, &m_logLength);
        m_log.resize(m_logLength);
        glGetShaderInfoLog(m_shaderHandle,m_logLength,&m_logLength,&m_log[0]);
        cout << m_log << endl;
    } else{
        cout << "Shader: " << m_shaderName << " compilation succesful" << endl;
    }
}

std::string Shader::getLog(){
    glGetShaderiv(m_shaderHandle,GL_INFO_LOG_LENGTH, &m_logLength);
    m_log.resize(m_logLength);
    glGetShaderInfoLog(m_shaderHandle,m_logLength, &m_logLength,&m_log[0]);
    return m_log;
}

GLuint Shader::getHandle(){
    return m_shaderHandle;
}

} //namespace OpenGL
