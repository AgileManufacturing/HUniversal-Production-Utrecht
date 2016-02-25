/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	stlobject.cpp
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This class represents the 3D object that is drawn from data that is parsed from STL files.
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
#include "stlobject.h"
#include <cmath>

#include "glm/ext.hpp"

#include <iostream>

STLObject::STLObject(Camera* camera):
    m_vertexShader{"STLObject vertex shader"},
    m_fragmentShader{"STLObject fragment shader"},
    m_camera(camera),
    m_shadeState{true}
{}

void STLObject::initialize(){
    m_vao.create();
    m_vao.bind();

    m_vbo.create(GL_ARRAY_BUFFER);
    m_vbo.bind();

    m_vertexShader.create(GL_VERTEX_SHADER);
    m_vertexShader.addSourceFile("STLObject.vert");
    m_vertexShader.compile();

    m_fragmentShader.create(GL_FRAGMENT_SHADER);
    m_fragmentShader.addSourceFile("STLObject.frag");
    m_fragmentShader.compile();

    m_objectProgram.create();
    m_objectProgram.attachShader(m_vertexShader.getHandle());
    m_objectProgram.attachShader(m_fragmentShader.getHandle());
    m_objectProgram.bindFragData(0,"outColor");
    m_objectProgram.link();
    m_objectProgram.use();

    m_vao.enableVertexAttrib(m_objectProgram,"position",
                           3,GL_FLOAT,GL_FALSE,6 * sizeof(GLfloat), nullptr);

    m_vao.enableVertexAttrib(m_objectProgram,"normal",
                           3,GL_FLOAT,GL_FALSE,6 * sizeof(GLfloat), (void*)(3 * sizeof(float)));

    m_uniFragColor = m_objectProgram.getUniform("fragColor");
    m_shadeBoolean = m_objectProgram.getUniform("shading");
    m_uniProj = m_objectProgram.getUniform("proj");
    m_uniView = m_objectProgram.getUniform("view");
    m_uniTrans = m_objectProgram.getUniform("trans");
    glUniform1i(m_shadeBoolean,1);
    glUniform3f(m_uniFragColor, 1.0,1.0,1.0);

    m_vao.release();
}

void STLObject::draw(){
    m_objectProgram.use();
    m_vao.bind();

    glUniformMatrix4fv(m_uniTrans,1,GL_FALSE,glm::value_ptr(m_trans));
    glUniformMatrix4fv(m_uniProj,1,GL_FALSE,glm::value_ptr(m_camera->projection));
    glm::mat4 view = m_camera->getView();
    glUniformMatrix4fv(m_uniView,1,GL_FALSE,glm::value_ptr(view));

    glDrawArrays(GL_TRIANGLES,0, m_verticeCount);
    m_vao.release();
    m_objectProgram.release();
}

void STLObject::switchShading(){
    m_objectProgram.use();
    if(!m_shadeState){
        glUniform1i(m_shadeBoolean,1);
        m_shadeState = true;
    }else{
        glUniform1i(m_shadeBoolean,0);
        m_shadeState = false;
    }
    m_objectProgram.release();
}

void STLObject::setData(vector<float> &data){
    modelData = data;
    m_verticeCount = modelData.size() / 6;
    m_vbo.bind();
    m_vbo.bindData(data,GL_STATIC_DRAW);
    //TODO: AUTOMATICLY CENTER OBJECT
    int minX = INFINITY,maxX = -INFINITY;
    int minY = INFINITY,maxY = -INFINITY;
    for(int i = 0; i < modelData.size();i+=6){
        minX = (modelData[i] < minX)? modelData[i] : minX;
        maxX = (modelData[i] > maxX)? modelData[i] : maxX;
        minY = (modelData[i+1] < minY)? modelData[i+1] : minY;
        maxY = (modelData[i+1] > maxY)? modelData[i+1] : maxY;
    }
    m_camera->partOffset = glm::vec3((maxX-minX)/2,(maxY-minY)/2,0);
}

vector<float> STLObject::getData(){
    return modelData;
}

bool STLObject::isCreated(){
    return m_verticeCount != 0;
}
