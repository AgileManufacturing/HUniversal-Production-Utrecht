/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	workplane.cpp
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This class represents the simulated workplane in the rendering.
 *       ..MMM#      ,MMr  .MMMMMMMMr
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Edwin Koek
 *   .dMMMMMF           7Y=d9  dMMMMMr
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		@section LICENSE
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	License:	newBSD
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	Copyright © 2013, HU University of Applied Sciences Utrecht.
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
#include "workplane.h"
#include <iostream>
#include <vector>

#include <glm/ext.hpp>
using namespace std;

Workplane::Workplane(Camera* camera):
    m_camera{camera},
    m_vertexShader{"Workplane vertex shader"},
    m_fragmentShader{"Workplane fragment shader"},
    m_lightGridColor{glm::vec4(0.8,0.8,0.8,1)},
    m_darkGridColor{glm::vec4(0.2,0.2,0.2,1)}
{
}

void Workplane::create(glm::vec2 gridSize, glm::vec2 cellSize){

    //Adjust the vector size according to the amount of vertices and the required
    //floats for the attributes of each vertex
    m_workplaneData.resize((gridSize.x * gridSize.y) * 36);
    vector<float> cellData;

    glm::vec3 gridStartPosition(0.0,0.0,0.0);
    //determine the position of the top-left corner of the grid
 
    gridStartPosition.y = -cellSize.y * (gridSize.y / 2);
    gridStartPosition.x = -cellSize.x * (gridSize.x / 2);

    glm::vec4 rowStartColor = m_darkGridColor;
    glm::vec4 nextCellColor;
    int vectorIndex = 0;


    for(int y = 0; y < gridSize.y * cellSize.y; y += cellSize.y){
        nextCellColor = rowStartColor;
        if(rowStartColor == m_darkGridColor){
            rowStartColor = m_lightGridColor;
        }else{
            rowStartColor = m_darkGridColor;
        }
        for(int x = 0; x < gridSize.x * cellSize.x; x += cellSize.x, vectorIndex += 36){

            cellData = generateGridCell(glm::vec3(gridStartPosition.x + x, gridStartPosition.y + y, 0),
                                        cellSize,
                                        nextCellColor);
            if(nextCellColor == m_darkGridColor){
                nextCellColor = m_lightGridColor;
            } else{
                nextCellColor = m_darkGridColor;
            }
            for(int i = 0; i < cellData.size(); ++i){
                m_workplaneData[vectorIndex + i] = cellData[i];
            }
        }
    }
    glBufferData(GL_ARRAY_BUFFER,m_workplaneData.size() * sizeof(float), m_workplaneData.data(), GL_STATIC_DRAW);
}

void Workplane::setGridColors(glm::vec3 lightColor, glm::vec3 darkColor){
    m_lightGridColor = glm::vec4(lightColor,0);
    m_darkGridColor = glm::vec4(darkColor,0);
}

vector<float> Workplane::getWorkplaneData(){
    for(int i = 0; i < m_workplaneData.size(); i+=6){
        cout << m_workplaneData[i] << "," << m_workplaneData[i+1] << "," << m_workplaneData[i+2] << " - "
             << m_workplaneData[i+3] << "," << m_workplaneData[i+4] << "," << m_workplaneData[i+5] << endl;
    }
    return m_workplaneData;
}

bool Workplane::isCreated(){
    return m_workplaneData.size() != 0;
}

void Workplane::draw(){
    m_workplaneProgram.use();
    m_vao.bind();

    glUniformMatrix4fv(m_uniProj,1,GL_FALSE,glm::value_ptr(m_camera->projection));
    glm::mat4 view = m_camera->getView();
    glUniformMatrix4fv(m_uniView,1,GL_FALSE,glm::value_ptr(view));

    glDrawArrays(GL_TRIANGLES, 0,  m_workplaneData.size() / 6);
    m_vao.release();
    m_workplaneProgram.release();
}

void Workplane::initialize(){

    m_vao.create();
    m_vao.bind();

    m_vbo.create(GL_ARRAY_BUFFER);
    m_vbo.bind();

    //Workplane generation currently hardcoded, oops
    create(glm::vec2(45,40),glm::vec2(10,10));

    m_vertexShader.create(GL_VERTEX_SHADER);
    m_vertexShader.addSourceFile("workplane.vert");
    m_vertexShader.compile();

    m_fragmentShader.create(GL_FRAGMENT_SHADER);
    m_fragmentShader.addSourceFile("workplane.frag");
    m_fragmentShader.compile();

    m_workplaneProgram.create();
    m_workplaneProgram.attachShader(m_vertexShader.getHandle());
    m_workplaneProgram.attachShader(m_fragmentShader.getHandle());
    m_workplaneProgram.bindFragData(0,"outColor");
    m_workplaneProgram.link();
    m_workplaneProgram.use();

    m_vao.enableVertexAttrib(m_workplaneProgram,"position",3,GL_FLOAT,GL_FALSE,
                             6 * sizeof(GLfloat), nullptr);

    m_vao.enableVertexAttrib(m_workplaneProgram,"color",3,GL_FLOAT,GL_FALSE,
                             6 * sizeof(GLfloat), (void*)(3 * sizeof(float)));


    m_uniProj = m_workplaneProgram.getUniform("proj");
    m_uniView = m_workplaneProgram.getUniform("view");

    m_vao.release();
}

vector<float> Workplane::generateGridCell(glm::vec3 startPos, glm::vec2 cellSize, glm::vec4 color){
    vector<float> cellData;
    cellData.resize(36);

    cellData[0] = startPos.x;
    cellData[1] = startPos.y;
    cellData[2] = startPos.z;
    cellData[3] = color.r;
    cellData[4] = color.g;
    cellData[5] = color.b;

    cellData[6] = startPos.x + cellSize.x;
    cellData[7] = startPos.y;
    cellData[8] = startPos.z;
    cellData[9] = color.r;
    cellData[10] = color.g;
    cellData[11] = color.b;

    cellData[12] = startPos.x;
    cellData[13] = startPos.y + cellSize.y;
    cellData[14] = startPos.z;
    cellData[15] = color.r;
    cellData[16] = color.g;
    cellData[17] = color.b;

    cellData[18] = startPos.x + cellSize.x;
    cellData[19] = startPos.y;
    cellData[20] = startPos.z;
    cellData[21] = color.r;
    cellData[22] = color.g;
    cellData[23] = color.b;

    cellData[24] = startPos.x + cellSize.x;
    cellData[25] = startPos.y + cellSize.y;
    cellData[26] = startPos.z;
    cellData[27] = color.r;
    cellData[28] = color.g;
    cellData[29] = color.b;

    cellData[30] = startPos.x;
    cellData[31] = startPos.y + cellSize.y;
    cellData[32] = startPos.z;
    cellData[33] = color.r;
    cellData[34] = color.g;
    cellData[35] = color.b;

    return cellData;
}
