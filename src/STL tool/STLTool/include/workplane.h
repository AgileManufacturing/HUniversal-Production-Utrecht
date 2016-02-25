/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	workplane.h
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
#ifndef WORKPLANE_H
#define WORKPLANE_H
#include "opengl.h"

#include <string>
#include <vector>


#include "camera.h"

using namespace std;
/**
 * @brief The Workplane class
 *
 * This class represents the simulated workplane in the rendering.
 */
class Workplane{
public:
    /**
     * @brief Workplane constructor
     * @param camera Camera Contains information regarding the current camera setup.
     */
    Workplane(Camera* camera);
    /**
     * @brief create
     *
     * This function generates the vertices that are required to draw
     * the workplane with openGL.
     * @param gridSize The size of the grid.
     * @param cellSize The size of each cell.
     */
    void create(glm::vec2 gridSize, glm::vec2 cellSize);
    /**
     * @brief setGridColors
     *
     * This function adjusts the colour of the cells of the checkerboard like
     * workplane.
     * @param lightColor The light colour of the checkerboard
     * @param darkColor The dark colour of the checkerboard.
     */
    void setGridColors(glm::vec3 lightColor, glm::vec3 darkColor);
    /**
     * @brief getWorkplaneData
     * @return Returns the vertice data of the work plane.
     */
    vector<float> getWorkplaneData();
    /**
     * @brief isCreated
     * @return Returns whether the workplane has been initialised and created.
     */
    bool isCreated();
    /**
     * @brief draw
     *
     * This function is responsible for calling the openGL draw function.
     */
    void draw();
    /**
     * @brief initialize
     *
     * This function is responsible for setting up the openGL buffers that are
     * needed in order to draw the workplane.
     */
    void initialize();

private:
    Camera* m_camera;
    vector<float> generateGridCell(glm::vec3 startPos, glm::vec2 cellSize, glm::vec4 color);

    glm::vec4 m_lightGridColor, m_darkGridColor;

    vector<float> m_workplaneData;

    GLint m_uniProj, m_uniView;

    OpenGL::Shader m_vertexShader;
    OpenGL::Shader m_fragmentShader;
    OpenGL::VertexArray m_vao;
    OpenGL::VertexBuffer m_vbo;
    OpenGL::ShaderProgram m_workplaneProgram;
};

#endif // WORKPLANE_H
