/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	stlobject.h
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This class represents the 3D object that is drawn from data that is parsed from STL files.
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
#ifndef STLOBJECT_H
#define STLOBJECT_H

#include <vector>

#include "opengl.h"

#include "camera.h"

using namespace std;
/**
 * @brief The STLObject class
 *
 * This class represents the 3D object that is drawn from data that is parsed from STL files.
 */
class STLObject{
public:
    /**
     * @brief STLObject constructor
     * @param camera Contains information regarding the current camera setup.
     */
    STLObject(Camera* camera);
    /**
     * @brief initialize
     *
     * This function initialises al the buffers that are needed by
     * openGL in order to draw the STL object.
     */
    void initialize();
    /**
     * @brief draw
     *
     * This function updates the positional values of the object and
     * then calls the opengl draw function.
     */
    void draw();
    /**
     * @brief setData
     *
     * This updates the model data with new vertices.
     * @param data The new vertices.
     */
    void setData(vector<float>& data);

    /**
     * @brief getData
     * @return Returns the model data
     */
    vector<float> getData();
    /**
     * @brief isCreated
     * @return Returns whether the object is initialised and has valid model data.
     */
    bool isCreated();
    /**
     * @brief switchShading
     *
     * This function switched between including and not including the diffuse lighting in
     * the rendering.
     */
    void switchShading();


//TODO: i dont think trans should be public
    glm::mat4 m_trans;
private:
    vector<float> modelData;
    bool m_shadeState;

    Camera* m_camera;

    GLuint m_verticeCount = 0;

    GLuint m_uniFragColor,m_uniView,m_uniProj,m_uniTrans,m_shadeBoolean;

    OpenGL::Shader m_vertexShader;
    OpenGL::Shader m_fragmentShader;
    OpenGL::VertexArray m_vao;
    OpenGL::VertexBuffer m_vbo;
    OpenGL::ShaderProgram m_objectProgram;
};

#endif // STLOBJECT_H
