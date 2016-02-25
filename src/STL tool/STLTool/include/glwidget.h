/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	glwidget.h
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	The widget that openGL draws to.
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
#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "opengl.h"

#include <chrono>
#include <QOpenGLWidget>
#include <QOpenGLContext>
#include <QTimer>
#include "workplane.h"
#include "stlobject.h"
/**
 * @brief The GLWidget class
 *
 * This class is responsible for the main widget of the STL tool. OpenGL
 * will draw onto this widget when the tool is used with a GUI.
 */
class GLWidget : public QOpenGLWidget{
public:
    /**
     * @brief GLWidget
     *
     * The constructor of the openGL widget.
     * @param width Defines the width of the widget.
     * @param height Defines the height of the widget.
     */
    GLWidget(float width, float height);
    ~GLWidget();
    /**
     * @brief setCamera
     *
     * This function call the setPosition and setDirection functions of the camera.
     * @param x The new x coördinate of the camera.
     * @param y The new y coördinate of the camera.
     * @param z The new z coördinate of the camera.
     * @param yaw The new yar rotation of the camera.
     * @param pitch The new pitch rotation of the camera.
     */
    void setCamera(float x, float y, float z, float yaw, float pitch);
    /**
     * @brief initializeGL
     *
     * This function initialises OpenGL.
     */
    void initializeGL() override;
    /**
     * @brief paintGL
     *
     * This function calls al the draw functions.
     */
    void paintGL() override;
    /**
     * @brief sizeHint
     *
     * Qt requires this to be defined in order to set the
     * widget dimensions.
     * @return The size of the widget.
     */
    QSize sizeHint() const;

    /**
     * @brief loadSTLObject
     *
     * This function passes the vertice data to the STL object.
     * @param data The vertice data.
     */
    void loadSTLObject(std::vector<float>& data);
    /**
     * @brief enableWorkplane
     *
     * This function sets the active state of the workplane. This determines
     * whether it is drawn or not.
     * @param newWorkplaneState The new workplane state.
     */
    void enableWorkplane(bool newWorkplaneState);
    /**
     * @brief switchShading
     *
     * Switches between including the including or not including the diffuse lighting
     * in the rendering.
     */
    void switchShading();
    /**
     * @brief save
     *
     * This function saves the rendered image and saves a part config file based on the part
     * that was rendered.
     * @param imageName The name that both the file and image are saved as.
     */
    void save(QString imageName);

//    void keyPressEvent(QKeyEvent* e) override;
private:
    Camera* camera;
    Workplane* workplane;
    STLObject* m_stlObject;

    QTimer drawTimer;

    bool m_activeWorkplane;

    //Test attributes
    //glm::mat4 trans;
};

#endif
