/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	glsurface.cpp
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This class is responsible for the offscreen rendering process.
 *          dM@      dMMM3  .ga...g,    	@date Created:	2016-2-25
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
#include "glsurface.h"
#include <iostream>
#include <QDebug>
#include <QDir>

#include "imagefactory.h"
#include "featurefactory.h"
#include "templater.h"

#include "opencv2/highgui/highgui.hpp"

GLSurface::GLSurface():
    m_surface{new QOffscreenSurface},
    m_context{new QOpenGLContext},
    m_camera{new Camera},
    m_workplane{new Workplane(m_camera)},
    m_stlObject{new STLObject(m_camera)},
    m_workplaneState{false},
    m_rbo{640,480}
{
    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(32);
    fmt.setVersion(3,3);

    m_context->setFormat(fmt);
    m_context->create();

    m_surface->setFormat(fmt);
    m_surface->create();

    m_context->makeCurrent(m_surface);
}

Mat GLSurface::renderGL(){
    m_fbo.bind(GL_DRAW_FRAMEBUFFER);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(m_workplane->isCreated() && m_workplaneState){
        m_workplane->draw();
    }
    if(m_stlObject->isCreated()){
        m_stlObject->draw();
    }
    texture.bind();
    vector<unsigned char> data = texture.getData();
    m_fbo.release();
    //Turns the raw pixel data into a CV image
    return Mat(cvSize(m_rbo.width(),m_rbo.height()), CV_8UC4, &data[0], cv::Mat::AUTO_STEP).clone();
}

void GLSurface::initializeGL(){
    GLenum error = glewInit();
    if (error != GLEW_OK) {
        qFatal("Failed to initialize OpenGL: "
               "failed to initialize GLEW (%s)", glewGetErrorString(error));
    }
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 1);
    glViewport(0,0,m_rbo.width(),m_rbo.height());

    texture.create();
    texture.bind();
    texture.setParameter(GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    texture.setParameter(GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    texture.initialise(m_rbo.width(),m_rbo.height());
    texture.release();

    m_rbo.create();
    m_rbo.bind();
    m_rbo.setBufferStorage(GL_DEPTH_COMPONENT);
    m_rbo.release();

    m_fbo.create();
    m_fbo.bind(GL_FRAMEBUFFER);
    m_fbo.bindTexture2D(texture.getHandle());
    m_fbo.bindRenderBuffer(GL_DEPTH_ATTACHMENT,m_rbo);
    m_fbo.release();

    m_workplane->initialize();
    m_stlObject->initialize();
}

void GLSurface::loadSTLObject(std::vector<float> &data){
    m_stlObject->setData(data);
}

bool GLSurface::workplaneState() const{
    return m_workplaneState;
}

void GLSurface::setWorkplaneState(bool workplaneState){
    m_workplaneState = workplaneState;
}

void GLSurface::switchShadeState(){
    m_stlObject->switchShading();
}

void GLSurface::executeTool(int argc, char *argv[]){
    //ignore first argument
    int argumentOffset = 1;
    QString fileName = argv[argumentOffset];
    QStringList splitName = fileName.split(".");

    ++argumentOffset;
    String currentArg = argv[argumentOffset];
    if(currentArg == "-r"){
        m_rbo.setBufferSize(atoi(argv[argumentOffset+1]),atoi(argv[argumentOffset+2]));
        argumentOffset += 3;
    }

    initializeGL();

    if((argc - argumentOffset)%5 == 0){
        parser.open(QDir::currentPath() + "/" + fileName);

        vector<float> vertices = parser.getVertices();
        if(vertices.size() != 0){
            loadSTLObject(vertices);

            int nrOfCameras = (argc-argumentOffset)/5;
            for(int i = 0; i < nrOfCameras;++i){
                // Setting up new filename
                QString newName = splitName[0] + " - " +  QString(to_string(i).data()) + "." + splitName[1];
                m_camera->setPosition(atof(argv[argumentOffset]),atof(argv[argumentOffset+1]),atof(argv[argumentOffset+2]));
                m_camera->setDirection(atof(argv[argumentOffset+3]),atof(argv[argumentOffset+4]));
                argumentOffset += 5;

                Mat image = renderGL();
                ImageFactory::save(image,newName.toStdString());
                waitKey(0);
                switchShadeState();
                image = renderGL();
                cvtColor(image,image,CV_RGB2GRAY);
                vector<VisionObject> objects = ImageFactory::filterObjects(FeatureFactory::findConnectedComponents(image),image);
                FeatureFactory::savePartConfig(FeatureFactory::createParameterMap(objects[0],m_stlObject->getData(),
                                               Templater::generateGripperTemp(3,3,3,12)),newName);
                switchShadeState();
                cout << i << " Done" << endl;
            }
        }else{
            cout << "Invalid STL file selected." << endl;
        }
    }
}

