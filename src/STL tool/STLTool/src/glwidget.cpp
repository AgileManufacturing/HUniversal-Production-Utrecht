/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	glwidget.cpp
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	The widget that openGL draws to.
 *          dM@      dMMM3  .ga...g,    	@date Created:	2016-2-25
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
#include "glwidget.h"
#include "QPainter"
#include "GL/glew.h"
#include <iostream>
#include <string>
#include "QOpenGLContext"
#include <QKeyEvent>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/ext.hpp"

#include <QLayout>
#include <QImage>

#include "featurefactory.h"
#include "imagefactory.h"
#include "templater.h"


using namespace std;

GLWidget::GLWidget(float width,float height):
    camera{new Camera},
    workplane{new Workplane(camera)},
    m_stlObject{new STLObject(camera)},
    m_activeWorkplane{false}
{

    resize(width,height);
    QSurfaceFormat fmt = format();
    fmt.setDepthBufferSize(32);
    fmt.setVersion(3,3);
    setFormat(fmt);
    
    connect(&drawTimer,SIGNAL(timeout()),this,SLOT(update()));
    //Timer that triggers the draw function 60 times a second
    drawTimer.start(static_cast<float>(1000/60));
}

GLWidget::~GLWidget(){
    delete workplane;
    delete m_stlObject;
    delete camera;
}

void GLWidget::setCamera(float x,float y, float z, float yaw, float pitch){
    camera->setPosition(x,y,z);
    camera->setDirection(yaw,pitch);
}

void GLWidget::initializeGL(){
    GLenum error = glewInit();
    if (error != GLEW_OK) {
        qFatal("Failed to initialize OpenGL: "
               "failed to initialize GLEW (%s)", glewGetErrorString(error));
    }
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1, 0.1, 0.1, 1);

    workplane->initialize();
    m_stlObject->initialize();
}

void GLWidget::paintGL(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(m_activeWorkplane){
        workplane->draw();
    }
    if(m_stlObject->isCreated()){
        m_stlObject->draw();
    }
}

void GLWidget::loadSTLObject(std::vector<float> &data){
    m_stlObject->setData(data);
}

void GLWidget::enableWorkplane(bool newWorkplaneState){
    m_activeWorkplane = newWorkplaneState;
}

void GLWidget::switchShading(){
    m_stlObject->switchShading();
}

void GLWidget::save(QString imageName){
    QImage image = grabFramebuffer();
    ImageFactory::save(image,imageName.toStdString());
    Mat cvImage = Mat(image.height(),image.width(),CV_8UC4,const_cast<uchar*>(image.bits()),image.bytesPerLine()).clone();
    cvtColor(cvImage,cvImage,CV_RGB2GRAY);
    vector<VisionObject> objects = ImageFactory::filterObjects(FeatureFactory::findConnectedComponents(cvImage),cvImage);
    //This function uses the first VisionObject since there should only ever be one part rendered.
    FeatureFactory::savePartConfig(FeatureFactory::createParameterMap(objects[0],m_stlObject->getData(),Templater::generateGripperTemp(3,3,3,12))
            ,imageName);
}

QSize GLWidget::sizeHint() const{
    return QSize(width(),height());
}
//void GLWidget::keyPressEvent(QKeyEvent* e){
//    switch(e->key()) {
//        case Qt::Key_W :
//            camera->position += glm::vec3(0.0f,0.0f,2.0f);
//            camera->facing += glm::vec3(0.0f,0.0f,2.0f);
//            break;
//        case Qt::Key_Z:
//            camera->position += glm::vec3(0.0f,-2.0f,0.0f);
//            camera->facing += glm::vec3(0.0f,-2.0f,0.0f);
//            break;
//        case Qt::Key_C:
//            camera->position += glm::vec3(0.0f,2.0f,0.0f);
//            camera->facing += glm::vec3(0.0f,2.0f,0.0f);
//            break;
//        case Qt::Key_Q:
//            m_stlObject->m_trans = glm::rotate(m_stlObject->m_trans,glm::radians(3.0f),glm::vec3(0.0f,0.0f,1.0f));
//            break;
//        case Qt::Key_E:
//            m_stlObject->m_trans = glm::rotate(m_stlObject->m_trans,glm::radians(-3.0f),glm::vec3(0.0f,0.0f,1.0f));
//            break;
//        case Qt::Key_Up:
//            m_stlObject->m_trans = glm::rotate(m_stlObject->m_trans,glm::radians(3.0f),glm::vec3(1.0f,0.0f,0.0f));
//            break;
//        case Qt::Key_A:
//            camera->position += glm::vec3(-2.0f,0.0f,0.0f);
//            camera->facing += glm::vec3(-2.0f,0.0f,0.0f);
//            break;
//        case Qt::Key_Left:
//            m_stlObject->m_trans  = glm::rotate(m_stlObject->m_trans,glm::radians(3.0f),glm::vec3(0.0f,1.0f,0.0f));
//            break;
//        case Qt::Key_S:
//            camera->position += glm::vec3(0.0f,0.0f,-2.0f);
//            camera->facing += glm::vec3(0.0f,0.0f,-2.0f);
//            break;
//        case Qt::Key_Down:
//            m_stlObject->m_trans  = glm::rotate(m_stlObject->m_trans,glm::radians(-3.0f),glm::vec3(1.0f,0.0f,0.0f));
//            break;
//        case Qt::Key_D:
//            camera->position += glm::vec3(2.0f,0.0f,0.0f);
//            camera->facing += glm::vec3(2.0f,0.0f,0.0f);
//            break;
//        case Qt::Key_Right:
//            m_stlObject->m_trans  = glm::rotate(m_stlObject->m_trans,glm::radians(-3.0f),glm::vec3(0.0f,1.0f,0.0f));
//            break;
//        case Qt::Key_X:
//            cout << "Pressing X" << endl;
//            break;
//    }
//}
