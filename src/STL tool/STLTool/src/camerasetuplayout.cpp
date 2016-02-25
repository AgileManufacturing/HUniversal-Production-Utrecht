/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	camerasetuplayout.cpp
 *         .MN.      MMMMM;  ?^ ,THM		@brief  This is a layout class that is used by the mainwindow. It contains several input fields that are used to specify a camera position and orientation.
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
#include "camerasetuplayout.h"
#include <iostream>

#include <glm/glm.hpp>
#include "glm/ext.hpp"

using namespace std;

cameraSetupLayout::cameraSetupLayout(GLWidget *_glwidget):
    cameraSetup{new QGridLayout},
    cameraPos{new QLabel("Position")},
    directionVec{new QLabel("Orientation")},
    errorLabel{new QLabel},
    name{new QLabel("Camera")},
    useCameraButton{new QPushButton("Set")},
    glwidget{_glwidget},
    xPosE{new QLineEdit},
    yPosE{new QLineEdit},
    zPosE{new QLineEdit},
    rollE{new QLineEdit},
    pitchE{new QLineEdit},
    yawE{new QLineEdit}
{
    cameraSetup->addWidget(name,0,0);
    cameraSetup->setHorizontalSpacing(1);

    cameraSetup->addWidget(cameraPos,1,1,Qt::AlignCenter);

    xPosE->setMaximumSize(editBoxWidth,editBoxHeight);
    yPosE->setMaximumSize(editBoxWidth,editBoxHeight);
    zPosE->setMaximumSize(editBoxWidth,editBoxHeight);

    xPosE->setMaxLength(4);
    yPosE->setMaxLength(4);
    zPosE->setMaxLength(4);

    xPosE->setText("x");
    yPosE->setText("y");
    zPosE->setText("z");
    xPosE->setAlignment(Qt::AlignCenter);
    yPosE->setAlignment(Qt::AlignCenter);
    zPosE->setAlignment(Qt::AlignCenter);

    cameraSetup->addWidget(xPosE,2,0);
    cameraSetup->addWidget(yPosE,2,1);
    cameraSetup->addWidget(zPosE,2,2);

    cameraSetup->addWidget(directionVec,3,1,Qt::AlignCenter);

    pitchE->setMaximumSize(editBoxWidth,editBoxHeight);
    yawE->setMaximumSize(editBoxWidth,editBoxHeight);
    pitchE->setText("Pitch");
    yawE->setText("Yaw");

    pitchE->setMaxLength(5);
    yawE->setMaxLength(5);

    pitchE->setAlignment(Qt::AlignCenter);
    yawE->setAlignment(Qt::AlignCenter);

    cameraSetup->addWidget(pitchE,4,0);
    cameraSetup->addWidget(yawE,4,2);

    errorLabel->setStyleSheet("QLabel {color : red;}");
    cameraSetup->addWidget(errorLabel,5,2,1,2);


    useCameraButton->setMaximumWidth(editBoxWidth);
    cameraSetup->addWidget(useCameraButton,0,3);
    connect(useCameraButton,&QPushButton::clicked, this, &cameraSetupLayout::useCamera);

    connect(xPosE, &QLineEdit::editingFinished,xPosE,&QLineEdit::clearFocus);
    connect(yPosE, &QLineEdit::editingFinished,yPosE,&QLineEdit::clearFocus);
    connect(zPosE, &QLineEdit::editingFinished,zPosE,&QLineEdit::clearFocus);

    connect(pitchE, &QLineEdit::editingFinished,pitchE,&QLineEdit::clearFocus);
    connect(yawE, &QLineEdit::editingFinished,yawE,&QLineEdit::clearFocus);
}

cameraSetupLayout::~cameraSetupLayout(){
    delete name;
    delete useCameraButton;

    delete cameraPos;
    delete directionVec;
    delete errorLabel;

    delete xPosE;
    delete yPosE;
    delete zPosE;

    delete rollE;
    delete pitchE;
    delete yawE;

    delete cameraSetup;
}

QGridLayout *cameraSetupLayout::getCameraSetupLayout(){
    return cameraSetup;
}

void cameraSetupLayout::setValues(){
    if(xPosE->text() != "" && xPosE->text() != "x" &&
            yPosE->text() != "" && xPosE->text() != "y" &&
            zPosE->text() != "" && xPosE->text() != "z" &&
            pitchE->text() != "" && pitchE->text() != "Pitch" &&
            yawE->text() != "" && yawE->text() != "Yaw"){

        x = xPosE->text().toFloat();
        y = yPosE->text().toFloat();
        z = zPosE->text().toFloat();
        yaw = yawE->text().toFloat();
        pitch = pitchE->text().toFloat();


        cout << "Valid information thank you" << endl;

        errorLabel->setText("");
    }else{
        errorLabel->setText("Information is invalid");
    }
}

void cameraSetupLayout::useCamera(){
    setValues();
    glwidget->setCamera(x,y,z,yaw,pitch);
}


