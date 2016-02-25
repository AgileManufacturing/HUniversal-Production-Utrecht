/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	mainwindow.cpp
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	The main window that contains al the other widgets for the GUI.
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
#include "mainwindow.h"
#include <iostream>
#include <QKeyEvent>

#include <QFileDialog>
#include <QImage>

#include <QPushButton>
#include <QLineEdit>
#include <QLabel>

#include "QInputDialog"

#include <QColor>

#include "camerasetuplayout.h"

using namespace std;

MainWindow::MainWindow():
    centerWidget{new QWidget},
    glwidget{new GLWidget(640,480)},
    remainingCameraSlots{3},
    mainLayout{new QHBoxLayout},
    leftLayout{new QHBoxLayout},
    rightLayout{new QVBoxLayout},
    m_buttonLayout{new QHBoxLayout},
    addCameraButton{new QPushButton("Add camera")},
    removeCameraButton{new QPushButton("Remove camera")},
    parser{new STLParser},
    menubar{new QMenuBar},
    m_open{new QAction("Open",nullptr)},
    m_save{new QAction("Save",nullptr)},
    m_enableWorkplane{new QAction("Workplane", nullptr)},
    m_enableShading{new QAction("Shade",nullptr)}
{
    resize(1280,720);
    setMinimumSize(1280,720);
    setMaximumSize(1280,720);
    setCentralWidget(centerWidget);
    glwidget->setSizePolicy(QSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed));

    rightLayout->setAlignment(Qt::AlignTop);
    m_buttonLayout->addWidget(addCameraButton);
    m_buttonLayout->addWidget(removeCameraButton);
    rightLayout->addLayout(m_buttonLayout);
    leftLayout->addWidget(glwidget);
    leftLayout->setSizeConstraint(QLayout::SetFixedSize);

    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);

    centerWidget->setLayout(mainLayout);
    setMenuBar(menubar);

    //TODO: Dit kan netter... Mainwindow functies opruimen en direkt callen.
    connect(m_enableShading,&QAction::triggered,glwidget,&GLWidget::switchShading);
    connect(m_open,&QAction::triggered,this,&MainWindow::open);
    connect(m_enableWorkplane,&QAction::triggered,this,&MainWindow::enableWorkplane);
    connect(m_save,&QAction::triggered,this,&MainWindow::save);
    connect(addCameraButton,&QPushButton::clicked,this,&MainWindow::addCamera);
    connect(removeCameraButton,&QPushButton::clicked,this,&MainWindow::removeCamera);



    menubar->addAction(m_open);
    menubar->addAction(m_save);
    menubar->addAction(m_enableShading);
    m_enableWorkplane->setCheckable(true);
    menubar->addAction(m_enableWorkplane);
    if(remainingCameraSlots > 0){
        cameras.push_back(new cameraSetupLayout(glwidget));
        --remainingCameraSlots;
    }
    rightLayout->addLayout(cameras.back()->getCameraSetupLayout());
}

void MainWindow::open()
{
    cout << "Open function triggered!" << endl;

    QString path = QFileDialog::getOpenFileName(glwidget,QDir::currentPath());
    parser->open(path);
    vector<float> vertices = parser->getVertices();
    if(vertices.size()!= 0){
        glwidget->loadSTLObject(vertices);
    }else{
        cout << "No valid file selected!" << endl;
    }
}

void MainWindow::save(){
    bool succes;
    QString imageName = QInputDialog::getText(this,"Save","Image name:",QLineEdit::Normal,"",&succes);
    if(succes){
        for(int i = 0; i < cameras.size();++i){
            cameras[i]->useCamera();
            glwidget->save(imageName+QString(to_string(i).data()));
        }
        cameras[0]->useCamera();
    }
}

void MainWindow::keyPressEvent(QKeyEvent * e){
    if(e->modifiers() == Qt::ControlModifier ){
        if(e->key() == Qt::Key_1){
            open();
        }
    }else{
//        glwidget->keyPressEvent(e);
    }
}


void MainWindow::addCamera(){
    if(remainingCameraSlots > 0){
        cameras.push_back(new cameraSetupLayout(glwidget));
        --remainingCameraSlots;
        rightLayout->addLayout(cameras.back()->getCameraSetupLayout());
    }
    cout << "Camera size: " << cameras.size() << endl;
}

void MainWindow::removeCamera(){
    if(cameras.size() > 1){
        ++remainingCameraSlots;
        delete cameras.back();
        cameras.pop_back();
    }
    cout << "Camera size: " << cameras.size() << endl;
}

void MainWindow::enableWorkplane(){
    if(m_enableWorkplane->isChecked()){
        glwidget->enableWorkplane(true);
    }else{
        glwidget->enableWorkplane(false);
    }
}
