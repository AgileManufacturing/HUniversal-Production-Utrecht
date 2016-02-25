/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	mainwindow.h
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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include "glwidget.h"
#include "stlparser.h"
#include <QLayout>
#include <QFormLayout>
#include <QPushButton>


class cameraSetupLayout;
/**
 * @brief The MainWindow class
 *
 * The main window that contains al the other widgets for the GUI.
 */
class MainWindow: public QMainWindow{
public:
    /**
     * @brief MainWindow constructor
     *
     * Sets up al the layouts and widgets and connects the buttons
     * to the corresponding functions.
     */
    MainWindow();
    /**
     * @brief open
     *
     * Calls the open function in order to open a STL file.
     */
    void open();
    /**
     * @brief save
     *
     * Calls the save function that saves information regarding a part and its
     * image.
     */
    void save();
    /**
     * @brief keyPressEvent
     *
     * Manages the keyboards inputs from the user.
     */
    void keyPressEvent(QKeyEvent *) override;


private:
    void enableWorkplane();
    void addCamera();
    void removeCamera();

    QPushButton* addCameraButton;
    QPushButton* removeCameraButton;

    int remainingCameraSlots;

    STLParser* parser;
    GLWidget* glwidget;
    QWidget* centerWidget;

    QHBoxLayout* mainLayout;
    QHBoxLayout* leftLayout;
    QHBoxLayout* m_buttonLayout;
    QVBoxLayout* rightLayout;

    std::vector<cameraSetupLayout*> cameras;

    QMenuBar* menubar;

    QAction* m_open;
    QAction* m_save;
    QAction* m_enableWorkplane;
    QAction* m_enableShading;
};

#endif // MAINWINDOW_H
