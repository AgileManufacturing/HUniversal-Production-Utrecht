/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	featureparser.cpp
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	Parses features to files and reads these files
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
#include <iostream>

#include "featureparser.h"
#include "QDir"
#include "QDirIterator"
#include "QDebug"


Part FeatureParser::parsePart(string partName){
    Part part;
    QFile partFile;

    if(!QDir(QDir::currentPath() + "/parts").exists()){
        qWarning() << "No parts directory found when parsing features, returning empty feature map.";
        return part;
    }

    QString path = QDir::currentPath() + QString("/parts/") + QString(partName.data());
    partFile.setFileName(path);
    partFile.open(QIODevice::ReadOnly);

    QTextStream read(&partFile);
    QString currentLine;
    QStringList wordList;

    while(!read.atEnd()){
        currentLine = read.readLine();
        wordList = currentLine.split(":=");
        if(wordList[0].toStdString() == "Partname"){
            part.name = wordList[1].toStdString();
        }
        if(wordList[0] == "/parameters"){
            while(!read.atEnd()){
                currentLine = read.readLine();
                if(currentLine != "parameters/"){
                    wordList = currentLine.split(":=");
                    part.parameters.insert(pair<string,double>(wordList[0].toStdString(),wordList[1].toDouble()));
                }else{
                    break;
                }
            }
        }
        if(wordList[0] == "/gripper"){
            while(!read.atEnd()){
                currentLine = read.readLine();
                if(currentLine != "gripper/"){
                    wordList = currentLine.split(":=");
                    part.parameters.insert(pair<string,double>(wordList[0].toStdString(),wordList[1].toDouble()));
                }
                else{
                    break;
                }
            }
        }
        break;
    }
    return part;
}

vector<Part> FeatureParser::parseAllParts(){
    vector<Part> parts;
    vector<string> partList = getPartList();
    parts.resize(partList.size());
    for(int i = 0; i < partList.size(); ++i){
        parts[i] = parsePart(partList[i]);
    }
    return parts;
}

vector<string> FeatureParser::getPartList(){
    vector<string> partList;
    QDir partDirectory(QDir::currentPath() + QString("/parts/"));
    QStringList fileList = partDirectory.entryList(QStringList() << "*.config");
    partList.resize(fileList.size());
    for(int i = 0; i < fileList.size(); ++i){
        partList[i] = fileList[i].toStdString();
    }
    return partList;
}

