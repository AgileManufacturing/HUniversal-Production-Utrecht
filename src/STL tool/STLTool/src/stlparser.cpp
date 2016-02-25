/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`
 *                     ...MMMMMF .
 *         dN.       .jMN, TMMM`.MM     	@file 	stlparser.cpp
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This function is responsible for parsing STL files and storing the vertice data in a vector.
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
#include "stlparser.h"
#include <iostream>

#include <QTextStream>
#include <vector>

#include <QElapsedTimer>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

using namespace std;

STLParser::STLParser():
    binary{false}
{}

STLParser::~STLParser(){
}

void STLParser::open(QString path){
    m_stlFile.close();
    m_stlFile.setFileName(path);
    m_stlFile.open(QIODevice::ReadOnly | QIODevice::Text);
    if(m_stlFile.isReadable()){
        textStream.setDevice(&m_stlFile);
        QByteArray headerArray = textStream.read(80).toLocal8Bit();
        QByteArray triangleCountArray = textStream.read(4).toLocal8Bit();
        char* tri = triangleCountArray.data();
        char triangleCount[4] = {tri[0],tri[1],tri[2],tri[3]};
        long triangles = *((long*) triangleCount);
        //Checking wether the file is in binary format by multiplying the size
        //of each triangle by the size of each triangle + the size of the file header
        if(((triangles*50) + 84) == m_stlFile.size()){
            binary = true;
            cout << "This is binary" << endl;
        } else{
            binary = false;
        }
        m_stlFile.close();
        m_stlFile.open(QIODevice::ReadOnly);
    }
}

std::vector<float> STLParser::getVertices(){
    QElapsedTimer timer;
    timer.start();
    glm::vec3 normal;
    if(m_stlFile.isReadable()){
        if(!vertices.empty()){
            vertices.clear();
        }
        textStream.setDevice(&m_stlFile);
        if(binary){

            cout << "Reading binary file!" << endl;
            //get file header (80 bytes)
            QByteArray headerArray = textStream.read(80).toLocal8Bit();
            //get amount of triangles in file
            QByteArray triangleCountArray = textStream.read(4).toLocal8Bit();
            char* tri = triangleCountArray.data();
            char triangleCount[4] = {tri[0],tri[1],tri[2],tri[3]};
            long triangles = *((long*) triangleCount);
            cout << triangles << endl;
            vertices.resize(triangles * 18);

            int offset;
            for(int i = 0; i < triangles; ++i){
                offset = i * 18;

                QByteArray facetArray = textStream.read(50).toLocal8Bit();
                char* facet = facetArray.data();
                char normalXBuffer[4] = {facet[0],facet[1],facet[2],facet[3]};
                float normalx = *((float*) normalXBuffer);
                char normalYBuffer[4] = {facet[4],facet[5],facet[6],facet[7]};
                float normaly = *((float*) normalYBuffer);
                char normalZBuffer[4] = {facet[8],facet[9],facet[10],facet[11]};
                float normalz = *((float*) normalZBuffer);

                char v1XBuffer[4] = {facet[12],facet[13],facet[14],facet[15]};
                float v1x = *((float*) v1XBuffer);
                vertices[offset] = v1x;
                char v1YBuffer[4] = {facet[16],facet[17],facet[18],facet[19]};
                float v1y = *((float*) v1YBuffer);
                vertices[offset + 1] = v1y;
                char v1ZBuffer[4] = {facet[20],facet[21],facet[22],facet[23]};
                float v1z = *((float*) v1ZBuffer);
                vertices[offset + 2] = v1z;

                vertices[offset + 3] = normalx;
                vertices[offset + 4] = normaly;
                vertices[offset + 5] = normalz;

                char v2XBuffer[4] = {facet[24],facet[25],facet[26],facet[27]};
                float v2x = *((float*) v2XBuffer);
                vertices[offset + 6] = v2x;
                char v2YBuffer[4] = {facet[28],facet[29],facet[30],facet[31]};
                float v2y = *((float*) v2YBuffer);
                vertices[offset + 7] = v2y;
                char v2ZBuffer[4] = {facet[32],facet[33],facet[34],facet[35]};
                float v2z = *((float*) v2ZBuffer);
                vertices[offset + 8] = v2z;

                vertices[offset + 9] = normalx;
                vertices[offset + 10] = normaly;
                vertices[offset + 11] = normalz;

                char v3XBuffer[4] = {facet[36],facet[37],facet[38],facet[39]};
                float v3x = *((float*) v3XBuffer);
                vertices[offset + 12] = v3x;
                char v3YBuffer[4] = {facet[40],facet[41],facet[42],facet[43]};
                float v3y = *((float*) v3YBuffer);
                vertices[offset + 13] = v3y;
                char v3ZBuffer[4] = {facet[44],facet[45],facet[46],facet[47]};
                float v3z = *((float*) v3ZBuffer);
                vertices[offset + 14] = v3z;

                vertices[offset + 15] = normalx;
                vertices[offset + 16] = normaly;
                vertices[offset + 17] = normalz;
            }
        }else{
            QString data;
            QStringList dataList;
            std::string currentWord;

            while(!textStream.atEnd()){
                data = textStream.readLine();
                dataList = data.split(" ");
                for(int i = 0; i < dataList.size(); ++i){
                    currentWord = dataList[i].toStdString();
                    if(currentWord == "vertex"){
                        vertices.push_back(dataList[i+1].toFloat());
                        vertices.push_back(dataList[i+2].toFloat());
                        vertices.push_back(dataList[i+3].toFloat());
                        vertices.push_back(normal.x);
                        vertices.push_back(normal.y);
                        vertices.push_back(normal.z);
                    }else if(currentWord == "normal"){
                          normal = glm::vec3(dataList[i+1].toFloat(),dataList[i+2].toFloat(),dataList[i+3].toFloat());
                    }
                }
            }
        }
        std::cout << "Reading took: " << timer.elapsed() << " milliseconds" << std::endl;
        return vertices;
    }
    return vertices;
}






