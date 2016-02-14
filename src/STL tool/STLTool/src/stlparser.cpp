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






