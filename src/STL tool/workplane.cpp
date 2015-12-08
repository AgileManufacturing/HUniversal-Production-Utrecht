#include "workplane.h"
#include <iostream>
#include <vector>

#include <glm/ext.hpp>
using namespace std;

Workplane::Workplane(Camera* camera):
    m_camera{camera},
    m_vertexShader{"Workplane vertex shader"},
    m_fragmentShader{"Workplane fragment shader"},
    m_lightGridColor{glm::vec4(0.8,0.8,0.8,1)},
    m_darkGridColor{glm::vec4(0.2,0.2,0.2,1)}
{
}

void Workplane::create(glm::vec2 gridSize, glm::vec2 cellSize){

    //Adjust the vector size according to the amount of vertices and the required
    //floats for the attributes of each vertex
    m_workplaneData.resize((gridSize.x * gridSize.y) * 36);
    vector<float> cellData;

    glm::vec3 gridStartPosition(0.0,0.0,0.0);
    //determine the position of the top-left corner of the grid
 
    gridStartPosition.y = -cellSize.y * (gridSize.y / 2);
    gridStartPosition.x = -cellSize.x * (gridSize.x / 2);

    glm::vec4 rowStartColor = m_darkGridColor;
    glm::vec4 nextCellColor;
    int vectorIndex = 0;


    for(int y = 0; y < gridSize.y * cellSize.y; y += cellSize.y){
        nextCellColor = rowStartColor;
        if(rowStartColor == m_darkGridColor){
            rowStartColor = m_lightGridColor;
        }else{
            rowStartColor = m_darkGridColor;
        }
        for(int x = 0; x < gridSize.x * cellSize.x; x += cellSize.x, vectorIndex += 36){

            cellData = generateGridCell(glm::vec3(gridStartPosition.x + x, gridStartPosition.y + y, 0),
                                        cellSize,
                                        nextCellColor);
            if(nextCellColor == m_darkGridColor){
                nextCellColor = m_lightGridColor;
            } else{
                nextCellColor = m_darkGridColor;
            }
            for(int i = 0; i < cellData.size(); ++i){
                m_workplaneData[vectorIndex + i] = cellData[i];
            }
        }
    }
    glBufferData(GL_ARRAY_BUFFER,m_workplaneData.size() * sizeof(float), m_workplaneData.data(), GL_STATIC_DRAW);
}

void Workplane::setGridColors(glm::vec3 lightColor, glm::vec3 darkColor){
    m_lightGridColor = glm::vec4(lightColor,0);
    m_darkGridColor = glm::vec4(darkColor,0);
}

vector<float> Workplane::getWorkplaneData(){
    for(int i = 0; i < m_workplaneData.size(); i+=6){
        cout << m_workplaneData[i] << "," << m_workplaneData[i+1] << "," << m_workplaneData[i+2] << " - "
             << m_workplaneData[i+3] << "," << m_workplaneData[i+4] << "," << m_workplaneData[i+5] << endl;
    }
    return m_workplaneData;
}

GLint Workplane::getWorkplaneProgram(){
    return m_workplaneProgram.getHandle();
}

void Workplane::draw(){
    m_workplaneProgram.use();
    m_vao.bind();

    glUniformMatrix4fv(m_uniProj,1,GL_FALSE,glm::value_ptr(m_camera->projection));
    glm::mat4 view = m_camera->getView();
    glUniformMatrix4fv(m_uniView,1,GL_FALSE,glm::value_ptr(view));

    glDrawArrays(GL_TRIANGLES, 0,  m_workplaneData.size() / 6);
    m_vao.release();
    m_workplaneProgram.release();
}

void Workplane::initialize(){

    m_vao.create();
    m_vao.bind();

    m_vbo.create(GL_ARRAY_BUFFER);
    m_vbo.bind();

    create(glm::vec2(45,40),glm::vec2(10,10));

    m_vertexShader.create(GL_VERTEX_SHADER);
    m_vertexShader.addSourceFile("workplane.vert");
    m_vertexShader.compile();

    m_fragmentShader.create(GL_FRAGMENT_SHADER);
    m_fragmentShader.addSourceFile("workplane.frag");
    m_fragmentShader.compile();

    m_workplaneProgram.create();
    m_workplaneProgram.attachShader(m_vertexShader.getHandle());
    m_workplaneProgram.attachShader(m_fragmentShader.getHandle());
    m_workplaneProgram.bindFragData(0,"outColor");
    m_workplaneProgram.link();
    m_workplaneProgram.use();

    m_vao.enableVertexAttrib(m_workplaneProgram,"position",3,GL_FLOAT,GL_FALSE,
                             6 * sizeof(GLfloat), nullptr);

    m_vao.enableVertexAttrib(m_workplaneProgram,"color",3,GL_FLOAT,GL_FALSE,
                             6 * sizeof(GLfloat), (void*)(3 * sizeof(float)));


    m_uniProj = m_workplaneProgram.getUniform("proj");
    m_uniView = m_workplaneProgram.getUniform("view");

    m_vao.release();
}

vector<float> Workplane::generateGridCell(glm::vec3 startPos, glm::vec2 cellSize, glm::vec4 color){
    vector<float> cellData;
    cellData.resize(36);

    cellData[0] = startPos.x;
    cellData[1] = startPos.y;
    cellData[2] = startPos.z;
    cellData[3] = color.r;
    cellData[4] = color.g;
    cellData[5] = color.b;

    cellData[6] = startPos.x + cellSize.x;
    cellData[7] = startPos.y;
    cellData[8] = startPos.z;
    cellData[9] = color.r;
    cellData[10] = color.g;
    cellData[11] = color.b;

    cellData[12] = startPos.x;
    cellData[13] = startPos.y + cellSize.y;
    cellData[14] = startPos.z;
    cellData[15] = color.r;
    cellData[16] = color.g;
    cellData[17] = color.b;

    cellData[18] = startPos.x + cellSize.x;
    cellData[19] = startPos.y;
    cellData[20] = startPos.z;
    cellData[21] = color.r;
    cellData[22] = color.g;
    cellData[23] = color.b;

    cellData[24] = startPos.x + cellSize.x;
    cellData[25] = startPos.y + cellSize.y;
    cellData[26] = startPos.z;
    cellData[27] = color.r;
    cellData[28] = color.g;
    cellData[29] = color.b;

    cellData[30] = startPos.x;
    cellData[31] = startPos.y + cellSize.y;
    cellData[32] = startPos.z;
    cellData[33] = color.r;
    cellData[34] = color.g;
    cellData[35] = color.b;

    return cellData;
}
