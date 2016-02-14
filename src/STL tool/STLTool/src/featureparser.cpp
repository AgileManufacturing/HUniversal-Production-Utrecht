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

