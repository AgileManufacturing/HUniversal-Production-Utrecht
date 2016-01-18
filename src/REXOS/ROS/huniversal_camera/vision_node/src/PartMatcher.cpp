#include "vision_node/PartMatcher.h"
#include "vision_node/ObjectDetector.h"

#include <unistd.h>
#include <dirent.h>
#include <iostream>
#include <fstream>


vector<string> PartMatcher::getPartList(){
    char currentPath[FILENAME_MAX];
    string directory;
    if (getcwd(currentPath, sizeof(currentPath))){
        directory = currentPath;
        directory += "/parts";
    }
    DIR* dir;
    struct dirent *ent;
    vector<string> fileList;
    if((dir = opendir(directory.c_str())) != NULL){
        while((ent = readdir(dir)) != NULL){
            string currentFile = ent->d_name;
            if(currentFile.substr(currentFile.find("."),100) == ".config"){
                fileList.push_back(currentFile);
            }
        }
        closedir(dir);
    }else{
        REXOS_INFO("COULD NOT OPEN DIRECTORY");
    }
    return fileList;
}

Part PartMatcher::parsePart(string partName){
    char currentPath[FILENAME_MAX];
    Part part;
    string directory;
    if (getcwd(currentPath, sizeof(currentPath))){
        directory = currentPath;
        directory += "/parts/" + partName;
    }
    ifstream file;
    file.open(directory);
    char buffer[256];
    string currentLine;
    while(!file.eof()){
        file.getline(buffer,256);
        currentLine = buffer;
        if(currentLine.substr(0,currentLine.find(":=")) == "Partname"){
            part.name = currentLine.substr(currentLine.find(":=")+2);
        }else if(currentLine == "/parameters"){
            while(!file.eof()){
                file.getline(buffer,256);
                currentLine = buffer;
                if(currentLine[0] != '/' && !file.eof()){
                    part.parameters.insert(pair<string,double>(
                                               currentLine.substr(0,currentLine.find(":=")),
                                               atof(currentLine.substr(currentLine.find(":=")+2).c_str())));
                }else{
                    break;
                }
            }
        }
    }
    return part;
}

vector<Part> PartMatcher::parseAllParts(){
    vector<Part> parts;
    vector<string> partList = getPartList();
    parts.resize(partList.size());
    for(unsigned int i = 0; i < partList.size(); ++i){
        parts[i] = parsePart(partList[i]);
    }
    return parts;
}

double PartMatcher::matchPart(map<string, double> partFeatures, map<string, double> matchFeatures){
    map<string,double>::iterator it;
    for(it = partFeatures.begin(); it != partFeatures.end();it++){
        if(matchFeatures.find(it->first) == matchFeatures.end()){
            return 0;
        }
    }
    double matchSum = 0;
        int averageWeight = 0;
        for(it = partFeatures.begin(); it != partFeatures.end();it++){
            //Matching is based of of the percentage that the current value
            //deviates from the "known value"
            if(it->first == "SurfacePercentage"){
                matchSum += 100 - abs(1 - (it->second / matchFeatures.find(it->first)->second)) * 100;
                averageWeight +=1;
            }
            if(it->first == "Height" || it->first == "Width"){
                matchSum += 100 - abs(1 -(pow(it->second,2) / pow(matchFeatures.find(it->first)->second,2))) * 100;
                averageWeight +=1;
            }
            if(it->first == "NumberOfHoles"){
                int holesWeight = 3;
                int numberOfHoles = matchFeatures.find(it->first)->second;
                REXOS_WARN_STREAM("Number of holes: " + to_string(it->second) +  " Found: " + to_string(numberOfHoles));
                if(numberOfHoles == it->second){
                    matchSum += 100 * holesWeight;
                    averageWeight +=holesWeight;
                }else if(numberOfHoles == 0){
                    matchSum += 0 * holesWeight;
                    averageWeight +=holesWeight;
                }else{
                    matchSum += (100 - abs(1 -(it->second / numberOfHoles)) * 100) * holesWeight;
                    averageWeight +=holesWeight;
                }
            }
        }
        return matchSum /averageWeight;
}

///TODO:UPDATE THIS FUNCTION
pair<Part, double> PartMatcher::matchPart(map<string, double> partFeatures, string partName){
    Part matchPart = parsePart(partName);
    map<string,double>::iterator it;
    for(it = partFeatures.begin(); it != partFeatures.end();++it){
        if(matchPart.parameters.find(it->first) == matchPart.parameters.end()){
            return make_pair(matchPart,0);
        }
    }
    double matchSum = 0;
    for(it = partFeatures.begin(); it != partFeatures.end();++it){
        //Matching is based of of the percentage that the current value
        //deviates from the "known value"
        if(it->first == "SurfacePercentage"){
            matchSum += 100 - abs(1 - (it->second / matchPart.parameters.find(it->first)->second)) * 100;
        }
        if(it->first == "Height" || it->first == "Width"){
            matchSum += 100 - abs(1 -(pow(it->second,2) / pow(matchPart.parameters.find(it->first)->second,2))) * 100;
        }
        if(it->first == "NumberOfHoles"){
            int numberOfHoles = matchPart.parameters.find(it->first)->second;
            if(numberOfHoles == it->second){
                matchSum += 100;
            }else if(numberOfHoles == 0){
                matchSum += 0;
            }else{
                matchSum += 100 - abs(1 -(it->second / numberOfHoles)) * 100;
            }
        }
    }
    return make_pair(matchPart,matchSum / partFeatures.size());
}

pair<Part, double> PartMatcher::matchPart(map<string, double> partFeatures, Part referencePart){
    map<string,double>::iterator it;
    for(it = partFeatures.begin(); it != partFeatures.end();it++){
        if(referencePart.parameters.find(it->first) == referencePart.parameters.end()){
            return make_pair(referencePart,0);
        }
    }
    double matchSum = 0;
        int averageWeight = 0;
        for(it = partFeatures.begin(); it != partFeatures.end();it++){
            //Matching is based of of the percentage that the current value
            //deviates from the "known value"
            if(it->first == "SurfacePercentage"){
                matchSum += 100 - abs(1 - (it->second / referencePart.parameters.find(it->first)->second)) * 100;
                averageWeight +=1;
            }
            if(it->first == "Height" || it->first == "Width"){
                matchSum += 100 -
                        abs(1 -(pow(it->second,2) / pow(referencePart.parameters.find(it->first)->second,2))) * 100;
                averageWeight +=1;
            }
            //Number of holes has a stronger weight
            if(it->first == "NumberOfHoles"){
                int holesWeight = 3;
                int numberOfHoles = referencePart.parameters.find(it->first)->second;
                if(numberOfHoles == it->second){
                    matchSum += 100 * holesWeight;
                    averageWeight +=holesWeight;
                }else if(numberOfHoles == 0){
                    matchSum += 0 * holesWeight;
                    averageWeight +=holesWeight;
                }else{
                    matchSum += (100 - abs(1 -(it->second / numberOfHoles)) * 100) * holesWeight;
                    averageWeight +=holesWeight;
                }
            }
        }
        return make_pair(referencePart,matchSum / averageWeight);
}

pair<Part, double> PartMatcher::matchPart(map<string, double> partFeatures){
    vector<Part> parts = parseAllParts();
    vector<pair<Part,double> > matchPercentages;
    matchPercentages.resize(parts.size());
    for(unsigned int i = 0; i < parts.size(); ++i){
        matchPercentages[i] = make_pair(parts[i],matchPart(partFeatures,parts[i].parameters));
        string matchResult = matchPercentages[i].first.name + " - " + to_string(matchPercentages[i].second);
        REXOS_WARN_STREAM(matchResult);
    }
    pair<Part,double> bestMatch;
    bestMatch.second = 0;

    for(unsigned int i = 0; i < matchPercentages.size(); ++i){
        if(matchPercentages[i].second > bestMatch.second){
            bestMatch = matchPercentages[i];
        }
    }
    return bestMatch;
}

map<string, double> PartMatcher::createParameterMap(const VisionObject& object){
    map<string,double> parameters;
    // calculating width and height of the object, the broadest side is always set
    // as width
    RotatedRect rect = minAreaRect(object.data);
    Point2f vertices[4];
    rect.points(vertices);
    double maxLength = -INFINITY, minLength = INFINITY;
    for(int i = 0; i < 4;++i){
        double dX,dY;
        dX = vertices[i].x - vertices[(i+1)%4].x;
        dY = vertices[i].y - vertices[(i+1)%4].y;
        double length = sqrt((dX * dX) + (dY * dY));
        (length <= minLength)? minLength = length : minLength = minLength;
        (length >= maxLength)? maxLength = length : maxLength = maxLength;
    }
    double height = minLength;
    double width = maxLength;
    double area = height * width;

    double surfacePercentage = (object.data.size()/area) * 100;
    vector<vector<Point>> holes =
            ObjectDetector::getHoles(ObjectDetector::getContoursHierarchy(object.objectImage));
    double numberOfHoles = holes.size();

    //Add parameters to map

    parameters.insert(make_pair("Height",height));
    parameters.insert(make_pair("Width",width));
    //parameters.insert(make_pair("Area",area));
    //surfacepercentage
    parameters.insert(make_pair("SurfacePercentage",surfacePercentage));
    //number of holes
    parameters.insert(make_pair("NumberOfHoles",numberOfHoles));
    return parameters;
}
