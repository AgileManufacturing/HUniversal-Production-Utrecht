#include "featurematcher.h"
#include "featureparser.h"

#include <map>
#include <iostream>



pair<Part, double> FeatureMatcher::matchPart(map<string,double>& partFeatures,string partName){
    Part matchPart = FeatureParser::parsePart(partName);
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

double FeatureMatcher::matchPart(map<string,double>& partFeatures,map<string,double>& matchFeatures){
    map<string,double>::iterator it;
    for(it = partFeatures.begin(); it != partFeatures.end();it++){
        if(matchFeatures.find(it->first) == matchFeatures.end()){
            return 0;
        }
    }
    double matchSum = 0;
    for(it = partFeatures.begin(); it != partFeatures.end();it++){
        //Matching is based of of the percentage that the current value
        //deviates from the "known value"
        if(it->first == "SurfacePercentage"){
            matchSum += 100 - abs(1 - (it->second / matchFeatures.find(it->first)->second)) * 100;
        }
        if(it->first == "Height" || it->first == "Width"){
            matchSum += 100 - abs(1 -(pow(it->second,2) / pow(matchFeatures.find(it->first)->second,2))) * 100;
        }
        if(it->first == "NumberOfHoles"){
            int numberOfHoles = matchFeatures.find(it->first)->second;
            if(numberOfHoles == it->second){
                matchSum += 100;
            }else if(numberOfHoles == 0){
                matchSum += 0;
            }else{
                matchSum += 100 - abs(1 -(it->second / numberOfHoles)) * 100;
            }
        }
    }
    return matchSum / partFeatures.size();
}

pair<Part, double> FeatureMatcher::matchPart(map<string, double>& partFeatures, Part referencePart){
    map<string,double>::iterator it;
    for(it = partFeatures.begin(); it != partFeatures.end();it++){
        if(referencePart.parameters.find(it->first) == referencePart.parameters.end()){
            return make_pair(referencePart,0);
        }
    }
    double matchSum = 0;
    for(it = partFeatures.begin(); it != partFeatures.end();it++){
        //Matching is based of of the percentage that the current value
        //deviates from the "known value"
        if(it->first == "SurfacePercentage"){
            matchSum += 100 - abs(1 - (it->second / referencePart.parameters.find(it->first)->second)) * 100;
        }
        if(it->first == "Height" || it->first == "Width"){
            matchSum += 100 -
                    abs(1 -(pow(it->second,2) / pow(referencePart.parameters.find(it->first)->second,2))) * 100;
        }
        if(it->first == "NumberOfHoles"){
            int numberOfHoles = referencePart.parameters.find(it->first)->second;
            if(numberOfHoles == it->second){
                matchSum += 100;
            }else if(numberOfHoles == 0){
                matchSum += 0;
            }else{
                matchSum += 100 - abs(1 -(it->second / numberOfHoles)) * 100;
            }
        }
    }
    return make_pair(referencePart,matchSum / partFeatures.size());
}

pair<Part,double> FeatureMatcher::matchPart(map<string, double>& partFeatures){
    vector<Part> parts = FeatureParser::parseAllParts();
    vector<pair<Part,double> > matchPercentages;
    matchPercentages.resize(parts.size());
    for(int i = 0; i < parts.size(); ++i){
        matchPercentages[i] = make_pair(parts[i],matchPart(partFeatures,parts[i].parameters));
//        cout <<  parts[i].name << " - " << matchPercentages[i] << endl;
    }
    pair<Part,double> bestMatch;
    bestMatch.second = 0;

    for(int i = 0; i < matchPercentages.size(); ++i){
        if(matchPercentages[i].second > bestMatch.second){
            bestMatch = matchPercentages[i];
        }
    }

    return bestMatch;
}

