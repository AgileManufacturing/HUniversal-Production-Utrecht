#ifndef FEATUREPARSER_H
#define FEATUREPARSER_H

#include "part.h"

#include <vector>
#include <map>

using namespace std;

class FeatureParser{
public:
    static Part parsePart(string partName);
    static vector<Part> parseAllParts();
    static vector<string> getPartList();
};



#endif // FEATUREPARSER_H
