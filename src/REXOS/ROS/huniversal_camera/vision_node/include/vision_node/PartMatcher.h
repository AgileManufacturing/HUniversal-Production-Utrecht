#ifndef PARTMATCHER_H
#define PARTMATCHER_H

#include <vector>
#include <string>
#include <map>

#include "Part.h"
#include "VisionObject.h"

class PartMatcher{
public:
    static vector<string> getPartList();
    static Part parsePart(string partName);
    static vector<Part> parseAllParts();

    static double matchPart(map<string, double> partFeatures, map<string, double> matchFeatures);

    static pair<Part, double> matchPart(map<string, double> partFeatures, string partName);
    static pair<Part, double> matchPart(map<string, double> partFeatures, Part referencePart);
    static pair<Part, double> matchPart(map<string, double> partFeatures);

    static map<string, double> createParameterMap(const VisionObject& object);

private:
};


#endif // PARTMATCHER_H
