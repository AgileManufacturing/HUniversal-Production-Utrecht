#ifndef FEATUREMATCHER_H
#define FEATUREMATCHER_H

#include "part.h"

#include "featureparser.h"
#include "featurefactory.h"

using namespace std;

class FeatureMatcher{
public:
    // Matches a part based on 2 parameter maps. This function has no further knowledge of what
    // part it is matching. Therefore it wil only return how well the two parameter maps match eachother.
    static double matchPart(map<string, double>& partFeatures, map<string, double>& matchFeatures);

    static pair<Part, double> matchPart(map<string, double>& partFeatures, string partName);
    static pair<Part, double> matchPart(map<string, double>& partFeatures, Part referencePart);
    static pair<Part, double> matchPart(map<string, double>& partFeatures);
};

#endif // FEATUREMATCHER_H

