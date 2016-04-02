#ifndef _PARTMATCHER_H
#define _PARTMATCHER_H
#include "Part.h"
#include "VisionObject.h"
#include <vector>
#include <map>

/**
 * @brief The PartMatcher class
 *
 *  This class contains functions that are used to extract the parameter maps from
 *  part files and match them with parameter maps of detected parts.
 */
class PartMatcher{
public:
    /**
     * @brief getPartList
     *
     * This function is used to extract the .config files from the
     * designated part folder.
     * @return Returns a vector containing all the located file names.
     */
    static vector<string> getPartList();
    /**
     * @brief parsePart
     *
     * This function reads a part file and creates a part struct with the knowledge
     * of its name and shape features.
     * @param partName The name of part whose file wil be read.
     * @return Returns a part struct.
     */
    static Part parsePart(string partName);
    /**
     * @brief parseAllParts
     *
     * This function reads al the part files that are detected by getPartList()
     * and creates the corresponding part stucts.
     * @return Returns a list of structs created from the detected part files.
     */
    static vector<Part> parseAllParts();
    /**
     * @brief matchPart
     *
     * This function matches the features of two parts.
     * @param partFeatures The parameter map of a part.
     * @param matchFeatures The parameter map that partFeatures is going to be matched with.
     * @return Returns a percentage based match result.
     */
    static double matchPart(map<string, double> partFeatures, map<string, double> matchFeatures);
    /**
     * @brief matchPart
     *
     * This function matches the features of two parts. The second part is extracted based on
     * its name.
     * @param partFeatures The parameter map of a part.
     * @param partName The name of a part that the parameter map is mathced with.
     * @return Returns a pair of a percentage based match and the part that was matched with.
     */
    static pair<Part, double> matchPart(map<string, double> partFeatures, string partName);
    /**
     * @brief matchPart
     *
     * This function matches the features of two parts. The second list of features comes
     * from the parameter map of the referencePart.
     * @param partFeatures The parameter map of a part.
     * @param referencePart The part that the parameter map is matched with.
     * @return Returns a pair of a percentage based match and the part that was matched with.
     */
    static pair<Part, double> matchPart(map<string, double> partFeatures, Part referencePart);
    /**
     * @brief matchPart
     *
     * This function matches the features of a part with the features of all known parts. These parts
     * are extracted by the parseAllParts() function.
     * @param partFeatures The parameter map of a part.
     * @return Returns a pair of a percentage based match and the corresponding part name.
     */
    static pair<Part, double> matchPart(map<string, double> partFeatures);
    /**
     * @brief createParameterMap
     *
     * This function creates a parameterMap that contains information regarding the features of an object
     * according to the data that is stored with a VisionObject struct.
     * @param object The VisionObject that the parameterMap is made from.
     * @return Returns the parameter map of the VisionObject.
     */
    static map<string, double> createParameterMap(const VisionObject& object);
};
#endif // PARTMATCHER_H