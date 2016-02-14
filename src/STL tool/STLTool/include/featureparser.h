#ifndef FEATUREPARSER_H
#define FEATUREPARSER_H

#include "part.h"

#include <vector>
#include <map>

using namespace std;
/**
 * @brief The FeatureParser class
 *
 * This class is responsible for parsing the contents of part
 * configuration files.
 */
class FeatureParser{
public:
    /**
     * @brief parsePart
     *
     * This function parses a configurion file based on the name of the file.
     * It also puts the information into a Part struct.
     * @param partName The name of the file that is parsed.
     * @return Returns a part struct that contains the information from the file.
     */
    static Part parsePart(string partName);
    /**
     * @brief parseAllParts
     *
     * This function uses the parsePart() function and the
     * getPartList() function to parse all the known parts.
     * @return Returns a vector of part structs of al the known parts.
     */
    static vector<Part> parseAllParts();
    /**
     * @brief getPartList
     *
     * This function checks the part directory and extracts the names of
     * all config files.
     * @return Returns a vector with the names of all known parts.
     */
    static vector<string> getPartList();
};



#endif // FEATUREPARSER_H
