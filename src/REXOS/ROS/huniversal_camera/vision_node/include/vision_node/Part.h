#ifndef PART_H
#define PART_H

#include <map>

using namespace std;
/**
 * @brief The Part struct
 *
 * Contains the name aswell as information regarding a parts features.
 */
struct Part{
    string name;
    map<string,double> parameters;
};

#endif // PART_H
