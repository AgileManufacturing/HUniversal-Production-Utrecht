#ifndef PART_H
#define PART_H

#include <map>

using namespace std;

struct Part{
    string name;
    map<string,double> parameters;
};

#endif // PART_H
