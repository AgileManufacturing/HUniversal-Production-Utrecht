#ifndef TEMPLATER_H
#define TEMPLATER_H

#include <vector>
#include <featurefactory.h>

class Templater{
public:
    static std::vector<std::vector<std::vector<int>>> generateGripperTemp
    (int arms,float armHeight,float armWidth,float maxDist);

    static pair<Point,int> getGripPoint(Mat &image, vector<vector<vector<int> > >& gTemplate);
private:
};

#endif // TEMPLATER_H
