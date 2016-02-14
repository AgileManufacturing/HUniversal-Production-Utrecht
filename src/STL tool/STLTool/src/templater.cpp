#include "templater.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

vector<vector<vector<int>>> Templater::generateGripperTemp(int arms,float armHeight,float armWidth,float maxDist){
    vector<vector<vector<int>>> templates;
    //currently only supports min distance and max distance as the gripper
    //can only fully grip or fully expand
    templates.resize(2);

    float angleBetweenArms =(360/arms)/(180/M_PI);
    //Possible orientations before the arm goes back to a similar one
    int orientations = 36 / arms;
    //Calculate minimal distance
    int minDist = ceil(cos((((arms-2)*180)/arms)/2) * armHeight);

    int tempHeight = (maxDist + armHeight) * 2;
    int tempWidth = (maxDist + armWidth) * 2;
    vector<int> firstArm(tempWidth * tempHeight);
    vector<int> gripperTemplate(firstArm.size());
    if(minDist == 0){
        minDist = armWidth;
    }

    int centerX = tempWidth / 2;
    int centerY = tempHeight / 2;
    double currentAngle = 0;

    int checkX, checkY;
    int currentDist;
    for(int distance = 0; distance < 2; ++distance){
        fill(firstArm.begin(),firstArm.end(),0);

        currentDist = (distance == 0)? minDist : maxDist;

        for(int i = 0; i < orientations;++i){
            currentAngle = (M_PI/18)*i;
            //clear vector
            fill(gripperTemplate.begin(),gripperTemplate.end(),0);
            //Add first arm first rotation to template
            if(i == 0){
                for(int y = centerY + (armHeight/2); y >= centerY - (armHeight/2);--y){
                    for(int x = centerX + currentDist; x < centerX + currentDist + armWidth;++x){
                        firstArm[(y * tempWidth) + x] = 1;
                    }
                    cout << endl;
                }
            }
            for(int y = 0; y < tempHeight;++y){
                for(int x = 0; x < tempWidth;++x){
                    for(int a = 0; a < arms;++a){
                        checkX = centerX + ((x - centerX) * cos((a*angleBetweenArms) + currentAngle))
                                - ((y - centerY) * sin((a*angleBetweenArms) + currentAngle));
                        checkY = centerY + ((x - centerX) * sin((a*angleBetweenArms) + currentAngle))
                                + ((y - centerY) * cos((a*angleBetweenArms) + currentAngle));

                        if(checkX > 0 && checkX < tempWidth && checkY > 0
                                && checkY < tempHeight){
                            if(firstArm[(checkY * tempWidth) + checkX] == 1){
                                gripperTemplate[(y * tempWidth) + x] = 1;
                            }
                        }
                    }
                }
            }
            templates[distance].push_back(gripperTemplate);
        }
    }
    return templates;
}

pair<Point, int> Templater::getGripPoint(cv::Mat& image, vector<vector<vector<int>>>& gTemplate){
    Point center = FeatureFactory::findCenter(FeatureFactory::findConnectedComponents(image)[0]);
    threshold(image,image,100,255,CV_THRESH_BINARY);

    pair<Point,int> bestPoint = make_pair(Point(0,0),0);
    int offset = -sqrt(gTemplate[0][0].size())/2;

    int sumPos =0, sumNeg= 0;
    //IMAGE
    for(int y = 0; y < image.size().height;++y){
        for(int x = 0; x < image.size().width;++x){
            if(static_cast<int>(image.data[(y * image.size().width)+x]) != 0){
                //KERNEL
                for(int a = 0; a < gTemplate[0].size();++a){
                    sumNeg = 0;
                    sumPos = 0;
                    for(int ky = 0; ky < sqrt(gTemplate[0][a].size());++ky){
                        for(int kx = 0;kx < sqrt(gTemplate[0][a].size());++kx){
                            if(gTemplate[0][a][(ky*sqrt(gTemplate[0][a].size()))+kx] == 1){
                                if(y +(ky + offset) >= 0 && y +(ky + offset) < image.size().height &&
                                        x +(kx + offset) >= 0 && x +(kx + offset) < image.size().width){
                                    if(static_cast<int>(image.data[((y +(ky + offset)) * image.size().width) + (x + (kx + offset))]) !=0){
                                        ++sumNeg;
                                    }else{
                                        ++sumPos;
                                    }
                                }else{
                                    ++sumPos;
                                }
                            }
                        }
                    }
                    if(sumNeg/(double)(sumNeg+sumPos) >= 1){
                        sumNeg = 0;
                        sumPos = 0;
                        for(int ky = 0; ky < sqrt(gTemplate[1][a].size());++ky){
                            for(int kx = 0;kx < sqrt(gTemplate[1][a].size());++kx){
                                if(gTemplate[1][a][(ky*sqrt(gTemplate[1][a].size()))+kx] == 1){
                                    if(y +(ky + offset) >= 0 && y +(ky + offset) < image.size().height &&
                                            x +(kx + offset) >= 0 && x +(kx + offset) < image.size().width){
                                        if(static_cast<int>(image.data[((y +(ky + offset)) * image.size().width) + (x + (kx + offset))]) != 0){
                                            ++sumNeg;
                                        }else{
                                            ++sumPos;
                                        }
                                    }else{
                                        ++sumPos;
                                    }
                                }
                            }
                        }
                        if(sumPos/(double)(sumNeg+sumPos) == 1){
                            if(FeatureFactory::getEuclideanDistance(Point(x,y),center) < FeatureFactory::getEuclideanDistance(bestPoint.first,center)){
                                bestPoint.first = Point(x,y);
                                bestPoint.second = a * 10;
                            }

                        }
                    }
                }
            }
        }
    }
    return make_pair(bestPoint.first-center,bestPoint.second);
}


