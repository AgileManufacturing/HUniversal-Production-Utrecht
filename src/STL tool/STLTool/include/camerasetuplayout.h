#ifndef CAMERASETUPLAYOUT_H
#define CAMERASETUPLAYOUT_H

#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <glm/glm.hpp>
#include "glwidget.h"

class cameraSetupLayout : public QObject{
public:
    cameraSetupLayout(GLWidget *_glwidget);
    ~cameraSetupLayout();

    QGridLayout* getCameraSetupLayout();

    void setValues();
    void useCamera();

private:
    GLWidget* glwidget;
    QGridLayout* cameraSetup;

    int editBoxHeight = 30;
    int editBoxWidth = 60;

    QLabel *cameraPos, *directionVec, *errorLabel, *name;

    QLineEdit* xPosE, *yPosE, *zPosE;
    QLineEdit* rollE, *pitchE, *yawE;

    QPushButton* useCameraButton;

    float x,y,z;
    float yaw,pitch;

};

#endif // CAMERASETUPLAYOUT_H
