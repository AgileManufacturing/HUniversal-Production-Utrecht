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

    glm::vec3 getCameraPosition();
    glm::vec3 getCameraDirection();

    void setValues();
    void useCamera();

private:
    GLWidget* glwidget;
    QGridLayout* cameraSetup;

    int editBoxHeight = 30;
    int editBoxWidth = 60;

    QLabel *cameraPos, *directionVec, *errorLabel;

    QLineEdit* xPosE, *yPosE, *zPosE;
    QLineEdit* rollE, *pitchE, *yawE;

    QPushButton* useCameraButton;

    glm::vec3 cameraPosition;
    glm::vec3 cameraDirection;

};

#endif // CAMERASETUPLAYOUT_H
