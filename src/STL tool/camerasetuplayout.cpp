#include "camerasetuplayout.h"
#include <iostream>

#include <glm/glm.hpp>
#include "glm/ext.hpp"

using namespace std;

cameraSetupLayout::cameraSetupLayout(GLWidget *_glwidget):
    cameraSetup{new QGridLayout},
    cameraPos{new QLabel("Position")},
    directionVec{new QLabel("Orientation")},
    errorLabel{new QLabel},
    useCameraButton{new QPushButton("Set")},
    glwidget{_glwidget},
    xPosE{new QLineEdit},
    yPosE{new QLineEdit},
    zPosE{new QLineEdit},
    rollE{new QLineEdit},
    pitchE{new QLineEdit},
    yawE{new QLineEdit}
{
    QLabel* title = new QLabel("Camera");
    cameraSetup->addWidget(title,0,0);
    cameraSetup->setHorizontalSpacing(1);

    cameraSetup->addWidget(cameraPos,1,1,Qt::AlignCenter);

    xPosE->setMaximumSize(editBoxWidth,editBoxHeight);
    yPosE->setMaximumSize(editBoxWidth,editBoxHeight);
    zPosE->setMaximumSize(editBoxWidth,editBoxHeight);

    xPosE->setMaxLength(4);
    yPosE->setMaxLength(4);
    zPosE->setMaxLength(4);

    xPosE->setText("x");
    yPosE->setText("y");
    zPosE->setText("z");
    xPosE->setAlignment(Qt::AlignCenter);
    yPosE->setAlignment(Qt::AlignCenter);
    zPosE->setAlignment(Qt::AlignCenter);

    cameraSetup->addWidget(xPosE,2,0);
    cameraSetup->addWidget(yPosE,2,1);
    cameraSetup->addWidget(zPosE,2,2);

    cameraSetup->addWidget(directionVec,3,1,Qt::AlignCenter);

    pitchE->setMaximumSize(editBoxWidth,editBoxHeight);
    yawE->setMaximumSize(editBoxWidth,editBoxHeight);
    pitchE->setText("Pitch");
    yawE->setText("Yaw");

    pitchE->setMaxLength(5);
    yawE->setMaxLength(5);

    pitchE->setAlignment(Qt::AlignCenter);
    yawE->setAlignment(Qt::AlignCenter);

    cameraSetup->addWidget(pitchE,4,0);
    cameraSetup->addWidget(yawE,4,2);

    errorLabel->setStyleSheet("QLabel {color : red;}");
    cameraSetup->addWidget(errorLabel,5,2,1,2);


    useCameraButton->setMaximumWidth(editBoxWidth);
    cameraSetup->addWidget(useCameraButton,0,3);
    connect(useCameraButton,&QPushButton::clicked, this, &cameraSetupLayout::useCamera);

    connect(xPosE, &QLineEdit::editingFinished,xPosE,&QLineEdit::clearFocus);
    connect(yPosE, &QLineEdit::editingFinished,yPosE,&QLineEdit::clearFocus);
    connect(zPosE, &QLineEdit::editingFinished,zPosE,&QLineEdit::clearFocus);

    connect(pitchE, &QLineEdit::editingFinished,pitchE,&QLineEdit::clearFocus);
    connect(yawE, &QLineEdit::editingFinished,yawE,&QLineEdit::clearFocus);
}

cameraSetupLayout::~cameraSetupLayout(){
    delete cameraPos;
    delete directionVec;
    delete errorLabel;

    delete xPosE;
    delete yPosE;
    delete zPosE;

    delete rollE;
    delete pitchE;
    delete yawE;

    delete cameraSetup;
}

QGridLayout *cameraSetupLayout::getCameraSetupLayout(){
    return cameraSetup;
}

void cameraSetupLayout::setValues(){
    if(xPosE->text() != "" && yPosE->text() != "" && zPosE->text() != "" &&
           pitchE->text() != "" && yawE->text() != ""){
        cameraPosition = glm::vec3(xPosE->text().toFloat(),
                                   yPosE->text().toFloat(),
                                   zPosE->text().toFloat());

        //TO-DO:limit input values
        cameraDirection = glm::vec3{1.0f,0.0f,0.0f};
        //yaw
        cameraDirection = glm::rotateZ(cameraDirection,glm::radians(yawE->text().toFloat()));
        //pitch
        cameraDirection = glm::rotateY(cameraDirection,glm::radians(pitchE->text().toFloat()));

        cameraDirection = glm::normalize(cameraDirection);
        cout << glm::to_string(cameraDirection) << endl;

        cout << "Valid information thank you" << endl;

        errorLabel->setText("");
    }else{
        errorLabel->setText("Information is invalid");

        cout << "Wat is dit nou" << endl;
    }
}

void cameraSetupLayout::useCamera(){
    setValues();
    glwidget->setCamera(cameraPosition,cameraDirection);
}

glm::vec3 cameraSetupLayout::getCameraPosition(){
    return cameraPosition;
}

glm::vec3 cameraSetupLayout::getCameraDirection(){
    return cameraDirection;
}

