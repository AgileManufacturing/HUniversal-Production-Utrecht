#ifndef CAMERASETUPLAYOUT_H
#define CAMERASETUPLAYOUT_H

#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <glm/glm.hpp>
#include "glwidget.h"

/**
 * @brief The cameraSetupLayout class
 *
 * This is a layout class that is used by the mainwindow. It contains
 * several input fields that are used to specify a camera position and orientation.
 */
class cameraSetupLayout : public QObject{
public:
    /**
     * @brief cameraSetupLayout constructor
     * @param _glwidget The widget that is updated when the camera
     * values are used.
     */
    cameraSetupLayout(GLWidget *_glwidget);
    ~cameraSetupLayout();
    /**
     * @brief getCameraSetupLayout
     * @return Returns the main layout.
     */
    QGridLayout* getCameraSetupLayout();
    /**
     * @brief setValues
     *
     * This function checks and saves the user specified parameters
     * in the variables.
     */
    void setValues();
    /**
     * @brief useCamera
     *
     * This function updates the camera of the glWidget with the
     * saved variables.
     */
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
