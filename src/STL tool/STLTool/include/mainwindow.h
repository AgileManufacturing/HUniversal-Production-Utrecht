#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include "glwidget.h"
#include "stlparser.h"
#include <QLayout>
#include <QFormLayout>
#include <QPushButton>


class cameraSetupLayout;

class MainWindow: public QMainWindow{
public:
    MainWindow();

    void open();
    void save();

    void keyPressEvent(QKeyEvent *) override;


private:
    void enableWorkplane();
    void addCamera();
    void removeCamera();

    QPushButton* addCameraButton;
    QPushButton* removeCameraButton;

    int remainingCameraSlots;

    STLParser* parser;
    GLWidget* glwidget;
    QWidget* centerWidget;

    QHBoxLayout* mainLayout;
    QHBoxLayout* leftLayout;
    QHBoxLayout* m_buttonLayout;
    QVBoxLayout* rightLayout;

    std::vector<cameraSetupLayout*> cameras;

    QMenuBar* menubar;

    QAction* m_open;
    QAction* m_save;
    QAction* m_enableWorkplane;
    QAction* m_enableShading;
};

#endif // MAINWINDOW_H
