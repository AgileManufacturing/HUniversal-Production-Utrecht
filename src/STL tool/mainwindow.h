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
#include "imagefactory.h"


class cameraSetupLayout;

class MainWindow: public QMainWindow{
public:
    MainWindow();

    void open();
    void save();

    void keyPressEvent(QKeyEvent *) override;

    void addCamera();
    void enableWorkplane();

private:
    QPushButton* addCameraButton;

    int remainingCameraSlots;

    ImageFactory* m_imageFactory;
    STLParser* parser;
    GLWidget* glwidget;
    QWidget* centerWidget;

    QHBoxLayout* mainLayout;
    QHBoxLayout* leftLayout;
    QVBoxLayout* rightLayout;

    std::vector<cameraSetupLayout*> cameras;

    QMenuBar* menubar;

    QAction* m_open;
    QAction* m_save;
    QAction* m_enableWorkplane;
};

#endif // MAINWINDOW_H
