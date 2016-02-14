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
/**
 * @brief The MainWindow class
 *
 * The main window that contains al the other widgets for the GUI.
 */
class MainWindow: public QMainWindow{
public:
    /**
     * @brief MainWindow constructor
     *
     * Sets up al the layouts and widgets and connects the buttons
     * to the corresponding functions.
     */
    MainWindow();
    /**
     * @brief open
     *
     * Calls the open function in order to open a STL file.
     */
    void open();
    /**
     * @brief save
     *
     * Calls the save function that saves information regarding a part and its
     * image.
     */
    void save();
    /**
     * @brief keyPressEvent
     *
     * Manages the keyboards inputs from the user.
     */
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
