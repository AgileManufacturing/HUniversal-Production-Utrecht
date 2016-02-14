#include "mainwindow.h"
#include <iostream>
#include <QKeyEvent>

#include <QFileDialog>
#include <QImage>

#include <QPushButton>
#include <QLineEdit>
#include <QLabel>

#include "QInputDialog"

#include <QColor>

#include "camerasetuplayout.h"

using namespace std;

MainWindow::MainWindow():
    centerWidget{new QWidget},
    glwidget{new GLWidget(640,480)},
    remainingCameraSlots{3},
    mainLayout{new QHBoxLayout},
    leftLayout{new QHBoxLayout},
    rightLayout{new QVBoxLayout},
    m_buttonLayout{new QHBoxLayout},
    addCameraButton{new QPushButton("Add camera")},
    removeCameraButton{new QPushButton("Remove camera")},
    parser{new STLParser},
    menubar{new QMenuBar},
    m_open{new QAction("Open",nullptr)},
    m_save{new QAction("Save",nullptr)},
    m_enableWorkplane{new QAction("Workplane", nullptr)},
    m_enableShading{new QAction("Shade",nullptr)}
{
    resize(1280,720);
    setMinimumSize(1280,720);
    setMaximumSize(1280,720);
    setCentralWidget(centerWidget);
    glwidget->setSizePolicy(QSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed));

    rightLayout->setAlignment(Qt::AlignTop);
    m_buttonLayout->addWidget(addCameraButton);
    m_buttonLayout->addWidget(removeCameraButton);
    rightLayout->addLayout(m_buttonLayout);
    leftLayout->addWidget(glwidget);
    leftLayout->setSizeConstraint(QLayout::SetFixedSize);

    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);

    centerWidget->setLayout(mainLayout);
    setMenuBar(menubar);

    //TODO: Dit kan netter... Mainwindow functies opruimen en direkt callen.
    connect(m_enableShading,&QAction::triggered,glwidget,&GLWidget::switchShading);
    connect(m_open,&QAction::triggered,this,&MainWindow::open);
    connect(m_enableWorkplane,&QAction::triggered,this,&MainWindow::enableWorkplane);
    connect(m_save,&QAction::triggered,this,&MainWindow::save);
    connect(addCameraButton,&QPushButton::clicked,this,&MainWindow::addCamera);
    connect(removeCameraButton,&QPushButton::clicked,this,&MainWindow::removeCamera);



    menubar->addAction(m_open);
    menubar->addAction(m_save);
    menubar->addAction(m_enableShading);
    m_enableWorkplane->setCheckable(true);
    menubar->addAction(m_enableWorkplane);
    if(remainingCameraSlots > 0){
        cameras.push_back(new cameraSetupLayout(glwidget));
        --remainingCameraSlots;
    }
    rightLayout->addLayout(cameras.back()->getCameraSetupLayout());
}

void MainWindow::open()
{
    cout << "Open function triggered!" << endl;

    QString path = QFileDialog::getOpenFileName(glwidget,QDir::currentPath());
    parser->open(path);
    vector<float> vertices = parser->getVertices();
    if(vertices.size()!= 0){
        glwidget->loadSTLObject(vertices);
    }else{
        cout << "No valid file selected!" << endl;
    }
}

void MainWindow::save(){
    bool succes;
    QString imageName = QInputDialog::getText(this,"Save","Image name:",QLineEdit::Normal,"",&succes);
    if(succes){
        for(int i = 0; i < cameras.size();++i){
            cameras[i]->useCamera();
            glwidget->save(imageName+QString(to_string(i).data()));
        }
        cameras[0]->useCamera();
    }
}

void MainWindow::keyPressEvent(QKeyEvent * e){
    if(e->modifiers() == Qt::ControlModifier ){
        if(e->key() == Qt::Key_1){
            open();
        }
    }else{
//        glwidget->keyPressEvent(e);
    }
}


void MainWindow::addCamera(){
    if(remainingCameraSlots > 0){
        cameras.push_back(new cameraSetupLayout(glwidget));
        --remainingCameraSlots;
        rightLayout->addLayout(cameras.back()->getCameraSetupLayout());
    }
    cout << "Camera size: " << cameras.size() << endl;
}

void MainWindow::removeCamera(){
    if(cameras.size() > 1){
        ++remainingCameraSlots;
        delete cameras.back();
        cameras.pop_back();
    }
    cout << "Camera size: " << cameras.size() << endl;
}

void MainWindow::enableWorkplane(){
    if(m_enableWorkplane->isChecked()){
        glwidget->enableWorkplane(true);
    }else{
        glwidget->enableWorkplane(false);
    }
}
