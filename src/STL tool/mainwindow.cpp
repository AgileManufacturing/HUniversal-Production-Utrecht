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
    glwidget{new GLWidget(800,640)},
    remainingCameraSlots{3},
    mainLayout{new QHBoxLayout},
    leftLayout{new QHBoxLayout},
    rightLayout{new QVBoxLayout},
    addCameraButton{new QPushButton("Add camera")},
    m_imageFactory{new ImageFactory},
    parser{new STLParser},
    menubar{new QMenuBar},
    m_open{new QAction("Open",nullptr)},
    m_save{new QAction("Save",nullptr)},
    m_enableWorkplane{new QAction("Workplane", nullptr)}
{
    resize(1280,720);
    setMinimumSize(1280,720);
    setMaximumSize(1280,720);

    //setStyleSheet("QMainWindow{background: rgb(51, 51, 51);}");

    setCentralWidget(centerWidget);
    glwidget->setSizePolicy(QSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed));

    rightLayout->setAlignment(Qt::AlignTop);

    rightLayout->addWidget(addCameraButton);
    leftLayout->addWidget(glwidget);
    leftLayout->setSizeConstraint(QLayout::SetFixedSize);

    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);

    centerWidget->setLayout(mainLayout);
    setMenuBar(menubar);

    connect(m_open,&QAction::triggered,this,&MainWindow::open);
    connect(m_enableWorkplane,&QAction::triggered,this,&MainWindow::enableWorkplane);
    connect(m_save,&QAction::triggered,this,&MainWindow::save);
    connect(addCameraButton,&QPushButton::clicked,this,&MainWindow::addCamera);

    menubar->addAction(m_open);
    menubar->addAction(m_save);
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

//    path = QFileDialog::getOpenFileName(glwidget,QDir::currentPath());
//    parser->open(path);
//    vector<float> vertices2 = parser->getVertices();

//    for(int i = 0; i < 50*18;++i){
//        if(static_cast<int>(vertices[i]) != static_cast<int>(vertices2[i])){
//            cout << i/18 << "-"<< i <<  " Er gaat wat mis kerel" << endl <<
//                    vertices[i] << " - " << vertices2[i] << endl;
//        }
//    }
//    for(int i = 0; i < 36; i+=3){
//        if(i%18 == 0){
//            cout << endl;
//        }
//        cout << vertices[i] << " - " << vertices[i+1] << " - " << vertices[i+2] << endl;
//    }
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
        m_imageFactory->save(glwidget->grabFramebuffer(),imageName.toStdString());
//        int cameraNr = 1;
//        for(cameraSetupLayout* camera : cameras){
//            camera->useCamera();
//            m_imageFactory->save(glwidget->grabFramebuffer(),imageName.toStdString() + to_string(cameraNr));
//            cameraNr++;
//        }
    }
}

void MainWindow::keyPressEvent(QKeyEvent * e){
    if(e->modifiers() == Qt::ControlModifier ){
        if(e->key() == Qt::Key_1){
            open();
        }
    }else{
        glwidget->keyPressEvent(e);
    }
}


void MainWindow::addCamera(){
    if(remainingCameraSlots > 0){
        cameras.push_back(new cameraSetupLayout(glwidget));
        --remainingCameraSlots;
        rightLayout->addLayout(cameras.back()->getCameraSetupLayout());
    }
}

void MainWindow::enableWorkplane(){
    if(m_enableWorkplane->isChecked()){
        glwidget->enableWorkplane(true);
    }else{
        glwidget->enableWorkplane(false);
    }
}
