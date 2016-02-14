#include "glsurface.h"
#include <iostream>
#include <QDebug>
#include <QDir>

#include "imagefactory.h"
#include "featurefactory.h"
#include "templater.h"

#include "opencv2/highgui/highgui.hpp"

GLSurface::GLSurface():
    m_surface{new QOffscreenSurface},
    m_context{new QOpenGLContext},
    m_camera{new Camera},
    m_workplane{new Workplane(m_camera)},
    m_stlObject{new STLObject(m_camera)},
    m_workplaneState{false},
    m_rbo{640,480}
{
    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(32);
    fmt.setVersion(3,3);

    m_context->setFormat(fmt);
    m_context->create();

    m_surface->setFormat(fmt);
    m_surface->create();

    m_context->makeCurrent(m_surface);
}

Mat GLSurface::renderGL(){
    m_fbo.bind(GL_DRAW_FRAMEBUFFER);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(m_workplane->isCreated() && m_workplaneState){
        m_workplane->draw();
    }
    if(m_stlObject->isCreated()){
        m_stlObject->draw();
    }
    texture.bind();
    vector<unsigned char> data = texture.getData();
    m_fbo.release();
    //Turns the raw pixel data into a CV image
    return Mat(cvSize(m_rbo.width(),m_rbo.height()), CV_8UC4, &data[0], cv::Mat::AUTO_STEP).clone();
}

void GLSurface::initializeGL(){
    GLenum error = glewInit();
    if (error != GLEW_OK) {
        qFatal("Failed to initialize OpenGL: "
               "failed to initialize GLEW (%s)", glewGetErrorString(error));
    }
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 1);
    glViewport(0,0,m_rbo.width(),m_rbo.height());

    texture.create();
    texture.bind();
    texture.setParameter(GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    texture.setParameter(GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    texture.initialise(m_rbo.width(),m_rbo.height());
    texture.release();

    m_rbo.create();
    m_rbo.bind();
    m_rbo.setBufferStorage(GL_DEPTH_COMPONENT);
    m_rbo.release();

    m_fbo.create();
    m_fbo.bind(GL_FRAMEBUFFER);
    m_fbo.bindTexture2D(texture.getHandle());
    m_fbo.bindRenderBuffer(GL_DEPTH_ATTACHMENT,m_rbo);
    m_fbo.release();

    m_workplane->initialize();
    m_stlObject->initialize();
}

void GLSurface::loadSTLObject(std::vector<float> &data){
    m_stlObject->setData(data);
}

bool GLSurface::workplaneState() const{
    return m_workplaneState;
}

void GLSurface::setWorkplaneState(bool workplaneState){
    m_workplaneState = workplaneState;
}

void GLSurface::switchShadeState(){
    m_stlObject->switchShading();
}

void GLSurface::executeTool(int argc, char *argv[]){
    //ignore first argument
    int argumentOffset = 1;
    QString fileName = argv[argumentOffset];
    QStringList splitName = fileName.split(".");

    ++argumentOffset;
    String currentArg = argv[argumentOffset];
    if(currentArg == "-r"){
        m_rbo.setBufferSize(atoi(argv[argumentOffset+1]),atoi(argv[argumentOffset+2]));
        argumentOffset += 3;
    }

    initializeGL();

    if((argc - argumentOffset)%5 == 0){
        parser.open(QDir::currentPath() + "/" + fileName);

        vector<float> vertices = parser.getVertices();
        if(vertices.size() != 0){
            loadSTLObject(vertices);

            int nrOfCameras = (argc-argumentOffset)/5;
            for(int i = 0; i < nrOfCameras;++i){
                // Setting up new filename
                QString newName = splitName[0] + " - " +  QString(to_string(i).data()) + "." + splitName[1];
                m_camera->setPosition(atof(argv[argumentOffset]),atof(argv[argumentOffset+1]),atof(argv[argumentOffset+2]));
                m_camera->setDirection(atof(argv[argumentOffset+3]),atof(argv[argumentOffset+4]));
                argumentOffset += 5;

                Mat image = renderGL();
                ImageFactory::save(image,newName.toStdString());
                waitKey(0);
                switchShadeState();
                image = renderGL();
                cvtColor(image,image,CV_RGB2GRAY);
                vector<VisionObject> objects = ImageFactory::filterObjects(FeatureFactory::findConnectedComponents(image),image);
                FeatureFactory::savePartConfig(FeatureFactory::createParameterMap(objects[0],m_stlObject->getData(),
                                               Templater::generateGripperTemp(3,3,3,12)),newName);
                switchShadeState();
                cout << i << " Done" << endl;
            }
        }else{
            cout << "Invalid STL file selected." << endl;
        }
    }
}

