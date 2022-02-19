#include "acquisitionthread.h"

using namespace cv;

AcquisitionThread::AcquisitionThread(QObject *parent)
    :QThread(parent)
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
    v_isInit = false;
    v_IdCam = new int;
    v_camIsOpen = false;
    v_isSnapshot = false;
}

AcquisitionThread::AcquisitionThread(int *numCam) : v_IdCam(numCam)
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
    v_isInit = false;
    v_camIsOpen = false;
    v_isSnapshot = false;

    init();
}

AcquisitionThread::~AcquisitionThread()
{
    delete v_IdCam;
}

void AcquisitionThread::run(){
    try {
        if(v_camIsOpen){
            if(!v_isSnapshot){
                while(v_cap.isOpened() && !isInterruptionRequested()){
                    v_cap >> v_frame;
                    emit newFrame(v_frame);
                }

                cv::Mat frame_ = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
                emit newFrame(frame_);
            }
            else{
                v_cap >> v_frame;
                v_cap >> v_frame;
            }
        }
    }
    catch (...) {

    }
}

cv::Mat AcquisitionThread::getFrame()
{
    if(v_isSnapshot) this->start();
    while(this->isRunning());
    return v_frame;
}

void AcquisitionThread::setIDCam(int numCam)
{
    if(!v_isInit) init();

    *v_IdCam = numCam;
}

void AcquisitionThread::setIsSnapshot(bool value)
{
    v_isSnapshot = value;
}

void AcquisitionThread::init()
{
    v_cap.set(CAP_PROP_FRAME_WIDTH, IMG_W);
    v_cap.set(CAP_PROP_FRAME_HEIGHT, IMG_H);
    v_cap.set(CAP_PROP_AUTOFOCUS, 0);
    //cap.set(CAP_PROP_BRIGHTNESS, 120.0);

    v_isInit = true;
}

bool AcquisitionThread::getCamIsOpen() const
{
    return v_camIsOpen;
}

bool AcquisitionThread::launchCamera()
{
    if(v_cap.open(*this->v_IdCam, cv::CAP_DSHOW)){
        v_camIsOpen = true;
        return true;
    }
    else{
        v_camIsOpen = false;
        return false;
    }
}

bool AcquisitionThread::stopCamera()
{
    if(v_cap.isOpened() && (v_camIsOpen || !v_camIsOpen)){
        v_cap.release();
        v_camIsOpen = false;
    }
    else if(v_camIsOpen) v_camIsOpen = false;

    return true;
}

int AcquisitionThread::getIdCam()
{
    return *v_IdCam;
}

