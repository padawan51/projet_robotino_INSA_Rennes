#include "cameraviewbutton.h"

CameraViewButton::CameraViewButton()
{
    v_index = -1;
    v_isPresent = false;
    v_isSelected = false;
    this->v_winW = 1000;
    this->v_winH = 1000;
}

CameraViewButton::CameraViewButton(int index, int winW, int winH)
{
    this->v_index = index;
    this->v_winW = winW;
    this->v_winH = winH;
    this->v_isSelected = false;
    v_isPresent = false;
    openCamera();
}

void CameraViewButton::sendPixmap()
{
    if(v_isSelected) emit image(v_pixmap);
}

void CameraViewButton::setIndex(int index)
{
    this->v_index = index;
    this->v_isSelected = false;
    openCamera();
}

void CameraViewButton::captureImage()
{
    //fx et fy sont des facteur de redimensionnement
    double fx;
    double fy;

    fx = (v_winW * 0.2601) / (IMG_W/2);
    fy = (v_winH * 0.25) / (IMG_H/2);

    if(!v_isSelected){
        cv::Mat frame = cv::imread(pathEmptyArea[0]);
        cv::VideoCapture cap(this->v_index, cv::CAP_DSHOW);

        cap.set(cv::CAP_PROP_FRAME_WIDTH, IMG_W);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_H);

        delay_s(1);
        for(int i = 0; i < 3; i++) cap >> frame;

        cv::resize(frame, frame, cv::Size(), fx, fy);
        this->v_pixmap = Mat2Pixmap(frame, true, 2); std::cout << "\tindex = " << v_index << std::endl;

        emit checkRunningCameras(v_index);
        v_isSelected = true;
    }
    else{
        cv::Mat frame = cv::Mat::zeros(static_cast<int>(IMG_H/2),
                                    static_cast<int>(IMG_W/2),
                                    CV_8UC3);
        cv::resize(frame, frame, cv::Size(), fx, fy);
        this->v_pixmap = Mat2Pixmap(frame);

        setStyleSheet(pushButtonEnabledStyle);

        emit image(v_pixmap);
        v_isSelected = false;
    }
}

bool CameraViewButton::getIsSelected() const
{
    return v_isSelected;
}

void CameraViewButton::setIsSelected(bool value)
{
    v_isSelected = value;

    if(!v_isSelected && v_isPresent) setStyleSheet(pushButtonEnabledStyle);
    else if(v_isSelected && v_isPresent) setStyleSheet(pushButtonStyle_1);
}

void CameraViewButton::setWinParam(const int w, const int h)
{
    //h et w sont respectivement la hauteur et la largeur de l'écran
    v_winH = h;
    v_winW = w;
}

bool CameraViewButton::getIsPresent() const
{
    return v_isPresent;
}

void CameraViewButton::openCamera()
{
    cv::VideoCapture cap(this->v_index, cv::CAP_DSHOW);

    this->v_isPresent = false;
    if(cap.isOpened()) v_isPresent = true;

    cap.release();

    if(v_isPresent){
        setEnabled(true);
        setStyleSheet(pushButtonEnabledStyle);
    }
    else{
        setEnabled(false);
        setStyleSheet(pushButtonDesabledStyle);
    }

    setText(QString::fromStdString(std::to_string(v_index)));
    setFixedSize(static_cast<int>(v_winW * 0.02772), static_cast<int>(v_winH * 0.01852));
}
