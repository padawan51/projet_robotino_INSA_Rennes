#include "thresholddialog.h"
#include "ui_thresholddialog.h"

ThresholdDialog::ThresholdDialog(int winW, int winH, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ThresholdDialog),
    v_winW(winW),
    v_winH(winH)
{
    init();
    configWidgets();
    connects();
}

ThresholdDialog::~ThresholdDialog()
{
    delete v_cam_L;
    delete v_cam_M;
    delete v_cam_R;
    delete ui;
}

void ThresholdDialog::thresholdChanged(int h_min,
                                       int h_max,
                                       int s_min,
                                       int s_max,
                                       int v_min,
                                       int v_max,
                                       const cv::Mat &image)
{
    cv::Mat imgHSV, mask;

    cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV);

    cv::inRange(imgHSV, cv::Scalar(h_min, s_min, v_min),
                cv::Scalar(h_max, s_max, v_max), mask);

    QImage qImage_mask(static_cast<uchar*>(mask.data),
                       mask.cols,
                       mask.rows,
                       static_cast<int>(mask.step),
                       QImage::Format_Grayscale8);
    auto *qPix_mask = new QPixmap(mask.cols, mask.rows);

    qPix_mask->convertFromImage(qImage_mask);
    *qPix_mask = qPix_mask->scaled(mask.cols/2, mask.rows/2);

    ui->imgThreshView->setPixmap(*qPix_mask);

    delete  qPix_mask; qPix_mask = nullptr;
}

void ThresholdDialog::configWidgets()
{
    cv::Mat frame_init;
    QPixmap qPix_init;
    QFont font = QFont("MS Shell Dlg 2", static_cast<int>(v_winH * 0.00741));

    //...
    setGeometry((v_winW - static_cast<int>(v_winW * 0.428900))/2,
                (v_winH - static_cast<int>(v_winH * 0.527800))/2,
                static_cast<int>(v_winW * 0.428900),
                static_cast<int>(v_winH * 0.527800));
    setFixedSize(static_cast<int>(v_winW * 0.428900), static_cast<int>(v_winH * 0.527800));
    //FIN ...

    //...
    frame_init = cv::Mat::zeros(static_cast<int>(IMG_H/2),
                                static_cast<int>(IMG_W/2),
                                CV_8UC3);
    cv::resize(frame_init, frame_init, cv::Size(), v_fx, v_fy);
    qPix_init = Mat2Pixmap(frame_init);

    ui->imgView->setGeometry(static_cast<int>(v_winW * 0.165600),
                             static_cast<int>(v_winH * 0.009300),
                             static_cast<int>(v_winW * 0.260600),
                             static_cast<int>(v_winH * 0.250100));
    ui->imgView->setFont(font);
    ui->imgView->setPixmap(qPix_init);
    ui->imgView->setStyleSheet(labelStyle_3);

    ui->imgThreshView->setGeometry(static_cast<int>(v_winW * 0.165600),
                                   static_cast<int>(v_winH * 0.268600),
                                   static_cast<int>(v_winW * 0.260600),
                                   static_cast<int>(v_winH * 0.250100));
    ui->imgThreshView->setFont(font);
    ui->imgThreshView->setPixmap(qPix_init);
    ui->imgThreshView->setStyleSheet(labelStyle_3);
    //FIN ...

    //...
    ui->cameraGroup->setGeometry(static_cast<int>(v_winW * 0.005500),
                                 static_cast<int>(v_winH * 0.009300),
                                 static_cast<int>(v_winW * 0.154800),
                                 static_cast<int>(v_winH * 0.056500));
    ui->cameraGroup->setFont(font);

    ui->camera_l->setGeometry(static_cast<int>(v_winW * 0.002800),
                              static_cast<int>(v_winH * 0.018600),
                              static_cast<int>(v_winW * 0.040800),
                              static_cast<int>(v_winH * 0.019500));
    ui->camera_l->setFont(font);

    ui->camera_m->setGeometry(static_cast<int>(v_winW * 0.057100),
                              static_cast<int>(v_winH * 0.018600),
                              static_cast<int>(v_winW * 0.040800),
                              static_cast<int>(v_winH * 0.019500));
    ui->camera_m->setFont(font);

    ui->camera_r->setGeometry(static_cast<int>(v_winW * 0.111300),
                              static_cast<int>(v_winH * 0.018600),
                              static_cast<int>(v_winW * 0.040800),
                              static_cast<int>(v_winH * 0.019500));
    ui->camera_r->setFont(font);
    //FIN ...

    //...
    ui->sourceGroup->setGeometry(static_cast<int>(v_winW * 0.005500),
                                 static_cast<int>(v_winH * 0.083400),
                                 static_cast<int>(v_winW * 0.154800),
                                 static_cast<int>(v_winH * 0.074100));
    ui->sourceGroup->setFont(font);

    ui->src_img->setGeometry(static_cast<int>(v_winW * 0.002800),
                             static_cast<int>(v_winH * 0.018600),
                             static_cast<int>(v_winW * 0.070600),
                             static_cast<int>(v_winH * 0.019500));
    ui->src_img->setFont(font);

    ui->src_cam->setGeometry(static_cast<int>(v_winW * 0.002800),
                             static_cast<int>(v_winH * 0.046300),
                             static_cast<int>(v_winW * 0.073300),
                             static_cast<int>(v_winH * 0.019500));
    ui->src_cam->setFont(font);

    ui->openImgFileBtn->setGeometry(static_cast<int>(v_winW * 0.084200),
                                    static_cast<int>(v_winH * 0.018600),
                                    static_cast<int>(v_winW * 0.021800),
                                    static_cast<int>(v_winH * 0.023200));
    ui->openImgFileBtn->setFont(font);

    ui->captureBtn->setGeometry(static_cast<int>(v_winW * 0.084200),
                                static_cast<int>(v_winH * 0.046300),
                                static_cast<int>(v_winW * 0.038100),
                                static_cast<int>(v_winH * 0.023200));
    ui->captureBtn->setFont(font);
    //FIN ...

    //...
    ui->originThresh->setGeometry(static_cast<int>(v_winW * 0.005500),
                                  static_cast<int>(v_winH * 0.176000),
                                  static_cast<int>(v_winW * 0.108600),
                                  static_cast<int>(v_winH * 0.019500));
    ui->originThresh->setFont(font);

    ui->hue_min_text->setGeometry(static_cast<int>(v_winW * 0.010900),
                                  static_cast<int>(v_winH * 0.222300),
                                  static_cast<int>(v_winW * 0.040800),
                                  static_cast<int>(v_winH * 0.019500));
    ui->hue_min_text->setFont(font);

    ui->hue_max_text->setGeometry(static_cast<int>(v_winW * 0.010900),
                                  static_cast<int>(v_winH * 0.254700),
                                  static_cast<int>(v_winW * 0.040800),
                                  static_cast<int>(v_winH * 0.019500));
    ui->hue_max_text->setFont(font);

    ui->sat_min_text->setGeometry(static_cast<int>(v_winW * 0.010900),
                                  static_cast<int>(v_winH * 0.287100),
                                  static_cast<int>(v_winW * 0.048900),
                                  static_cast<int>(v_winH * 0.019500));
    ui->sat_min_text->setFont(font);

    ui->sat_max_text->setGeometry(static_cast<int>(v_winW * 0.010900),
                                  static_cast<int>(v_winH * 0.319500),
                                  static_cast<int>(v_winW * 0.048900),
                                  static_cast<int>(v_winH * 0.019500));
    ui->sat_max_text->setFont(font);

    ui->val_min_text->setGeometry(static_cast<int>(v_winW * 0.010900),
                                  static_cast<int>(v_winH * 0.351900),
                                  static_cast<int>(v_winW * 0.040800),
                                  static_cast<int>(v_winH * 0.019500));
    ui->val_min_text->setFont(font);

    ui->val_max_text->setGeometry(static_cast<int>(v_winW * 0.010900),
                                  static_cast<int>(v_winH * 0.384300),
                                  static_cast<int>(v_winW * 0.040800),
                                  static_cast<int>(v_winH * 0.019500));
    ui->val_max_text->setFont(font);

    ui->hue_min->setGeometry(static_cast<int>(v_winW * 0.065200),
                             static_cast<int>(v_winH * 0.222300),
                             static_cast<int>(v_winW * 0.054300),
                             static_cast<int>(v_winH * 0.019500));
    ui->hue_min->setFont(font);

    ui->hue_max->setGeometry(static_cast<int>(v_winW * 0.065200),
                             static_cast<int>(v_winH * 0.254700),
                             static_cast<int>(v_winW * 0.054300),
                             static_cast<int>(v_winH * 0.019500));
    ui->hue_max->setFont(font);

    ui->sat_min->setGeometry(static_cast<int>(v_winW * 0.065200),
                             static_cast<int>(v_winH * 0.287100),
                             static_cast<int>(v_winW * 0.054300),
                             static_cast<int>(v_winH * 0.019500));
    ui->sat_min->setFont(font);

    ui->sat_max->setGeometry(static_cast<int>(v_winW * 0.065200),
                             static_cast<int>(v_winH * 0.319500),
                             static_cast<int>(v_winW * 0.054300),
                             static_cast<int>(v_winH * 0.019500));
    ui->sat_max->setFont(font);

    ui->val_min->setGeometry(static_cast<int>(v_winW * 0.065200),
                             static_cast<int>(v_winH * 0.351900),
                             static_cast<int>(v_winW * 0.054300),
                             static_cast<int>(v_winH * 0.019500));
    ui->val_min->setFont(font);

    ui->val_max->setGeometry(static_cast<int>(v_winW * 0.065200),
                             static_cast<int>(v_winH * 0.384300),
                             static_cast<int>(v_winW * 0.054300),
                             static_cast<int>(v_winH * 0.019500));
    ui->val_max->setFont(font);

    ui->lcd_hue_min->setGeometry(static_cast<int>(v_winW * 0.124900),
                                 static_cast<int>(v_winH * 0.222300),
                                 static_cast<int>(v_winW * 0.027200),
                                 static_cast<int>(v_winH * 0.021300));
    ui->lcd_hue_min->setFont(font);

    ui->lcd_hue_max->setGeometry(static_cast<int>(v_winW * 0.124900),
                                 static_cast<int>(v_winH * 0.254700),
                                 static_cast<int>(v_winW * 0.027200),
                                 static_cast<int>(v_winH * 0.021300));
    ui->lcd_hue_max->setFont(font);

    ui->lcd_sat_min->setGeometry(static_cast<int>(v_winW * 0.124900),
                                 static_cast<int>(v_winH * 0.287100),
                                 static_cast<int>(v_winW * 0.027200),
                                 static_cast<int>(v_winH * 0.021300));
    ui->lcd_sat_min->setFont(font);

    ui->lcd_sat_max->setGeometry(static_cast<int>(v_winW * 0.124900),
                                 static_cast<int>(v_winH * 0.319500),
                                 static_cast<int>(v_winW * 0.027200),
                                 static_cast<int>(v_winH * 0.021300));
    ui->lcd_sat_max->setFont(font);

    ui->lcd_val_min->setGeometry(static_cast<int>(v_winW * 0.124900),
                                 static_cast<int>(v_winH * 0.351900),
                                 static_cast<int>(v_winW * 0.027200),
                                 static_cast<int>(v_winH * 0.021300));
    ui->lcd_val_min->setFont(font);

    ui->lcd_val_max->setGeometry(static_cast<int>(v_winW * 0.124900),
                                 static_cast<int>(v_winH * 0.384300),
                                 static_cast<int>(v_winW * 0.027200),
                                 static_cast<int>(v_winH * 0.021300));
    ui->lcd_val_max->setFont(font);

    ui->razBtn->setGeometry(static_cast<int>(v_winW * 0.054300),
                            static_cast<int>(v_winH * 0.416700),
                            static_cast<int>(v_winW * 0.043500),
                            static_cast<int>(v_winH * 0.023200));
    ui->razBtn->setFont(font);

    ui->saveHSVBtn->setGeometry(static_cast<int>(v_winW * 0.032600),
                                static_cast<int>(v_winH * 0.490800),
                                static_cast<int>(v_winW * 0.043500),
                                static_cast<int>(v_winH * 0.023200));
    ui->saveHSVBtn->setFont(font);

    ui->canceledBtn->setGeometry(static_cast<int>(v_winW * 0.086900),
                                 static_cast<int>(v_winH * 0.490800),
                                 static_cast<int>(v_winW * 0.043500),
                                 static_cast<int>(v_winH * 0.023200));
    ui->canceledBtn->setFont(font);
    //FIN ...
}

void ThresholdDialog::connects()
{
    connect(ui->hue_min, SIGNAL(valueChanged(int)), ui->lcd_hue_min, SLOT(display(int)));
    connect(ui->hue_max, SIGNAL(valueChanged(int)), ui->lcd_hue_max, SLOT(display(int)));
    connect(ui->sat_min, SIGNAL(valueChanged(int)), ui->lcd_sat_min, SLOT(display(int)));
    connect(ui->sat_max, SIGNAL(valueChanged(int)), ui->lcd_sat_max, SLOT(display(int)));
    connect(ui->val_min, SIGNAL(valueChanged(int)), ui->lcd_val_min, SLOT(display(int)));
    connect(ui->val_max, SIGNAL(valueChanged(int)), ui->lcd_val_max, SLOT(display(int)));
    connect(ui->hue_min, SIGNAL(valueChanged(int)), this, SLOT(threshChanged()));
    connect(ui->hue_max, SIGNAL(valueChanged(int)), this, SLOT(threshChanged()));
    connect(ui->sat_min, SIGNAL(valueChanged(int)), this, SLOT(threshChanged()));
    connect(ui->sat_max, SIGNAL(valueChanged(int)), this, SLOT(threshChanged()));
    connect(ui->val_min, SIGNAL(valueChanged(int)), this, SLOT(threshChanged()));
    connect(ui->val_max, SIGNAL(valueChanged(int)), this, SLOT(threshChanged()));
    connect(ui->camera_l, &QPushButton::clicked, this, &ThresholdDialog::setHSVValue);
    connect(ui->camera_r, &QPushButton::clicked, this, &ThresholdDialog::setHSVValue);
    connect(ui->camera_m, &QPushButton::clicked, this, &ThresholdDialog::setHSVValue);
    connect(ui->openImgFileBtn, &QPushButton::clicked, this, &ThresholdDialog::openFile);
    connect(ui->captureBtn, &QPushButton::clicked, this, &ThresholdDialog::capture);
    connect(ui->src_cam, &QPushButton::clicked, this, &ThresholdDialog::useCamera);
    connect(ui->src_img, &QPushButton::clicked, this, &ThresholdDialog::useImageFile);
    connect(ui->razBtn, &QPushButton::clicked, this, &ThresholdDialog::raz);
    connect(ui->saveHSVBtn, &QPushButton::clicked, this, &ThresholdDialog::changeHSV);
    connect(ui->canceledBtn, &QPushButton::clicked, this, &ThresholdDialog::close);
    connect(ui->originThresh, &QPushButton::clicked, this, &ThresholdDialog::configOriginThresh);
    connect(this, &ThresholdDialog::displayCamView, this, &ThresholdDialog::displayView);
}

void ThresholdDialog::manageCameras()
{
    AcquisitionThread *acqT = nullptr;

    if(ui->camera_l->isChecked()) acqT = v_cam_L;
    else if(ui->camera_m->isChecked()) acqT = v_cam_M;
    else if(ui->camera_r->isChecked()) acqT = v_cam_R;

    if(ui->src_cam->isChecked()){
        this->v_img = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
        threshChanged();
        this->v_img.release();

        ui->openImgFileBtn->setEnabled(false);
        ui->openImgFileBtn->setStyleSheet(pushButtonDesabledStyle);
        ui->openImgFileBtn->clearFocus();

        ui->captureBtn->setEnabled(true);
        ui->captureBtn->setStyleSheet(pushButtonEnabledStyle);

        if(!acqT->getCamIsOpen()){
            if(acqT->launchCamera()){
                connect(acqT, SIGNAL(newFrame(cv::Mat)), this, SLOT(dispFrame(cv::Mat)));
                acqT->start();
            }
        }else{
            connect(acqT, SIGNAL(newFrame(cv::Mat)), this, SLOT(dispFrame(cv::Mat)));
            acqT->start();
        }

    }
    else if(ui->src_img->isChecked()){
        ui->openImgFileBtn->setEnabled(true);
        ui->openImgFileBtn->setStyleSheet(pushButtonEnabledStyle);

        ui->captureBtn->setEnabled(false);
        ui->captureBtn->setStyleSheet(pushButtonDesabledStyle);
        ui->captureBtn->clearFocus();

        if(acqT->isRunning()){
            acqT->requestInterruption();
            acqT->wait();
            disconnect(acqT, SIGNAL(newFrame(cv::Mat)), this, SLOT(dispFrame(cv::Mat)));
            acqT->stopCamera();
        }else if(acqT->getCamIsOpen()) acqT->stopCamera();
    }

    acqT = nullptr;
}

void ThresholdDialog::init()
{
    ui->setupUi(this);
    qRegisterMetaType<cv::Mat>("cv::Mat");

    cv::Mat frame;
    cv::FileStorage fs(camerasIDFile, cv::FileStorage::READ);
    QPixmap pix;

    setModal(true); //fenêtre bloquante
    setWindowTitle("Configuration du seuillage");
    setStyleSheet(dialogStyle_1);

    v_fx = (v_winW * 0.260600) / (IMG_W/2.);
    v_fy = (v_winH * 0.250100) / (IMG_H/2.);

    if(fs.isOpened()){
        v_cam_L = new AcquisitionThread();
        v_cam_M = new AcquisitionThread();
        v_cam_R = new AcquisitionThread();

        v_cam_L->setIDCam(fs["ID_camera_G"]);
        v_cam_M->setIDCam(fs["ID_camera_M"]);
        v_cam_R->setIDCam(fs["ID_camera_D"]);

        bool snap = false;
        v_cam_L->setIsSnapshot(snap);
        v_cam_M->setIsSnapshot(snap);
        v_cam_R->setIsSnapshot(snap);

        fs.release();
    }

    std::vector<std::vector<std::vector<int>>> hsv = readHSVConfig();
    std::vector<std::vector<std::vector<int>>> hsv_ori = readHSVConfig(true);

    for(uint i = 0; i < hsv.size(); i++){
        v_hue.push_back(hsv[HSVCode::HUE][i]);
        v_sat.push_back(hsv[HSVCode::SAT][i]);
        v_val.push_back(hsv[HSVCode::VAL][i]);

        v_hue_ori.push_back(hsv_ori[HSVCode::HUE][i]);
        v_sat_ori.push_back(hsv_ori[HSVCode::SAT][i]);
        v_val_ori.push_back(hsv_ori[HSVCode::VAL][i]);
    }

    ui->hue_min->setMinimum(HUE_MIN); ui->hue_min->setMaximum(HUE_MAX);
    ui->hue_max->setMinimum(HUE_MIN); ui->hue_max->setMaximum(HUE_MAX);
    ui->sat_min->setMinimum(SAT_MIN); ui->sat_min->setMaximum(SAT_MAX);
    ui->sat_max->setMinimum(SAT_MIN); ui->sat_max->setMaximum(SAT_MAX);
    ui->val_min->setMinimum(VAL_MIN); ui->val_min->setMaximum(VAL_MAX);
    ui->val_max->setMinimum(VAL_MIN); ui->val_max->setMaximum(VAL_MAX);

    ui->hue_min_text->setStyleSheet(labelStyle_2);
    ui->hue_max_text->setStyleSheet(labelStyle_2);
    ui->sat_min_text->setStyleSheet(labelStyle_2);
    ui->sat_max_text->setStyleSheet(labelStyle_2);
    ui->val_min_text->setStyleSheet(labelStyle_2);
    ui->val_max_text->setStyleSheet(labelStyle_2);

    ui->lcd_hue_min->setSegmentStyle(QLCDNumber::Flat); ui->lcd_hue_min->setStyleSheet(lcdStyle_6);
    ui->lcd_hue_max->setSegmentStyle(QLCDNumber::Flat); ui->lcd_hue_max->setStyleSheet(lcdStyle_6);
    ui->lcd_sat_min->setSegmentStyle(QLCDNumber::Flat); ui->lcd_sat_min->setStyleSheet(lcdStyle_6);
    ui->lcd_sat_max->setSegmentStyle(QLCDNumber::Flat); ui->lcd_sat_max->setStyleSheet(lcdStyle_6);
    ui->lcd_val_min->setSegmentStyle(QLCDNumber::Flat); ui->lcd_val_min->setStyleSheet(lcdStyle_6);
    ui->lcd_val_max->setSegmentStyle(QLCDNumber::Flat); ui->lcd_val_max->setStyleSheet(lcdStyle_6);

    ui->canceledBtn->setStyleSheet(pushButtonEnabledStyle);
    ui->razBtn->setStyleSheet(pushButtonEnabledStyle);
    ui->openImgFileBtn->setStyleSheet(pushButtonEnabledStyle);
    ui->captureBtn->setStyleSheet(pushButtonDesabledStyle);
    ui->saveHSVBtn->setStyleSheet(pushButtonEnabledStyle);

    ui->camera_l->setChecked(true);
    ui->src_img->setChecked(true);

    ui->originThresh->setStyleSheet(checkBoxStyle_1);

    ui->captureBtn->setEnabled(false);

    uint cam = Camera::GAUCHE;

    ui->hue_min->setValue(this->v_hue[cam][0]); ui->lcd_hue_min->display(this->v_hue[cam][0]);
    ui->hue_max->setValue(this->v_hue[cam][1]); ui->lcd_hue_max->display(this->v_hue[cam][1]);
    ui->sat_min->setValue(this->v_sat[cam][0]); ui->lcd_sat_min->display(this->v_sat[cam][0]);
    ui->sat_max->setValue(this->v_sat[cam][1]); ui->lcd_sat_max->display(this->v_sat[cam][1]);
    ui->val_min->setValue(this->v_val[cam][0]); ui->lcd_val_min->display(this->v_val[cam][0]);
    ui->val_max->setValue(this->v_val[cam][1]); ui->lcd_val_max->display(this->v_val[cam][1]);

    ui->sourceGroup->setStyleSheet(groupBoxStyle_5);
    ui->cameraGroup->setStyleSheet(groupBoxStyle_5);
}

void ThresholdDialog::closeEvent(QCloseEvent *event)
{
    if(ui->src_cam->isChecked()){
        ui->src_cam->setChecked(false);
        ui->src_img->setChecked(true);
    }

    this->v_img = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    threshChanged();
    this->v_img.release();

    manageCameras();

    ui->camera_l->setChecked(true);
    ui->camera_m->setChecked(false);
    ui->camera_r->setChecked(false);

    v_imgCaptureL.release();
    v_imgCaptureM.release();
    v_imgCaptureR.release();

    event->accept();
}

void ThresholdDialog::setHSVValue(){
    uint cam;

    if(ui->camera_r->isChecked()) cam = Camera::DROITE;
    else if(ui->camera_l->isChecked()) cam = Camera::GAUCHE;
    else cam = Camera::MILIEU;

    if(ui->originThresh->isChecked()){
        ui->hue_min->setValue(this->v_hue_ori[cam][0]); ui->lcd_hue_min->display(this->v_hue_ori[cam][0]);
        ui->hue_max->setValue(this->v_hue_ori[cam][1]); ui->lcd_hue_max->display(this->v_hue_ori[cam][1]);
        ui->sat_min->setValue(this->v_sat_ori[cam][0]); ui->lcd_sat_min->display(this->v_sat_ori[cam][0]);
        ui->sat_max->setValue(this->v_sat_ori[cam][1]); ui->lcd_sat_max->display(this->v_sat_ori[cam][1]);
        ui->val_min->setValue(this->v_val_ori[cam][0]); ui->lcd_val_min->display(this->v_val_ori[cam][0]);
        ui->val_max->setValue(this->v_val_ori[cam][1]); ui->lcd_val_max->display(this->v_val_ori[cam][1]);

        manageCameras();
    }else{
        ui->hue_min->setValue(this->v_hue[cam][0]); ui->lcd_hue_min->display(this->v_hue[cam][0]);
        ui->hue_max->setValue(this->v_hue[cam][1]); ui->lcd_hue_max->display(this->v_hue[cam][1]);
        ui->sat_min->setValue(this->v_sat[cam][0]); ui->lcd_sat_min->display(this->v_sat[cam][0]);
        ui->sat_max->setValue(this->v_sat[cam][1]); ui->lcd_sat_max->display(this->v_sat[cam][1]);
        ui->val_min->setValue(this->v_val[cam][0]); ui->lcd_val_min->display(this->v_val[cam][0]);
        ui->val_max->setValue(this->v_val[cam][1]); ui->lcd_val_max->display(this->v_val[cam][1]);

        manageCameras();
    }

    if(ui->src_cam->isChecked()){
        emit displayCamView();
    }
}

void ThresholdDialog::openFile(){
    QString file = QFileDialog::getOpenFileName(this, "Ouvrir un fichier", QString(), "Images(*.png *.jpg *.jpeg)");

    if(!file.isEmpty()){
        std::string fileString = file.toLocal8Bit().constData();

        this->v_img = cv::imread(fileString);
        cv::Mat img_;

        cv::resize(v_img, img_, cv::Size(), v_fx, v_fy);

        ui->imgView->setPixmap(Mat2Pixmap(img_, true, 2));
        ThresholdDialog::thresholdChanged(ui->hue_min->value(), ui->hue_max->value(),
                                        ui->sat_min->value(), ui->sat_max->value(),
                                        ui->val_min->value(), ui->val_max->value(), img_);
    }

}

void ThresholdDialog::useCamera(){
    cv::FileStorage fs(camerasIDFile, cv::FileStorage::READ);
    if(fs.isOpened()){
        v_cam_L->setIDCam(fs["ID_camera_G"]);
        v_cam_M->setIDCam(fs["ID_camera_M"]);
        v_cam_R->setIDCam(fs["ID_camera_D"]);

        fs.release();
    }
    manageCameras();
}

void ThresholdDialog::useImageFile(){
    manageCameras();
}

void ThresholdDialog::threshChanged(){
    cv::Mat img_;
    if(!this->v_img.empty() && ui->src_img->isChecked()){
        cv::resize(v_img, img_, cv::Size(), v_fx, v_fy);
        ThresholdDialog::thresholdChanged(ui->hue_min->value(), ui->hue_max->value(),
                                        ui->sat_min->value(), ui->sat_max->value(),
                                        ui->val_min->value(), ui->val_max->value(),
                                        img_);
    }
    else if(ui->src_cam->isChecked()){
        if(ui->camera_l->isChecked()){
            if(!this->v_imgCaptureL.empty()) img_ = this->v_imgCaptureL.clone();
        }
        else if(ui->camera_m->isChecked()){
            if(!this->v_imgCaptureM.empty())
                img_ = this->v_imgCaptureM.clone();
        }
        else if(ui->camera_r->isChecked()){
            if(!this->v_imgCaptureR.empty())
                img_ = this->v_imgCaptureR.clone();
        }

        if(img_.empty()) img_ = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);

        cv::resize(img_, img_, cv::Size(), v_fx, v_fy);
        ThresholdDialog::thresholdChanged(ui->hue_min->value(), ui->hue_max->value(),
                                        ui->sat_min->value(), ui->sat_max->value(),
                                        ui->val_min->value(), ui->val_max->value(),
                                        img_);
    }
}

void ThresholdDialog::raz(){
    uint cam;

    ui->hue_min->setValue(HUE_MIN); ui->lcd_hue_min->display(HUE_MIN);
    ui->hue_max->setValue(HUE_MAX); ui->lcd_hue_max->display(HUE_MAX);
    ui->sat_min->setValue(SAT_MIN); ui->lcd_sat_min->display(SAT_MIN);
    ui->sat_max->setValue(SAT_MAX); ui->lcd_sat_max->display(SAT_MAX);
    ui->val_min->setValue(VAL_MIN); ui->lcd_val_min->display(VAL_MIN);
    ui->val_max->setValue(VAL_MAX); ui->lcd_val_max->display(VAL_MAX);

    if(ui->camera_r->isChecked()) cam = Camera::DROITE;
    else if(ui->camera_l->isChecked()) cam = Camera::GAUCHE;
    else cam = Camera::MILIEU;

    if(ui->originThresh->isChecked()){
        this->v_hue_ori[cam][0] = ui->hue_min->value();
        this->v_hue_ori[cam][1] = ui->hue_max->value();
        this->v_sat_ori[cam][0] = ui->sat_min->value();
        this->v_sat_ori[cam][1] = ui->sat_max->value();
        this->v_val_ori[cam][0] = ui->val_min->value();
        this->v_val_ori[cam][1] = ui->val_max->value();
    }else{
        this->v_hue[cam][0] = ui->hue_min->value();
        this->v_hue[cam][1] = ui->hue_max->value();
        this->v_sat[cam][0] = ui->sat_min->value();
        this->v_sat[cam][1] = ui->sat_max->value();
        this->v_val[cam][0] = ui->val_min->value();
        this->v_val[cam][1] = ui->val_max->value();
    }

}

void ThresholdDialog::configOriginThresh(){
    uint cam;

    if(ui->camera_r->isChecked()) cam = Camera::DROITE;
    else if(ui->camera_l->isChecked()) cam = Camera::GAUCHE;
    else cam = Camera::MILIEU;

    if(ui->originThresh->isChecked()){
        ui->hue_min->setValue(this->v_hue_ori[cam][0]); ui->lcd_hue_min->display(this->v_hue_ori[cam][0]);
        ui->hue_max->setValue(this->v_hue_ori[cam][1]); ui->lcd_hue_max->display(this->v_hue_ori[cam][1]);
        ui->sat_min->setValue(this->v_sat_ori[cam][0]); ui->lcd_sat_min->display(this->v_sat_ori[cam][0]);
        ui->sat_max->setValue(this->v_sat_ori[cam][1]); ui->lcd_sat_max->display(this->v_sat_ori[cam][1]);
        ui->val_min->setValue(this->v_val_ori[cam][0]); ui->lcd_val_min->display(this->v_val_ori[cam][0]);
        ui->val_max->setValue(this->v_val_ori[cam][1]); ui->lcd_val_max->display(this->v_val_ori[cam][1]);

        ui->originThresh->setStyleSheet(checkBoxStyle_2);
    }
    else{
        ui->hue_min->setValue(this->v_hue[cam][0]); ui->lcd_hue_min->display(this->v_hue[cam][0]);
        ui->hue_max->setValue(this->v_hue[cam][1]); ui->lcd_hue_max->display(this->v_hue[cam][1]);
        ui->sat_min->setValue(this->v_sat[cam][0]); ui->lcd_sat_min->display(this->v_sat[cam][0]);
        ui->sat_max->setValue(this->v_sat[cam][1]); ui->lcd_sat_max->display(this->v_sat[cam][1]);
        ui->val_min->setValue(this->v_val[cam][0]); ui->lcd_val_min->display(this->v_val[cam][0]);
        ui->val_max->setValue(this->v_val[cam][1]); ui->lcd_val_max->display(this->v_val[cam][1]);

        ui->originThresh->setStyleSheet(checkBoxStyle_1);
    }
}

void ThresholdDialog::canceled()
{
    close();
}

void ThresholdDialog::dispFrame(cv::Mat frame)
{
    cv::Mat img_;
    this->v_img = frame.clone();
    cv::resize(frame, img_, cv::Size(), v_fx, v_fy);
    ui->imgView->setPixmap(Mat2Pixmap(img_, true, 2));
}

void ThresholdDialog::capture()
{
    AcquisitionThread *acqT = nullptr;
    cv::Mat img_;

    if(ui->camera_l->isChecked()) acqT = v_cam_L;
    else if(ui->camera_m->isChecked()) acqT = v_cam_M;
    else if(ui->camera_r->isChecked()) acqT = v_cam_R;

    disconnect(acqT, SIGNAL(newFrame(cv::Mat)), this, SLOT(dispFrame(cv::Mat)));

    if(ui->camera_l->isChecked()){
        v_imgCaptureL = v_img.clone();
        img_ = v_imgCaptureL.clone();
    }
    else if(ui->camera_m->isChecked()){
        v_imgCaptureM = v_img.clone();
        img_ = v_imgCaptureM.clone();
    }
    else if(ui->camera_r->isChecked()){
        v_imgCaptureR = v_img.clone();
        img_ = v_imgCaptureR.clone();
    }

    cv::resize(img_, img_, cv::Size(), v_fx, v_fy);
    connect(acqT, SIGNAL(newFrame(cv::Mat)), this, SLOT(dispFrame(cv::Mat)));
    acqT = nullptr;

    ThresholdDialog::thresholdChanged(ui->hue_min->value(), ui->hue_max->value(),
                                    ui->sat_min->value(), ui->sat_max->value(),
                                      ui->val_min->value(), ui->val_max->value(), img_);
}

void ThresholdDialog::displayView()
{
    if(!ui->camera_l->isChecked() && v_cam_L->isRunning()){
        v_cam_L->requestInterruption();
        v_cam_L->wait();
        v_cam_L->stopCamera();
    }else if(!ui->camera_l->isChecked() && v_cam_L->getCamIsOpen()) v_cam_L->stopCamera();

    if(!ui->camera_m->isChecked() && v_cam_M->isRunning()){
        v_cam_M->requestInterruption();
        v_cam_M->wait();
        v_cam_M->stopCamera();
    }else if(!ui->camera_m->isChecked() && v_cam_M->getCamIsOpen()) v_cam_M->stopCamera();

    if(!ui->camera_r->isChecked() && v_cam_R->isRunning()){
        v_cam_R->requestInterruption();
        v_cam_R->wait();
        v_cam_R->stopCamera();
    }else if(!ui->camera_r->isChecked() && v_cam_R->getCamIsOpen()) v_cam_R->stopCamera();

    if(ui->camera_l->isChecked()){
        if(!v_cam_L->launchCamera()){
            this->v_img = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
            threshChanged();
            this->v_img.release();
        }
    }
    else if(ui->camera_m->isChecked()){
        if(!v_cam_M->launchCamera()){
            this->v_img = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
            threshChanged();
            this->v_img.release();
        }
    }
    else if(ui->camera_r->isChecked()){
        if(!v_cam_R->launchCamera()){
            this->v_img = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
            threshChanged();
            this->v_img.release();
        }
    }
    manageCameras();
}

void ThresholdDialog::changeHSV(){
    uint cam;
    bool ori = ui->originThresh->isChecked();

    if(ui->camera_r->isChecked()) cam = Camera::DROITE;
    else if(ui->camera_l->isChecked()) cam = Camera::GAUCHE;
    else cam = Camera::MILIEU;

    if(!ori){
        this->v_hue[cam][0] = ui->hue_min->value();
        this->v_hue[cam][1] = ui->hue_max->value();
        this->v_sat[cam][0] = ui->sat_min->value();
        this->v_sat[cam][1] = ui->sat_max->value();
        this->v_val[cam][0] = ui->val_min->value();
        this->v_val[cam][1] = ui->val_max->value();
    }else{
        this->v_hue_ori[cam][0] = ui->hue_min->value();
        this->v_hue_ori[cam][1] = ui->hue_max->value();
        this->v_sat_ori[cam][0] = ui->sat_min->value();
        this->v_sat_ori[cam][1] = ui->sat_max->value();
        this->v_val_ori[cam][0] = ui->val_min->value();
        this->v_val_ori[cam][1] = ui->val_max->value();
    }

    saveHSVConfig(static_cast<int>(cam),
                  ui->hue_min->value(), ui->hue_max->value(),
                  ui->sat_min->value(), ui->sat_max->value(),
                  ui->val_min->value(), ui->val_max->value(),
                  ori);
}
