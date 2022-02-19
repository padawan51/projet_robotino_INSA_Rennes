#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;
using namespace cv;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    this->init();
    this->loadParamIHM();
    this->connects();

    this->initCameras();
    this->createActions();
    this->addActionsToMenu();

    v_checkNodes_T->start();
}

MainWindow::~MainWindow()
{
    for(uint i = 0; i < 3; i++){
        if(v_cam[i]->useCam()){
            if(v_cam[i]->acq->isRunning()){
                v_cam[i]->acq->requestInterruption();
                v_cam[i]->acq->wait();
            }
            delete v_cam[i]->acq;
        }

        if(v_cam[i]->wa != nullptr) delete v_cam[i]->wa;

        if(v_cam[i]->rp != nullptr){
            if(v_cam[i]->rp->isRunning()){
                v_cam[i]->rp->requestInterruption();
                v_cam[i]->rp->wait();
            }
            delete v_cam[i]->rp;
        }

        delete v_cam[i];
    }

    if(v_rc->isRunning()){
        v_rc->requestInterruption();
        v_rc->wait();
    }

    if(v_cam_G_acq->isRunning()){
        v_cam_G_acq->requestInterruption();
        v_cam_G_acq->wait();
    }
    else if(v_cam_D_acq->isRunning()){
        v_cam_D_acq->requestInterruption();
        v_cam_D_acq->wait();
    }
    else if(v_cam_M_acq->isRunning()){
        v_cam_M_acq->requestInterruption();
        v_cam_M_acq->wait();
    }

    if(v_moveRobotThread->isRunning()){
        v_moveRobotThread->requestInterruption();
        v_moveRobotThread->wait();
    }

    if(v_chronoImg->isRunning()){
        v_chronoImg->requestInterruption();
        v_chronoImg->wait();
    }

    if(v_checkNodes_T->isRunning()){
        v_checkNodes_T->requestInterruption();
        v_checkNodes_T->wait();
    }

    delete v_threshDial;
    delete v_corrector;
    delete v_rc;
    delete v_cam_G_acq;
    delete v_cam_M_acq;
    delete v_cam_D_acq;
    if(v_hasProjectnamePointer) delete v_projectName;
    delete [] v_cameraBtn;
    delete v_moveRobotThread;
    delete v_chronoImg;
    delete v_checkNodes_T;
    delete ui;
}

void MainWindow::init(){
     QPalette palette;
     cv::Mat frame_init;
     auto scaleWindow = 1.3;

     ui->setupUi(this);

     QFile file(QString::fromStdString(saveProject) + "current_project.name");
     if(file.exists()){
         file.remove();
     }

     //Configuration de la fenêtre principale de l'IHM
     setWindowTitle("IHM Projet Robotino LGCGM");
     palette.setColor(QPalette::Window, QColor(15, 15, 15));
     setPalette(palette);

     //Récupération des dimensions de l'écran
     this->v_maxWinW = (QApplication::desktop()->availableGeometry(this).width());
     this->v_maxWinH = (QApplication::desktop()->availableGeometry(this).height());

     //Dimensionnement de l'IHM
     this->v_windowWidth = static_cast<int>(QApplication::desktop()->availableGeometry(this).width()/scaleWindow);
     this->v_windowHeight = static_cast<int>(QApplication::desktop()->availableGeometry(this).height()/scaleWindow);
     setGeometry((this->v_maxWinW - this->v_windowWidth)/2,
                 (this->v_maxWinH - this->v_windowHeight)/2,
                 v_windowWidth,
                 v_windowHeight);
     setFixedSize(this->v_windowWidth, this->v_windowHeight);

     //Configuration de la zone des messages
     palette = ui->message->palette();
     palette.setColor(QPalette::Base, QColor(0,0,0));
     palette.setColor(QPalette::Text, QColor(255,255,255));
     ui->message->setPalette(palette);

     //Réglage des facteurs de redimensionnement de l'image issue des caméras
     v_fx = (v_windowWidth * 0.339) / (IMG_W/2.);
     v_fy = (v_windowHeight * 0.3254) / (IMG_H/2.);

     //Réglage des facteurs de redimensionnement de l'image de la zone de travail sur l'IHM
     v_resizeWidthImage = (v_windowWidth * 1.96) / (2775);
     v_resizeHeightImage = (v_windowHeight * 3.494) / (2900);

     //...
     v_camerasDialog = false;

     configWidgets();
     setToolTip();
     keyboardShortcuts();
     initViews();
     enableWidgets(true);

     //Allocations dynamique
     this->v_qPixWA = new QPixmap(static_cast<int>(stoi(ui->waLength->text().toStdString())*scale),
                          static_cast<int>(stoi(ui->waWidth->text().toStdString())*scale));
     this->v_threshDial = new ThresholdDialog(v_maxWinW, v_maxWinH);
     this->v_corrector = new CorrectorDialog(v_maxWinW, v_maxWinH);
     this->v_rc = new RobotControllerThread();
     this->v_cameraBtn = new CameraViewButton[static_cast<unsigned long long>(v_nbCamera)]();
     this->v_cam_G_acq = new AcquisitionThread();
     this->v_cam_M_acq = new AcquisitionThread();
     this->v_cam_D_acq = new AcquisitionThread();
     this->v_chronoImg = new ChronoThread();
     this->v_checkNodes_T = new CheckNodesThread();

     this->v_rosMasterNodeStatus = true;
     this->v_pcControllerNodeStatus = true;
     this->v_intermediaryNodeStatus = true;
     this->v_velNodeStatus = true;

     for(int i = 0; i < v_nbCamera; i++){
         v_cameraBtn[i].setWinParam(v_maxWinW, v_maxWinH);
         v_cameraBtn[i].setIndex(i);
         connect(&v_cameraBtn[i], SIGNAL(checkRunningCameras(int)), this, SLOT(checkCameras(int)));
     }
     this->v_hasProjectnamePointer = false;
     this->v_left_btn_joy_status = false;
     this->v_right_btn_joy_status = false;
     this->v_fw_btn_joy_status = false;
     this->v_bw_btn_joy_status = false;
     this->v_cw_btn_joy_status = false;
     this->v_ccw_btn_joy_status = false;
     this->v_moveRobotThread = new MoveRobotThread(&v_fw_btn_joy_status,
                                                 &v_bw_btn_joy_status,
                                                 &v_left_btn_joy_status,
                                                 &v_right_btn_joy_status,
                                                 &v_ccw_btn_joy_status,
                                                 &v_cw_btn_joy_status);
}

void MainWindow::initCameras()
{
    cv::FileStorage fs(camerasIDFile, cv::FileStorage::READ);

    if(fs.isOpened()){
        bool useCamera = false;
        v_cam.resize(3);

#ifdef CHECK_CAMERAS
        useCamera = true;
#endif

        v_cam[Camera::DROITE] = new Cameras(fs["ID_camera_D"], "droite", useCamera);
        v_cam[Camera::GAUCHE] = new Cameras(fs["ID_camera_G"], "gauche", useCamera);
        v_cam[Camera::MILIEU] = new Cameras(fs["ID_camera_M"], "milieu", useCamera);
        fs.release();

        connect(v_cam[Camera::DROITE], SIGNAL(sendMessage(QString, QColor)), this, SLOT(myAppend(QString, QColor)));
        connect(v_cam[Camera::GAUCHE], SIGNAL(sendMessage(QString, QColor)), this, SLOT(myAppend(QString, QColor)));
        connect(v_cam[Camera::MILIEU], SIGNAL(sendMessage(QString, QColor)), this, SLOT(myAppend(QString, QColor)));

#ifndef CHECK_CAMERAS
        v_cam[Camera::DROITE]->run();
        v_cam[Camera::GAUCHE]->run();
        v_cam[Camera::MILIEU]->run();
#endif
    }

}

void MainWindow::connects(){
    connect(this, SIGNAL(sendMessage(QString)), ui->message, SLOT(append(QString)));

    connect(this, &MainWindow::displayMsgSignal, this, &MainWindow::displayMsgSlot);

    connect(ui->ros_master, SIGNAL(clicked()), this, SLOT(rosNodeMaster()));

    connect(ui->gen_map, SIGNAL(clicked()), this, SLOT(generateMap()));

    connect(ui->ros_node, SIGNAL(clicked()), this, SLOT(nodeList()));

    connect(ui->threshold, SIGNAL(clicked()), this, SLOT(setTresh()));

    connect(ui->measure, SIGNAL(clicked()), this, SLOT(startMeasure()));

    connect(ui->IPButton, SIGNAL(clicked()), this, SLOT(changeIP()));

    connect(ui->highButton, SIGNAL(clicked()), this, SLOT(heightList()));

    connect(ui->savedHigh, SIGNAL(textEdited(QString)), this, SLOT(changeHeightList(QString)));

    connect(ui->otherHigh, SIGNAL(textEdited(QString)), this, SLOT(changeHeightList(QString)));

    connect(ui->stop, SIGNAL(clicked()), this, SLOT(stop()));

    connect(ui->clean_msg_window, SIGNAL(clicked()), ui->message, SLOT(clear()));

    connect(ui->calibrationBtn, &QPushButton::clicked, this, &MainWindow::calibrateCameras);

    connect(ui->start_chronoBtn, &QPushButton::clicked, this, &MainWindow::runChrono);

    connect(ui->log, SIGNAL(clicked(bool)), this, SLOT(messageView(bool)));

    connect(ui->cameras, SIGNAL(toggled(bool)), this, SLOT(configView(bool)));

    connect(ui->cam_left, SIGNAL(clicked()), this, SLOT(camLeftView()));

    connect(ui->cam_middle, SIGNAL(clicked()), this, SLOT(camMiddleView()));

    connect(ui->cam_right, SIGNAL(clicked()), this, SLOT(camRightView()));

    connect(ui->action_Quitter, SIGNAL(triggered()), this, SLOT(quitIHM()));

    connect(ui->action_New_measure, SIGNAL(triggered()), this, SLOT(newProject()));

    connect(ui->action_Save, SIGNAL(triggered()), this, SLOT(saveParamIHM()));

    connect(ui->start_pc_controller_node,SIGNAL(triggered()), this, SLOT(runControllerNode()));

    connect(ui->start_intermediary_node, SIGNAL(triggered()), this, SLOT(runIntermediaryNode()));

    connect(ui->start_velocity_node, SIGNAL(triggered()), this, SLOT(runVelocityNode()));

    connect(ui->start_ros_master_node, SIGNAL(triggered()), this, SLOT(rosNodeMaster()));

    connect(ui->start_all_nodes, SIGNAL(triggered()), this, SLOT(runAllNodes()));

    connect(ui->stop_pc_controller_node,SIGNAL(triggered()), this, SLOT(stopControllerNode()));

    connect(ui->stop_intermediary_node, SIGNAL(triggered()), this, SLOT(stopIntermediaryNode()));

    connect(ui->stop_velocity_node, SIGNAL(triggered()), this, SLOT(stopVelocityNode()));

    connect(ui->stop_ros_master_node, SIGNAL(triggered()), this, SLOT(stopRosMasterNode()));

    connect(ui->stop_all_nodes, SIGNAL(triggered()), this, SLOT(stopAllNodes()));

    connect(ui->camerasButton, SIGNAL(clicked()), this, SLOT(camerasView()));

    connect(ui->ID_cam_g, SIGNAL(currentIndexChanged(int)), this, SLOT(saveIDsCam()));

    connect(ui->ID_cam_m, SIGNAL(currentIndexChanged(int)), this, SLOT(saveIDsCam()));

    connect(ui->ID_cam_d, SIGNAL(currentIndexChanged(int)), this, SLOT(saveIDsCam()));

    connect(ui->manualControl_checkbox, SIGNAL(clicked()), this, SLOT(manualControlView()));

    connect(ui->calibration_checkbox, SIGNAL(clicked()), this, SLOT(calibrationView()));

    connect(ui->radioSavedHigh, SIGNAL(clicked()), this, SLOT(targetHeight()));

    connect(ui->radioOtherHigh, SIGNAL(clicked()), this, SLOT(targetHeight()));

    connect(ui->robot_fw, SIGNAL(clicked()), this, SLOT(xVelPlusManualControl()));

    connect(ui->robot_bw, SIGNAL(clicked()), this, SLOT(xVelMinusManualControl()));

    connect(ui->robot_left, SIGNAL(clicked()), this, SLOT(yVelMinusManualControl()));

    connect(ui->robot_right, SIGNAL(clicked()), this, SLOT(yVelPlusManualControl()));

    connect(ui->robot_rot_ccw, SIGNAL(clicked()), this, SLOT(zAngVelPlusManualControl()));

    connect(ui->robot_rot_cw, SIGNAL(clicked()), this, SLOT(zAngVelMinusManualControl()));

    connect(ui->robot_stop, SIGNAL(clicked()), this, SLOT(stopManualControl()));

    connect(ui->waWidth, SIGNAL(valueChanged(int)), this, SLOT(robotFPXMax(int)));

    connect(ui->waLength, SIGNAL(valueChanged(int)), this, SLOT(robotFPYMax(int)));

    connect(ui->robot_free_pos_go, &QPushButton::clicked, this, &MainWindow::robotFPLaunchBtn);

    connect(ui->robot_free_positionning, SIGNAL(toggled(bool)), this, SLOT(robotFreePos(bool)));

    connect(ui->robot_free_pos_with_measure, SIGNAL(clicked()), this, SLOT(takeMeasureManualControl()));

    connect(ui->robot_free_pos_without_measure, SIGNAL(clicked()), this, SLOT(takeMeasureManualControl()));

    connect(v_rc, SIGNAL(measurementPoint(int)), this, SLOT(processedPoint(int)));

    connect(v_rc, SIGNAL(processedMeasure(int)), this, SLOT(processedMeasure(int)));

    connect(v_rc, SIGNAL(measureFinish()), this, SLOT(stop()));

    connect(v_checkNodes_T, SIGNAL(nodeStatus(int, bool)), this, SLOT(updateNodesStatus(int, bool)));
}

void MainWindow::createActions()
{
    cv::FileStorage fs(saveVVOType, cv::FileStorage::READ);
    int type;

    if(fs.isOpened()) fs["type"] >> type;
    else type = -1;

    v_correctorP_action = new QAction("Correcteur P", this);
    v_correctorP_action->setCheckable(true);
    connect(v_correctorP_action, &QAction::triggered, this, &MainWindow::showCorrectorDial);

    v_correctorPI_action = new QAction("Correcteur PI", this);
    v_correctorPI_action->setCheckable(true);
    v_correctorPI_action->setEnabled(false);
    connect(v_correctorPI_action, &QAction::triggered, this, &MainWindow::showCorrectorDial);

    v_correctorPID_action = new QAction("Correcteur PID", this);
    v_correctorPID_action->setCheckable(true);
    v_correctorPID_action->setEnabled(false);
    connect(v_correctorPID_action, &QAction::triggered, this, &MainWindow::showCorrectorDial);

    v_0_5_V_action = new QAction("0-5V", this);
    v_0_5_V_action->setCheckable(true);
    connect(v_0_5_V_action, &QAction::triggered, this, &MainWindow::_0_5_V);

    v_1_5_V_action = new QAction("1-5V", this);
    v_1_5_V_action->setCheckable(true);
    connect(v_1_5_V_action, &QAction::triggered, this, &MainWindow::_1_5_V);

    v_0_10_V_action = new QAction("0-10V", this);
    v_0_10_V_action->setCheckable(true);

    v_2_10_V_action = new QAction("2-10V", this);
    v_2_10_V_action->setCheckable(true);

    v_0_20_mA_action = new QAction("0-20mA", this);
    v_0_20_mA_action->setCheckable(true);

    v_4_20_mA_action = new QAction("4-20mA", this);
    v_4_20_mA_action->setCheckable(true);

    v_correctorGroup = new QActionGroup(this);
    v_correctorGroup->addAction(v_correctorP_action);
    v_correctorGroup->addAction(v_correctorPI_action);
    v_correctorGroup->addAction(v_correctorPID_action);
    v_correctorP_action->setChecked(true);

    v_voltageGroup = new QActionGroup(this);
    v_voltageGroup->addAction(v_0_5_V_action);
    v_voltageGroup->addAction(v_1_5_V_action);

    if(type == VelocityVoltageOutputType::TYPE_0_5_V || type == -1) v_0_5_V_action->setChecked(true);
    else if(type == VelocityVoltageOutputType::TYPE_1_5_V) v_1_5_V_action->setChecked(true);

    v_0_10_V_action->setEnabled(false);
    v_2_10_V_action->setEnabled(false);
    v_0_20_mA_action->setEnabled(false);
    v_4_20_mA_action->setEnabled(false);
}

void MainWindow::addActionsToMenu()
{
    ui->menuCorrecteur->addAction(v_correctorP_action);
    ui->menuCorrecteur->addSeparator();
    ui->menuCorrecteur->addAction(v_correctorPI_action);
    ui->menuCorrecteur->addSeparator();
    ui->menuCorrecteur->addAction(v_correctorPID_action);

    ui->menuType_de_sortie->addAction(v_0_5_V_action);
    ui->menuType_de_sortie->addSeparator();
    ui->menuType_de_sortie->addAction(v_1_5_V_action);
    ui->menuType_de_sortie->addSeparator();
    ui->menuType_de_sortie->addAction(v_0_10_V_action);
    ui->menuType_de_sortie->addSeparator();
    ui->menuType_de_sortie->addAction(v_2_10_V_action);
    ui->menuType_de_sortie->addSeparator();
    ui->menuType_de_sortie->addAction(v_0_20_mA_action);
    ui->menuType_de_sortie->addSeparator();
    ui->menuType_de_sortie->addAction(v_4_20_mA_action);
}

void MainWindow::saveParamIHM(bool force)
{
    if(v_IHMStatus || force){
        QFile file(QString::fromStdString(saveParamIHMFile));
        file.setPermissions(QFileDevice::WriteUser);

        cv::FileStorage fs;
        int motor_precision_divider;
        int extra;
        std::vector<int> height;
        QString text = ui->savedHigh->text();
        QStringList strList = text.split(QChar(';'), Qt::SkipEmptyParts);
        std::string tempStr;

        if(strList.size() >= 1 ){
            for(int i = 0; i < strList.size(); i++){
                bool ok;
                int val = strList[i].toInt(&ok);

                if(ok) height.emplace_back(val);
            }

            if(height.size() > 1){
                for(uint i = 0; i < height.size() - 1; i++){
                    for(uint j = i+1; j < height.size(); j++){
                        if(height[i] > height[j]){
                            int temp = height[i];
                            height[i] = height[j];
                            height[j] = temp;
                        }
                    }
                }
            }
        }

        if(ui->motor_precision->currentText() == "0.0225") motor_precision_divider = 16;
        else if(ui->motor_precision->currentText() == "0.045") motor_precision_divider = 8;
        else if(ui->motor_precision->currentText() == "0.09") motor_precision_divider = 4;
        else if(ui->motor_precision->currentText() == "0.18") motor_precision_divider = 2;
        else if(ui->motor_precision->currentText() == "0.36") motor_precision_divider = 1;

        fs.open(saveParamIHMFile, cv::FileStorage::READ);
        fs["extra"] >> extra;
        fs.release();

        fs.open(saveParamIHMFile, cv::FileStorage::WRITE);
        fs << "IP_PC" << ui->IP_PC->text().toStdString();
        fs << "IP_Robotino" << ui->IP_Robotino->text().toStdString();
        fs << "robot_diameter" << ui->diameter->value();
        fs << "tool_length" << ui->toolLength->value();
        fs << "tool_width" << ui->toolWidth->value();
        fs << "extra" << extra;
        fs << "leds_height" << ui->ledHigh->value();
        fs << "max_linear_velocity" << ui->linearVelMax->value();
        fs << "min_angular_velocity" << ui->angularVelMin->value();
        fs << "working_area_length" << ui->waLength->value();
        fs << "working_area_width" << ui->waWidth->value();
        fs << "measurement_point_dx" << ui->dx->value();
        fs << "measurement_point_dy" << ui->dy->value();
        fs << "rest_time" << ui->rest->value();
        fs << "measure_period" << ui->period->value();
        fs << "measure_timer" << ui->measure_timer->value();
        fs << "pulley_radius" << ui->pulley_radius->value();
        fs << "motor_precision_index" << ui->motor_precision->currentIndex();
        fs << "motor_precision" << ui->motor_precision->currentData().toFloat();
        fs << "motor_precision_divider" << motor_precision_divider;
        fs << "duty_cycle" << ui->duty_cycle->currentData().toInt();
        fs << "duty_cycle_index" << ui->duty_cycle->currentIndex();
        fs << "lin_vel_motor_mast" << ui->lin_vel_motor_mast->value();

        if(static_cast<int>(height.size()) > 0){
            fs << "number_of_heights" << static_cast<int>(height.size());

            for(uint i = 0; i < height.size() - 1; i++){
                for(uint j = i+1; j < height.size(); j++){
                    if(height[i] > height[j]){
                        int temp = height[i];
                        height[i] = height[j];
                        height[j] = temp;
                    }
                }
            }

            for(uint i = 0; i < height.size(); i++){
                fs << "h_" + to_string(i+1) << height[i];
            }
        }

        fs.release();
        file.setPermissions(QFileDevice::ReadUser);

        QString msg = "Les paramètres de l'IHM ont été sauvegardés";
        emit displayMsgSignal(msg, GREEN_COLOR);
    }

}

void MainWindow::processedPoint(int val)
{
    ui->processedPoints->display(val);

    if(val > 0){
        int radius = 3;
        cv::Mat map;

        if(val < v_generatedPath.size()){
            int x_i = static_cast<int>(v_generatedPath[val-1].ptr<double>(v_generatedPath[val-1].rows-1)[0] * scale);
            int y_i = static_cast<int>(v_generatedPath[val-1].ptr<double>(v_generatedPath[val-1].rows-1)[1] * scale);
            int x_f = static_cast<int>(v_generatedPath[val].ptr<double>(0)[0] * scale);
            int y_f = static_cast<int>(v_generatedPath[val].ptr<double>(0)[1] * scale);

            circle(v_map,
                   Point(static_cast<int>(v_robotPositionPoints[val].x*scale),
                         static_cast<int>(v_robotPositionPoints[val].y*scale)),
                   radius,
                   Scalar(0,255,255),
                   cv::FILLED);

            cv::arrowedLine(v_map,
                             Point(y_i, x_i),
                             Point(y_f, x_f),
                             Scalar(0, 0, 255),
                             1);

            if(v_generatedPath[val].rows > 1){
                for(int i = 0; i < v_generatedPath[val].rows - 1; i++){
                    x_i = static_cast<int>(v_generatedPath[val].ptr<double>(i)[0] * scale);
                    y_i = static_cast<int>(v_generatedPath[val].ptr<double>(i)[1] * scale);

                    x_f = static_cast<int>(v_generatedPath[val].ptr<double>(i+1)[0] * scale);
                    y_f = static_cast<int>(v_generatedPath[val].ptr<double>(i+1)[1] * scale);

                    cv::arrowedLine(v_map,
                             Point(y_i, x_i),
                             Point(y_f, x_f),
                             Scalar(0, 0, 255),
                             1);
                }
            }
        }

        circle(v_map,
               Point(static_cast<int>(v_reachablePoints[val-1].x*scale),
                     static_cast<int>(v_reachablePoints[val-1].y*scale)),
               radius,
               Scalar(0,255,0),
               cv::FILLED);

        cv::resize(v_map, map, cv::Size(), this->v_resizeWidthImage, this->v_resizeHeightImage);
        *v_qPixWA = Mat2Pixmap(map);
        ui->dispWA->setPixmap(*v_qPixWA);
    }
}

void MainWindow::processedMeasure(int val)
{
    ui->processedMeasures->display(val);
}

void MainWindow::newProject()
{
    v_projectName = new QDialog();
    v_hasProjectnamePointer = true;

    auto *fileNameLayout = new QHBoxLayout();
    auto *buttonLayout = new QHBoxLayout();
    auto *mainLayout = new QVBoxLayout();
    QLabel label("Nom du projet :");
    QLabel consigne;
    QLineEdit fileName;
    QPushButton ok("OK");
    QPushButton cancel("ANNULER");

    ok.setFixedSize(QSize(75,25));
    cancel.setFixedSize(QSize(75,25));
    consigne.setText("CONSIGNES:\n"
                     "  1) Le premier caractère doit être\n"
                     "      une lettre\n"
                     "  2) Caractères autorisés:\n"
                     "      -> Lettres minuscules\n"
                     "      -> Chiffres\n"
                     "      -> Underscore (_)"
                     );

    label.setStyleSheet(labelStyle_2);
    consigne.setStyleSheet(labelStyle_2);

    ok.setStyleSheet(pushButtonEnabledStyle);
    cancel.setStyleSheet(pushButtonEnabledStyle);

    fileName.setStyleSheet(lineEditStyle_4);

    v_projectName->setStyleSheet(dialogStyle_1);

    fileNameLayout->addWidget(&label);
    fileNameLayout->addWidget(&fileName);

    buttonLayout->addWidget(&ok);
    buttonLayout->addWidget(&cancel);

    mainLayout->addLayout(fileNameLayout);
    mainLayout->addWidget(&consigne);
    mainLayout->addLayout(buttonLayout);

    connect(&cancel, SIGNAL(clicked()), v_projectName, SLOT(close()));
    connect(&fileName, SIGNAL(textEdited(QString)), this, SLOT(newProjectName(QString)));
    connect(&ok, SIGNAL(clicked()), this, SLOT(saveNewProjectName()));

    v_projectName->setModal(true);
    v_projectName->setWindowTitle("Projet");
    v_projectName->setFixedSize(QSize(250,200));
    v_projectName->setLayout(mainLayout);
    v_projectName->exec();
}

void MainWindow::saveNewProjectName()
{
    if(!projectFileName.isEmpty()){
        bool fileNameOK = false;
        std::string str = projectFileName.toStdString();

        if((str[0] >= 97) && str[0] <= 122){
            for(uint i = 1; i < str.length(); i++){
                if((str[i] >= 97) && (str[i] <= 122)) fileNameOK = true;
                else if((str[i] >= 48) && (str[i] <= 57)) fileNameOK = true;
                else if(str[i] == 95) fileNameOK = true;
                else{
                    fileNameOK = false;
                    break;
                }
            }

            if(fileNameOK){
                QFile file(QString::fromStdString(saveProject) + "current_project.name");

                file.open(QIODevice::OpenModeFlag::WriteOnly);
                file.write(projectFileName.toStdString().c_str());
                file.close();

                QString msg("NOM DU PROJET : " + projectFileName);
                emit displayMsgSignal(msg, GREEN_COLOR);

                v_projectName->close();
                //enableWidgets(true);
            }else{
                QString msg("NOM DU PROJET : Veuillez saisir un nom de projet valide");
                emit displayMsgSignal(msg, RED_COLOR);
            }

        }else{
            QString msg("NOM DU PROJET : Premier caractère incorrect");
            emit displayMsgSignal(msg, RED_COLOR);
        }
    }else{
        QString msg("CHAMP VIDE : Veuillez saisir un nom de projet valide");
        emit displayMsgSignal(msg, RED_COLOR);
    }

}

void MainWindow::newProjectName(const QString &text)
{
    this->projectFileName = text;
}

void MainWindow::camerasView()
{
    bool hasCamera = false;

    for(int i = 0; i < v_nbCamera; i++){
        if(v_cameraBtn[i].getIsPresent()){
            hasCamera = true;
            break;
        }
    }

    if(hasCamera){
        QLabel view;
        auto *vLayout = new QVBoxLayout();
        auto *hl1 = new QHBoxLayout();
        auto *hl2 = new QHBoxLayout();
        QPixmap pix;

        cv::Mat frame;

        double fx_;
        double fy_;

        v_viewCameras = new QDialog();
        v_viewCameras->setStyleSheet(dialogStyle_1);
        v_viewCameras->setWindowTitle("Caméras");

        vLayout->addWidget(&view);

        frame = cv::Mat::zeros(static_cast<int>(IMG_H/2),
                                    static_cast<int>(IMG_W/2),
                                    CV_8UC3);

        fx_ = (v_maxWinW * 0.2601) / (IMG_W/2);
        fy_ = (v_maxWinH * 0.25) / (IMG_H/2);

        if(!v_camerasDialog) v_camerasDialog = true;

        cv::resize(frame, frame, cv::Size(), fx_, fy_);
        pix = Mat2Pixmap(frame);

        view.setFixedSize(static_cast<int>(v_maxWinW * 0.2601), static_cast<int>(v_maxWinH * 0.25));
        view.setPixmap(pix);
        view.setStyleSheet(labelStyle_3);

        for(int i = 0; i < v_nbCamera; i++){
            if(v_cameraBtn[i].getIsPresent() && v_cameraBtn[i].getIsSelected()){
                v_cameraBtn[i].setIsSelected(false);
            }
        }

        for(int i = 0; i < v_nbCamera; i++){

            if(i < v_nbCamera/2) hl1->addWidget(&v_cameraBtn[i]);
            else hl2->addWidget(&v_cameraBtn[i]);

            if(v_camerasDialog){
                disconnect(&v_cameraBtn[i], SIGNAL(clicked()), &v_cameraBtn[i], SLOT(captureImage()));
                disconnect(&v_cameraBtn[i], SIGNAL(image(QPixmap)), &view, SLOT(setPixmap(QPixmap)));
            }
            connect(&v_cameraBtn[i], SIGNAL(clicked()), &v_cameraBtn[i], SLOT(captureImage()));
            connect(&v_cameraBtn[i], SIGNAL(image(QPixmap)), &view, SLOT(setPixmap(QPixmap)));
        }

        vLayout->addLayout(hl1);
        vLayout->addLayout(hl2);

        v_viewCameras->setFixedSize(static_cast<int>(v_maxWinW * 0.2715), static_cast<int>(v_maxWinH * 0.3426));
        v_viewCameras->setModal(true);
        v_viewCameras->setLayout(vLayout);

        v_viewCameras->exec();
    }
}

void MainWindow::saveIDsCam()
{
    if(
            ui->ID_cam_g->currentData().toInt() != ui->ID_cam_m->currentData().toInt() &&
            ui->ID_cam_g->currentData().toInt() != ui->ID_cam_d->currentData().toInt() &&
            ui->ID_cam_m->currentData().toInt() != ui->ID_cam_d->currentData().toInt()
       ){
        QFile file(QString::fromStdString(camerasIDFile));
        file.setPermissions(QFileDevice::WriteUser);

        cv::FileStorage fs(camerasIDFile, cv::FileStorage::WRITE);

        if(fs.isOpened()){
            fs << "ID_camera_G" << ui->ID_cam_g->currentData().toInt();
            fs << "ID_camera_M" << ui->ID_cam_m->currentData().toInt();
            fs << "ID_camera_D" << ui->ID_cam_d->currentData().toInt();
            fs << "ID_camera_G_index" << ui->ID_cam_g->currentIndex();
            fs << "ID_camera_M_index" << ui->ID_cam_m->currentIndex();
            fs << "ID_camera_D_index" << ui->ID_cam_d->currentIndex();

            if(ui->ID_cam_g->currentData().toInt() != v_cam_G_acq->getIdCam()) v_cam_G_acq->setIDCam(ui->ID_cam_g->currentData().toInt());
            if(ui->ID_cam_m->currentData().toInt() != v_cam_M_acq->getIdCam()) v_cam_M_acq->setIDCam(ui->ID_cam_m->currentData().toInt());
            if(ui->ID_cam_d->currentData().toInt() != v_cam_D_acq->getIdCam()) v_cam_D_acq->setIDCam(ui->ID_cam_d->currentData().toInt());

            if(v_cam[Camera::DROITE]->getId() != ui->ID_cam_d->currentData().toInt()){
                v_cam[Camera::DROITE]->setId(ui->ID_cam_d->currentData().toInt());
            }

            if(v_cam[Camera::GAUCHE]->getId() != ui->ID_cam_g->currentData().toInt()){
                v_cam[Camera::GAUCHE]->setId(ui->ID_cam_g->currentData().toInt());
            }

            if(v_cam[Camera::MILIEU]->getId() != ui->ID_cam_m->currentData().toInt()){
                v_cam[Camera::MILIEU]->setId(ui->ID_cam_m->currentData().toInt());
            }
        }

        fs.release();
        file.setPermissions(QFileDevice::ReadUser);
    }
    else{
        cv::FileStorage fs(camerasIDFile, cv::FileStorage::READ);

        if(fs.isOpened()){
            ui->ID_cam_g->setCurrentIndex(fs["ID_camera_G_index"]);
            ui->ID_cam_m->setCurrentIndex(fs["ID_camera_M_index"]);
            ui->ID_cam_d->setCurrentIndex(fs["ID_camera_D_index"]);

            emit displayMsgSignal("IDs non sauvegardés: les caméras doivent avoir des IDs différents", RED_COLOR);
            fs.release();
        }
    }

}

void MainWindow::manualControlView()
{
    if(ui->manualControl_checkbox->isChecked()){
        ui->manualControl_checkbox->setStyleSheet(checkBoxStyle_3);

        if(ui->calibration_checkbox->isChecked()){
            ui->calibration_checkbox->setStyleSheet(checkBoxStyle_1);
            ui->calibration_checkbox->setChecked(false);
            ui->calibration_test_groupbox->hide();

            ui->frame_cameras->hide();

            if(ui->start_chronoBtn->text() != "GO"){
                if(v_chronoImg->isRunning()){
                     disconnect(v_chronoImg, SIGNAL(chrono(int)), this, SLOT(updateChrono(int)));
                     v_chronoImg->requestInterruption();
                     v_chronoImg->wait();
                }

                ui->cameras->setEnabled(true);
                ui->calib_img_progressbar->setValue(0);

                ui->start_chronoBtn->setText("GO");
                ui->start_chronoBtn->setStyleSheet(QString::fromStdString(v_pushButtonStyle_3));

                bool enable = true;
                ui->calib_delay_img->setEnabled(enable);
                ui->calib_number_of_img->setEnabled(enable);
                ui->calib_img_file_name->setEnabled(enable);
            }

            if(ui->calib_left_checkbox->isChecked()) ui->calib_left_checkbox->setChecked(false);
            if(ui->calib_middle_checkbox->isChecked()) ui->calib_middle_checkbox->setChecked(false);
            if(ui->calib_right_checkbox->isChecked()) ui->calib_right_checkbox->setChecked(false);
        }

        ui->camView->show();
        ui->cameras->show();
        ui->robot_fw->show();
        ui->robot_bw->show();
        ui->robot_left->show();
        ui->robot_right->show();
        ui->robot_stop->show();
        ui->robot_rot_ccw->show();
        ui->robot_rot_cw->show();
        ui->frame->show();
        ui->robot_free_positionning->show();

        ui->dispWA->hide();
        //ui->gen_map->hide();
        ui->measure->hide();
        ui->stop->hide();
        //ui->reuseTrajectory->hide();

#ifdef IHM_DEBUG
        emit displayMsgSignal("hide manual control view", YELLOW_COLOR);
#endif
    }
    else{
        ui->manualControl_checkbox->setStyleSheet(checkBoxStyle_1);

        ui->camView->hide();
        ui->cameras->hide();
        ui->robot_fw->hide();
        ui->robot_bw->hide();
        ui->robot_left->hide();
        ui->robot_right->hide();
        ui->robot_stop->hide();
        ui->robot_rot_ccw->hide();
        ui->robot_rot_cw->hide();
        ui->frame->hide();
        ui->robot_free_positionning->hide();

        ui->dispWA->show();
        //ui->gen_map->show();
        ui->measure->show();
        ui->stop->show();
        //ui->reuseTrajectory->show();

        if(ui->robot_free_positionning->isChecked()) ui->robot_free_positionning->setChecked(false);
        if(ui->robot_stop->isEnabled()) stopManualControl();

        if(ui->cameras->isChecked()){
            ui->cameras->setChecked(false);
            ui->cameras->setStyleSheet(groupBoxStyle_3);
        }

#ifdef IHM_DEBUG
        emit displayMsgSignal("show map view", GREEN_COLOR);
#endif
    }
}

void MainWindow::calibrationView()
{
    if(ui->calibration_checkbox->isChecked()){
        ui->calibration_checkbox->setStyleSheet(checkBoxStyle_3);

        if(ui->manualControl_checkbox->isChecked()){
            ui->manualControl_checkbox->setStyleSheet(checkBoxStyle_1);

            ui->manualControl_checkbox->setChecked(false);
            if(ui->robot_stop->isEnabled()) stopManualControl();
            if(ui->robot_free_positionning->isChecked()) ui->robot_free_positionning->setChecked(false);

            ui->robot_fw->hide();
            ui->robot_bw->hide();
            ui->robot_left->hide();
            ui->robot_right->hide();
            ui->robot_stop->hide();
            ui->robot_rot_ccw->hide();
            ui->robot_rot_cw->hide();
            ui->frame->hide();
            ui->robot_free_positionning->hide();
        }
        else{
            ui->dispWA->hide();
            //ui->gen_map->hide();
            ui->measure->hide();
            ui->stop->hide();
            //ui->reuseTrajectory->hide();
        }

        ui->camView->show();
        ui->cameras->show();
        ui->start_chronoBtn->show();
        ui->calibration_test_groupbox->show();
        ui->frame_cameras->show();

#ifdef IHM_DEBUG
        emit displayMsgSignal("hide calibration view", YELLOW_COLOR);
#endif
    }
    else{
        ui->calibration_checkbox->setStyleSheet(checkBoxStyle_1);

        ui->camView->hide();
        ui->cameras->hide();
        ui->start_chronoBtn->hide();
        ui->frame_cameras->hide();
        ui->calibration_test_groupbox->hide();

        ui->dispWA->show();
        //ui->gen_map->show();
        ui->measure->show();
        ui->stop->show();
        //ui->reuseTrajectory->show();

        if(ui->cameras->isChecked()){
            ui->cameras->setChecked(false);
            ui->cameras->setStyleSheet(groupBoxStyle_3);
        }

        if(ui->calib_left_checkbox->isChecked()) ui->calib_left_checkbox->setChecked(false);
        if(ui->calib_middle_checkbox->isChecked()) ui->calib_middle_checkbox->setChecked(false);
        if(ui->calib_right_checkbox->isChecked()) ui->calib_right_checkbox->setChecked(false);
    }
}

void MainWindow::targetHeight()
{
    if(ui->highButton->text() == "VALIDER"){

        if(ui->radioSavedHigh->isChecked()){
            ui->savedHigh->setReadOnly(false);
            ui->otherHigh->setReadOnly(true);

            ui->savedHigh->setStyleSheet(lineEditStyle_3);
            ui->otherHigh->setStyleSheet(lineEditStyle_2);
            ui->radioSavedHigh->setStyleSheet(radioStyle_1);
            ui->radioOtherHigh->setStyleSheet(radioStyle_2);
        }
        else{
            ui->savedHigh->setReadOnly(true);
            ui->otherHigh->setReadOnly(false);

            ui->savedHigh->setStyleSheet(lineEditStyle_2);
            ui->otherHigh->setStyleSheet(lineEditStyle_3);
            ui->radioSavedHigh->setStyleSheet(radioStyle_2);
            ui->radioOtherHigh->setStyleSheet(radioStyle_1);
        }
    }
}

void MainWindow::myAppend(QString msg, QColor color)
{
    emit displayMsgSignal(msg, color);
}

void MainWindow::messageView(bool checked)
{
    if(checked) ui->clean_msg_window->setStyleSheet(pushButtonEnabledStyle);
    else{
        ui->message->clear();
        ui->clean_msg_window->setStyleSheet(pushButtonDesabledStyle);
    }
}

void MainWindow::yVelPlusManualControl()
{
    manageJoystick(RobotMove::RGT,
                   this->v_right_btn_joy_status, this->v_left_btn_joy_status,
                   ui->robot_right, ui->robot_left);
}

void MainWindow::yVelMinusManualControl()
{
    manageJoystick(RobotMove::LFT,
                   this->v_left_btn_joy_status, this->v_right_btn_joy_status,
                   ui->robot_left, ui->robot_right);
}

void MainWindow::xVelPlusManualControl()
{
    manageJoystick(RobotMove::FWD,
                   this->v_fw_btn_joy_status, this->v_bw_btn_joy_status,
                   ui->robot_fw, ui->robot_bw);
}

void MainWindow::xVelMinusManualControl()
{
    manageJoystick(RobotMove::BWD,
                   this->v_bw_btn_joy_status, this->v_fw_btn_joy_status,
                   ui->robot_bw, ui->robot_fw);
}

void MainWindow::zAngVelPlusManualControl()
{
    manageJoystick(RobotMove::CCW,
                   this->v_ccw_btn_joy_status, this->v_cw_btn_joy_status,
                   ui->robot_rot_ccw, ui->robot_rot_cw);
}

void MainWindow::zAngVelMinusManualControl()
{
    manageJoystick(RobotMove::CW,
                   this->v_cw_btn_joy_status, this->v_ccw_btn_joy_status,
                   ui->robot_rot_cw, ui->robot_rot_ccw);
}

void MainWindow::stopManualControl()
{
    bool state1, state2;
    manageJoystick(RobotMove::STOP, state1, state2);
}

void MainWindow::checkCameras(int camIndex)
{
    int count = 0;

    for(int i = 0; i < v_nbCamera; i++){
        if(v_cameraBtn[i].getIsPresent()){
            count++;
            if(v_cameraBtn[i].getIsSelected() && (i != camIndex))
            {
                v_cameraBtn[i].setIsSelected(false);
            }
        }
    }

    v_cameraBtn[camIndex].setIsSelected(true);
    v_cameraBtn[camIndex].sendPixmap();
}

void MainWindow::_0_5_V()
{
    saveVelVoltOutputType(VelocityVoltageOutputType::TYPE_0_5_V);
    emit displayMsgSignal("Vitesse aérodynamique: type de sortie -> 0-5 V", GREEN_COLOR);
}

void MainWindow::_1_5_V()
{
    saveVelVoltOutputType(VelocityVoltageOutputType::TYPE_1_5_V);
    emit displayMsgSignal("Vitesse aérodynamique: type de sortie -> 1-5 V", GREEN_COLOR);
}

void MainWindow::camLeftView()
{
    runCamera(v_cam_G_acq, v_cam_D_acq, v_cam_M_acq);

    ui->cam_left->setStyleSheet(radioStyle_1);
    ui->cam_right->setStyleSheet(radioStyle_3);
    ui->cam_middle->setStyleSheet(radioStyle_3);
}

void MainWindow::camMiddleView()
{
    runCamera(v_cam_M_acq, v_cam_D_acq, v_cam_G_acq);

    ui->cam_left->setStyleSheet(radioStyle_3);
    ui->cam_right->setStyleSheet(radioStyle_3);
    ui->cam_middle->setStyleSheet(radioStyle_1);
}

void MainWindow::camRightView()
{
    runCamera(v_cam_D_acq, v_cam_M_acq, v_cam_G_acq);

    ui->cam_left->setStyleSheet(radioStyle_3);
    ui->cam_right->setStyleSheet(radioStyle_1);
    ui->cam_middle->setStyleSheet(radioStyle_3);
}

void MainWindow::calibrateCameras()
{
    QDir dir(QString::fromStdString(projectDirectory + "others"));

    if(dir.exists("pattern_config.yml")){
        cv::FileStorage fs(patternConfig, cv::FileStorage::READ);

        if(fs.isOpened()){
            int css = fs["checkerboard_sqare_size"];
            int ccw = fs["checkerboard_corner_width"];
            int cch = fs["checkerboard_corner_height"];

            fs.release();

            if(css > 0 && ccw > 0 && cch > 0){
                if(ui->calib_left_checkbox->isChecked()) v_cam[Camera::GAUCHE]->calcIntrinsicParams();
                if(ui->calib_middle_checkbox->isChecked()) v_cam[Camera::MILIEU]->calcIntrinsicParams();
                if(ui->calib_right_checkbox->isChecked()) v_cam[Camera::DROITE]->calcIntrinsicParams();
            }
            else{
                QMessageBox::warning(this, "", "Calibration impossible. \nVeuillez configurer "
                                               "correctement les paramètres de la mire de calibration"
                                               " dans le fichier ../others/pattern_config.yml");
            }
        }
    }
    else{
        QMessageBox::warning(this, "", "Calibration impossible. \nLe fichier pattern_config.yml est "
                                       "absent du dossier ../others/.");
    }

}

void MainWindow::runChrono()
{
    bool enable;
    bool camIsOpen = false;
    int camPosition = -1;

    v_countImage = 0;

    if(ui->cam_left->isChecked()){
        if(v_cam_G_acq->getCamIsOpen()) camIsOpen = true;
        camPosition = Camera::GAUCHE;
    }

    else if(ui->cam_right->isChecked()){
        if(v_cam_D_acq->getCamIsOpen()) camIsOpen = true;
        camPosition = Camera::DROITE;
    }

    else if(ui->cam_middle->isChecked()){
        if(v_cam_M_acq->getCamIsOpen()) camIsOpen = true;
        camPosition = Camera::MILIEU;
    }

    if(camIsOpen){

#ifdef IHM_DEBUG
        emit displayMsgSignal("Run chrono ...", QColor(255, 255, 0));
#endif
        if(ui->start_chronoBtn->text() == "GO"){
            QDir dir(QString::fromStdString(imageFiles4Calibration[camPosition]));
            QStringList fileList = dir.entryList();

            for(int i = 0; i < fileList.size(); i++){
                QString absolutePath = dir.absoluteFilePath(fileList[i]);
                if(fileList[i] != "." || fileList[i] != "..") dir.remove(absolutePath);
            }

            ui->cameras->setEnabled(false);

            ui->start_chronoBtn->setText("0");
            ui->start_chronoBtn->setStyleSheet(QString::fromStdString(v_pushButtonStyle_2));

            enable = false;
            ui->calib_delay_img->setEnabled(enable);
            ui->calib_number_of_img->setEnabled(enable);
            ui->calib_img_file_name->setEnabled(enable);

            connect(v_chronoImg, SIGNAL(chrono(int)), this, SLOT(updateChrono(int)));
            v_chronoImg->setTempo(ui->calib_delay_img->value());
            v_chronoImg->start();
        }
        else{
            if(v_chronoImg->isRunning()){
                 disconnect(v_chronoImg, SIGNAL(chrono(int)), this, SLOT(updateChrono(int)));
                 v_chronoImg->requestInterruption();
                 v_chronoImg->wait();
            }

            ui->cameras->setEnabled(true);
            ui->calib_img_progressbar->setValue(0);

            ui->start_chronoBtn->setText("GO");
            ui->start_chronoBtn->setStyleSheet(QString::fromStdString(v_pushButtonStyle_3));

            bool enable = true;
            ui->calib_delay_img->setEnabled(enable);
            ui->calib_number_of_img->setEnabled(enable);
            ui->calib_img_file_name->setEnabled(enable);
        }
    }

    else{
        emit displayMsgSignal("Caméra indisponible", RED_COLOR);
    }
}

void MainWindow::updateChrono(int chrono)
{
    ui->start_chronoBtn->setText(QString::fromStdString(to_string(chrono)));
    if(chrono == 0){

#ifdef IHM_DEBUG
        emit displayMsgSignal("FINI", YELLOW_COLOR);
#endif
        calibrationImage();
    }
}

void MainWindow::configView(bool status)
{
    manageCameras(status);

    ui->IDs_cams->setEnabled(!status);
    ui->camerasButton->setEnabled(!status);
    ui->threshold->setEnabled(!status);
    ui->calibration_test_groupbox->setEnabled(status);

    if(status){
        ui->calibration_test_groupbox->setStyleSheet(groupBoxStyle_9);
        ui->cameras->setStyleSheet(groupBoxStyle_2);
        ui->IDs_cams->setStyleSheet(groupBoxStyle_6);
        ui->camerasButton->setStyleSheet(pushButtonDesabledStyle);
        ui->threshold->setStyleSheet(pushButtonDesabledStyle);
        ui->start_chronoBtn->setStyleSheet(QString::fromStdString(v_pushButtonStyle_3));
    }
    else{
        ui->calibration_test_groupbox->setStyleSheet(groupBoxStyle_10);
        ui->IDs_cams->setStyleSheet(groupBoxStyle_7);
        ui->cameras->setStyleSheet(groupBoxStyle_3);
        ui->camerasButton->setStyleSheet(pushButtonEnabledStyle);
        ui->threshold->setStyleSheet(pushButtonEnabledStyle);

        if(ui->start_chronoBtn->text() != "GO"){
            if(v_chronoImg->isRunning()){
                 disconnect(v_chronoImg, SIGNAL(chrono(int)), this, SLOT(updateChrono(int)));
                 v_chronoImg->requestInterruption();
                 v_chronoImg->wait();
            }

            if(!ui->cameras->isEnabled()) ui->cameras->setEnabled(true);
            ui->calib_img_progressbar->setValue(0);

            ui->start_chronoBtn->setText("GO");

            ui->calib_delay_img->setEnabled(!status);
            ui->calib_number_of_img->setEnabled(!status);
            ui->calib_img_file_name->setEnabled(!status);
        }
        ui->start_chronoBtn->setStyleSheet(QString::fromStdString(v_pushButtonStyle_4));
    }
}

void MainWindow::changeHeightList(QString text)
{
    int sz = text.size();

    if(sz > 0){
        if(ui->radioSavedHigh->isChecked()) v_cursorPos = ui->savedHigh->cursorPosition();
        else v_cursorPos = ui->otherHigh->cursorPosition();

        for(int i = 0; i < sz; i++){
            if((text[i] < 48 || text[i] > 57) && text[i] != QChar(';')){
                QStringList list = text.split(text[i], Qt::SkipEmptyParts);
                QString newText = "";

                for(int j = 0; j < list.size(); j++){
                    newText += list[j];
                }

                if(newText.size() > 0) text = newText;
                else text = "";

                v_cursorPos--;

                break;
            }
        }

        if(ui->radioSavedHigh->isChecked()){
            ui->savedHigh->setText(text);
            ui->savedHigh->setCursorPosition(v_cursorPos);
        }
        else{
            ui->otherHigh->setText(text);
            ui->otherHigh->setCursorPosition(v_cursorPos);
        }
    }
}

void MainWindow::loadParamIHM()
{
    cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);
    int nh;
    int val;
    int temp;
    double vald;
    std::string heights;

    ui->IP_PC->setText(QString::fromStdString(fs["IP_PC"]));
    ui->IP_Robotino->setText(QString::fromStdString(fs["IP_Robotino"]));

    fs["robot_diameter"] >> val;
    ui->diameter->setValue(val);

    fs["tool_length"] >> val;
    ui->toolLength->setValue(val);

    fs["tool_width"] >> val;
    ui->toolWidth->setValue(val);

    fs["leds_height"] >> val;
    ui->ledHigh->setValue(val);

    fs["max_linear_velocity"] >> val;
    ui->linearVelMax->setValue(val);

    fs["min_angular_velocity"] >> vald;
    ui->angularVelMin->setValue(vald);

    fs["working_area_length"] >> val;
    ui->waLength->setValue(val);

    fs["working_area_width"] >> val;
    ui->waWidth->setValue(val);

    fs["measurement_point_dx"] >> val;
    ui->dx->setValue(val);

    fs["measurement_point_dy"] >> val;
    ui->dy->setValue(val);

    fs["rest_time"] >> val;
    ui->rest->setValue(val);

    fs["measure_period"] >> vald;
    ui->period->setValue(vald);

    fs["measure_timer"] >> val;
    ui->measure_timer->setValue(val);

    fs["number_of_heights"] >> nh;

    if(nh > 0){
        for(int i = 0; i < nh; i++){
            fs["h_" + to_string(i+1)] >> temp;
            heights += to_string(temp);

            if(i < nh - 1){
                heights += ';';
            }
        }
        ui->savedHigh->setText(QString::fromStdString(heights));
    }
    else ui->savedHigh->setText("");

    fs["pulley_radius"] >> vald;
    ui->pulley_radius->setValue(vald);

    fs["motor_precision_index"] >> val;
    ui->motor_precision->setCurrentIndex(val);

    fs["duty_cycle_index"] >> val;
    ui->duty_cycle->setCurrentIndex(val);

    fs["lin_vel_motor_mast"] >> val;
    ui->lin_vel_motor_mast->setValue(val);

    fs.release();

    //...
    fs.open(camerasIDFile, cv::FileStorage::READ);

    if(fs.isOpened()){
        ui->ID_cam_g->setCurrentIndex(fs["ID_camera_G_index"]);
        ui->ID_cam_m->setCurrentIndex(fs["ID_camera_M_index"]);
        ui->ID_cam_d->setCurrentIndex(fs["ID_camera_D_index"]);

        v_cam_G_acq->setIDCam(fs["ID_camera_G"]);
        v_cam_M_acq->setIDCam(fs["ID_camera_M"]);
        v_cam_D_acq->setIDCam(fs["ID_camera_D"]);
    }

    fs.release();

    ui->robot_free_pos_x->setMaximum(ui->waWidth->value());
    ui->robot_free_pos_y->setMaximum(ui->waLength->value());
}

void MainWindow::displayMsgSlot(const QString &msg, const QColor &textColor, const QColor &textBackgroundColor)
{
    if(ui->log->isChecked()){
        ui->message->setTextColor(textColor);
        ui->message->setTextBackgroundColor(textBackgroundColor);
        emit sendMessage(">>> " + msg);
    }
}

void MainWindow::robotFreePos(bool checked)
{
    if(checked){
        ui->robot_free_pos_go->setStyleSheet(pushButtonEnabledStyle);
        ui->robot_free_positionning->setStyleSheet(groupBoxStyle_12);

        if(ui->robot_free_pos_with_measure->isChecked()){
            ui->robot_free_pos_with_measure->setStyleSheet(radioStyle_1);
            ui->robot_free_pos_without_measure->setStyleSheet(radioStyle_3);
        }
        else{
            ui->robot_free_pos_with_measure->setStyleSheet(radioStyle_3);
            ui->robot_free_pos_without_measure->setStyleSheet(radioStyle_1);
        }
    }
    else{
        ui->robot_free_pos_go->setStyleSheet(pushButtonDesabledStyle);
        ui->robot_free_positionning->setStyleSheet(groupBoxStyle_11);

        if(ui->robot_free_pos_go->text() == "STOP"){
            robotFPLaunchBtn();
        }
    }

    enableJoystick(!checked);
}

void MainWindow::robotFPXMax(int val)
{
    ui->robot_free_pos_x->setMaximum(val);

    if(ui->robot_free_pos_x->value() > val){
        ui->robot_free_pos_x->setValue(val);
    }
}

void MainWindow::robotFPYMax(int val)
{
    ui->robot_free_pos_y->setMaximum(val);

    if(ui->robot_free_pos_y->value() > val){
        ui->robot_free_pos_y->setValue(val);
    }
}

void MainWindow::robotFPLaunchBtn()
{
    if(ui->robot_free_pos_go->text() == "GO") startMeasure();
    else if(ui->robot_free_pos_go->text() == "STOP") stop();
}

void MainWindow::takeMeasureManualControl()
{
    if(ui->robot_free_pos_with_measure->isChecked() && !v_IHMStatus){
        ui->highToAchieve->setEnabled(true);
        ui->highToAchieve->setStyleSheet(groupBoxStyle_2);
        ui->highButton->setStyleSheet(pushButtonEnabledStyle);
    }
    else if(ui->robot_free_pos_without_measure->isChecked() && !v_IHMStatus){
        ui->highToAchieve->setEnabled(false);
        ui->highToAchieve->setStyleSheet(groupBoxStyle_3);
        ui->highButton->setStyleSheet(pushButtonDesabledStyle);
    }

    if(ui->robot_free_pos_with_measure->isChecked()){
        ui->robot_free_pos_with_measure->setStyleSheet(radioStyle_1);
        ui->robot_free_pos_without_measure->setStyleSheet(radioStyle_3);
    }
    else if(ui->robot_free_pos_without_measure->isChecked()){
        ui->robot_free_pos_with_measure->setStyleSheet(radioStyle_3);
        ui->robot_free_pos_without_measure->setStyleSheet(radioStyle_1);
    }
}

void MainWindow::showCorrectorDial()
{
    delete v_corrector;

    int type;

    if(v_correctorP_action->isChecked()) type = CorrectorType::P;
    else if(v_correctorPI_action->isChecked()) type = CorrectorType::PI;
    else type = CorrectorType::PID;

    v_corrector = new CorrectorDialog(v_maxWinW, v_maxWinH, type);
    v_corrector->open();
}

void MainWindow::startMeasure()
{
    QFile file(QString::fromStdString(saveProject) + "current_project.name");
    if((file.exists() && !ui->manualControl_checkbox->isChecked()) ||
            ui->manualControl_checkbox->isChecked()){
        QString text;
        bool launch = true;

        if(!ui->manualControl_checkbox->isChecked()){
            char data[255];

            if(file.open(QIODevice::OpenModeFlag::ReadOnly)){
                file.readLine(data, 255);
                std::string pfn(data);

                if(QFile(QString::fromStdString(saveProject + pfn + ".xlsx")).exists()){
                    text = "Un fichier nommé " + QString::fromStdString(pfn + ".xlsx");
                    text += " existe déjà dans le dossier " + QDir(QString::fromStdString(saveProject)).absolutePath();
                    text += " et sera remplacé si vous continuer.\n";

                    int response = QMessageBox::information(this, "", text, QMessageBox::Cancel | QMessageBox::Ok);

                    if(response == QMessageBox::Cancel) launch = false;
                    else if(response == QMessageBox::Ok){
                        QFile(QString::fromStdString(saveProject + pfn + ".xlsx")).remove();
                        launch = true;
                    }
                }
            }
        }

        if (launch){
            text = "Afin de procéder au déplacement du robot et à la prise des mesures, "
                   "la zone de travail va être calibrée. Avant de continuer, veuillez procéder aux vérifications suivantes:"
                   "\n\n1) Assurez-vous que les index des caméras de la chambre climatique sont correctement configurés sur l'IHM"
                   "\n\n2) Assurez-vous que les cibles au sol, qui permettent la détection de la zone de travail, sont bien visibles à partir de toutes les caméras"
                   "\n\n3) Assurez-vous qu'aucun objet n'interfère avec la détection des couleurs des cibles au sol. Le cas échéant, "
                   "veuillez recouvrir les zones parasites."
                   "\n\nAppuyez sur \"Annuler\" pour revenir en arrière, sinon appuyez sur \"Ok\"";

            int response = QMessageBox::information(this, "Calibration de la zone de travail", text, QMessageBox::Cancel | QMessageBox::Ok);

            if(response == QMessageBox::Ok) measure();
        }
    }
    else if(!file.exists() && !ui->manualControl_checkbox->isChecked()){
        QMessageBox::critical(this, "", "Veuillez définir un nom de projet avant de continuer.");
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    quitIHM();
    event->accept();
}

void MainWindow::extractMeasurePoints(Mat map, std::vector<Point> mesPoint, std::vector<Point> &reachablePoints)
{
    for(uint i = 0; i < mesPoint.size(); i++){
        if(map.ptr(static_cast<int>(static_cast<double>(mesPoint[i].y)*scale))[static_cast<int>(static_cast<double>(mesPoint[i].x)*scale)] == 255){
            reachablePoints.emplace_back(mesPoint[i]);
        }
        else{
            v_unReachablePoints.emplace_back(mesPoint[i]);
        }
    }
}

void MainWindow::orderingMeasurePoints(std::vector<Point> &reachablePoints)
{
    uint step;
    bool direction = Direction::LEFT_2_RIGHT;
    for(uint i = 0; i < reachablePoints.size()-1; i++){
        for(uint j = i+1; j < reachablePoints.size(); j++){
            if(reachablePoints[i].y > reachablePoints[j].y){
                Point2f temp = reachablePoints[i];
                reachablePoints[i] = reachablePoints[j];
                reachablePoints[j] = temp;
            }
        }
    }

    for(uint i = 0; i < reachablePoints.size(); i+= step){
        step = 1;
        for(uint j = i+1; j < reachablePoints.size();j++){
            if(reachablePoints[i].y == reachablePoints[j].y) step++;
            else break;
        }

        if(step != 1){
            for(uint k = i; k < (step+i-1); k++){
                for(uint l = k + 1; l < (step +i); l++){
                    if(direction == Direction::LEFT_2_RIGHT){
                        if(reachablePoints[k].x > reachablePoints[l].x){
                            Point2f temp = reachablePoints[k];
                            reachablePoints[k] = reachablePoints[l];
                            reachablePoints[l] = temp;
                        }
                    }else if(direction == Direction::RIGHT_2_LEFT){
                        if(reachablePoints[k].x < reachablePoints[l].x){
                            Point2f temp = reachablePoints[k];
                            reachablePoints[k] = reachablePoints[l];
                            reachablePoints[l] = temp;
                        }
                    }
                }
            }
        }
        direction = 1 - direction;
    }

    /*cv::FileStorage fs("C:\\Users\\vanec\\Bureau\\reachablePoints.yml", cv::FileStorage::WRITE);
    fs << "Points" << "[";
    for(uint i = 0; i < reachablePoints.size(); i++){
        std::string input = " ( " + to_string(reachablePoints[i].y) + " ; " + to_string(reachablePoints[i].x) + " )";
        fs << input;
    }
    fs << "]";
    fs.release();*/

    v_reachablePoints = reachablePoints;

#ifdef IHM_DEBUG
    emit displayMsgSignal("Points classés", GREEN_COLOR);
#endif
}

void MainWindow::computeRobotPositions(
        cv::Mat map4RobotPosition, std::vector<cv::Point> reachablePoints,
        int length_tool, int areaW, int areaL)
{
    for(uint i = 0; i < reachablePoints.size(); i++){
        auto limInf = static_cast<int>((reachablePoints[i].y - length_tool)*scale);
        auto limSup = static_cast<int>((reachablePoints[i].y + length_tool)*scale);
        int y1, y2;
        int x;

        if(limInf < 0) limInf = 0;
        if(limSup > static_cast<int>(areaW*scale)) limSup = static_cast<int>(areaW*scale);

        std::uniform_int_distribution<int> distributionX(limInf, limSup);

        for(;;){
            x = distributionX(*QRandomGenerator::global());
            y1 = cvFloor(reachablePoints[i].x*scale + sqrt((length_tool*length_tool*scale*scale) -
                                             (x - reachablePoints[i].y*scale)*(x - reachablePoints[i].y*scale)));

            y2 = cvFloor(reachablePoints[i].x*scale - sqrt((length_tool*length_tool*scale*scale) -
                                             (x - reachablePoints[i].y*scale)*(x - reachablePoints[i].y*scale)));

            if((y1 > 0 && y1 < areaL*scale) && (static_cast<int>(map4RobotPosition.ptr(x)[y1]) == 255)){
                v_robotPositionPoints.emplace_back(Point(static_cast<int>(y1/scale), static_cast<int>(x/scale)));
                break;
            }
            if((y2 > 0  && y2 < areaL*scale) && (static_cast<int>(map4RobotPosition.ptr(x)[y2]) == 255)){
                v_robotPositionPoints.emplace_back(Point(static_cast<int>(y2/scale), static_cast<int>(x/scale)));
                break;
            }

        }
    }

#ifdef IHM_DEBUG
    emit displayMsgSignal("Points de positionnement du robot OK ...", GREEN_COLOR);
#endif
}

void MainWindow::calcPosition(cv::Mat &initialRobotPosition, std::vector<cv::Mat> &robotPositionByCam, int areaW, int areaL)
{
    int countCamera = 0;
    std::vector<bool> positionIsFound(3);
    /*CLOCK_TIME_POINT t_test;

    t_test = CLOCK_NOW;*/

    for(uint i = 0; i < 3; i++){
        positionIsFound[i] = false;
    }

    for(uint i = 0; i < 3; i++){
        v_cam[i]->capture();
    }

    for(uint i = 0; i < 3; i++){
        robotPositionByCam[i] = v_cam[i]->getRobotPosition();

        if(robotPositionByCam[i].ptr<double>(0)[0] > 0 &&
           robotPositionByCam[i].ptr<double>(0)[1] > 0 &&
           robotPositionByCam[i].ptr<double>(0)[0] <= areaW &&
           robotPositionByCam[i].ptr<double>(0)[1] <= areaL){
           countCamera++;
           positionIsFound[i] = true;
        }
    }

    if(countCamera != 0){
        double pos_cam_L_x = robotPositionByCam[Camera::GAUCHE].ptr<double>(0)[0];
        double pos_cam_L_y = robotPositionByCam[Camera::GAUCHE].ptr<double>(0)[1];
        double pos_cam_L_t = robotPositionByCam[Camera::GAUCHE].ptr<double>(0)[2];
        double weight_L = positionIsFound[Camera::GAUCHE] ? 1.0 : 0.0;

        double pos_cam_M_x = robotPositionByCam[Camera::MILIEU].ptr<double>(0)[0];
        double pos_cam_M_y = robotPositionByCam[Camera::MILIEU].ptr<double>(0)[1];
        double pos_cam_M_t = robotPositionByCam[Camera::MILIEU].ptr<double>(0)[2];
        double weight_M = positionIsFound[Camera::MILIEU] ? 1.0 : 0.0;

        double pos_cam_R_x = robotPositionByCam[Camera::DROITE].ptr<double>(0)[0];
        double pos_cam_R_y = robotPositionByCam[Camera::DROITE].ptr<double>(0)[1];
        double pos_cam_R_t = robotPositionByCam[Camera::DROITE].ptr<double>(0)[2];
        double weight_R = positionIsFound[Camera::DROITE] ? 1.0 : 0.0;

        cv::Mat weights = (cv::Mat_<double>(1, 3) <<
                          weight_L, weight_M, weight_R);

        cv::Mat pos_cam = (cv::Mat_<double>(3, 3) <<
                           pos_cam_L_x, pos_cam_L_y, pos_cam_L_t,
                           pos_cam_M_x, pos_cam_M_y, pos_cam_M_t,
                           pos_cam_R_x, pos_cam_R_y, pos_cam_R_t);

        initialRobotPosition = weights * pos_cam;
        initialRobotPosition /= countCamera;
    }
    else{
        initialRobotPosition = (cv::Mat_<double>(1,3) <<
                                POSITION_NOT_FOUND,
                                POSITION_NOT_FOUND,
                                POSITION_NOT_FOUND);
    }

#ifdef IHM_DEBUG
    string msg;

    msg = "Position initiale du robot OK ...";
    msg += "\n\t(" + std::to_string(initialRobotPosition.ptr<double>(0)[0]) +
            " ; " + std::to_string(initialRobotPosition.ptr<double>(0)[1]) +
            " ; " + std::to_string(initialRobotPosition.ptr<double>(0)[2]) + ")";
    emit displayMsgSignal(QString::fromStdString(msg), GREEN_COLOR);
#endif
}
void MainWindow::generatePath(cv::Mat initialRobotPosition, Robot &robot)
{
    cv::FileStorage fs;
    string s;
    cv::Mat tempGP;

    if(!ui->manualControl_checkbox->isChecked()){
        if(!ui->reuseTrajectory->isChecked()){
            size_t sizeGP;
            TopographicMap tm(robot);
            QFile file(QString::fromStdString(saveTrajectory));

            Path path(tm, robot);

            v_generatedPath = path.generate(std::move(initialRobotPosition));
            sizeGP = v_generatedPath.size();

            computeRobotOrientation();

            file.setPermissions(QFileDevice::WriteUser);
            fs.open(saveTrajectory, cv::FileStorage::WRITE);
            fs << "size" << static_cast<int>(sizeGP);
            for(uint i = 0; i < sizeGP; i++){
                s = "_" + to_string(i);
                fs << s << v_generatedPath[i];
            }
            fs.release();
            file.setPermissions(QFileDevice::ReadUser);
        }
        else{
            int sgp;
            fs.open(saveTrajectory, cv::FileStorage::READ);
            fs["size"] >> sgp;
            v_generatedPath.clear();
            for(int i = 0; i < sgp; i++){
                tempGP.release();
                s = "_" + to_string(i);
                fs[s] >> tempGP;
                v_generatedPath.emplace_back(tempGP);
            }
            fs.release();
        }
    }
    else{
        TopographicMap tm(robot);

        Path path(tm, robot);

        v_generatedPath = path.generate(std::move(initialRobotPosition));
        computeRobotOrientation();
    }

#ifdef IHM_DEBUG
    emit displayMsgSignal("Trajectoire générée ...", GREEN_COLOR);
#endif
}

void MainWindow::computeRobotOrientation(){
    int nbRow;
    cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);
    int height_LED;
    double theta;
    double sigma;

    cv::Mat robot_center;
    cv::Mat sensor_end;
    cv::Mat measure_pt;

    fs["leds_height"] >> height_LED;
    fs.release();

    for(int i = 0; i < v_generatedPath.size(); i++){
        nbRow = v_generatedPath[i].rows;

        sigma = 0.;
        theta = v_generatedPath[i].ptr<double>(nbRow - 1)[2];

        robot_center = (Mat_<double>(4, 1) <<
                        v_generatedPath[i].ptr<double>(nbRow - 1)[0],
                        v_generatedPath[i].ptr<double>(nbRow - 1)[1],
                        height_LED,
                        1);

        sensor_end = (Mat_<double>(4, 1) <<
                        v_generatedPath[i].ptr<double>(nbRow - 1)[0] + 300 * cos(theta),
                        v_generatedPath[i].ptr<double>(nbRow - 1)[1] + 300 * sin(theta),
                        height_LED,
                        1);

        measure_pt = (Mat_<double>(4, 1) <<
                      v_reachablePoints[i].y,
                      v_reachablePoints[i].x,
                      0,
                      1);

        sigma = atan2(measure_pt.ptr<double>(1)[0] - robot_center.ptr<double>(1)[0],
                measure_pt.ptr<double>(0)[0] - robot_center.ptr<double>(0)[0]);

        sigma -= atan2(sensor_end.ptr<double>(1)[0] - robot_center.ptr<double>(1)[0],
                sensor_end.ptr<double>(0)[0] - robot_center.ptr<double>(0)[0]);

        theta += sigma;

        if (theta <= -M_PI) theta += 2 * M_PI;
        else if (theta >= M_PI) theta -= 2 * M_PI;

        v_generatedPath[i].ptr<double>(nbRow - 1)[2] = theta;
    }
}

void MainWindow::generateFinalMap(cv::Mat &map, std::vector<cv::Point> reachablePoints, int areaW)
{
    //Affichages des points:
    //  - En rouge: points inaccessibles
    //  - En bleu: positionnement du robot par rapport à chaque points accessibles

    int radius = 3;
    int gapy = 120;
    int gapx = 200;
    int axeLength = 100;
    int x_i, x_f, y_i, y_f;
    cv::Point org(static_cast<int>(gapy*scale), static_cast<int>((areaW - gapx)*scale));
    cv::Point orgX, orgY;
    //cv::Mat final_init, final_temp;

    cv::bitwise_not(map, map);
    cv::cvtColor(map, map, cv::COLOR_GRAY2BGR);

    //Points inatteignables
    for(uint i = 0; i < v_unReachablePoints.size(); i ++){
        circle(map,
               Point(static_cast<int>(static_cast<double>(v_unReachablePoints[i].x)*scale),
                     static_cast<int>(static_cast<double>(v_unReachablePoints[i].y)*scale)),
               radius,
               Scalar(0,0,255),
               cv::FILLED);
    }

    //Points atteignables
    for(uint i = 0; i < reachablePoints.size(); i ++){
        circle(map,
               Point(static_cast<int>(reachablePoints[i].x*scale),
                     static_cast<int>(reachablePoints[i].y*scale)),
               radius,
               Scalar(50,50,50),
               cv::FILLED);
    }

    //Premier point de positionnement du centre du robot
    circle(map,
           Point(static_cast<int>(v_robotPositionPoints[0].x*scale),
                 static_cast<int>(v_robotPositionPoints[0].y*scale)),
           radius,
           Scalar(0,255,255),
           cv::FILLED);

    //Affichage du repère *********************************************************************************************

    cv::arrowedLine(map,
                    org,
                    cv::Point(static_cast<int>(gapy*scale), static_cast<int>((areaW - gapx + axeLength)*scale)),
                    Scalar(0,0,255), 2); // axe x
    cv::arrowedLine(map,
                    org,
                    cv::Point(static_cast<int>((gapy + axeLength)*scale), static_cast<int>((areaW - gapx)*scale)),
                    Scalar(0,255,0), 2); // axe y
    orgX = org + cv::Point(static_cast<int>(-60*scale), static_cast<int>((axeLength/2)*scale));
    orgY = org + cv::Point(static_cast<int>((axeLength/2)*scale), static_cast<int>(-20*scale));
    cv::putText(map, "x", orgX, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));
    cv::putText(map, "y", orgY, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 0));

    //Fin affichage du repère *****************************************************************************************

    x_i = static_cast<int>(v_initialRobotPosition.ptr<double>(0)[0] * scale);
    y_i = static_cast<int>(v_initialRobotPosition.ptr<double>(0)[1] * scale);

    x_f = static_cast<int>(v_generatedPath[0].ptr<double>(0)[0] * scale);
    y_f = static_cast<int>(v_generatedPath[0].ptr<double>(0)[1] * scale);

    cv::arrowedLine(map,
             Point(y_i, x_i),
             Point(y_f, x_f),
             Scalar(0, 0, 255),
             1);

    if(v_generatedPath[0].rows > 1){
        for(int i = 0; i < v_generatedPath[0].rows - 1; i++){
            x_i = static_cast<int>(v_generatedPath[0].ptr<double>(i)[0] * scale);
            y_i = static_cast<int>(v_generatedPath[0].ptr<double>(i)[1] * scale);

            x_f = static_cast<int>(v_generatedPath[0].ptr<double>(i+1)[0] * scale);
            y_f = static_cast<int>(v_generatedPath[0].ptr<double>(i+1)[1] * scale);

            cv::arrowedLine(map,
                     Point(y_i, x_i),
                     Point(y_f, x_f),
                     Scalar(0, 0, 255),
                     1);
        }
    }

    v_map = map.clone();
}

void MainWindow::updateIHM(cv::Mat &map, int nbPoint)
{
    cv::FileStorage fs;

    //Affichage de la map sur l'IHM
    cv::Mat img;
    cv::resize(map, img, cv::Size(), this->v_resizeWidthImage, this->v_resizeHeightImage);
    *v_qPixWA = Mat2Pixmap(img);
    ui->dispWA->setPixmap(*v_qPixWA);


    //Affichage du nombre de mesures à effectuer et du nombre de point au sol
    if(!ui->manualControl_checkbox->isChecked() || ui->robot_free_pos_with_measure->isChecked()){
        this->v_nb_of_measurement_points = nbPoint;

        if(ui->radioSavedHigh->isChecked()){
            fs.open(saveParamIHMFile, cv::FileStorage::READ);
            fs["number_of_heights"] >> this->v_nb_of_height;
            fs.release();

            this->nb_of_process_points = this->v_nb_of_measurement_points * this->v_nb_of_height;
        }else if(ui->radioOtherHigh->isChecked()){
            QString text = ui->otherHigh->text();
            QStringList strList = text.split(QChar(';'), Qt::SkipEmptyParts);

            if(strList.size() > 0){

                this->v_nb_of_height = strList.size();
                this->nb_of_process_points = this->v_nb_of_measurement_points * this->v_nb_of_height;
            }
        }

        ui->numberOfPoints->display(this->v_nb_of_measurement_points);
        ui->numberOfMeasures->display(this->nb_of_process_points);
    }
}

void MainWindow::sendMotorParamToNode()
{
    ParametersIHM data;
    Serializer serializer;

    uint8 index = Index::PARAMETERS;
    cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);

    fs["duty_cycle"] >> data.dutyCycle;
    fs["pulley_radius"] >> data.pulleyRadius;
    fs["motor_precision"] >> data.motorAccuracy;
    fs["motor_precision_divider"] >> data.motorAccuracyDivider;
    fs["lin_vel_motor_mast"] >> data.linVelMotorMast;
    fs.release();

    serializer.clearBuffer();
    serializer.write(index);
    serializer.write(data.dutyCycle);
    serializer.write(data.pulleyRadius);
    serializer.write(data.motorAccuracy);
    serializer.write(data.motorAccuracyDivider);
    serializer.write(data.linVelMotorMast);

    runClient(serializer.buffer(), serializer.bufferSize());
}

void MainWindow::sendHeightListToNode()
{
    HeightsList data;
    Serializer serializer;

    uint8 index = Index::HEIGHTS;
    data.heights.clear();

    if(ui->radioSavedHigh->isChecked()){
        int nb_height;
        cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);

        fs["number_of_heights"] >> nb_height;

        for(int i = 0; i < nb_height; i++){
            uint32 val;
            int t;
            fs["h_" + to_string(i+1)] >> t;

            val = static_cast<uint32>(t);
            data.heights.push_back(val);
        }

        fs.release();
    }
    else if(ui->radioOtherHigh->isChecked()){
        std::vector<int> height;
        QString text = ui->otherHigh->text();
        QStringList strList = text.split(QChar(';'), Qt::SkipEmptyParts);

        if(strList.size() > 0){
            for(int i = 0; i < strList.size(); i++){
                bool ok = false;
                int val = strList[i].toInt(&ok);

                if(ok) height.emplace_back(val);
            }

            for(uint i = 0; i < height.size() - 1; i++){
                for(uint j = i+1; j < height.size(); j++){
                    if(height[i] > height[j]){
                        int temp = height[i];
                        height[i] = height[j];
                        height[j] = temp;
                    }
                }
            }
        }

        for(uint i = 0; i < height.size(); i++) data.heights.push_back(static_cast<uint32>(height[i]));
    }

    serializer.clearBuffer();
    serializer.write(index);
    serializer.write(data.heights);

    runClient(serializer.buffer(), serializer.bufferSize());
}

void MainWindow::sendDelaysToNode()
{
    Delay data;
    Serializer serializer;

    uint8 index = Index::DELAY;
    cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);

    if(fs.isOpened()){
        fs["rest_time"] >> data.restDelay;
        fs["measure_timer"] >> data.measureDelay;
        fs["measure_period"] >> data.periodDelay;
        fs.release();

        serializer.clearBuffer();
        serializer.write(index);
        serializer.write(data.restDelay);
        serializer.write(data.measureDelay);
        serializer.write(data.periodDelay);

        runClient(serializer.buffer(), serializer.bufferSize());
    }
}

void MainWindow::sendVoltOutputTypeToNode()
{
    int32 type;
    uint8 index;
    Serializer serial;
    cv::FileStorage fs(saveVVOType, cv::FileStorage::READ);

    if(fs.isOpened()){
        fs["type"] >> type;
        fs.release();

        index = Index::VEL_VOLT_OUT_TYPE;
        serial.write(index);
        serial.write(type);

        runClient(serial.buffer(), serial.bufferSize());
    }
}

void MainWindow::sendStopToNode()
{
    uint8 index = Index::STOP_MEASURE;
    Serializer serial;

    serial.write(index);
    runClient(serial.buffer(), serial.bufferSize());
}

void MainWindow::enableWidgets(bool enabled)
{
    QString styleSheet;
    QString css;

    if(enabled){
        css = pushButtonEnabledStyle;

        ui->processedPoints->setStyleSheet(lcdStyle_1);
        ui->numberOfPoints->setStyleSheet(lcdStyle_2);
        ui->processedMeasures->setStyleSheet(lcdStyle_3);
        ui->numberOfMeasures->setStyleSheet(lcdStyle_4);

        qApp->setStyleSheet(qAppStyle_1);

        if(ui->manualControl->isChecked() && !ui->cameras->isChecked()) ui->cameras->setStyleSheet(groupBoxStyle_3);
    }
    else{
        css = pushButtonDesabledStyle;

        ui->processedPoints->setStyleSheet(lcdStyle_5);
        ui->numberOfPoints->setStyleSheet(lcdStyle_5);
        ui->processedMeasures->setStyleSheet(lcdStyle_5);
        ui->numberOfMeasures->setStyleSheet(lcdStyle_5);

        qApp->setStyleSheet(qAppStyle_2);
    }

    ui->IP_PC->setStyleSheet(lineEditStyle_1);
    ui->IP_Robotino->setStyleSheet(lineEditStyle_1);

    styleSheet = groupBoxStyle_4;

    //ui->IPButton->setStyleSheet(css);
    ui->highButton->setStyleSheet(css);
    ui->ros_node->setStyleSheet(pushButtonEnabledStyle);
    ui->ros_master->setStyleSheet(pushButtonEnabledStyle);
    ui->gen_map->setStyleSheet(css);
    ui->measure->setStyleSheet(css);
    ui->stop->setStyleSheet(pushButtonDesabledStyle);

    ui->IPAdress->setStyleSheet(styleSheet);
    if(ui->cameras->isChecked()) ui->IDs_cams->setStyleSheet(groupBoxStyle_6);
    else ui->IDs_cams->setStyleSheet(styleSheet);
    ui->config_motor_mast->setStyleSheet(styleSheet);
    ui->config_robot->setStyleSheet(styleSheet);
    ui->delay->setStyleSheet(styleSheet);
    ui->measurementPoints->setStyleSheet(styleSheet);
    ui->workingArea->setStyleSheet(styleSheet);
    ui->highToAchieve->setStyleSheet(styleSheet);
    ui->manualControl->setStyleSheet(styleSheet);
    ui->calibration->setStyleSheet(styleSheet);
    ui->advancement->setStyleSheet(styleSheet);
    ui->log->setStyleSheet(styleSheet);
    //ui->calibration_test_groupbox->setStyleSheet(styleSheet);

    //ui->IPButton->setEnabled(enabled);
    ui->config_robot->setEnabled(enabled);
    ui->advancement->setEnabled(enabled);
    ui->config_motor_mast->setEnabled(enabled);
    ui->delay->setEnabled(enabled);
    ui->measurementPoints->setEnabled(enabled);
    ui->workingArea->setEnabled(enabled);
    ui->highToAchieve->setEnabled(enabled);
    //ui->ros_master->setEnabled(enabled);
    //ui->ros_node->setEnabled(enabled);
    ui->gen_map->setEnabled(enabled);
    ui->measure->setEnabled(enabled);
    ui->stop->setEnabled(false);
    ui->reuseTrajectory->setEnabled(enabled);

    this->v_IHMStatus = enabled;
}

void MainWindow::saveVelVoltOutputType(int type)
{
    cv::FileStorage fs;
    QFile file(QString::fromStdString(saveVVOType));
    file.setPermissions(QFileDevice::WriteUser);

    fs.open(saveVVOType, cv::FileStorage::WRITE);
    if(fs.isOpened()) fs << "type" << type;
    fs.release();

    file.setPermissions(QFileDevice::ReadUser);
}

void rosMaster(){
    QDir dir(QString::fromStdString(projectDirectory));
    string path = dir.absolutePath().toStdString();
    string newPath = path.append("/scripts");
    string batchFile = "setup.bat";
    string command = "cd " + newPath + " && " + batchFile;

    system(command.c_str());
}

void rosNodeList(){
    QDir dir(QString::fromStdString(projectDirectory));
    string path = dir.absolutePath().toStdString();
    string newPath = path.append("/others/node_list.txt");
    string command = "C:/opt/ros/noetic/x64/setup.bat && rosnode list && pause";
    system(command.c_str());
}

void controllerNodeThread(){
#ifndef IHM_DEBUG
    string path = QDir::currentPath().toStdString();
    string command = "cd " + path + " && " + "pc_controller_node.exe";
    system(command.c_str());
#endif
}

void intermediaryNodeThread(){
#ifndef IHM_DEBUG
    string path = QDir::currentPath().toStdString();
    string command = "cd " + path + " && " + "intermediary_node.exe";
    system(command.c_str());
#endif
}

void velocityNodeThread(){
#ifndef IHM_DEBUG
    string path = QDir::currentPath().toStdString();
    string command = "cd " + path + " && " + "vel_node.exe";
    system(command.c_str());
#endif
}

bool MainWindow::runControllerNode(QString *msg)
{
    return launchNode("pc_controller_node.exe", msg, v_pcControllerNodeStatus, controllerNodeThread);
}

bool MainWindow::runIntermediaryNode(QString *msg)
{
    return launchNode("intermediary_node.exe", msg, v_intermediaryNodeStatus, intermediaryNodeThread);
}

bool MainWindow::runVelocityNode(QString *msg)
{
    return launchNode("vel_node.exe", msg, v_velNodeStatus, velocityNodeThread);
}

void MainWindow::runAllNodes()
{
    if(v_rosMasterNodeStatus){
        if(!v_pcControllerNodeStatus) runControllerNode();
        if(!v_intermediaryNodeStatus) runIntermediaryNode();
        if(!v_velNodeStatus) runVelocityNode();
    }
}

void MainWindow::stopControllerNode()
{
    stopNode("pc_controller_node.exe", v_pcControllerNodeStatus);
}

void MainWindow::stopIntermediaryNode()
{
    stopNode("intermediary_node.exe", v_intermediaryNodeStatus);
}

void MainWindow::stopVelocityNode()
{
    stopNode("vel_node.exe", v_velNodeStatus);
}

void MainWindow::stopRosMasterNode()
{
    if(v_rosMasterNodeStatus){
        stopNode("pc_controller_node.exe", v_pcControllerNodeStatus);
        stopNode("intermediary_node.exe", v_intermediaryNodeStatus);
        stopNode("vel_node.exe", v_velNodeStatus);
        stopNode("roscore.exe", v_rosMasterNodeStatus);
    }
}

void MainWindow::stopAllNodes()
{
    if(v_rosMasterNodeStatus){
        stopControllerNode();
        stopIntermediaryNode();
        stopVelocityNode();
    }
}

void MainWindow::updateNodesStatus(int node, bool status)
{
    QString text1 = "application en cours d'exécution";
    QString text2 = "application arrêtée";

    switch(node){
    case ProjectNode::ROS_MASTER:
    {
        if(status != v_rosMasterNodeStatus){
            v_rosMasterNodeStatus = status;

            ui->start_all_nodes->setEnabled(status);
            ui->stop_all_nodes->setEnabled(status);

            if(status) emit displayMsgSignal("rosmaster.exe : " + text1, GREEN_COLOR);
            else emit displayMsgSignal("rosmaster.exe : " + text2, RED_COLOR);
        }
        break;
    }

    case ProjectNode::PC_CONTROLLER:
    {
        if(status != v_pcControllerNodeStatus){
            v_pcControllerNodeStatus = status;

            if(status) emit displayMsgSignal("pc_controller_node.exe : " + text1, GREEN_COLOR);
            else emit displayMsgSignal("pc_controller_node.exe : " + text2, RED_COLOR);
        }
        break;
    }

    case ProjectNode::INTERMEDIARY:
    {
        if(status != v_intermediaryNodeStatus){
            v_intermediaryNodeStatus = status;

            if(status) emit displayMsgSignal("intermediary_node.exe : " + text1, GREEN_COLOR);
            else emit displayMsgSignal("intermediary_node.exe : " + text2, RED_COLOR);
        }
        break;
    }

    case ProjectNode::VELOCITY:
    {
        if(status != v_velNodeStatus){
            v_velNodeStatus = status;

            if(status) emit displayMsgSignal("vel_node.exe : " + text1, GREEN_COLOR);
            else emit displayMsgSignal("vel_node.exe : " + text2, RED_COLOR);
        }
        break;
    }
    }
}

void MainWindow::calibrationImage()
{
    cv::Mat image = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    int camPosition = -1;
    string imageName = ui->calib_img_file_name->text().toStdString();
    string extension = ".jpg";
    string absolutePath;
    AcquisitionThread *tempAcq = nullptr;

    if(v_countImage < ui->calib_number_of_img->value()){
        if(ui->cam_left->isChecked()){
            tempAcq = v_cam_G_acq;
            camPosition = Camera::GAUCHE;
        }
        else if(ui->cam_right->isChecked()){
            tempAcq = v_cam_D_acq;
            camPosition = Camera::DROITE;
        }
        else if(ui->cam_middle->isChecked()){
            tempAcq = v_cam_M_acq;
            camPosition = Camera::MILIEU;
        }

        absolutePath = QDir(QString::fromStdString(imageFiles4Calibration[camPosition])).absolutePath().toStdString();

        tempAcq->requestInterruption();
        tempAcq->wait();
        image = tempAcq->getFrame().clone();
        tempAcq->start();
        tempAcq = nullptr;

        cv::imwrite(absolutePath + "\\" + imageName + "_" + to_string(++v_countImage) + extension, image);

        ui->calib_img_progressbar->setValue((v_countImage*100)/ui->calib_number_of_img->value());
        if(v_countImage < ui->calib_number_of_img->value()) v_chronoImg->start();
    }

    if(v_countImage == ui->calib_number_of_img->value()){
        disconnect(v_chronoImg, SIGNAL(chrono(int)), this, SLOT(updateChrono(int)));

        ui->start_chronoBtn->setText("GO");
        ui->start_chronoBtn->setStyleSheet(QString::fromStdString(v_pushButtonStyle_3));

        if(!ui->cameras->isEnabled()) ui->cameras->setEnabled(true);

        ui->calib_img_progressbar->setValue(0);

        bool enable = true;
        ui->calib_delay_img->setEnabled(enable);
        ui->calib_number_of_img->setEnabled(enable);
        ui->calib_img_file_name->setEnabled(enable);
    }
}

void MainWindow::setToolTip()
{
    ui->robot_rot_ccw->setToolTip("Sens anti horaire");
    ui->robot_rot_cw->setToolTip("Sens horaire");
    ui->robot_right->setToolTip("Droite");
    ui->robot_left->setToolTip("Gauche");
    ui->robot_bw->setToolTip("Reculer");
    ui->robot_fw->setToolTip("Avancer");
}

void MainWindow::configWidgets()
{
    cv::Mat frame_init;
    QPixmap qPix_init;
    QFont font = QFont("MS Shell Dlg 2", static_cast<int>(v_windowHeight * 0.0097));

    ui->dispWA->setGeometry(static_cast<int>(v_windowWidth * 0.504),
                            static_cast<int>(v_windowHeight * 0.01205),
                            static_cast<int>(v_windowWidth * scale * 1.96),
                            static_cast<int>(v_windowHeight * scale * 3.494));

    std::cout << "\nwindow width = " << v_windowWidth << std::endl;
    std::cout << "window height = " << v_windowHeight << std::endl;
    std::cout << "window width max = " << v_maxWinW << std::endl;
    std::cout << "window height max = " << v_maxWinH << std::endl;
    std::cout << "alpha = " << static_cast<double>(v_windowWidth) / v_windowHeight << std::endl << std::endl;

    frame_init = cv::Mat::zeros(static_cast<int>(2900 * scale),
                                static_cast<int>(2775 * scale),
                                CV_8UC3);
    cv::resize(frame_init, frame_init, cv::Size(), v_resizeWidthImage, v_resizeHeightImage);
    qPix_init = Mat2Pixmap(frame_init);
    ui->dispWA->setPixmap(qPix_init);

    //Initialisation de la zone de visualisation du flux des caméras
    ui->camView->setGeometry(static_cast<int>(v_windowWidth * 0.546),
                             static_cast<int>(v_windowHeight * 0.01205),
                             static_cast<int>(v_windowWidth * 0.339),
                             static_cast<int>(v_windowHeight * 0.3254));



    frame_init.release();
    frame_init = cv::Mat::zeros(static_cast<int>(IMG_H/2),
                                static_cast<int>(IMG_W/2),
                                CV_8UC3);
    cv::resize(frame_init, frame_init, cv::Size(), v_fx, v_fy);
    qPix_init = Mat2Pixmap(frame_init);
    ui->camView->setPixmap(qPix_init);

    ui->robot_free_positionning->setStyleSheet(groupBoxStyle_11);
    //GROUPBOX robot_free_positionning
    ui->robot_free_positionning->setGeometry(static_cast<int>(v_windowWidth * 0.579100),
                                             static_cast<int>(v_windowHeight * 0.813300),
                                             static_cast<int>(v_windowWidth * 0.289600),
                                             static_cast<int>(v_windowHeight * 0.096400));
    ui->robot_free_positionning->setFont(font);

    ui->robot_free_pos_without_measure->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                                    static_cast<int>(v_windowHeight * 0.030200),
                                                    static_cast<int>(v_windowWidth * 0.077700),
                                                    static_cast<int>(v_windowHeight * 0.025400));
    ui->robot_free_pos_without_measure->setFont(font);

    ui->robot_free_pos_with_measure->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                                 static_cast<int>(v_windowHeight * 0.060300),
                                                 static_cast<int>(v_windowWidth * 0.077700),
                                                 static_cast<int>(v_windowHeight * 0.025400));
    ui->robot_free_pos_with_measure->setFont(font);

    ui->robot_free_pos_x_label->setGeometry(static_cast<int>(v_windowWidth * 0.127200),
                                            static_cast<int>(v_windowHeight * 0.030200),
                                            static_cast<int>(v_windowWidth * 0.014200),
                                            static_cast<int>(v_windowHeight * 0.025400));
    ui->robot_free_pos_x_label->setFont(font);

    ui->robot_free_pos_x->setGeometry(static_cast<int>(v_windowWidth * 0.106000),
                                      static_cast<int>(v_windowHeight * 0.054300),
                                      static_cast<int>(v_windowWidth * 0.064300),
                                      static_cast<int>(v_windowHeight * 0.025400));
    ui->robot_free_pos_x->setFont(font);

    ui->robot_free_pos_y_label->setGeometry(static_cast<int>(v_windowWidth * 0.197800),
                                            static_cast<int>(v_windowHeight * 0.030200),
                                            static_cast<int>(v_windowWidth * 0.014200),
                                            static_cast<int>(v_windowHeight * 0.025400));
    ui->robot_free_pos_y_label->setFont(font);

    ui->robot_free_pos_y->setGeometry(static_cast<int>(v_windowWidth * 0.176600),
                                      static_cast<int>(v_windowHeight * 0.054300),
                                      static_cast<int>(v_windowWidth * 0.064300),
                                      static_cast<int>(v_windowHeight * 0.025400));
    ui->robot_free_pos_y->setFont(font);

    ui->robot_free_pos_go->setGeometry(static_cast<int>(v_windowWidth * 0.247200),
                                       static_cast<int>(v_windowHeight * 0.050700),
                                       static_cast<int>(v_windowWidth * 0.035400),
                                       static_cast<int>(v_windowHeight * 0.030200));
    ui->robot_free_pos_go->setFont(font);
    ui->robot_free_pos_go->setStyleSheet(pushButtonDesabledStyle);
    //FIN GROUPBOX robot_free_positionning

    //GROUPBOX calibration_test_groupbox
    ui->calibration_test_groupbox->setGeometry( static_cast<int>(v_windowWidth * 0.546),
                                                static_cast<int>(v_windowHeight * 0.35),
                                                static_cast<int>(v_windowWidth * 0.339),
                                                static_cast<int>(v_windowHeight * 0.181));
    ui->calibration_test_groupbox->setFont(font);

    ui->calib_img_delay_label->setGeometry(static_cast<int>(v_windowWidth * 0.014200),
                                           static_cast<int>(v_windowHeight * 0.036200),
                                           static_cast<int>(v_windowWidth * 0.056500),
                                           static_cast<int>(v_windowHeight * 0.025400));
    ui->calib_img_delay_label->setFont(font);

    ui->calib_numb_of_img_label->setGeometry(static_cast<int>(v_windowWidth * 0.014200),
                                             static_cast<int>(v_windowHeight * 0.072300),
                                             static_cast<int>(v_windowWidth * 0.056500),
                                             static_cast<int>(v_windowHeight * 0.025400));
    ui->calib_numb_of_img_label->setFont(font);

    ui->calib_file_name_label->setGeometry(static_cast<int>(v_windowWidth * 0.014200),
                                           static_cast<int>(v_windowHeight * 0.108500),
                                           static_cast<int>(v_windowWidth * 0.063600),
                                           static_cast<int>(v_windowHeight * 0.025400));
    ui->calib_file_name_label->setFont(font);

    ui->calib_delay_img->setGeometry(static_cast<int>(v_windowWidth * 0.091900),
                                     static_cast<int>(v_windowHeight * 0.036200),
                                     static_cast<int>(v_windowWidth * 0.064300),
                                     static_cast<int>(v_windowHeight * 0.025400));
    ui->calib_delay_img->setFont(font);

    ui->calib_number_of_img->setGeometry(static_cast<int>(v_windowWidth * 0.091900),
                                         static_cast<int>(v_windowHeight * 0.072300),
                                         static_cast<int>(v_windowWidth * 0.064300),
                                         static_cast<int>(v_windowHeight * 0.025400));
    ui->calib_number_of_img->setFont(font);

    ui->calib_img_file_name->setGeometry(static_cast<int>(v_windowWidth * 0.091900),
                                         static_cast<int>(v_windowHeight * 0.108500),
                                         static_cast<int>(v_windowWidth * 0.084800),
                                         static_cast<int>(v_windowHeight * 0.025400));
    ui->calib_img_file_name->setFont(font);

    ui->calib_img_progressbar->setGeometry(static_cast<int>(v_windowWidth * 0.091900),
                                          static_cast<int>(v_windowHeight * 0.144600),
                                          static_cast<int>(v_windowWidth * 0.084800),
                                          static_cast<int>(v_windowHeight * 0.025400));
    ui->calib_img_progressbar->setFont(font);

    ui->start_chronoBtn->setGeometry(static_cast<int>(v_windowWidth * 0.219000),
                                     static_cast<int>(v_windowHeight * 0.024100),
                                     static_cast<int>(v_windowWidth * 0.084800),
                                     static_cast<int>(v_windowHeight * 0.144600));
    ui->start_chronoBtn->setFont(QFont("MS Shell Dlg 2", static_cast<int>(v_windowHeight * 0.0362)));

    v_pushButtonStyle_2 = "QPushButton "
                        "{ background-color: rgb(0, 255, 0);"
                        "border-style: outset;";
    v_pushButtonStyle_2 += "border-radius: " + to_string(ui->start_chronoBtn->geometry().width()/2) + "px;"
                         "border-width: " + to_string(static_cast<int>(v_windowWidth * 0.0071)) + "px;"
                         "border-color: rgb(0, 150, 0); }"
                         "QPushButton:pressed "
                         "{ background-color: rgb(0, 255, 0);"
                         "border-style: inset;"
                         "border-color: rgb(0, 255, 0); }"
                         "QPushButton:hover:!pressed "
                         "{ background-color: rgb(0, 150, 0); "
                         "border-color: rgb(0, 255, 0); }";

    v_pushButtonStyle_3 = "QPushButton "
                        "{ background-color: rgb(150, 150, 150);"
                        "border-style: outset;";

    v_pushButtonStyle_3 += "border-radius: " + to_string(ui->start_chronoBtn->geometry().width()/2) + "px;"
                        "border-width: " + to_string(static_cast<int>(v_windowWidth * 0.0071)) + "px;"
                        "border-color: rgb(150, 150, 150);"
                        "color: rgb(0, 0, 0); }"
                        "QPushButton:pressed "
                        "{ background-color: rgb(150, 150, 150);"
                        "border-style: inset;"
                        "border-color: rgb(0, 255, 0); }"
                        "QPushButton:hover:!pressed "
                        "{ background-color: rgb(215, 215, 215); "
                        "border-color: rgb(150, 150, 150); }";

    v_pushButtonStyle_4 = "QPushButton "
                        "{ background-color: rgb(150, 150, 150);"
                        "border-style: outset;";

    v_pushButtonStyle_4 += "border-radius: " + to_string(ui->start_chronoBtn->geometry().width()/2) + "px;"
                        "border-width: " + to_string(static_cast<int>(v_windowWidth * 0.0071)) + "px;"
                        "border-color: rgb(150, 150, 150);"
                        "color: rgb(100, 100, 100); }"
                        "QPushButton:pressed "
                        "{ background-color: rgb(150, 150, 150);"
                        "border-style: inset;"
                        "border-color: rgb(0, 255, 0); }"
                        "QPushButton:hover:!pressed "
                        "{ background-color: rgb(215, 215, 215); "
                        "border-color: rgb(150, 150, 150); }";
    //FIN GROUPBOX calibration_test_groupbox

    //GROUPBOX IPAdress
    ui->IPAdress->setGeometry(static_cast<int>(v_windowWidth * 0.0212),
                              static_cast<int>(v_windowHeight * 0.0121),
                              static_cast<int>(v_windowWidth * 0.2267),
                              static_cast<int>(v_windowHeight * 0.0976));
    ui->IPAdress->setFont(font);

    ui->ip_pc_label->setGeometry(static_cast<int>(v_windowWidth * 0.0071),
                                 static_cast<int>(v_windowHeight * 0.0241),
                                 static_cast<int>(v_windowWidth * 0.039),
                                 static_cast<int>(v_windowHeight * 0.026));
    ui->ip_pc_label->setFont(font);

    ui->ip_robotino_label->setGeometry(static_cast<int>(v_windowWidth * 0.0071),
                                 static_cast<int>(v_windowHeight * 0.061),
                                 static_cast<int>(v_windowWidth * 0.039),
                                 static_cast<int>(v_windowHeight * 0.026));
    ui->ip_robotino_label->setFont(font);

    ui->IP_PC->setGeometry(static_cast<int>(v_windowWidth * 0.05),
                           static_cast<int>(v_windowHeight * 0.0241),
                           static_cast<int>(v_windowWidth * 0.079),
                           static_cast<int>(v_windowHeight * 0.026));
    ui->IP_PC->setFont(font);

    ui->IP_Robotino->setGeometry(static_cast<int>(v_windowWidth * 0.05),
                                 static_cast<int>(v_windowHeight * 0.061),
                                 static_cast<int>(v_windowWidth * 0.079),
                                 static_cast<int>(v_windowHeight * 0.026));
    ui->IP_Robotino->setFont(font);

    ui->IPButton->setGeometry(static_cast<int>(v_windowWidth * 0.15),
                              static_cast<int>(v_windowHeight * 0.039),
                              static_cast<int>(v_windowWidth * 0.066),
                              static_cast<int>(v_windowHeight * 0.031));
    ui->IPButton->setFont(font);
    //FIN GROUPBOX IPAdress

    //GROUPBOX IDs_cams
    ui->IDs_cams->setGeometry(static_cast<int>(v_windowWidth * 0.255),
                              static_cast<int>(v_windowHeight * 0.0121),
                              static_cast<int>(v_windowWidth * 0.185),
                              static_cast<int>(v_windowHeight * 0.097));
    ui->IDs_cams->setFont(font);

    ui->cam_id_left_label->setGeometry(static_cast<int>(0),
                                       static_cast<int>(v_windowHeight * 0.031),
                                       static_cast<int>(v_windowWidth * 0.061),
                                       static_cast<int>(v_windowHeight * 0.0254));
    ui->cam_id_left_label->setFont(font);

    ui->cam_id_middle_label->setGeometry(static_cast<int>(v_windowWidth * 0.0565),
                                         static_cast<int>(v_windowHeight * 0.031),
                                         static_cast<int>(v_windowWidth * 0.061),
                                         static_cast<int>(v_windowHeight * 0.0254));
    ui->cam_id_middle_label->setFont(font);

    ui->cam_id_right_label->setGeometry(static_cast<int>(v_windowWidth * 0.121),
                                        static_cast<int>(v_windowHeight * 0.031),
                                        static_cast<int>(v_windowWidth * 0.061),
                                        static_cast<int>(v_windowHeight * 0.0254));
    ui->cam_id_right_label->setFont(font);

    ui->ID_cam_g->setGeometry(static_cast<int>(v_windowWidth * 0.0043),
                              static_cast<int>(v_windowHeight * 0.061),
                              static_cast<int>(v_windowWidth * 0.05),
                              static_cast<int>(v_windowHeight * 0.0254));
    ui->ID_cam_g->setFont(font);

    ui->ID_cam_m->setGeometry(static_cast<int>(v_windowWidth * 0.068),
                              static_cast<int>(v_windowHeight * 0.061),
                              static_cast<int>(v_windowWidth * 0.05),
                              static_cast<int>(v_windowHeight * 0.0254));
    ui->ID_cam_m->setFont(font);

    ui->ID_cam_d->setGeometry(static_cast<int>(v_windowWidth * 0.132),
                              static_cast<int>(v_windowHeight * 0.061),
                              static_cast<int>(v_windowWidth * 0.05),
                              static_cast<int>(v_windowHeight * 0.0254));
    ui->ID_cam_d->setFont(font);

    //GROUPBOX config_robot
    ui->config_robot->setGeometry(static_cast<int>(v_windowWidth * 0.0212),
                                  static_cast<int>(v_windowHeight * 0.1205),
                                  static_cast<int>(v_windowWidth * 0.2267),
                                  static_cast<int>(v_windowHeight * 0.2181));
    ui->config_robot->setFont(font);

    ui->diameter_label->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                    static_cast<int>(v_windowHeight * 0.024100),
                                    static_cast<int>(v_windowWidth * 0.063600),
                                    static_cast<int>(v_windowHeight * 0.025400));
    ui->diameter_label->setFont(font);

    ui->tool_label->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                static_cast<int>(v_windowHeight * 0.072300),
                                static_cast<int>(v_windowWidth * 0.070700),
                                static_cast<int>(v_windowHeight * 0.025400));
    ui->tool_label->setFont(font);

    ui->height_led_label->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                      static_cast<int>(v_windowHeight * 0.108500),
                                      static_cast<int>(v_windowWidth * 0.070700),
                                      static_cast<int>(v_windowHeight * 0.025400));
    ui->height_led_label->setFont(font);

    ui->lin_vel_max_label->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                       static_cast<int>(v_windowHeight * 0.144600),
                                       static_cast<int>(v_windowWidth * 0.070700),
                                       static_cast<int>(v_windowHeight * 0.025400));
    ui->lin_vel_max_label->setFont(font);

    ui->ang_vel_min_label->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                       static_cast<int>(v_windowHeight * 0.180800),
                                       static_cast<int>(v_windowWidth * 0.070700),
                                       static_cast<int>(v_windowHeight * 0.025400));
    ui->ang_vel_min_label->setFont(font);

    ui->tool_length_label->setGeometry(static_cast<int>(v_windowWidth * 0.084800),
                                       static_cast<int>(v_windowHeight * 0.048200),
                                       static_cast<int>(v_windowWidth * 0.070700),
                                       static_cast<int>(v_windowHeight * 0.025400));
    ui->tool_length_label->setFont(font);

    ui->tool_width_label->setGeometry(static_cast<int>(v_windowWidth * 0.155400),
                                      static_cast<int>(v_windowHeight * 0.048200),
                                      static_cast<int>(v_windowWidth * 0.070700),
                                      static_cast<int>(v_windowHeight * 0.025400));
    ui->tool_width_label->setFont(font);

    ui->diameter->setGeometry(static_cast<int>(v_windowWidth * 0.084800),
                              static_cast<int>(v_windowHeight * 0.024100),
                              static_cast<int>(v_windowWidth * 0.064300),
                              static_cast<int>(v_windowHeight * 0.025400));
    ui->diameter->setFont(font);

    ui->toolLength->setGeometry(static_cast<int>(v_windowWidth * 0.084800),
                                static_cast<int>(v_windowHeight * 0.072300),
                                static_cast<int>(v_windowWidth * 0.064300),
                                static_cast<int>(v_windowHeight * 0.025400));
    ui->toolLength->setFont(font);

    ui->ledHigh->setGeometry(static_cast<int>(v_windowWidth * 0.084800),
                             static_cast<int>(v_windowHeight * 0.108500),
                             static_cast<int>(v_windowWidth * 0.064300),
                             static_cast<int>(v_windowHeight * 0.025400));
    ui->ledHigh->setFont(font);

    ui->linearVelMax->setGeometry(static_cast<int>(v_windowWidth * 0.084800),
                                  static_cast<int>(v_windowHeight * 0.144600),
                                  static_cast<int>(v_windowWidth * 0.064300),
                                  static_cast<int>(v_windowHeight * 0.025400));
    ui->linearVelMax->setFont(font);

    ui->angularVelMin->setGeometry(static_cast<int>(v_windowWidth * 0.084800),
                                   static_cast<int>(v_windowHeight * 0.180800),
                                   static_cast<int>(v_windowWidth * 0.064300),
                                   static_cast<int>(v_windowHeight * 0.025400));
    ui->angularVelMin->setFont(font);

    ui->toolWidth->setGeometry(static_cast<int>(v_windowWidth * 0.155400),
                               static_cast<int>(v_windowHeight * 0.072300),
                               static_cast<int>(v_windowWidth * 0.064300),
                               static_cast<int>(v_windowHeight * 0.025400));
    ui->toolWidth->setFont(font);
    //FIN GROUPBOX config_robot

    //GROUPBOX config_motor_mast
    ui->config_motor_mast->setGeometry(static_cast<int>(v_windowWidth * 0.254300),
                                       static_cast<int>(v_windowHeight * 0.120500),
                                       static_cast<int>(v_windowWidth * 0.184400),
                                       static_cast<int>(v_windowHeight * 0.218100));
    ui->config_motor_mast->setFont(font);

    ui->pulley_radius_label->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                         static_cast<int>(v_windowHeight * 0.048200),
                                         static_cast<int>(v_windowWidth * 0.106700),
                                         static_cast<int>(v_windowHeight * 0.025400));
    ui->pulley_radius_label->setFont(font);

    ui->motor_accuracy_label->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                          static_cast<int>(v_windowHeight * 0.084400),
                                          static_cast<int>(v_windowWidth * 0.106700),
                                          static_cast<int>(v_windowHeight * 0.025400));
    ui->motor_accuracy_label->setFont(font);

    ui->motor_mast_lin_vel_label->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                              static_cast<int>(v_windowHeight * 0.120500),
                                              static_cast<int>(v_windowWidth * 0.099600),
                                              static_cast<int>(v_windowHeight * 0.025400));
    ui->motor_mast_lin_vel_label->setFont(font);

    ui->duty_cycle_label->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                      static_cast<int>(v_windowHeight * 0.156700),
                                      static_cast<int>(v_windowWidth * 0.092600),
                                      static_cast<int>(v_windowHeight * 0.025400));
    ui->duty_cycle_label->setFont(font);

    ui->pulley_radius->setGeometry(static_cast<int>(v_windowWidth * 0.113000),
                                   static_cast<int>(v_windowHeight * 0.048200),
                                   static_cast<int>(v_windowWidth * 0.064300),
                                   static_cast<int>(v_windowHeight * 0.025400));
    ui->pulley_radius->setFont(font);

    ui->motor_precision->setGeometry(static_cast<int>(v_windowWidth * 0.113000),
                                     static_cast<int>(v_windowHeight * 0.084400),
                                     static_cast<int>(v_windowWidth * 0.064300),
                                     static_cast<int>(v_windowHeight * 0.025400));
    ui->motor_precision->setFont(font);

    ui->lin_vel_motor_mast->setGeometry(static_cast<int>(v_windowWidth * 0.113000),
                                        static_cast<int>(v_windowHeight * 0.120500),
                                        static_cast<int>(v_windowWidth * 0.064300),
                                        static_cast<int>(v_windowHeight * 0.025400));
    ui->lin_vel_motor_mast->setFont(font);

    ui->duty_cycle->setGeometry(static_cast<int>(v_windowWidth * 0.113000),
                                static_cast<int>(v_windowHeight * 0.156700),
                                static_cast<int>(v_windowWidth * 0.064300),
                                static_cast<int>(v_windowHeight * 0.025400));
    ui->duty_cycle->setFont(font);
    //FIN GROUPBOX config_motor_mast

    //GROUPBOX delay
    ui->delay->setGeometry(static_cast<int>(v_windowWidth * 0.021200),
                           static_cast<int>(v_windowHeight * 0.349400),
                           static_cast<int>(v_windowWidth * 0.219700),
                           static_cast<int>(v_windowHeight * 0.097600));
    ui->delay->setFont(font);

    ui->delay_measure_label->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                         static_cast<int>(v_windowHeight * 0.024100),
                                         static_cast<int>(v_windowWidth * 0.064300),
                                         static_cast<int>(v_windowHeight * 0.025400));
    ui->delay_measure_label->setFont(font);

    ui->delay_period_label->setGeometry(static_cast<int>(v_windowWidth * 0.077700),
                                        static_cast<int>(v_windowHeight * 0.024100),
                                        static_cast<int>(v_windowWidth * 0.064300),
                                        static_cast<int>(v_windowHeight * 0.025400));
    ui->delay_period_label->setFont(font);

    ui->delay_rest_label->setGeometry(static_cast<int>(v_windowWidth * 0.148400),
                                      static_cast<int>(v_windowHeight * 0.024100),
                                      static_cast<int>(v_windowWidth * 0.064300),
                                      static_cast<int>(v_windowHeight * 0.025400));
    ui->delay_rest_label->setFont(font);

    ui->measure_timer->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                   static_cast<int>(v_windowHeight * 0.060300),
                                   static_cast<int>(v_windowWidth * 0.064300),
                                   static_cast<int>(v_windowHeight * 0.025400));
    ui->measure_timer->setFont(font);

    ui->period->setGeometry(static_cast<int>(v_windowWidth * 0.077700),
                            static_cast<int>(v_windowHeight * 0.060300),
                            static_cast<int>(v_windowWidth * 0.064300),
                            static_cast<int>(v_windowHeight * 0.025400));
    ui->period->setFont(font);

    ui->rest->setGeometry(static_cast<int>(v_windowWidth * 0.148400),
                          static_cast<int>(v_windowHeight * 0.060300),
                          static_cast<int>(v_windowWidth * 0.064300),
                          static_cast<int>(v_windowHeight * 0.025400));
    ui->rest->setFont(font);
    //FIN GROUPBOX delay

    //GROUPBOX measurementPoints
    ui->measurementPoints->setGeometry(static_cast<int>(v_windowWidth * 0.247200),
                                       static_cast<int>(v_windowHeight * 0.349400),
                                       static_cast<int>(v_windowWidth * 0.092600),
                                       static_cast<int>(v_windowHeight * 0.097600));
    ui->measurementPoints->setFont(font);

    ui->dx_label->setGeometry(static_cast<int>(v_windowWidth * 0.000100),
                              static_cast<int>(v_windowHeight * 0.024100),
                              static_cast<int>(v_windowWidth * 0.021200),
                              static_cast<int>(v_windowHeight * 0.025400));
    ui->dx_label->setFont(font);

    ui->dy_label->setGeometry(static_cast<int>(v_windowWidth * 0.000100),
                              static_cast<int>(v_windowHeight * 0.060300),
                              static_cast<int>(v_windowWidth * 0.021200),
                              static_cast<int>(v_windowHeight * 0.025400));
    ui->dy_label->setFont(font);

    ui->dx->setGeometry(static_cast<int>(v_windowWidth * 0.021200),
                        static_cast<int>(v_windowHeight * 0.024100),
                        static_cast<int>(v_windowWidth * 0.064300),
                        static_cast<int>(v_windowHeight * 0.025400));
    ui->dx->setFont(font);

    ui->dy->setGeometry(static_cast<int>(v_windowWidth * 0.021200),
                        static_cast<int>(v_windowHeight * 0.060300),
                        static_cast<int>(v_windowWidth * 0.064300),
                        static_cast<int>(v_windowHeight * 0.025400));
    ui->dy->setFont(font);
    //FIN GROUPBOX measurementPoints

    //GROUPBOX workingArea
    ui->workingArea->setGeometry(static_cast<int>(v_windowWidth * 0.346100),
                                 static_cast<int>(v_windowHeight * 0.349400),
                                 static_cast<int>(v_windowWidth * 0.092600),
                                 static_cast<int>(v_windowHeight * 0.097600));
    ui->workingArea->setFont(font);

    ui->wa_x_label->setGeometry(static_cast<int>(v_windowWidth * 0.000100),
                                static_cast<int>(v_windowHeight * 0.024100),
                                static_cast<int>(v_windowWidth * 0.021900),
                                static_cast<int>(v_windowHeight * 0.019300));
    ui->wa_x_label->setFont(font);

    ui->wa_y_label->setGeometry(static_cast<int>(v_windowWidth * 0.000100),
                                static_cast<int>(v_windowHeight * 0.060300),
                                static_cast<int>(v_windowWidth * 0.021900),
                                static_cast<int>(v_windowHeight * 0.019300));
    ui->wa_y_label->setFont(font);

    ui->waWidth->setGeometry(static_cast<int>(v_windowWidth * 0.021200),
                             static_cast<int>(v_windowHeight * 0.024100),
                             static_cast<int>(v_windowWidth * 0.064300),
                             static_cast<int>(v_windowHeight * 0.025400));
    ui->waWidth->setFont(font);

    ui->waLength->setGeometry(static_cast<int>(v_windowWidth * 0.021200),
                              static_cast<int>(v_windowHeight * 0.060300),
                              static_cast<int>(v_windowWidth * 0.064300),
                              static_cast<int>(v_windowHeight * 0.025400));
    ui->waLength->setFont(font);
    //FIN GROUPBOX workingArea

    //GROUPBOX highToAchieve
    ui->highToAchieve->setGeometry(static_cast<int>(v_windowWidth * 0.021200),
                                   static_cast<int>(v_windowHeight * 0.457900),
                                   static_cast<int>(v_windowWidth * 0.417400),
                                   static_cast<int>(v_windowHeight * 0.109700));
    ui->highToAchieve->setFont(font);

    ui->radioSavedHigh->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                    static_cast<int>(v_windowHeight * 0.036200),
                                    static_cast<int>(v_windowWidth * 0.113800),
                                    static_cast<int>(v_windowHeight * 0.025400));
    ui->radioSavedHigh->setFont(font);

    ui->radioOtherHigh->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                    static_cast<int>(v_windowHeight * 0.072300),
                                    static_cast<int>(v_windowWidth * 0.113800),
                                    static_cast<int>(v_windowHeight * 0.025400));
    ui->radioOtherHigh->setFont(font);

    ui->savedHigh->setGeometry(static_cast<int>(v_windowWidth * 0.120100),
                               static_cast<int>(v_windowHeight * 0.036200),
                               static_cast<int>(v_windowWidth * 0.198500),
                               static_cast<int>(v_windowHeight * 0.025400));
    ui->savedHigh->setFont(font);

    ui->otherHigh->setGeometry(static_cast<int>(v_windowWidth * 0.120100),
                               static_cast<int>(v_windowHeight * 0.072300),
                               static_cast<int>(v_windowWidth * 0.198500),
                               static_cast<int>(v_windowHeight * 0.025400));
    ui->otherHigh->setFont(font);

    ui->highButton->setGeometry(static_cast<int>(v_windowWidth * 0.332000),
                                static_cast<int>(v_windowHeight * 0.048200),
                                static_cast<int>(v_windowWidth * 0.065700),
                                static_cast<int>(v_windowHeight * 0.030200));
    ui->highButton->setFont(font);
    //FIN GROUPBOX highToAchieve

    //GROUPBOX manualControl
    ui->manualControl->setGeometry(static_cast<int>(v_windowWidth * 0.021200),
                                   static_cast<int>(v_windowHeight * 0.578400),
                                   static_cast<int>(v_windowWidth * 0.205600),
                                   static_cast<int>(v_windowHeight * 0.073500));
    ui->manualControl->setFont(font);

    ui->manualControl_checkbox->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                            static_cast<int>(v_windowHeight * 0.024100),
                                            static_cast<int>(v_windowWidth * 0.149100),
                                            static_cast<int>(v_windowHeight * 0.025400));
    ui->manualControl_checkbox->setFont(font);
    //FIN GROUPBOX manualControl

    //GROUPBOX calibration
    ui->calibration->setGeometry(static_cast<int>(v_windowWidth * 0.233100),
                                 static_cast<int>(v_windowHeight * 0.578400),
                                 static_cast<int>(v_windowWidth * 0.205600),
                                 static_cast<int>(v_windowHeight * 0.073500));
    ui->calibration->setFont(font);

    ui->calibration_checkbox->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                          static_cast<int>(v_windowHeight * 0.024100),
                                          static_cast<int>(v_windowWidth * 0.120800),
                                          static_cast<int>(v_windowHeight * 0.025400));
    ui->calibration_checkbox->setFont(font);
    //FIN GROUPBOX calibration

    //GROUPBOX advancement
    ui->advancement->setGeometry(static_cast<int>(v_windowWidth * 0.021200),
                                 static_cast<int>(v_windowHeight * 0.662700),
                                 static_cast<int>(v_windowWidth * 0.417400),
                                 static_cast<int>(v_windowHeight * 0.062700));
    ui->advancement->setFont(font);

    ui->processed_pts_label->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                         static_cast<int>(v_windowHeight * 0.030200),
                                         static_cast<int>(v_windowWidth * 0.064300),
                                         static_cast<int>(v_windowHeight * 0.025400));
    ui->processed_pts_label->setFont(font);

    ui->processedPoints->setGeometry(static_cast<int>(v_windowWidth * 0.077700),
                                     static_cast<int>(v_windowHeight * 0.030200),
                                     static_cast<int>(v_windowWidth * 0.028300),
                                     static_cast<int>(v_windowHeight * 0.025400));
    ui->processedPoints->setFont(font);

    ui->label_1->setGeometry(static_cast<int>(v_windowWidth * 0.113000),
                             static_cast<int>(v_windowHeight * 0.030200),
                             static_cast<int>(v_windowWidth * 0.021900),
                             static_cast<int>(v_windowHeight * 0.025400));
    ui->label_1->setFont(font);

    ui->numberOfPoints->setGeometry(static_cast<int>(v_windowWidth * 0.141300),
                                    static_cast<int>(v_windowHeight * 0.030200),
                                    static_cast<int>(v_windowWidth * 0.028300),
                                    static_cast<int>(v_windowHeight * 0.025400));
    ui->numberOfPoints->setFont(font);

    ui->processed_measures_label->setGeometry(static_cast<int>(v_windowWidth * 0.219000),
                                              static_cast<int>(v_windowHeight * 0.030200),
                                              static_cast<int>(v_windowWidth * 0.092600),
                                              static_cast<int>(v_windowHeight * 0.025400));
    ui->processed_measures_label->setFont(font);

    ui->processedMeasures->setGeometry(static_cast<int>(v_windowWidth * 0.317800),
                                       static_cast<int>(v_windowHeight * 0.030200),
                                       static_cast<int>(v_windowWidth * 0.028300),
                                       static_cast<int>(v_windowHeight * 0.025400));
    ui->processedMeasures->setFont(font);

    ui->label_3->setGeometry(static_cast<int>(v_windowWidth * 0.353200),
                             static_cast<int>(v_windowHeight * 0.030200),
                             static_cast<int>(v_windowWidth * 0.021900),
                             static_cast<int>(v_windowHeight * 0.025400));
    ui->label_3->setFont(font);

    ui->numberOfMeasures->setGeometry(static_cast<int>(v_windowWidth * 0.381400),
                                      static_cast<int>(v_windowHeight * 0.030200),
                                      static_cast<int>(v_windowWidth * 0.028300),
                                      static_cast<int>(v_windowHeight * 0.025400));
    ui->numberOfMeasures->setFont(font);
    //FIN GROUPBOX advancement

    //GROUPBOX log
    ui->log->setGeometry(static_cast<int>(v_windowWidth * 0.021200),
                         static_cast<int>(v_windowHeight * 0.735000),
                         static_cast<int>(v_windowWidth * 0.417400),
                         static_cast<int>(v_windowHeight * 0.157900));
    ui->log->setFont(font);

    ui->message->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                             static_cast<int>(v_windowHeight * 0.024100),
                             static_cast<int>(v_windowWidth * 0.346800),
                             static_cast<int>(v_windowHeight * 0.126600));
    ui->message->setFont(font);

    ui->clean_msg_window->setGeometry(static_cast<int>(v_windowWidth * 0.358800),
                                      static_cast<int>(v_windowHeight * 0.072300),
                                      static_cast<int>(v_windowWidth * 0.053000),
                                      static_cast<int>(v_windowHeight * 0.030200));
    ui->clean_msg_window->setFont(font);
    //FIN GROUPBOX log

    //GROUPBOX cameras
    ui->cameras->setGeometry(static_cast<int>(v_windowWidth * 0.896900),
                             static_cast<int>(v_windowHeight * 0.108500),
                             static_cast<int>(v_windowWidth * 0.078400),
                             static_cast<int>(v_windowHeight * 0.121700));
    ui->cameras->setFont(font);

    ui->cam_left->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                              static_cast<int>(v_windowHeight * 0.024100),
                              static_cast<int>(v_windowWidth * 0.068600),
                              static_cast<int>(v_windowHeight * 0.025400));
    ui->cam_left->setFont(font);

    ui->cam_middle->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                static_cast<int>(v_windowHeight * 0.048200),
                                static_cast<int>(v_windowWidth * 0.068600),
                                static_cast<int>(v_windowHeight * 0.025400));
    ui->cam_middle->setFont(font);

    ui->cam_right->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                               static_cast<int>(v_windowHeight * 0.072300),
                               static_cast<int>(v_windowWidth * 0.068600),
                               static_cast<int>(v_windowHeight * 0.025400));
    ui->cam_right->setFont(font);
    //FIN GROUPBOX cameras

    //...
    ui->ros_master->setGeometry(static_cast<int>(v_windowWidth * 0.042400),
                                static_cast<int>(v_windowHeight * 0.903700),
                                static_cast<int>(v_windowWidth * 0.067100),
                                static_cast<int>(v_windowHeight * 0.030200));
    ui->ros_master->setFont(font);

    ui->ros_node->setGeometry(static_cast<int>(v_windowWidth * 0.120100),
                              static_cast<int>(v_windowHeight * 0.903700),
                              static_cast<int>(v_windowWidth * 0.095400),
                              static_cast<int>(v_windowHeight * 0.030200));
    ui->ros_node->setFont(font);

    ui->threshold->setGeometry(static_cast<int>(v_windowWidth * 0.226000),
                               static_cast<int>(v_windowHeight * 0.903700),
                               static_cast<int>(v_windowWidth * 0.053000),
                               static_cast<int>(v_windowHeight * 0.030200));
    ui->threshold->setFont(font);

    ui->camerasButton->setGeometry(static_cast<int>(v_windowWidth * 0.289600),
                                   static_cast<int>(v_windowHeight * 0.903700),
                                   static_cast<int>(v_windowWidth * 0.053000),
                                   static_cast<int>(v_windowHeight * 0.030200));
    ui->camerasButton->setFont(font);

    ui->gen_map->setGeometry(static_cast<int>(v_windowWidth * 0.586200),
                             static_cast<int>(v_windowHeight * 0.903700),
                             static_cast<int>(v_windowWidth * 0.060100),
                             static_cast<int>(v_windowHeight * 0.030200));
    ui->gen_map->setFont(font);

    ui->measure->setGeometry(static_cast<int>(v_windowWidth * 0.656800),
                             static_cast<int>(v_windowHeight * 0.903700),
                             static_cast<int>(v_windowWidth * 0.046000),
                             static_cast<int>(v_windowHeight * 0.030200));
    ui->measure->setFont(font);

    ui->stop->setGeometry(static_cast<int>(v_windowWidth * 0.713300),
                          static_cast<int>(v_windowHeight * 0.903700),
                          static_cast<int>(v_windowWidth * 0.046000),
                          static_cast<int>(v_windowHeight * 0.030200));
    ui->stop->setFont(font);

    ui->reuseTrajectory->setGeometry(static_cast<int>(v_windowWidth * 0.791000),
                                                       static_cast<int>(v_windowHeight * 0.903700),
                                                       static_cast<int>(v_windowWidth * 0.149100),
                                                       static_cast<int>(v_windowHeight * 0.030200));
    ui->reuseTrajectory->setFont(font);

    ui->frame_cameras->setGeometry( static_cast<int>(v_windowWidth * 0.546),
                                    static_cast<int>(v_windowHeight * 0.543),
                                    static_cast<int>(v_windowWidth * 0.220),
                                    static_cast<int>(v_windowHeight * 0.121));

    ui->calib_left_checkbox->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                         static_cast<int>(v_windowHeight * 0.012100),
                                         static_cast<int>(v_windowWidth * 0.106000),
                                         static_cast<int>(v_windowHeight * 0.025400));
    ui->calib_left_checkbox->setFont(font);

    ui->calib_right_checkbox->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                          static_cast<int>(v_windowHeight * 0.084400),
                                          static_cast<int>(v_windowWidth * 0.106000),
                                          static_cast<int>(v_windowHeight * 0.025400));
    ui->calib_right_checkbox->setFont(font);

    ui->calib_middle_checkbox->setGeometry(static_cast<int>(v_windowWidth * 0.007100),
                                           static_cast<int>(v_windowHeight * 0.048200),
                                           static_cast<int>(v_windowWidth * 0.106000),
                                           static_cast<int>(v_windowHeight * 0.025400));
    ui->calib_middle_checkbox->setFont(font);

    ui->calibrationBtn->setGeometry(static_cast<int>(v_windowWidth * 0.127200),
                                    static_cast<int>(v_windowHeight * 0.048200),
                                    static_cast<int>(v_windowWidth * 0.063600),
                                    static_cast<int>(v_windowHeight * 0.030200));
    ui->calibrationBtn->setFont(font);
    //FIN ...

    //...
    ui->frame->setGeometry(static_cast<int>(v_windowWidth * 0.579100),
                           static_cast<int>(v_windowHeight * 0.361500),
                           static_cast<int>(v_windowWidth * 0.247900),
                           static_cast<int>(v_windowHeight * 0.085600));
    ui->frame->setFont(font);

    ui->x_lin_vel_label->setGeometry(static_cast<int>(v_windowWidth * 0.014200),
                                     static_cast<int>(v_windowHeight * 0.000100),
                                     static_cast<int>(v_windowWidth * 0.070700),
                                     static_cast<int>(v_windowHeight * 0.025400));
    ui->x_lin_vel_label->setFont(font);

    ui->label_4->setGeometry(static_cast<int>(v_windowWidth * 0.028300),
                             static_cast<int>(v_windowHeight * 0.018100),
                             static_cast<int>(v_windowWidth * 0.038900),
                             static_cast<int>(v_windowHeight * 0.025400));
    ui->label_4->setFont(font);

    ui->label_5->setGeometry(static_cast<int>(v_windowWidth * 0.106000),
                             static_cast<int>(v_windowHeight * 0.018100),
                             static_cast<int>(v_windowWidth * 0.038900),
                             static_cast<int>(v_windowHeight * 0.025400));
    ui->label_5->setFont(font);

    ui->y_lin_vel_label->setGeometry(static_cast<int>(v_windowWidth * 0.091900),
                                     static_cast<int>(v_windowHeight * 0.000100),
                                     static_cast<int>(v_windowWidth * 0.070700),
                                     static_cast<int>(v_windowHeight * 0.025400));
    ui->y_lin_vel_label->setFont(font);

    ui->z_ang_vel_label->setGeometry(static_cast<int>(v_windowWidth * 0.169500),
                                     static_cast<int>(v_windowHeight * 0.000100),
                                     static_cast<int>(v_windowWidth * 0.073500),
                                     static_cast<int>(v_windowHeight * 0.025400));
    ui->z_ang_vel_label->setFont(font);

    ui->label_6->setGeometry(static_cast<int>(v_windowWidth * 0.183700),
                             static_cast<int>(v_windowHeight * 0.018100),
                             static_cast<int>(v_windowWidth * 0.038900),
                             static_cast<int>(v_windowHeight * 0.025400));
    ui->label_6->setFont(font);

    ui->x_lin_vel_value->setGeometry(static_cast<int>(v_windowWidth * 0.021200),
                                     static_cast<int>(v_windowHeight * 0.048200),
                                     static_cast<int>(v_windowWidth * 0.064300),
                                     static_cast<int>(v_windowHeight * 0.025400));
    ui->x_lin_vel_value->setFont(font);

    ui->y_lin_vel_value->setGeometry(static_cast<int>(v_windowWidth * 0.098900),
                                     static_cast<int>(v_windowHeight * 0.048200),
                                     static_cast<int>(v_windowWidth * 0.064300),
                                     static_cast<int>(v_windowHeight * 0.025400));
    ui->y_lin_vel_value->setFont(font);

    ui->z_ang_vel_value->setGeometry(static_cast<int>(v_windowWidth * 0.176600),
                                     static_cast<int>(v_windowHeight * 0.048200),
                                     static_cast<int>(v_windowWidth * 0.064300),
                                     static_cast<int>(v_windowHeight * 0.025400));
    ui->z_ang_vel_value->setFont(font);
    //FIN ...

    //...
    string ss;

    font.setBold(true);

    ss = "QPushButton "
         "{ background-color: rgb(255, 255, 255);"
         "border-radius: " + to_string(static_cast<int>(v_windowWidth * 0.01413)) + "px;";
    ss += "border-style: outset;"
          "border-color: rgb(255, 0, 0);"
          "border-width: " + to_string(static_cast<int>(v_windowWidth * 0.00212)) + "px; }";
    ss += "QPushButton:pressed "
          "{ border-style: inset;"
          "background-color: rgb(250, 0, 0);"
          "border-color: rgb(255, 0, 0); }"
          "QPushButton:hover:!pressed "
          "{ background-color: rgb(255, 100, 100); }";

    v_joystickEnabledStyle = QString::fromStdString(ss);

    ss = "QPushButton "
         "{ background-color: rgb(100, 255, 100);"
         "color: rgb(0, 0, 0);"
         "border-radius: " + to_string(static_cast<int>(v_windowWidth * 0.01413)) + "px;";
    ss += "border-style: outset;"
          "border-color: rgb(0, 255, 0);"
          "border-width: " + to_string(static_cast<int>(v_windowWidth * 0.00212)) + "px; }";
    ss += "QPushButton:hover:!pressed "
          "{ background-color: rgb(0, 255, 0); }";

    v_joystickDesabledStyle = QString::fromStdString(ss);

    ss = "QPushButton "
         "{ background-color: rgb(230, 230, 230);"
         "color: rgb(100, 100, 100);"
         "border-radius: " + to_string(static_cast<int>(v_windowWidth * 0.01413)) + "px;";
    ss += "border-style: outset;"
          "border-color: rgb(100, 100, 100);"
          "border-width: " + to_string(static_cast<int>(v_windowWidth * 0.00212)) + "px; }";

    v_joystickDesabledStyle_2 = QString::fromStdString(ss);

    ui->robot_fw->setGeometry(static_cast<int>(v_windowWidth * 0.699200),
                              static_cast<int>(v_windowHeight * 0.457900),
                              static_cast<int>(v_windowWidth * 0.028300),
                              static_cast<int>(v_windowHeight * 0.084400));
    ui->robot_fw->setFont(font);
    ui->robot_fw->setStyleSheet(v_joystickEnabledStyle);

    ui->robot_bw->setGeometry(static_cast<int>(v_windowWidth * 0.699200),
                              static_cast<int>(v_windowHeight * 0.638600),
                              static_cast<int>(v_windowWidth * 0.028300),
                              static_cast<int>(v_windowHeight * 0.084400));
    ui->robot_bw->setFont(font);
    ui->robot_bw->setStyleSheet(v_joystickEnabledStyle);

    ui->robot_left->setGeometry(static_cast<int>(v_windowWidth * 0.635600),
                                static_cast<int>(v_windowHeight * 0.566300),
                                static_cast<int>(v_windowWidth * 0.049500),
                                static_cast<int>(v_windowHeight * 0.048200));
    ui->robot_left->setFont(font);
    ui->robot_left->setStyleSheet(v_joystickEnabledStyle);

    ui->robot_right->setGeometry(static_cast<int>(v_windowWidth * 0.741600),
                                 static_cast<int>(v_windowHeight * 0.566300),
                                 static_cast<int>(v_windowWidth * 0.049500),
                                 static_cast<int>(v_windowHeight * 0.048200));
    ui->robot_right->setFont(font);
    ui->robot_right->setStyleSheet(v_joystickEnabledStyle);

    ui->robot_stop->setGeometry(static_cast<int>(v_windowWidth * 0.685100),
                                static_cast<int>(v_windowHeight * 0.542200),
                                static_cast<int>(v_windowWidth * 0.056500),
                                static_cast<int>(v_windowHeight * 0.096400));
    ui->robot_stop->setFont(font);
    ui->robot_stop->setStyleSheet(v_joystickEnabledStyle);

    ui->robot_rot_cw->setGeometry(static_cast<int>(v_windowWidth * 0.663900),
                                  static_cast<int>(v_windowHeight * 0.722900),
                                  static_cast<int>(v_windowWidth * 0.042400),
                                  static_cast<int>(v_windowHeight * 0.072300));
    ui->robot_rot_cw->setFont(font);

    ui->robot_rot_ccw->setGeometry(static_cast<int>(v_windowWidth * 0.720400),
                                   static_cast<int>(v_windowHeight * 0.722900),
                                   static_cast<int>(v_windowWidth * 0.042400),
                                   static_cast<int>(v_windowHeight * 0.072300));
    ui->robot_rot_ccw->setFont(font);

    ss = "QPushButton "
         "{ background-color: rgb(255, 255, 255);"
         "border-radius: " + to_string(static_cast<int>(ui->robot_rot_ccw->geometry().width() / 2)) + "px;";
    ss += "border-style: outset;"
          "border-color: rgb(255, 0, 0);"
          "border-width: " + to_string(static_cast<int>(v_windowWidth * 0.00212)) + "px; }";
    ss += "QPushButton:pressed "
          "{ border-style: inset;"
          "background-color: rgb(250, 0, 0);"
          "border-color: rgb(255, 0, 0); }"
          "QPushButton:hover:!pressed "
          "{ background-color: rgb(255, 100, 100); }";

    v_joystickRotEnabledStyle = QString::fromStdString(ss);

    ss = "QPushButton "
         "{ background-color: rgb(100, 255, 100);"
         "color: rgb(0, 0, 0);"
         "border-radius: " + to_string(static_cast<int>(ui->robot_rot_ccw->geometry().width() / 2)) + "px;";
    ss += "border-style: outset;"
          "border-color: rgb(0, 255, 0);"
          "border-width: " + to_string(static_cast<int>(v_windowWidth * 0.00212)) + "px; }";
    ss += "QPushButton:hover:!pressed "
          "{ background-color: rgb(0, 255, 0); }";

    v_joystickRotDesabledStyle = QString::fromStdString(ss);

    ss = "QPushButton "
         "{ background-color: rgb(230, 230, 230);"
         "color: rgb(100, 100, 100);"
         "border-radius: " + to_string(static_cast<int>(ui->robot_rot_ccw->geometry().width() / 2)) + "px;";
    ss += "border-style: outset;"
          "border-color: rgb(100, 100, 100);"
          "border-width: " + to_string(static_cast<int>(v_windowWidth * 0.00212)) + "px; }";

    v_joystickRotDesabledStyle_2 = QString::fromStdString(ss);

    ui->robot_rot_cw->setStyleSheet(v_joystickRotEnabledStyle);
    ui->robot_rot_ccw->setStyleSheet(v_joystickRotEnabledStyle);

    font.setBold(false);
    //FIN ...
}

void MainWindow::keyboardShortcuts()
{
    ui->action_Quitter->setShortcut(QKeySequence("Ctrl+Q"));
    ui->action_Save->setShortcut(QKeySequence("Ctrl+S"));
    ui->action_New_measure->setShortcut(QKeySequence("Ctrl+N"));
}

void MainWindow::initViews()
{
    double userData;

    userData = 0.0225;
    ui->motor_precision->addItem("0.0225", userData);
    userData = 0.045;
    ui->motor_precision->addItem("0.045", userData);
    userData = 0.09;
    ui->motor_precision->addItem("0.09", userData);
    userData = 0.18;
    ui->motor_precision->addItem("0.18", userData);
    userData = 0.36;
    ui->motor_precision->addItem("0.36", userData);
    ui->motor_precision->setCurrentIndex(0);

    ui->duty_cycle->addItem("10", 10);
    ui->duty_cycle->addItem("15", 15);
    ui->duty_cycle->addItem("20", 20);
    ui->duty_cycle->addItem("25", 25);
    ui->duty_cycle->addItem("30", 30);
    ui->duty_cycle->addItem("35", 35);
    ui->duty_cycle->addItem("40", 40);
    ui->duty_cycle->addItem("45", 45);
    ui->duty_cycle->addItem("50", 50);

    ui->ID_cam_g->addItem("", -1);
    ui->ID_cam_m->addItem("", -1);
    ui->ID_cam_d->addItem("", -1);
    for(int i = 0; i < 10; i++){
        ui->ID_cam_g->addItem(QString::fromStdString(std::to_string(i)), i);
        ui->ID_cam_m->addItem(QString::fromStdString(std::to_string(i)), i);
        ui->ID_cam_d->addItem(QString::fromStdString(std::to_string(i)), i);
    }

    ui->calib_img_progressbar->setValue(0);

    //...
    if(!ui->manualControl_checkbox->isChecked()){
        ui->camView->hide();
        ui->cameras->hide();
        ui->robot_fw->hide();
        ui->robot_bw->hide();
        ui->robot_left->hide();
        ui->robot_right->hide();
        ui->robot_stop->hide();
        ui->robot_rot_ccw->hide();
        ui->robot_rot_cw->hide();
        ui->frame->hide();
        ui->robot_free_positionning->hide();
    }

    if(!ui->calibration_checkbox->isChecked()){
        ui->start_chronoBtn->hide();
        ui->calibration_test_groupbox->hide();
        ui->frame_cameras->hide();
    }

    ui->gen_map->hide();
    ui->reuseTrajectory->hide();

    ui->frame_cameras->setStyleSheet(frameStyle_1);

    if(ui->cameras->isChecked()){
        ui->calibration_test_groupbox->setEnabled(true);
        ui->cameras->setStyleSheet(groupBoxStyle_2);
    }
    else{
        ui->calibration_test_groupbox->setEnabled(false);
        ui->cameras->setStyleSheet(groupBoxStyle_3);
    }

    ui->cam_left->setStyleSheet(radioStyle_1);
    ui->cam_right->setStyleSheet(radioStyle_3);
    ui->cam_middle->setStyleSheet(radioStyle_3);

    ui->threshold->setStyleSheet(pushButtonEnabledStyle);
    ui->camerasButton->setStyleSheet(pushButtonEnabledStyle);

    if(!ui->log->isChecked()) ui->clean_msg_window->setStyleSheet(pushButtonDesabledStyle);
    else ui->clean_msg_window->setStyleSheet(pushButtonEnabledStyle);

    ui->robot_stop->setEnabled(false);
    ui->robot_stop->setStyleSheet(v_joystickDesabledStyle);

    ui->IPButton->setStyleSheet(pushButtonEnabledStyle);
    ui->calibrationBtn->setStyleSheet(pushButtonEnabledStyle);
    ui->start_chronoBtn->setStyleSheet(QString::fromStdString(v_pushButtonStyle_4));

    ui->calibration_test_groupbox->setStyleSheet(groupBoxStyle_10);
    ui->calib_left_checkbox->setStyleSheet(checkBoxStyle_4);
    ui->calib_right_checkbox->setStyleSheet(checkBoxStyle_4);
    ui->calib_middle_checkbox->setStyleSheet(checkBoxStyle_4);

    ui->robot_free_pos_without_measure->setStyleSheet(radioStyle_1);
    ui->robot_free_pos_with_measure->setStyleSheet(radioStyle_3);
}

void MainWindow::manageCameras(bool launch)
{
    AcquisitionThread *acqT = nullptr;
    if(!launch){
        if(ui->cam_left->isChecked() && v_cam_G_acq->isRunning()) acqT = v_cam_G_acq;
        else if(ui->cam_middle->isChecked() && v_cam_M_acq->isRunning()) acqT = v_cam_M_acq;
        else if(ui->cam_right->isChecked() && v_cam_D_acq->isRunning()) acqT = v_cam_D_acq;

        if(acqT != nullptr){
            acqT->requestInterruption();
            acqT->wait();
            disconnect(acqT, SIGNAL(newFrame(cv::Mat)), this, SLOT(dispFrame(cv::Mat)));
            acqT->stopCamera();
        }
    }
    else{
        if(ui->cam_left->isChecked()) acqT = v_cam_G_acq;
        else if(ui->cam_middle->isChecked()) acqT = v_cam_M_acq;
        else if(ui->cam_right->isChecked()) acqT = v_cam_D_acq;

        connect(acqT, SIGNAL(newFrame(cv::Mat)), this, SLOT(dispFrame(cv::Mat)));
        acqT->launchCamera();
        acqT->start();
    }

    acqT = nullptr;
}

void MainWindow::runCamera(AcquisitionThread *acqT_1, //Thread à lancer
                           AcquisitionThread *acqT_2, //Thread à arrêter
                           AcquisitionThread *acqT_3  //Thread à arrêter
                           )
{
    AcquisitionThread *th1 = acqT_1;
    AcquisitionThread *th2 = acqT_2;
    AcquisitionThread *th3 = acqT_3;

    cv::Mat empty = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    cv::resize(empty, empty, cv::Size(), v_fx, v_fy);
    QPixmap pix = Mat2Pixmap(empty, true, 2);
    ui->camView->setPixmap(pix);

    if(th2->isRunning()){
        th2->requestInterruption();
        th2->wait();
        disconnect(th2, SIGNAL(newFrame(cv::Mat)), this, SLOT(dispFrame(cv::Mat)));
        th2->stopCamera();
    }
    else if(th3->isRunning()){
        th3->requestInterruption();
        th3->wait();
        disconnect(th3, SIGNAL(newFrame(cv::Mat)), this, SLOT(dispFrame(cv::Mat)));
        th3->stopCamera();
    }

    if(th1->launchCamera()){
        connect(th1, SIGNAL(newFrame(cv::Mat)), this, SLOT(dispFrame(cv::Mat)));
        th1->start();
    }

    th1 = nullptr;
    th2 = nullptr;
    th3 = nullptr;
}

void MainWindow::enableJoystick(bool enable)
{
    if(ui->robot_stop->isEnabled()) stopManualControl();

    ui->robot_fw->setEnabled(enable);
    ui->robot_bw->setEnabled(enable);
    ui->robot_left->setEnabled(enable);
    ui->robot_right->setEnabled(enable);
    ui->robot_rot_cw->setEnabled(enable);
    ui->robot_rot_ccw->setEnabled(enable);

    if(enable){
        ui->robot_fw->setStyleSheet(v_joystickEnabledStyle);
        ui->robot_bw->setStyleSheet(v_joystickEnabledStyle);
        ui->robot_left->setStyleSheet(v_joystickEnabledStyle);
        ui->robot_right->setStyleSheet(v_joystickEnabledStyle);
        ui->robot_rot_cw->setStyleSheet(v_joystickRotEnabledStyle);
        ui->robot_rot_ccw->setStyleSheet(v_joystickRotEnabledStyle);
        ui->robot_stop->setStyleSheet(v_joystickDesabledStyle);
    }else{
        ui->robot_fw->setStyleSheet(v_joystickDesabledStyle_2);
        ui->robot_bw->setStyleSheet(v_joystickDesabledStyle_2);
        ui->robot_left->setStyleSheet(v_joystickDesabledStyle_2);
        ui->robot_right->setStyleSheet(v_joystickDesabledStyle_2);
        ui->robot_rot_cw->setStyleSheet(v_joystickRotDesabledStyle_2);
        ui->robot_rot_ccw->setStyleSheet(v_joystickRotDesabledStyle_2);
        ui->robot_stop->setStyleSheet(v_joystickDesabledStyle_2);
    }
}

void MainWindow::checkRPiNodes(bool &motorNode, bool &sensorNode)
{
    QDir dir(QString::fromStdString(projectDirectory));
    string path = dir.absolutePath().toStdString();
    string newPath = path.append("/others/node_list.txt");
    string command = "C:/opt/ros/noetic/x64/setup.bat && rosnode list > " + newPath;
    QFile fl(dir.absolutePath() + "/others/node_list.txt");
    QByteArray data;

    dir.setPath(dir.absolutePath() + "/others");

    system(command.c_str());

    if(fl.open(QIODevice::ReadOnly)){
        data = fl.readAll();
        QString str(data);
        QStringList strList = str.split(QChar('\n'), Qt::SkipEmptyParts);

        for(int i = 0; i < strList.size(); i++){
            strList[i].truncate(strList[i].size() - 1);
            if(strList[i] == "/motor_mast_node"){
                motorNode = true;
                break;
            }
        }

        strList = str.split(QChar('\n'), Qt::SkipEmptyParts);
        for(int i = 0; i < strList.size(); i++){
            strList[i].truncate(strList[i].size() - 1);
            if(strList[i] == "/sensor_node"){
                sensorNode = true;
                break;
            }
        }
    }
    fl.close();
}

bool MainWindow::checkPCNodes(QString &outputMsg)
{
    bool check = true;

    if(!v_pcControllerNodeStatus){
        check &= false;
        outputMsg += "\n-> L'application pc_controller_node.exe est arrêtée";
    }
    if(!v_intermediaryNodeStatus){
        check &= false;
        outputMsg += "\n-> L'application intermediary_node.exe est arrêtée";
    }

    if(!check) outputMsg += "\nVeuillez démarrer le(s) noeud(s) ci-dessus avant de continuer";

    return check;
}

bool MainWindow::checkHeights()
{
    QString text;
    QStringList strList;
    bool isOk = true;

    if(ui->radioSavedHigh->isChecked()) text = ui->savedHigh->text();
    else if(ui->radioOtherHigh->isChecked()) text = ui->otherHigh->text();

    strList = text.split(QChar(';'), Qt::SkipEmptyParts);

    if(strList.size() > 0){
        for(int i = 0; i < strList.size(); i++){
            bool ok;
            int val = strList[i].toInt(&ok);

            if(ok && (val < 100)){
                isOk = false;
                break;
            }
        }
    }
    else isOk = false;

    return isOk;
}

bool MainWindow::checkCameras(QString &msg){
    bool status = true;

    QString text1 = "Fichier de calibration absent";
    QString text2 = "ID invalide";

    msg = "";

    if(v_cam[Camera::GAUCHE]->getCamIsPresent()){
        if(!v_cam[Camera::GAUCHE]->run()){
            status &= false;
            msg += "\n[ CAMERA GAUCHE ] : " + text1;
        }
    }
    else{
        status &= false;
        msg += "\n[ CAMERA GAUCHE ] : " + text2;
    }

    if(v_cam[Camera::MILIEU]->getCamIsPresent()){
        if(!v_cam[Camera::MILIEU]->run()){
            status &= false;
            msg += "\n[ CAMERA MILIEU ] : " + text1;
        }
    }
    else{
        status &= false;
        msg += "\n[ CAMERA MILIEU ] : " + text2;
    }

    if(v_cam[Camera::DROITE]->getCamIsPresent()){
        if(!v_cam[Camera::DROITE]->run()){
            status &= false;
            msg += "\n[ CAMERA DROITE ] : " + text1;
        }
    }
    else{
        status &= false;
        msg += "\n[ CAMERA DROITE ] : " + text2;
    }

    return status;
}

void MainWindow::manageJoystick(int robotDirection, bool &pressedBtnState, bool &unPressedBtnState,
                                QPushButton *pressedBtn, QPushButton *unpressedBtn)
{
    if(v_velNodeStatus && v_pcControllerNodeStatus && v_rosMasterNodeStatus){
        if(robotDirection != RobotMove::STOP){
            if(!ui->robot_stop->isEnabled()){
                ui->robot_stop->setEnabled(true);
                ui->robot_stop->setStyleSheet(v_joystickEnabledStyle);
            }

            if(pressedBtnState){
                pressedBtnState = false;

                if(pressedBtn == ui->robot_rot_ccw || pressedBtn == ui->robot_rot_cw)
                    pressedBtn->setStyleSheet(v_joystickRotEnabledStyle);
                else pressedBtn->setStyleSheet(v_joystickEnabledStyle);
            }
            else{
                if(unPressedBtnState){
                    unPressedBtnState = false;
                    while(v_moveRobotThread->isRunning()){}
                    v_moveRobotThread->start();

                    if(unpressedBtn == ui->robot_rot_ccw || unpressedBtn == ui->robot_rot_cw)
                        unpressedBtn->setStyleSheet(v_joystickRotEnabledStyle);
                    else unpressedBtn->setStyleSheet(v_joystickEnabledStyle);
                }

                pressedBtnState = true;

                if(pressedBtn == ui->robot_rot_ccw || pressedBtn == ui->robot_rot_cw)
                    pressedBtn->setStyleSheet(v_joystickRotDesabledStyle);
                else pressedBtn->setStyleSheet(v_joystickDesabledStyle);
            }

            if(!v_left_btn_joy_status &&
               !v_right_btn_joy_status &&
               !v_fw_btn_joy_status &&
               !v_bw_btn_joy_status &&
               !v_ccw_btn_joy_status &&
               !v_cw_btn_joy_status){
                ui->robot_stop->setEnabled(false);
                ui->robot_stop->setStyleSheet(v_joystickDesabledStyle);

                v_moveRobotThread->joystick(RobotMove::STOP);
            }
            else{
                v_moveRobotThread->joystick(robotDirection,
                                          ui->x_lin_vel_value->value(),
                                          ui->y_lin_vel_value->value(),
                                          ui->z_ang_vel_value->value());
            }
        }
        else{
            ui->robot_stop->setEnabled(false);

            ui->robot_fw->setStyleSheet(v_joystickEnabledStyle);
            ui->robot_bw->setStyleSheet(v_joystickEnabledStyle);
            ui->robot_left->setStyleSheet(v_joystickEnabledStyle);
            ui->robot_right->setStyleSheet(v_joystickEnabledStyle);
            ui->robot_rot_ccw->setStyleSheet(v_joystickRotEnabledStyle);
            ui->robot_rot_cw->setStyleSheet(v_joystickRotEnabledStyle);
            ui->robot_stop->setStyleSheet(v_joystickDesabledStyle);

            v_moveRobotThread->joystick(robotDirection);
        }

        while(v_moveRobotThread->isRunning()){}
        v_moveRobotThread->start();
    }
    else{
        QString warn = "Veuillez exécuter : ";

        if(!v_rosMasterNodeStatus) warn += "\n\t-> rosmaster.exe";
        if(!v_velNodeStatus) warn += "\n\t-> vel_node.exe";
        if(!v_pcControllerNodeStatus) warn += "\n\t-> pc_controller_node.exe";
        QMessageBox::warning(this, "", warn);
    }
}

bool MainWindow::launchNode(QString nodeExe, QString &msg, void (*function)())
{
    QDir dir(QDir::currentPath());

    if(dir.exists(nodeExe)){
        std::thread t(function);
        t.detach();

        return true;
    }
    else{
        msg += "-> L'application " + nodeExe + " n'a pas été détecté"
                                               " dans le dossier " + QDir::currentPath() + " pour un démarrage automatique.";
        msg += " Veuillez la démarrer manuellement avant de poursuivre.\n";
        return false;
    }
}

bool MainWindow::launchNode(QString nodeExe, QString *msg, bool nodeStatus, void (*function)())
{
    if(v_rosMasterNodeStatus){
        if(!nodeStatus){
            if(msg == nullptr){
                QString out;
                bool result = launchNode(nodeExe, out, function);
                if(!result) emit displayMsgSignal(out, YELLOW_COLOR);

                return result;
            }
            else return launchNode("vel_node.exe", *msg, function);
        }
        else{
            QMessageBox::information(this, "", "L'application " + nodeExe + " est déjà en cours d'exécution");
            return true;
        }
    }
    else{
        QMessageBox::warning(this, "", "Veuillez exécuter l'application rosmaster.exe");
        return false;
    }
}

void MainWindow::stopNode(string nodeExe, bool nodeStatus)
{
    if(nodeStatus){
        string command = "taskkill /t /f /im " + nodeExe + " && exit";
        system(command.c_str());
    }
}

void MainWindow::quitIHM(){
    QFile file(QString::fromStdString(saveProject) + "current_project.name");

    if(file.exists()){
        file.remove();
    }

    if(v_rc->isRunning()){
        v_rc->requestInterruption();
        v_rc->wait();
    }

    stopNode("vel_node.exe", v_velNodeStatus);
    stopNode("pc_controller_node.exe", v_pcControllerNodeStatus);
    stopNode("intermediary_node.exe", v_intermediaryNodeStatus);
    stopNode("roscore.exe", v_rosMasterNodeStatus);

    saveParamIHM();

    qApp->quit();
}

void MainWindow::rosNodeMaster(){
    if(!v_rosMasterNodeStatus){
        std::thread t(rosMaster);
        t.detach();
    }
    else{
        QMessageBox::information(this, "", "L'application rosmaster.exe est déjà en cours d'exécution");
    }
}

void MainWindow::nodeList(){
    std::thread t(rosNodeList);
    t.detach();
}

void MainWindow::dispFrame(const cv::Mat &frame){
    cv::Mat img;
    cv::resize(frame, img, cv::Size(), v_fx, v_fy);
    QPixmap pix = Mat2Pixmap(img, true, 2);
    ui->camView->setPixmap(pix);
}

void MainWindow::setTresh(){
    v_threshDial->show();
}

void MainWindow::generateMap(){
    cv::Mat result;
    cv::Mat map4RobotPosition = cv::Mat::zeros(Size(static_cast<int>(stoi(ui->waLength->text().toStdString())*scale),
                                                    static_cast<int>(stoi(ui->waWidth->text().toStdString())*scale)), CV_8UC1);
    int diameter = ui->diameter->value(); //mm
    auto R = static_cast<int>((diameter/2)*scale);
    string msg;

    CLOCK_TIME_POINT ti;

#ifdef IHM_DEBUG
    double time_s;
    int min, sec;
#endif

    //Calcul des matrices de rotation et de translation des caméras avant de générer la map
    for(uint i = 0; i < 3; i++){
        if(v_cam[i]->useCam()) v_cam[i]->calcExtrinsicParams();
    }

    ti = CLOCK_NOW;

    //Génération de la map vue par chaque caméra
    cv::Mat map1 = v_cam[Camera::GAUCHE]->generateTopographicMap(CanalBGR::B);
    cv::Mat map2 = v_cam[Camera::DROITE]->generateTopographicMap(CanalBGR::G);
    cv::Mat map3 = v_cam[Camera::MILIEU]->generateTopographicMap(CanalBGR::R);

    //Association des maps de chaque caméra
    cv::Mat realMap = map1 + map2 + map3;

    //Redimensionnement de l'image de la map
    cv::resize(map1, map1, Size(), static_cast<double>(scale), static_cast<double>(scale));
    cv::resize(map2, map2, Size(), static_cast<double>(scale), static_cast<double>(scale));
    cv::resize(map3, map3, Size(), static_cast<double>(scale), static_cast<double>(scale));
    cv::resize(realMap, realMap, Size(), static_cast<double>(scale), static_cast<double>(scale));

    //Mappage de la zone de travail du robot
    dilate(realMap, realMap, Mat(), Point(-1, -1), 3);

    cv::Mat map = cv::Mat::zeros(realMap.size(), CV_8UC3);
    std::vector<cv::Point> pts;

    for (int i = 0; i < realMap.rows; i++) {
        auto p = realMap.ptr<cv::Vec3b>(i);
        for (int j = 0; j < realMap.cols; j++) {
            if (p[j].val[2] != 0 && p[j].val[0] != 0 && p[j].val[1] != 0) {
                map.ptr<cv::Vec3b>(i)[j].val[2] = 255;
                pts.emplace_back(cv::Point(j, i));
            }
        }
    }

    cv::Rect rect = boundingRect(pts);

    dilate(map, map, cv::Mat(), cv::Point(-1, -1), 1);
    rectangle(map, rect, cv::Scalar(255, 255, 255), cv::FILLED);

#ifdef SHOW_IMG_MAP
    cv::imshow("Map cam left", map1); pause();
    cv::imshow("Map cam right", map2); pause();
    cv::imshow("Map cam middle", map3); pause();
    cv::imshow("Map", realMap); pause();
    cv::imshow("Map", realMap); pause();
    cv::imshow("Map with isolated obstacle", map); pause();
#endif

    for(int i = 0; i < map.rows; i++){
        auto p = map.ptr<cv::Vec3b>(i);
        for(int j = 0; j < map.cols; j++){
            if(p[j].val[0] == 255 && p[j].val[1] == 255 && p[j].val[2] == 255) continue;

            p[j].val[0] = 0;
            p[j].val[1] = 0;
            p[j].val[2] = 0;
        }
    }

    cv::cvtColor(map, result, cv::COLOR_BGR2GRAY);
    cv::bitwise_not(result, result);

    //Génération de la carte de déplacement du robot
    auto limInfX = R;
    auto limSupX = static_cast<int>(stoi(ui->waWidth->text().toStdString())*scale - R - 1);
    auto limInfY = R;
    auto limSupY = static_cast<int>(stoi(ui->waLength->text().toStdString())*scale - R - 1);

    for(auto j = limInfX; j <= limSupX; j++){
        for(auto i = limInfY; i <= limSupY; i++){
            int average = 0;
            for(auto x = j-R; x <= j+R; x++){
                for(auto y = i-R; y <= i+R; y++){
                    average += static_cast<int>(result.ptr(x)[y]);
                }
            }
            average /= ((2*R + 1)*(2*R + 1));
            if(average == 255) *map4RobotPosition.ptr<uchar>(j, i) = 255;
        }
    }

    //Enregistrement des cartes:
    //  - realTopographicMap = carte de la zone de travail avec ou sans obstacles (matrice result)
    //  - topographicMap = carte de déplacement du robot (matrice map4RobotPosition)
    cv::imwrite(pathRealTopographicMap, result);
    cv::imwrite(pathTopographicMap, map4RobotPosition);

#ifdef IHM_DEBUG
    elapsedTime(ti, time_s, min, sec);
    msg = "La map a été générée en " + to_string(time_s) + " s (" + to_string(min) + " min " + to_string(sec) + " s)";
    emit displayMsgSignal(QString::fromStdString(msg), GREEN_COLOR);
#endif

#ifdef SHOW_IMG_MAP
    cv::destroyAllWindows();
#endif

}

void MainWindow::measure(){
    QString camCheckMsg;
    QString nodePCCheckMsg;

#ifdef CHECK_CAMERAS
    if(checkCameras(camCheckMsg)){ //!1
#else
    if(true){ //!1
#endif
        if(checkHeights()){ //!2

            if(v_rosMasterNodeStatus){ //!3
                if(checkPCNodes(nodePCCheckMsg)){ //!4
                    bool motorNode = false;
                    bool sensorNode = false;

                    checkRPiNodes(motorNode, sensorNode);

                    QString text1 = "Veuillez démarrer, sur le Raspberry Pi, le(s) noeud(s) suivant(s): ";

                    if(motorNode && !sensorNode){ //!5
                        QMessageBox::warning(this, "", text1 + "\n-> /sensor_node");
                    }
                    else if(!motorNode && sensorNode){
                        QMessageBox::warning(this, "", text1 + "\n-> /motor_node");
                    }
                    else if(!motorNode && !sensorNode){
                        QMessageBox::warning(this, "", text1 + "\n-> /sensor_node"
                                                               "\n-> /motor_mast_node");
                    }
                    else{
                        if(!ui->manualControl_checkbox->isChecked()){
                            saveParamIHM();
                            delay_s(0.5);
                        }

                        //(Dés)activation de certains widgets
                        bool activate = false;

                        if(!ui->manualControl_checkbox->isChecked()){
                            ui->measure->setEnabled(activate);
                            ui->measure->setStyleSheet(pushButtonDesabledStyle);

                            ui->stop->setEnabled(!activate);
                            ui->stop->setStyleSheet(pushButtonEnabledStyle);
                        }
                        else{
                            ui->robot_free_pos_go->setText("STOP");
                            ui->robot_free_pos_go->setStyleSheet(pushButtonEnabledStyle_2);

                            ui->manualControl_checkbox->setEnabled(false);
                            ui->calibration_checkbox->setEnabled(false);
                            ui->robot_free_pos_without_measure->setEnabled(false);
                        }

                        ui->IDs_cams->setEnabled(activate);
                        ui->IDs_cams->setStyleSheet(groupBoxStyle_6);

                        ui->manualControl->setEnabled(activate);
                        ui->manualControl->setStyleSheet(groupBoxStyle_3);

                        ui->calibration->setEnabled(activate);
                        ui->calibration->setStyleSheet(groupBoxStyle_3);

                        ui->ros_master->setEnabled(activate);
                        ui->ros_master->setStyleSheet(pushButtonDesabledStyle);

                        ui->threshold->setEnabled(activate);
                        ui->threshold->setStyleSheet(pushButtonDesabledStyle);

                        ui->camerasButton->setEnabled(activate);
                        ui->camerasButton->setStyleSheet(pushButtonDesabledStyle);

                        //ui->gen_map->setEnabled(activate);
                        //ui->gen_map->setStyleSheet(pushButtonDesabledStyle);

                        generateMap(); ras();

                        int length_tool = stoi(ui->toolLength->text().toStdString());
                        int width_tool = stoi(ui->toolWidth->text().toStdString());

                        cv::Mat tool = (Mat_<double>(1,3) << length_tool, width_tool, 0.0);
                        cv::Mat map4RobotPosition = cv::imread(pathTopographicMap, cv::IMREAD_GRAYSCALE);
                        cv::Mat map = cv::imread(pathRealTopographicMap, cv::IMREAD_GRAYSCALE);
                        cv::Mat result = map.clone();
                        int diameter = ui->diameter->value(); //mm
                        int extra; //mm
                        int area_width = stoi(ui->waWidth->text().toStdString());
                        int area_length = stoi(ui->waLength->text().toStdString());
                        int dx = 0;
                        int dy = 0;
                        double linVelMax = stod(ui->linearVelMax->text().toStdString());
                        double angVelMax = M_PI/2;
                        cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);

                        if(fs.isOpened()) fs["extra"] >> extra;
                        else extra = 30;

                        fs.release();
                        length_tool += length_tool + extra;

            #ifdef IHM_DEBUG
                        int min, sec;
                        double time_s;
            #endif
                        Measurement mes;
                        std::vector<cv::Point> mesPoint;

                        if(!ui->manualControl_checkbox->isChecked()){
                            dx = stoi(ui->dx->text().toStdString());
                            dy = stoi(ui->dy->text().toStdString());
                            mes.setDeltaX(dx);
                            mes.setDeltaY(dy);

                            mesPoint = mes.getMeasurePoints();
                        }
                        else{
                            mesPoint.emplace_back(cv::Point(ui->robot_free_pos_y->value(),
                                                            ui->robot_free_pos_x->value()));
                        }

                        Robot robot(diameter, tool, linVelMax, angVelMax);
                        std::vector<cv::Point> reachablePoints;
                        std::vector<cv::Point> robotPositionPoints;
                        std::vector<cv::Mat> robotPositionByCam(3);
                        QString msg;
                        std::string msgStr;

                        CLOCK_TIME_POINT ti;

                        if(v_rc->isRunning()){
                            v_rc->requestInterruption();
                            v_rc->wait();
                        }

                        ti = CLOCK_NOW;

                        if(!v_reachablePoints.empty()) v_reachablePoints.clear();
                        if(!v_unReachablePoints.empty()) v_unReachablePoints.clear();
                        if(!v_robotPositionPoints.empty()) v_robotPositionPoints.clear();
                        if(!v_map.empty()) v_map.release();

                        //Extraction des points de mesures atteignables par le robot
                        extractMeasurePoints(result, mesPoint, reachablePoints); ras(2);

                        if(reachablePoints.size() >= 1){
                #ifdef IHM_DEBUG
                            emit displayMsgSignal("Séquence de mesure lancée ...", GREEN_COLOR);
                #endif
                            if(reachablePoints.size() > 1) orderingMeasurePoints(reachablePoints); ras(3);
                            computeRobotPositions(map4RobotPosition, v_reachablePoints, length_tool, area_width, area_length); ras(4);
                            robot.setPosition4Measure(v_robotPositionPoints);
                            calcPosition(v_initialRobotPosition, robotPositionByCam, area_width, area_length); ras(5);

                            std::cout << "\n\nposition initiale = " << v_initialRobotPosition << std::endl << std::endl;

                            for(uint i = 0; i < 3; i++){
                                if(v_cam[i]->rp->isRunning()){
                                    v_cam [i]->rp->requestInterruption();
                                    v_cam[i]->rp->wait();
                                }
                            }

                            generatePath(v_initialRobotPosition, robot);
                            generateFinalMap(map, v_reachablePoints, area_width);
                            updateIHM(map, static_cast<int>(v_reachablePoints.size()));

                #ifndef IHM_DEBUG
                            if(!ui->manualControl_checkbox->isChecked() || ui->robot_free_pos_with_measure->isChecked()){
                                sendMotorParamToNode();
                                delay_s(0.5);
                                sendHeightListToNode();
                                delay_s(0.5);
                                sendDelaysToNode();
                                delay_s(0.5);
                                sendVoltOutputTypeToNode();
                                delay_s(0.5);
                            }
                #endif

                #ifdef IHM_DEBUG
                            elapsedTime(ti, time_s, min, sec);
                            msgStr = "durée du traitement = " + to_string(time_s) + " s (" + to_string(min) + " min " + to_string(sec) + " s)";
                            msg = QString::fromStdString(msgStr);
                            emit displayMsgSignal(msg, GREEN_COLOR);
                #endif

                            msg = "Mesures lancées ...";
                            emit displayMsgSignal(msg, YELLOW_COLOR);

                            //Lancement du thread qui gère le déplacement du robot
                            if(!ui->manualControl_checkbox->isChecked()) v_rc->init(robot, v_generatedPath, v_cam, v_reachablePoints);
                            else{
                                if(ui->robot_free_pos_with_measure->isChecked()) v_rc->init(robot, v_generatedPath, v_cam, v_reachablePoints, true, true);
                                else v_rc->init(robot, v_generatedPath, v_cam, v_reachablePoints, false, true);
                            }
                            v_rc->start();
                        }
                        else{
                            emit displayMsgSignal("La coordonnée indiquée est inatteignable", RED_COLOR);
                        }
                    } //!5
                }
                else{
                    QMessageBox::warning(this, "", nodePCCheckMsg);
                } //!4
            }
            else{
                QMessageBox::critical(this, "", "Veuillez lancer l'application rosmaster.exe avant de continuer");
            } //!3
        }
        else{
            QMessageBox::critical(this, "", "La hauteur ne peut être inférieure à 100 mm");
        } //!2
    }
#ifdef CHECK_CAMERAS
    else{
        QMessageBox::critical(this, "", camCheckMsg);
    }
#endif
    //!1
}

void MainWindow::changeIP()
{
    QString msg;

    if(ui->IPButton->text() == "MODIFIER"){
        ui->IPButton->setText("VALIDER");
        ui->IPButton->setStyleSheet(validatePushButtonStyle);
        ui->IP_PC->setEnabled(true);
        ui->IP_PC->setStyleSheet(lineEditStyle_4);
        ui->IP_Robotino->setEnabled(true);
        ui->IP_Robotino->setStyleSheet(lineEditStyle_4);
    }
    else if(ui->IPButton->text() == "VALIDER"){
        ui->IPButton->setText("MODIFIER");
        ui->IPButton->setStyleSheet(pushButtonEnabledStyle);
        ui->IP_PC->setEnabled(false);
        ui->IP_PC->setStyleSheet(lineEditStyle_5);
        ui->IP_Robotino->setEnabled(false);
        ui->IP_Robotino->setStyleSheet(lineEditStyle_5);

        cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);
        std::string ipPC;
        std::string ipRobotino;

        fs["IP_PC"] >> ipPC;
        fs["IP_Robotino"] >> ipRobotino;
        if(ui->IP_PC->text().toStdString() != ipPC && ui->IP_Robotino->text().toStdString() == ipRobotino){
            if(validateIPAddress(ui->IP_PC->text())){
                saveParamIHM(true);

                generateBatchFile(ui->IP_PC->text().toStdString());
                msg = "L'adresse IP du PC a bien été modifiée";
                emit displayMsgSignal(msg, GREEN_COLOR);
            }
            else{
                QMessageBox::critical(this, "", "\n[ IP-PC ] : l'adresse renseignée est incorrecte.");
            }
        }
        else if(ui->IP_Robotino->text().toStdString() != ipRobotino && ui->IP_PC->text().toStdString() != ipPC){
            if(validateIPAddress(ui->IP_PC->text()) && validateIPAddress(ui->IP_Robotino->text())){
                saveParamIHM(true);

                generateBatchFile(ui->IP_PC->text().toStdString());
                msg = "L'adresse IP du PC a bien été modifiée";
                emit displayMsgSignal(msg, GREEN_COLOR);

                msg = "L'adresse IP du Robotino a bien été modifiée";
                emit displayMsgSignal(msg, GREEN_COLOR);
            }
            else{
                QMessageBox::critical(this, "", "\n[ IP-PC ] : l'adresse renseignée est incorrecte."
                                                "\n[ IP-Robotino ] : l'adresse renseignée est incorrecte.");
            }

        }
        else if(ui->IP_PC->text().toStdString() == ipPC && ui->IP_Robotino->text().toStdString() != ipRobotino){
            if(validateIPAddress(ui->IP_Robotino->text())){
                saveParamIHM(true);
                msg = "L'adresse IP du Robotino a bien été modifiée";
                emit displayMsgSignal(msg, GREEN_COLOR);
            }
            else{
                QMessageBox::critical(this, "", "\n[ IP-Robotino ] : l'adresse renseignée est incorrecte.");
            }
        }

        fs.release();
    }
}

void MainWindow::heightList()
{
    bool modif = false;
    bool heightOk = true;

    if(ui->highButton->text() == "MODIFIER"){
        ui->highButton->setText("VALIDER");
        ui->highButton->setStyleSheet(validatePushButtonStyle);

        if(ui->radioSavedHigh->isChecked()){
            ui->savedHigh->setReadOnly(false);
            ui->savedHigh->setStyleSheet(lineEditStyle_3);
            ui->radioSavedHigh->setStyleSheet(radioStyle_1);
        }
        else if(ui->radioOtherHigh->isChecked()){
            ui->otherHigh->setReadOnly(false);
            ui->otherHigh->setStyleSheet(lineEditStyle_3);
            ui->radioOtherHigh->setStyleSheet(radioStyle_1);
        }

        modif = false;
    }else if(ui->highButton->text() == "VALIDER"){
        ui->savedHigh->setReadOnly(true);
        ui->otherHigh->setReadOnly(true);
        ui->highButton->setText("MODIFIER");
        ui->highButton->setStyleSheet(pushButtonEnabledStyle);

        ui->savedHigh->setStyleSheet(lineEditStyle_2);
        ui->otherHigh->setStyleSheet(lineEditStyle_2);
        ui->radioSavedHigh->setStyleSheet(radioStyle_2);
        ui->radioOtherHigh->setStyleSheet(radioStyle_2);

        modif = true;
    }

    if(ui->radioSavedHigh->isChecked() && modif){
        int n;
        cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);

        std::vector<int> height;
        std::string tempStr;
        QString text = ui->savedHigh->text();
        QStringList strList = text.split(QChar(';'), Qt::SkipEmptyParts);

        if(strList.size() > 0){

            for(int i = 0; i < strList.size(); i++){
                bool ok = false;
                int val = strList[i].toInt(&ok);

                if(ok){
                    height.emplace_back(val);
                    if(val < 100) heightOk = false;
                }
            }

            if(height.size() > 1){
                for(uint i = 0; i < height.size() - 1; i++){
                    for(uint j = i+1; j < height.size(); j++){
                        if(height[i] > height[j]){
                            int temp = height[i];
                            height[i] = height[j];
                            height[j] = temp;
                        }
                    }
                }
            }

            fs["number_of_heights"] >> n;

            tempStr = "";
            for(uint i = 0; i < height.size(); i++){
                tempStr += to_string(height[i]);
                if(i < height.size()-1) tempStr += ";";
            }

            ui->savedHigh->setText(QString::fromStdString(tempStr));

            if(static_cast<int>(height.size()) != n){
                saveParamIHM();

                QString msg = "Les hauteurs ont été enregistrées";
                emit displayMsgSignal(msg, GREEN_COLOR);

            }else{
                std::string heights = ui->savedHigh->text().toStdString();
                std::string tempStr;
                bool ok = true;

                for(uint i = 0; i < height.size(); i++){
                    int h;
                    fs["h_" + to_string(i+1)] >> h;
                    if(h == height[i]) ok = false;
                    else break;
                }

                if(ok){
                    saveParamIHM();

                    QString msg = "Les hauteurs ont été enregistrées";
                    emit displayMsgSignal(msg, GREEN_COLOR);
                }
                else{
#ifdef IHM_DEBUG
                    QString msg = "RAS...";
                    emit displayMsgSignal(msg, GREEN_COLOR);
#endif
                }
            }
            fs.release();
        }
    }

    else if(ui->radioOtherHigh->isChecked() && modif){
        uint szString = static_cast<uint>(ui->otherHigh->text().size());

        if(szString > 0){
            std::vector<int> height;
            std::string tempStr;

            QString text = ui->otherHigh->text();
            QStringList strList = text.split(QChar(';'), Qt::SkipEmptyParts);

            if(strList.size() > 0){
                for(int i = 0; i < strList.size(); i++){
                    bool ok = false;
                    int val = strList[i].toInt(&ok);

                    if(ok){
                        height.emplace_back(val);
                        if(val < 100) heightOk = false;
                    }
                }

                for(uint i = 0; i < height.size() - 1; i++){
                    for(uint j = i+1; j < height.size(); j++){
                        if(height[i] > height[j]){
                            int temp = height[i];
                            height[i] = height[j];
                            height[j] = temp;
                        }
                    }
                }

                tempStr = "";
                for(uint i = 0; i < height.size(); i++){
                    tempStr += to_string(height[i]);
                    if(i < height.size()-1) tempStr += ";";
                }

                ui->otherHigh->setText(QString::fromStdString(tempStr));
            }
        }
    }

    if(!heightOk){
        QMessageBox::warning(this, "", "La hauteur ne peut être inférieure à 100 mm");
    }
}

void MainWindow::stop()
{
    sendStopToNode();

    for(uint i = 0; i < 3; i++){
        if(v_cam[i]->rp->isRunning()){
            v_cam [i]->rp->requestInterruption();
            v_cam[i]->rp->wait();
        }
    }

    if(!ui->manualControl_checkbox->isChecked()){
        cv::Mat frame_init = cv::Mat::zeros(static_cast<int>(2900 * scale),
                                            static_cast<int>(2775 * scale),
                                            CV_8UC3);
        cv::resize(frame_init, frame_init, cv::Size(), this->v_resizeWidthImage, this->v_resizeHeightImage);
        QPixmap qPix_init = Mat2Pixmap(frame_init);
        ui->dispWA->setPixmap(qPix_init);
    }

    if(!ui->manualControl_checkbox->isChecked()){
        ui->measure->setEnabled(true);
        ui->measure->setStyleSheet(pushButtonEnabledStyle);

        ui->stop->setEnabled(false);
        ui->stop->setStyleSheet(pushButtonDesabledStyle);
    }

    ui->IDs_cams->setEnabled(true);
    ui->IDs_cams->setStyleSheet(groupBoxStyle_7);

    ui->manualControl->setEnabled(true);
    ui->manualControl->setStyleSheet(groupBoxStyle_7);

    ui->calibration->setEnabled(true);
    ui->calibration->setStyleSheet(groupBoxStyle_7);

    ui->ros_master->setEnabled(true);
    ui->ros_master->setStyleSheet(pushButtonEnabledStyle);

    ui->threshold->setEnabled(true);
    ui->threshold->setStyleSheet(pushButtonEnabledStyle);

    ui->camerasButton->setEnabled(true);
    ui->camerasButton->setStyleSheet(pushButtonEnabledStyle);

    ui->gen_map->setEnabled(true);
    ui->gen_map->setStyleSheet(pushButtonEnabledStyle);

    ui->processedMeasures->display(0);
    ui->processedPoints->display(0);
    ui->numberOfMeasures->display(0);
    ui->numberOfPoints->display(0);

    if(ui->manualControl_checkbox->isChecked() && ui->robot_free_pos_go->text() == "STOP"){
        ui->robot_free_pos_go->setText("GO");
        if(ui->robot_free_positionning->isChecked()) ui->robot_free_pos_go->setStyleSheet(pushButtonEnabledStyle);
        else ui->robot_free_pos_go->setStyleSheet(pushButtonDesabledStyle);

        ui->manualControl_checkbox->setEnabled(true);
        ui->calibration_checkbox->setEnabled(true);
        ui->robot_free_pos_without_measure->setEnabled(true);
    }

    if(v_rc->isRunning()){
        v_rc->requestInterruption();
        v_rc->wait();
    }

    emit displayMsgSignal("... Mesures terminées", YELLOW_COLOR);
}
