#include "correctordialog.h"
#include "ui_correctordialog.h"

CorrectorDialog::CorrectorDialog(int winW, int winH, int type, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CorrectorDialog),
    v_winW(winW),
    v_winH(winH),
    v_type(type)
{
    ui->setupUi(this);

    std::string correctorName = "";

    if(v_type == CorrectorType::P) correctorName = "P";
    else if(v_type == CorrectorType::PI) correctorName = "PI";
    else if(v_type == CorrectorType::PID) correctorName = "PID";

    setWindowTitle("Réglage du correcteur : " + QString::fromStdString(correctorName));
    setStyleSheet(dialogStyle_1);
    setModal(true); //fenêtre bloquante

    configWidgets();
    loadValues();
    connects();
}

CorrectorDialog::~CorrectorDialog()
{
    delete ui;
}

void CorrectorDialog::configWidgets()
{
    QFont font = QFont("MS Shell Dlg 2", static_cast<int>(v_winH * 0.00741));

    ui->position_label->setStyleSheet(labelStyle_2);
    ui->orientation_label->setStyleSheet(labelStyle_2);

    if(v_type == CorrectorType::P){
        ui->kp_label->setStyleSheet(labelStyle_2);
        ui->ki_label->setStyleSheet(labelStyle_4);
        ui->kd_label->setStyleSheet(labelStyle_4);

        ui->kp_pos->setStyleSheet(doubleSpinBoxStyle_1);
        ui->kp_ori->setStyleSheet(doubleSpinBoxStyle_1);

        ui->ki_pos->setStyleSheet(doubleSpinBoxStyle_2);
        ui->ki_ori->setStyleSheet(doubleSpinBoxStyle_2);
        ui->ki_pos->setEnabled(false);
        ui->ki_ori->setEnabled(false);

        ui->kd_pos->setStyleSheet(doubleSpinBoxStyle_2);
        ui->kd_ori->setStyleSheet(doubleSpinBoxStyle_2);
        ui->kd_pos->setEnabled(false);
        ui->kd_ori->setEnabled(false);
    }
    else if(v_type == CorrectorType::PI){
        ui->kp_label->setStyleSheet(labelStyle_2);
        ui->ki_label->setStyleSheet(labelStyle_2);
        ui->kd_label->setStyleSheet(labelStyle_4);

        ui->kp_pos->setStyleSheet(doubleSpinBoxStyle_1);
        ui->kp_ori->setStyleSheet(doubleSpinBoxStyle_1);

        ui->ki_pos->setStyleSheet(doubleSpinBoxStyle_1);
        ui->ki_ori->setStyleSheet(doubleSpinBoxStyle_1);

        ui->kd_pos->setStyleSheet(doubleSpinBoxStyle_2);
        ui->kd_ori->setStyleSheet(doubleSpinBoxStyle_2);
        ui->kd_pos->setEnabled(false);
        ui->kd_ori->setEnabled(false);
    }
    else{
        ui->kp_label->setStyleSheet(labelStyle_2);
        ui->ki_label->setStyleSheet(labelStyle_2);
        ui->kd_label->setStyleSheet(labelStyle_2);

        ui->kp_pos->setStyleSheet(doubleSpinBoxStyle_1);
        ui->kp_ori->setStyleSheet(doubleSpinBoxStyle_1);

        ui->ki_pos->setStyleSheet(doubleSpinBoxStyle_1);
        ui->ki_ori->setStyleSheet(doubleSpinBoxStyle_1);

        ui->kd_pos->setStyleSheet(doubleSpinBoxStyle_1);
        ui->kd_ori->setStyleSheet(doubleSpinBoxStyle_1);
    }

    ui->validate_btn->setStyleSheet(pushButtonEnabledStyle);
    ui->cancel_btn->setStyleSheet(pushButtonEnabledStyle);

    setFixedSize(static_cast<int>(v_winW * 0.203600), static_cast<int>(v_winH * 0.152800));

    ui->position_label->setGeometry(static_cast<int>(v_winW * 0.039700),
                                    static_cast<int>(v_winH * 0.013900),
                                    static_cast<int>(v_winW * 0.029900),
                                    static_cast<int>(v_winH * 0.019500));
    ui->position_label->setFont(font);

    ui->orientation_label->setGeometry(static_cast<int>(v_winW * 0.089600),
                                       static_cast<int>(v_winH * 0.013900),
                                       static_cast<int>(v_winW * 0.038100),
                                       static_cast<int>(v_winH * 0.019500));
    ui->orientation_label->setFont(font);

    ui->kp_label->setGeometry(static_cast<int>(v_winW * 0.010900),
                              static_cast<int>(v_winH * 0.037100),
                              static_cast<int>(v_winW * 0.016300),
                              static_cast<int>(v_winH * 0.019500));
    ui->kp_label->setFont(font);

    ui->ki_label->setGeometry(static_cast<int>(v_winW * 0.010900),
                              static_cast<int>(v_winH * 0.074100),
                              static_cast<int>(v_winW * 0.016300),
                              static_cast<int>(v_winH * 0.019500));
    ui->ki_label->setFont(font);

    ui->kd_label->setGeometry(static_cast<int>(v_winW * 0.010900),
                              static_cast<int>(v_winH * 0.111200),
                              static_cast<int>(v_winW * 0.016300),
                              static_cast<int>(v_winH * 0.019500));
    ui->kd_label->setFont(font);

    ui->kp_pos->setGeometry(static_cast<int>(v_winW * 0.032600),
                            static_cast<int>(v_winH * 0.037100),
                            static_cast<int>(v_winW * 0.043500),
                            static_cast<int>(v_winH * 0.019500));
    ui->kp_pos->setFont(font);

    ui->kp_ori->setGeometry(static_cast<int>(v_winW * 0.086900),
                            static_cast<int>(v_winH * 0.037100),
                            static_cast<int>(v_winW * 0.043500),
                            static_cast<int>(v_winH * 0.019500));
    ui->kp_ori->setFont(font);

    ui->ki_pos->setGeometry(static_cast<int>(v_winW * 0.032600),
                            static_cast<int>(v_winH * 0.074100),
                            static_cast<int>(v_winW * 0.043500),
                            static_cast<int>(v_winH * 0.019500));
    ui->ki_pos->setFont(font);

    ui->ki_ori->setGeometry(static_cast<int>(v_winW * 0.086900),
                            static_cast<int>(v_winH * 0.074100),
                            static_cast<int>(v_winW * 0.043500),
                            static_cast<int>(v_winH * 0.019500));
    ui->ki_ori->setFont(font);

    ui->kd_pos->setGeometry(static_cast<int>(v_winW * 0.032600),
                            static_cast<int>(v_winH * 0.111200),
                            static_cast<int>(v_winW * 0.043500),
                            static_cast<int>(v_winH * 0.019500));
    ui->kd_pos->setFont(font);

    ui->kd_ori->setGeometry(static_cast<int>(v_winW * 0.086900),
                            static_cast<int>(v_winH * 0.111200),
                            static_cast<int>(v_winW * 0.043500),
                            static_cast<int>(v_winH * 0.019500));
    ui->kd_ori->setFont(font);

    ui->validate_btn->setGeometry(static_cast<int>(v_winW * 0.146600),
                                  static_cast<int>(v_winH * 0.046300),
                                  static_cast<int>(v_winW * 0.043500),
                                  static_cast<int>(v_winH * 0.023200));
    ui->validate_btn->setFont(font);

    ui->cancel_btn->setGeometry(static_cast<int>(v_winW * 0.146600),
                                static_cast<int>(v_winH * 0.083400),
                                static_cast<int>(v_winW * 0.043500),
                                static_cast<int>(v_winH * 0.023200));
    ui->cancel_btn->setFont(font);
}

void CorrectorDialog::connects()
{
    connect(ui->cancel_btn, &QPushButton::clicked, this, &QDialog::close);
    connect(ui->validate_btn, &QPushButton::clicked, this, &CorrectorDialog::save);
}

void CorrectorDialog::loadValues()
{
    cv::FileStorage fs(saveCorrectorSettings, cv::FileStorage::READ);

    if(fs.isOpened()){
        ui->kp_pos->setValue(fs["Kp_position"]);
        ui->kp_ori->setValue(fs["Kp_orientation"]);

        ui->ki_pos->setValue(fs["Ki_position"]);
        ui->ki_ori->setValue(fs["Ki_orientation"]);

        ui->kd_pos->setValue(fs["Kd_position"]);
        ui->kd_ori->setValue(fs["Kd_orientation"]);
    }

    fs.release();
}

void CorrectorDialog::save()
{
    QFile file(QString::fromStdString(saveCorrectorSettings));
    file.setPermissions(QFileDevice::WriteUser);

    cv::FileStorage fs(saveCorrectorSettings, cv::FileStorage::WRITE);

    if(fs.isOpened()){
        fs << "type" << v_type;

        fs << "Kp_position" << ui->kp_pos->value();
        fs << "Kp_orientation" << ui->kp_ori->value();

        fs << "Ki_position" << ui->ki_pos->value();
        fs << "Ki_orientation" << ui->ki_ori->value();

        fs << "Kd_position" << ui->kd_pos->value();
        fs << "Kd_orientation" << ui->kd_ori->value();

        fs.release();
        file.setPermissions(QFileDevice::ReadUser);
    }

    fs.release();

    close();
}

