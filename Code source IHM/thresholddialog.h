#ifndef THRESHOLDDIALOG_H
#define THRESHOLDDIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QFont>
#include <QWindow>
#include <QCloseEvent>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include "utils.h"
#include "acquisitionthread.h"
#include "css.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class ThresholdDialog;
}
QT_END_NAMESPACE

class ThresholdDialog : public QDialog
{
    Q_OBJECT

public:
    ThresholdDialog(int winW, int winH, QWidget *parent = nullptr);
    ~ThresholdDialog();

private: //properties
    Ui::ThresholdDialog *ui;

    std::vector<std::vector<int>> v_hue;
    std::vector<std::vector<int>> v_sat;
    std::vector<std::vector<int>> v_val;

    std::vector<std::vector<int>> v_hue_ori;
    std::vector<std::vector<int>> v_sat_ori;
    std::vector<std::vector<int>> v_val_ori;

    cv::Mat v_img;
    cv::Mat v_imgCaptureL;
    cv::Mat v_imgCaptureM;
    cv::Mat v_imgCaptureR;
    cv::Mat v_img_mask;
    int v_winW;
    int v_winH;
    double v_fx;
    double v_fy;
    AcquisitionThread *v_cam_L;
    AcquisitionThread *v_cam_M;
    AcquisitionThread *v_cam_R;

private: //methods
    void thresholdChanged(int h_min, int h_max,
                         int s_min, int s_max,
                         int v_min, int v_max,
                         const cv::Mat &image);
    void configWidgets();
    void connects();
    void manageCameras();
    void init();
    void closeEvent(QCloseEvent *event);
public slots:
    void setHSVValue();
    void openFile();
    void useCamera();
    void useImageFile();
    void threshChanged();
    void raz();
    void changeHSV();
    void configOriginThresh();
    void canceled();
    void dispFrame(cv::Mat frame);
    void capture();
    void displayView();
public: signals:
    void displayCamView();
};

#endif // THRESHOLDDIALOG_H
