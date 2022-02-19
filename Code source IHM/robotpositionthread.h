#ifndef ROBOTPOSITION_H
#define ROBOTPOSITION_H

#include <QThread>
#include <QMutex>

#include <opencv2/core.hpp>

#include "acquisitionthread.h"
#include "utils.h"

class Cameras;

class RobotPositionThread : public QThread
{
    Q_OBJECT
public:
    explicit RobotPositionThread(QObject *parent = nullptr);
    RobotPositionThread(Cameras *cam);
    ~RobotPositionThread() = default;
    void run();
    cv::Point* getPosition(cv::Mat capture = cv::Mat());
    cv::Mat showRobotPosition();

private:
    cv::Point v_position[3];
    cv::Point v_previousPosition;
    cv::Mat v_mask;
    Cameras *v_cam;

    cv::Mat v_img;
    cv::Mat v_imgHSV;
    cv::Mat v_mask_leds;
    cv::Mat v_snap;
    std::vector<std::vector<cv::Point>> v_contours;
    //std::vector<std::vector<cv::Point>> contour;
    std::vector<cv::Point> v_contour_R;
    std::vector<cv::Point> v_contour_G;
    int v_hmin_R, v_smin_R, v_vmin_R;
    int v_hmax_R, v_smax_R, v_vmax_R;
    int v_hmin_V, v_smin_V, v_vmin_V;
    int v_hmax_V, v_smax_V, v_vmax_V;
    /*double test_sec;
    int test_min, test_s;
    CLOCK_TIME_POINT t_test;*/
};

#endif // ROBOTPOSITION_H
