#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <WinSock2.h>
#pragma comment(lib, "Ws2_32.lib")

#include <QObject>
#include <QThread>
#include <xlsxdocument.h>
#include <string>

#include "robot.h"
#include "cameras.h"
#include <iostream>

#include "utils.h"

#include <opencv2/opencv.hpp>

#define MOVE

typedef int socklen_t;

struct Data{
    std::vector<int32> heights;
    std::vector<float32> airSpeedMean;
    std::vector<float32> temperatureMean;
    std::vector<float32> airSpeedDispersion;
    std::vector<float32> temperatureDispersion;
};

typedef struct Data Data;

class RobotControllerThread: public QThread
{
    Q_OBJECT
public:
    RobotControllerThread();
    ~RobotControllerThread() = default;

    void run();
    void init(const Robot &i_robot,
              std::vector<cv::Mat> i_path,
              std::vector<Cameras*> i_cam,
              std::vector<cv::Point> reachablePoints,
              bool takeMeasure = true,
              bool manualControl = false);
private:
    Robot v_robot;
    std::vector<cv::Mat> v_path;
    std::vector<Cameras*> v_cam;
    std::vector<cv::Point> v_reachablePoints;
    bool v_isInit;
    bool v_manualControl;
    bool v_takeMeasure;
    double v_Kp;
    double v_Kp_phi;
    std::string v_msg;
private:
    void moveInPath();
    void calcPosition(cv::Mat &position);
    void takeMeasures();
    bool waitEndOfMeasurements(std::vector<Results> &rawData);
    void data(Results &rawData, std::vector<Data> &reduceData);
    void saveData(std::vector<Results> &rawData, std::vector<Data> &reduceData, std::vector<cv::Point2d> &measurementPoints);
public: signals:
    void realRobotPosition(cv::Mat position);
    void measurementPoint(int num);
    void processedMeasure(int num);
    void measureFinish();
};

#endif // ROBOTCONTROLLER_H
