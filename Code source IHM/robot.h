#ifndef ROBOT_H
#define ROBOT_H

#include <WinSock2.h>
#include <opencv2/opencv.hpp>
#include <chrono>

#include <iostream>
#include <string>
#include "mymath.h"

#pragma comment(lib, "ws2_32.lib")

class Cameras;


class Robot
{
public:
    Robot() = default;
    Robot(int i_diameter, cv::Mat i_toolZone, double i_maxLinVel, double i_maxAngVel);
    ~Robot() = default;
    bool setVelocity(cv::Mat velocity);
    cv::Mat getPosition(Cameras &camerasClass);
    bool initSensorPosition();

    int getDiameter() const;
    void setDiameter(int value);

    double getMaxLinVel() const;
    void setMaxLinVel(double value);

    double getMaxAngVel() const;
    void setMaxAngVel(double value);

    cv::Mat getToolZone() const;
    void setToolZone(const cv::Mat &value);

    cv::Mat getSensorsPosition() const;
    void setSensorsPosition(const cv::Mat &value);

    std::vector<cv::Point> getPosition4Measure() const;
    void setPosition4Measure(const std::vector<cv::Point> &value);

private:
    int v_diameter;
    double v_maxLinVel;
    double v_maxAngVel;
    cv::Mat v_toolZone;
    cv::Mat v_sensorsPosition;
    std::vector<cv::Point> v_position4Measure;
private:
    cv::Mat generateToolZone(cv::Mat toolZone);
};

#endif // ROBOT_H
