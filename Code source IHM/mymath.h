#ifndef MYMATH_H
#define MYMATH_H

#include <opencv2/opencv.hpp>
#include <math.h>

class Robot;

namespace MyMath {
    //cv::Mat getRobotPosition(Robot robot, cv::Mat sensorsPosition, double theta);
    cv::Mat getPolarCoordinates(cv::Mat cartesianCoordiantes);
}

#endif // MYMATH_H
