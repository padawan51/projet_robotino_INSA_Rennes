//#include "settings.h"
#include <opencv2/core/affine.hpp>
#include "mymath.h"
#include "robot.h"

using namespace cv;

/*Mat MyMath::getRobotPosition(Robot robot, Mat sensorsPosition, double theta) {
    Mat robotCenter = Mat::ones(3, 1, CV_64F);
    Mat robotSensorsPosition = robot.getSensorsPosition();

    #ifdef DEBUG
        std::cout << std::endl << std::endl << "----- Calculating robot center position with sensors position -----";
        std::cout << std::endl << "INPUTS: ";
        std::cout << std::endl << "   - sensorsPositionInRobotCoordinateSystem: " << std::endl << sensorsPosition;
        std::cout << std::endl << "   - sensorsPositionInChamberCoordinateSystem: " << std::endl << robotSensorsPosition;
        std::cout << std::endl << "   - theta: " << theta;
    #endif

    double neutralTheta = atan2(robotSensorsPosition.ptr<double>(1)[0], robotSensorsPosition.ptr<double>(0)[0]);
    double lengthSensorsVector = sqrt(robotSensorsPosition.ptr<double>(0)[0] * robotSensorsPosition.ptr<double>(0)[0] +
                                      robotSensorsPosition.ptr<double>(1)[0] * robotSensorsPosition.ptr<double>(1)[0]);

   // Mat robotCenter = Mat::ones(3, 1, CV_64F);

    robotCenter.ptr<double>(0)[0] = sensorsPosition.ptr<double>(0)[0] - lengthSensorsVector * cos(theta + neutralTheta);
    robotCenter.ptr<double>(1)[0] = sensorsPosition.ptr<double>(1)[0] - lengthSensorsVector * sin(theta + neutralTheta);
    robotCenter.ptr<double>(2)[0] = theta;

    #ifdef DEBUG
        std::cout << std::endl << "OUTPUTS: ";
        std::cout << std::endl << "   - robotCenter: " << std::endl << robotCenter << std::endl;
    #endif

    return robotCenter;
}*/

cv::Mat MyMath::getPolarCoordinates(cv::Mat cartesianCoordinates) {
    double phi = atan2(cartesianCoordinates.ptr<double>(0)[1], cartesianCoordinates.ptr<double>(0)[0]);
    double r;

    if (cartesianCoordinates.ptr<double>(0)[0] == 0.0) {
        r = abs(cartesianCoordinates.ptr<double>(0)[1]);
    }else if (cartesianCoordinates.ptr<double>(0)[0] == 0.0 && cartesianCoordinates.ptr<double>(0)[1] == 0.0){
        r = 0.0;
    }else{
        r = abs(cartesianCoordinates.ptr<double>(0)[0] / cos(phi));
    }

    cv::Mat polarCoordinates = (cv::Mat_<double>(1, 2) << r, phi);

    return polarCoordinates;

    //return cv::Mat();
}
