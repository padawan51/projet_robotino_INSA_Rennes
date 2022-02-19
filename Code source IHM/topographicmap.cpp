#include "topographicmap.h"

TopographicMap::TopographicMap(Robot m_robot)
{
    this->v_robot = m_robot;
    this->v_map = cv::imread(pathTopographicMap, cv::IMREAD_GRAYSCALE);
    this->v_realMap = cv::imread(pathRealTopographicMap, cv::IMREAD_GRAYSCALE);
}

TopographicMap::TopographicMap(){}

void TopographicMap::setMap(cv::Mat m_map){
    this->v_map = m_map.clone();
}

void TopographicMap::setRealMap(cv::Mat m_realMap){
    this->v_realMap = m_realMap.clone();
}

bool TopographicMap::isFree(cv::Mat robotPose) {
    bool isFree = false;
    int rows = cvFloor((this->v_map.rows));
    int cols = cvFloor((this->v_map.cols));
    cv::Mat map_;
    cv::Mat realMap_;

    int imageRow = static_cast<int>(robotPose.ptr<double>(0)[0]*scale);
    int imageColumn = static_cast<int>(robotPose.ptr<double>(0)[1]*scale);

    int pixelValue;

    if(scale == 1.0){
        map_ = v_map.clone();
        realMap_ = v_realMap.clone();
    }
    else{
        //cv::resize(this->map, map_, cv::Size(), scale, scale);
        //cv::resize(this->realMap, realMap_, cv::Size(), scale, scale);
        map_ = v_map.clone();
        realMap_ = v_realMap.clone();
    }

    if ((imageRow > 0 && imageColumn > 0) && (imageRow < rows && imageColumn < cols)) {
        pixelValue = static_cast<int>(map_.ptr<uchar>(imageRow)[imageColumn]);

        if (pixelValue == 255) isFree = true;
    }

    cv::Mat robotToolZone = v_robot.getToolZone();

    if (isFree && robotToolZone.rows > 0) {
        cv::Mat point, point_double;
        cv::Mat vectorToAdd = (cv::Mat_<double>(1, 2) << 0.0, 0.0);
        double t = robotPose.ptr<double>(0)[2];

        for (int i = 0; i < robotToolZone.rows; i++) {
            double a = robotToolZone.ptr<double>(i)[0];

            point_double = (cv::Mat_<double>(1, 2) << robotPose.ptr<double>(0)[0], robotPose.ptr<double>(0)[1]);

            vectorToAdd.ptr<double>(0)[0] = cos(a + t) * (robotToolZone.ptr<double>(i)[1]);
            vectorToAdd.ptr<double>(0)[1] = sin(a + t) * (robotToolZone.ptr<double>(i)[1]);

            point_double += vectorToAdd;

            point = (cv::Mat_<int>(1, 2) << static_cast<int>(point_double.ptr<double>(0)[0]),
                                            static_cast<int>(point_double.ptr<double>(0)[1]));

            imageRow = static_cast<int>((point.ptr<int>(0)[0])*scale);
            imageColumn = static_cast<int>((point.ptr<int>(0)[1])*scale);

            if ((imageRow >= 0 && imageColumn >= 0) && (imageRow < rows && imageColumn < cols)) {
                pixelValue = static_cast<int>(realMap_.ptr<uchar>(imageRow)[imageColumn]);
                if (pixelValue == 0) {
                    isFree = false;
                    break;
                }
            }else{
                isFree = false;
            }
        }
    }

    return isFree;
}
