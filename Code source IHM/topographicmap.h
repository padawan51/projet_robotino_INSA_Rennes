#ifndef TOPOGRAPHICMAP_H
#define TOPOGRAPHICMAP_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/fast_math.hpp>

#include "robot.h"
#include "utils.h"

class TopographicMap
{
public:
    TopographicMap(Robot m_robot);
    TopographicMap();
    inline Robot getRobot() { return v_robot; }
    bool isFree(cv::Mat robotPose);
    inline cv::Mat getMap() { return v_map; }
    inline cv::Mat getRealMap() { return v_realMap; }
    void setMap(cv::Mat m_map);
    void setRealMap(cv::Mat m_realMap);
private:
    Robot v_robot;
    cv::Mat v_map;
    cv::Mat v_realMap;
};

#endif // TOPOGRAPHICMAP_H
