#ifndef PATH_H
#define PATH_H

#include <QtMath>
#include <opencv2/core.hpp>

#include <utility>

#include "pathplanning.h"

#include "measurement.h"
#include "cameras.h"
#include "topographicmap.h"
#include "robot.h"
#include "mymath.h"

class Path
{
public:
    Path(TopographicMap i_map, const Robot &i_robot);
    std::vector<cv::Mat> generate(cv::Mat initialPosition);

private:
    TopographicMap v_map;
    Robot v_robot;
};

#endif // PATH_H
