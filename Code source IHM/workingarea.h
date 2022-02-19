#ifndef WORKINGAREA_H
#define WORKINGAREA_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

#include "utils.h"

//#define SHOW_IMG

class WorkingArea
{
public:
    WorkingArea(int camera, cv::Mat img);
    std::vector<cv::Point> getCorners();
    cv::Point getOrigin();
    cv::Mat getMask();
    void setMask(const cv::Mat &mask);

private:
    std::vector<cv::Point> v_corners;
    cv::Mat v_mask;
    cv::Point origin;
};

#endif // WORKINGAREA_H
