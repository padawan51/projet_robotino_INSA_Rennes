#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <opencv2/core.hpp>
#include "utils.h"

class Measurement
{
public:
    Measurement(int deltaX, int deltaY);
    Measurement() = default;
    std::vector<cv::Point> getMeasurePoints();
    void setDeltaX(int m_deltaX);
    void setDeltaY(int m_deltaY);
    void calcPoint(std::vector<cv::Point>& points, int x);

private:
    int v_deltaX;
    int v_deltaY;
};

#endif // MEASUREMENT_H
