#include "measurement.h"

Measurement::Measurement(int deltaX, int deltaY)
{
    this->v_deltaX = deltaX;
    this->v_deltaY = deltaY;
}

void Measurement::setDeltaX(int m_deltaX){
    this->v_deltaX = m_deltaX;
}

void Measurement::setDeltaY(int m_deltaY){
    this->v_deltaY = m_deltaY;
}

void Measurement::calcPoint(std::vector<cv::Point> &points, int x)
{
    cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);
    int AREA_HIGH;

    fs["working_area_length"] >> AREA_HIGH;
    fs.release();
    int y = AREA_HIGH/2;

    while(y < AREA_HIGH){
        points.emplace_back(cv::Point(y, x));
        y += v_deltaY;

        if(y >= AREA_HIGH){
            y = AREA_HIGH - 1;
            points.emplace_back(cv::Point(y, x));
            break;
        }
    }

    y = (AREA_HIGH/2) - v_deltaY;

    while(y > 0){
        points.emplace_back((cv::Point(y, x)));
        y -= v_deltaY;

        if(y <= 0){
            y = 0;
            points.emplace_back(cv::Point(y, x));
            break;
        }
    }
}

std::vector<cv::Point> Measurement::getMeasurePoints()
{
    std::vector<cv::Point> points;
    cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);
    int AREA_WIDTH;

    fs["working_area_width"] >> AREA_WIDTH;
    fs.release();

    int x = AREA_WIDTH/2;

    while(x < AREA_WIDTH){
        this->calcPoint(points, x);

        x += this->v_deltaX;

        if(x >= AREA_WIDTH){
            x = AREA_WIDTH - 1;
            this->calcPoint(points, x);
            break;
        }
    }

    x = (AREA_WIDTH/2) - v_deltaX;
    while(x > 0){
        this->calcPoint(points, x);

        x -= this->v_deltaX;

        if(x <= 0){
            x = 0;
            this->calcPoint(points, x);
            break;
        }
    }

    return points;
}
