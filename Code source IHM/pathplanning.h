#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <QtMath>
#include <QRandomGenerator64>
#include <random>
#include <opencv2/opencv.hpp>
#include "measurement.h"
#include "topographicmap.h"
#include "utils.h"

namespace PathPlanning {
    cv::Mat generatePath(cv::Mat startPoint, cv::Mat endPoint, TopographicMap topographicMap);
    cv::Mat generateNewNode(const cv::Mat &nearstNode, const cv::Mat &sample, double step);
    void getNearestNode(cv::Mat newNode, cv::Mat tree, int actualTree, cv::Mat& nearstNode, int& indexNearstNode);
    cv::Mat optimizePath(cv::Mat primitivePath, const TopographicMap &topographicMap);
    cv::Mat getPrimitivePath(cv::Mat tree, int indexStartTree, int indexEndTree);
    void getParent(cv::Mat newNode, cv::Mat tree, int actualTree, double beta, const TopographicMap &topographicMap, int& idParent, double& minDistance);
    cv::Mat reWrite(int newNodeIndex, cv::Mat newNode, cv::Mat tree, int actualTree, double beta, const TopographicMap &topographicMap);
    bool verifyCollision(cv::Mat startPoint, cv::Mat endPoint, TopographicMap topographicMap);
};

#endif // PATHPLANNING_H
