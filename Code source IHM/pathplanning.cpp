//#include "settings.h"
#include "pathplanning.h"
#include "mymath.h"

#include <cstdlib>
#include <ctime>

bool PathPlanning::verifyCollision(cv::Mat startPoint, cv::Mat endPoint, TopographicMap topographicMap) {
    cv::Mat vector = endPoint - startPoint;
    double errorPhi = endPoint.ptr<double>(0)[2] - startPoint.ptr<double>(0)[2];
    cv::Mat poseError = (cv::Mat_<double>(1, 3) << vector.ptr<double>(0)[0], vector.ptr<double>(0)[1], errorPhi);
    double vectorMagnitude = cv::norm(poseError, cv::NORM_L2);

    double n = vectorMagnitude / 1.0;

    if (n < 10.0) {
        n = 10.0;
    }

    bool collisionDetected = false;
    double t = 0;

    while (t <= 1) {
        cv::Mat point = startPoint + t * vector;

        if (point.ptr<double>(0)[2] < -M_PI) {
            std::cout << std::endl << "phi is passing -pi in pathPlanning verifyCollision" << std::endl;
        } else if (point.ptr<double>(0)[2] > M_PI) {
            std::cout << std::endl << "phi is passing +pi in pathPlanning verifyCollision" << std::endl;
        }

        if (!topographicMap.isFree(point)) {
            collisionDetected = true;
            break;
        }

        t += (1.0 / static_cast<double>(n));
    }

    return collisionDetected;
}

cv::Mat PathPlanning::generateNewNode(const cv::Mat &nearstNode, const cv::Mat &sample, double step) {
    cv::Mat vector = sample - nearstNode;
    vector = vector * (1.0/norm(vector, cv::NORM_L2));

    cv::Mat translationVector = (cv::Mat_<double>(1, 3) << vector.ptr<double>(0)[0], vector.ptr<double>(0)[1], 0.0);
    cv::Mat rotationVector = (cv::Mat_<double>(1, 3) << 0.0, 0.0, vector.ptr<double>(0)[2]);

    cv::Mat newNode = nearstNode + step * translationVector + 0.1 * rotationVector;

    return newNode;
}

void PathPlanning::getNearestNode(cv::Mat newNode, cv::Mat tree, int actualTree, cv::Mat& nearestNode, int& indexNearstNode) {
    int rowsCountTree = tree.rows;
    double minDistance = 1000000.0;
    double distance;

    for (int i = 0; i < rowsCountTree; i++) {
        if (static_cast<int>(tree.ptr<double>(i)[5]) == actualTree) {
            if (minDistance == 1000000.0) {
                indexNearstNode = i;
                nearestNode = (cv::Mat_<double>(1, 3) << tree.ptr<double>(i)[0], tree.ptr<double>(i)[1], tree.ptr<double>(i)[2]);
                minDistance = sqrt(
                            pow((newNode.ptr<double>(0)[0] - tree.ptr<double>(i)[0]), 2) +
                            pow((newNode.ptr<double>(0)[1] - tree.ptr<double>(i)[1]), 2) +
                            (800 * pow((newNode.ptr<double>(0)[2] - tree.ptr<double>(i)[2]), 2)));
            } else {
                distance = sqrt(
                            pow((newNode.ptr<double>(0)[0] - tree.ptr<double>(i)[0]), 2) +
                            pow((newNode.ptr<double>(0)[1] - tree.ptr<double>(i)[1]), 2) +
                            (800 * pow((newNode.ptr<double>(0)[2] - tree.ptr<double>(i)[2]), 2)));

                if (distance < minDistance) {
                    indexNearstNode = i;
                    nearestNode = (cv::Mat_<double>(1, 3) << tree.ptr<double>(i)[0], tree.ptr<double>(i)[1], tree.ptr<double>(i)[2]);
                    minDistance = distance;
                }
            }
        }
    }
}

cv::Mat PathPlanning::reWrite(int newNodeIndex, cv::Mat newNode, cv::Mat tree, int actualTree, double beta, const TopographicMap &topographicMap) {
    int rowsCountTree = tree.rows;
    double distance;

    for (int i = 0; i < rowsCountTree; i++) {
        if (static_cast<int>(tree.ptr<double>(i)[5]) == actualTree) {
            if (i != (newNodeIndex-1)) {
                cv::Mat Spoint = (cv::Mat_<double>(1, 3) <<
                                  tree.ptr<double>(i)[0],
                                  tree.ptr<double>(i)[1],
                                  tree.ptr<double>(i)[2]);

                distance = sqrt(pow((newNode.ptr<double>(0)[0] - tree.ptr<double>(i)[0]), 2) +
                                pow((newNode.ptr<double>(0)[1] - tree.ptr<double>(i)[1]), 2) +
                                (800 * pow((newNode.ptr<double>(0)[2] - tree.ptr<double>(i)[2]), 2)));

                if (distance < beta) {
                    distance = distance + tree.ptr<double>(newNodeIndex-1)[4];
                    if (distance < tree.ptr<double>(i)[4]) {
                        if (!PathPlanning::verifyCollision(Spoint, newNode, topographicMap)) {
                            tree.ptr<double>(i)[3] = newNodeIndex - 1;
                            tree.ptr<double>(i)[4] = distance;
                        }
                    }
                }
            }
        }
    }
    return tree;
}

void PathPlanning::getParent(cv::Mat newNode, cv::Mat tree, int actualTree, double beta,
                             const TopographicMap &topographicMap, int& idParent, double& minDistance) {
    int rowsCountTree = tree.rows;
    double distance;

    idParent = -1;
    minDistance = 1000000.0;
    for (int i = 0; i < rowsCountTree; i++) {
        if (static_cast<int>(tree.ptr<double>(i)[5]) == actualTree) {
           cv::Mat Spoint = (cv::Mat_<double>(1, 3) << tree.ptr<double>(i)[0], tree.ptr<double>(i)[1], tree.ptr<double>(i)[2]);
            distance = sqrt(
                        pow((newNode.ptr<double>(0)[0] - tree.ptr<double>(i)[0]), 2) +
                        pow((newNode.ptr<double>(0)[1] - tree.ptr<double>(i)[1]), 2) +
                        (800 * pow((newNode.ptr<double>(0)[2] - tree.ptr<double>(i)[2]), 2)));

            if (distance < beta){
                distance = distance + tree.ptr<double>(i)[4];

                if (minDistance == 1000000.0) {
                    if (!PathPlanning::verifyCollision(Spoint, newNode, topographicMap)) {
                        idParent = i;
                        minDistance = distance;
                    }
                }
                else {
                    if (distance < minDistance) {
                        if (!PathPlanning::verifyCollision(Spoint, newNode, topographicMap)) {
                            idParent = i;
                            minDistance = distance;
                        }
                    }
                }
            }
        }
    }
}

cv::Mat PathPlanning::getPrimitivePath(cv::Mat tree, int indexStartTree, int indexEndTree) {
    bool finished = false;
    int index = indexStartTree;
    int parentIndex;
    cv::Mat path;
    cv::Mat newPath;

    while (!finished) {
        parentIndex = static_cast<int>(tree.ptr<double>(index)[3]);
        newPath = (cv::Mat_<double>(1, 3) << tree.ptr<double>(index)[0], tree.ptr<double>(index)[1], tree.ptr<double>(index)[2]);
        path.push_back(newPath);

        if (parentIndex != -1) index = parentIndex;
        else finished = true;
    }

    flip(path, path, 0);
    finished = false;
    index = indexEndTree;

    while (!finished) {
        parentIndex = static_cast<int>(tree.ptr<double>(index)[3]);
        newPath = (cv::Mat_<double>(1, 3) << tree.ptr<double>(index)[0], tree.ptr<double>(index)[1], tree.ptr<double>(index)[2]);
        path.push_back(newPath);

        if (parentIndex != -1) index = parentIndex;
        else finished = true;
    }

    return path;
}

cv::Mat PathPlanning::optimizePath(cv::Mat primitivePath, const TopographicMap &topographicMap) {
    cv::Mat path;
    int index = 0;
    cv::Mat rowToAdd;
    cv::Mat rowVerifyCollision_i;
    cv::Mat rowVerifyCollision_j;

    rowToAdd = (cv::Mat_<double>(1, 3) <<
                primitivePath.ptr<double>(0)[0],
                primitivePath.ptr<double>(0)[1],
                primitivePath.ptr<double>(0)[2]);

    path.push_back(rowToAdd);

    for (int i = 0; i < (primitivePath.rows - 1); i++) {
        if (i >= index) {
            rowVerifyCollision_i = (cv::Mat_<double>(1, 3) <<
                                    primitivePath.ptr<double>(i)[0],
                                    primitivePath.ptr<double>(i)[1],
                                    primitivePath.ptr<double>(i)[2]);

            for (int j = i + 1; j < primitivePath.rows; j++) {
                rowVerifyCollision_j = (cv::Mat_<double>(1, 3) <<
                                        primitivePath.ptr<double>(j)[0],
                                        primitivePath.ptr<double>(j)[1],
                                        primitivePath.ptr<double>(j)[2]);

                if(PathPlanning::verifyCollision(rowVerifyCollision_i, rowVerifyCollision_j, topographicMap)) {
                    rowToAdd = (cv::Mat_<double>(1, 3) <<
                                primitivePath.ptr<double>(j-1)[0],
                                primitivePath.ptr<double>(j-1)[1],
                                primitivePath.ptr<double>(j-1)[2]);

                    path.push_back(rowToAdd);
                    index = j - 1;
                    break;
                }

                if (j == (primitivePath.rows - 1)) {
                    rowToAdd = (cv::Mat_<double>(1, 3) <<
                                primitivePath.ptr<double>(j)[0],
                                primitivePath.ptr<double>(j)[1],
                                primitivePath.ptr<double>(j)[2]);

                    path.push_back(rowToAdd);
                    index = j;
                    break;
                }
            }
        }
    }

    return path;
}

cv::Mat PathPlanning::generatePath(cv::Mat startPoint, cv::Mat endPoint, TopographicMap topographicMap) {
    // Setting up constants

    cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);
    double step = 20.0;
    double beta = 50.0;
    double width;
    double height;
    int count = 2;
    int actualTree = 1;
    int k = 0;

    fs["working_area_width"] >> width;
    fs["working_area_length"] >> height;
    fs.release();

    auto max_x = static_cast<int>(height);
    auto max_y = static_cast<int>(width);
    int min_om = -314;
    int max_om = 314;

    cv::Mat tree;
    cv::Mat map = getTopographicMap();
    cv::Mat rowToAdd;
    cv::Mat optimizedPath;

    CLOCK_TIME_POINT ti;
    double time_s;
    int min, sec;

    ti = CLOCK_NOW;

    if (!topographicMap.isFree(startPoint) || !topographicMap.isFree(endPoint)) {
        std::cout << std::endl << "ERROR: One position in the path is not free" << std::endl;
        std::cout << "\nstart point = " << startPoint << std::endl;
        std::cout << "\nend point = " << endPoint << std::endl;
        return cv::Mat();
    }

    // Generating RRT * -double - optimized path

    rowToAdd = (cv::Mat_<double>(1, 6) <<
                startPoint.ptr<double>(0)[0],
                startPoint.ptr<double>(0)[1],
                startPoint.ptr<double>(0)[2],
                -1,
                0,
                1);

    tree.push_back(rowToAdd);
    rowToAdd.release();
    rowToAdd = (cv::Mat_<double>(1, 6) <<
                endPoint.ptr<double>(0)[0],
                endPoint.ptr<double>(0)[1],
                endPoint.ptr<double>(0)[2],
                -1,
                0,
                0);

    tree.push_back(rowToAdd);

    cv::Mat sample;

    int rand_x;
    int rand_y;
    double rand_om;
    int index;
    int idParent;
    int indexNearestNode;
    double distance;
    int x_min, y_min;
    int ray = 600;

    x_min = static_cast<int>(startPoint.ptr<double>(0)[0]) - ray;
    y_min = static_cast<int>(startPoint.ptr<double>(0)[1]) - ray;

    std::uniform_int_distribution<int> distributionX(0,max_x);
    std::uniform_int_distribution<int> distributionY(0,max_y);
    std::uniform_int_distribution<int> distributionT(0,2*max_om);

    int iteration = 2000;
    while(++k <= iteration){
        bool solutionFound;

        cv::Mat nearstNode;
        cv::Mat newNode;
        cv::Mat nearestNodeOtherTree;

        //initialize random seed
        srand(static_cast<uint>(time(nullptr)));

        do{
            do{
                rand_x = distributionX(*QRandomGenerator::global());
                rand_y = distributionY(*QRandomGenerator::global());
            }while(rand_x < 0 && rand_x > width && rand_y < 0 && rand_y > height);

            rand_om = (distributionT(*QRandomGenerator::global()) + min_om)/100.0;

            sample = (cv::Mat_<double>(1,3) << rand_x, rand_y, rand_om);
        }while(!topographicMap.isFree(sample));

        PathPlanning::getNearestNode(sample, tree, actualTree, nearstNode, index);
        newNode = PathPlanning::generateNewNode(nearstNode, sample, step);
        PathPlanning::getParent(newNode, tree, actualTree, beta, topographicMap, idParent, distance);

        if(idParent != -1){
            count++;

            rowToAdd = (cv::Mat_<double>(1,6) <<
                        newNode.ptr<double>(0)[0],
                        newNode.ptr<double>(0)[1],
                        newNode.ptr<double>(0)[2],
                        idParent,
                        distance,
                        actualTree);

            tree.push_back(rowToAdd);
            tree = PathPlanning::reWrite(count, newNode, tree, actualTree, beta, topographicMap);

            if(actualTree == 1){
                PathPlanning::getNearestNode(newNode, tree, 0, nearestNodeOtherTree, indexNearestNode);
                actualTree = 0;
            }else{
                PathPlanning::getNearestNode(newNode, tree, 1, nearestNodeOtherTree, indexNearestNode);
                actualTree = 1;
            }

            bool collision = PathPlanning::verifyCollision(nearestNodeOtherTree, newNode, topographicMap);
            if(!collision){
                cv::Mat primitivePath;

                if(actualTree == 1) {
                    primitivePath = PathPlanning::getPrimitivePath(tree, indexNearestNode, count-1);
                }
                else{
                    primitivePath = PathPlanning::getPrimitivePath(tree, count-1, indexNearestNode);
                }

                optimizedPath = PathPlanning::optimizePath(primitivePath, topographicMap);
                solutionFound = true;

                break;
            }
        }
    }

    elapsedTime(ti, time_s, min, sec);
    std::cout << "\t>>> duration = " << time_s << " s (" << min << " mn " << sec << " s)" << std::endl;

    return optimizedPath;
}

