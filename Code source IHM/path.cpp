#include "path.h"

Path::Path(TopographicMap i_map, const Robot &i_robot)
    :v_map(std::move(i_map)),
    v_robot(i_robot)
{}

std::vector<cv::Mat> Path::generate(cv::Mat initialPosition) {
    int count = 0;
    uint i = 0;

    double theta = 0.0;

    std::vector<cv::Point> measurementPoints;
    cv::Mat points;
    cv::Mat robotPosition = (cv::Mat_<double>(1,3) << 0.0, 0.0, 0.0);
    cv::Mat path;
    std::vector<cv::Mat> global_path;
    std::vector<cv::Mat> primitive_path;
    bool free;

    measurementPoints = v_robot.getPosition4Measure();

    while(i < measurementPoints.size()){
        theta = 0.0;
        while(theta <= M_PI){
            robotPosition.release();
            robotPosition = (cv::Mat_<double>(1,3) <<
                             measurementPoints[i].y,
                             measurementPoints[i].x,
                             -theta);

            free = v_map.isFree(robotPosition);
            if (free) {
                points.push_back(robotPosition);
                break;
            }

            robotPosition.release();
            robotPosition = (cv::Mat_<double>(1,3) <<
                             measurementPoints[i].y,
                             measurementPoints[i].x,
                             theta);

            free = v_map.isFree(robotPosition);
            if (free) {
                count++;
                points.push_back(robotPosition);
                break;
            }

            theta += (M_PI / 100);
        }
        i++;
    }

    int lengthPoints = points.rows;

    std::cout << "\nPoint 1/" << lengthPoints << std::endl;
    cv::Mat path1 = PathPlanning::generatePath(std::move(initialPosition), points.row(0), v_map);
    cv::Mat temp;
    count = 0;

    int lengthPath1 = (path1.rows);

    for(int k = 0; k < lengthPath1; k++){
        if (k > 0) {

            if (k < (lengthPath1 - 1)) {
                temp = (cv::Mat_<double>(1,4) <<
                          (path1.ptr<double>(k)[0]),
                          (path1.ptr<double>(k)[1]),
                          (path1.ptr<double>(k)[2]),
                          0);
                path.push_back(temp);
            }
            else{
                temp = (cv::Mat_<double>(1,4) <<
                        (path1.ptr<double>(k)[0]),
                        (path1.ptr<double>(k)[1]),
                        (path1.ptr<double>(k)[2]),
                          1);
                path.push_back(temp);
            }
        }
    }
    global_path.push_back(path);

    if (lengthPoints > 0) {
        for (int j = 0; j < lengthPoints - 1; j++) {
            std::cout << "\nPoint " << j+2 << "/" << lengthPoints << std::endl;
            cv::Mat newPath = PathPlanning::generatePath(points.row(j), points.row(j + 1), v_map);

            path.release();

            int lengthNewPath = newPath.rows;
            for (int k = 0; k < lengthNewPath; k++) {
                if (k > 0) {
                    if (k < lengthNewPath - 1) {
                        temp = (cv::Mat_<double>(1,4) <<
                                (newPath.ptr<double>(k)[0]),
                                (newPath.ptr<double>(k)[1]),
                                (newPath.ptr<double>(k)[2]),
                                  0);
                        path.push_back(temp);
                    }
                    else{
                        temp = (cv::Mat_<double>(1,4) <<
                                (newPath.ptr<double>(k)[0]),
                                (newPath.ptr<double>(k)[1]),
                                (newPath.ptr<double>(k)[2]),
                                  1);
                        path.push_back(temp);
                    }
                }
            }

            global_path.push_back(path);
        }
    }

    return global_path;
}
