#include "robotpositionthread.h"
#include "cameras.h"

RobotPositionThread::RobotPositionThread(QObject *parent) : QThread(parent)
{

}

RobotPositionThread::RobotPositionThread(Cameras* cam){
    this->v_cam = cam;
    this->v_mask = cam->wa->getMask();
    this->v_position[Position::RED_P].x = 0;
    this->v_position[Position::RED_P].y = 0;
    this->v_position[Position::GREEN_P].x = 0;
    this->v_position[Position::GREEN_P].y = 0;

    cv::FileStorage fs;

    fs.open(HSVLed_R, cv::FileStorage::READ);
    fs["h_min"] >> v_hmin_R; fs["h_max"] >> v_hmax_R;
    fs["s_min"] >> v_smin_R; fs["s_max"] >> v_smax_R;
    fs["v_min"] >> v_vmin_R; fs["v_max"] >> v_vmax_R;
    fs.release();

    fs.open(HSVLed_V, cv::FileStorage::READ);
    fs["h_min"] >> v_hmin_V; fs["h_max"] >> v_hmax_V;
    fs["s_min"] >> v_smin_V; fs["s_max"] >> v_smax_V;
    fs["v_min"] >> v_vmin_V; fs["v_max"] >> v_vmax_V;
    fs.release();
}

void RobotPositionThread::run(){
    try {
        //t_test = CLOCK_NOW;

        v_img = this->v_snap.clone();

        if(!v_img.empty()){
            cv::cvtColor(v_img, v_imgHSV, cv::COLOR_BGR2HSV);

            cv::inRange(v_imgHSV,
                        cv::Scalar(v_hmin_R, v_smin_R, v_vmin_R),
                        cv::Scalar(v_hmax_R, v_smax_R, v_vmax_R),
                        v_mask_leds);

            //LED ROUGE
            cv::findContours(v_mask_leds, v_contours, cv::RetrievalModes::RETR_LIST, cv::ContourApproximationModes::CHAIN_APPROX_NONE);

            if(v_contours.size() > 0){
                for(uint i = 0; i < v_contours.size() - 1; i++){
                    for(uint j = i+1; j < v_contours.size(); j++){
                        if(cv::contourArea(v_contours[i]) < cv::contourArea(v_contours[j])){
                            std::vector<cv::Point> temp = v_contours[i];
                            v_contours[i] = v_contours[j];
                            v_contours[j] = temp;
                        }
                    }
                }

                v_contour_R = v_contours[0];
                //contour.clear();
                //contour.push_back(contour_R);
                //cv::drawContours(mask_red_led, contour, -1, cv::Scalar(255,255,255), cv::FILLED);
                //cv::imshow("mask red led " + m_cam->getCamPosition(), mask_red_led); pause();
                //contour.clear();
                v_mask_leds.release();

                //LED VERTE
                cv::inRange(v_imgHSV,
                            cv::Scalar(v_hmin_V, v_smin_V, v_vmin_V),
                            cv::Scalar(v_hmax_V, v_smax_V, v_vmax_V),
                            v_mask_leds);

                cv::findContours(v_mask_leds, v_contours, cv::RetrievalModes::RETR_LIST, cv::ContourApproximationModes::CHAIN_APPROX_NONE);

                if(v_contours.size() > 0){
                    for(uint i = 0; i < v_contours.size() - 1; i++){
                        for(uint j = i+1; j < v_contours.size(); j++){
                            if(cv::contourArea(v_contours[i]) < cv::contourArea(v_contours[j])){
                                std::vector<cv::Point> temp = v_contours[i];
                                v_contours[i] = v_contours[j];
                                v_contours[j] = temp;
                            }
                        }
                    }

                    v_contour_G = v_contours[0];
                    //contour.push_back(contour_G);
                    //cv::drawContours(mask_green_led, contour, -1, cv::Scalar(255,255,255), cv::FILLED);
                    //cv::imshow("mask green led " + m_cam->getCamPosition(), mask_green_led); pause();
                    //contour.clear();
                    v_mask_leds.release();

                    for(uint i = 0; i < 3; i++){
                        this->v_position[i].x = 0;
                        this->v_position[i].y = 0;
                    }

                    for(uint i = 0; i < v_contour_R.size(); i++){
                        this->v_position[Position::RED_P].x += v_contour_R[i].x;
                        this->v_position[Position::RED_P].y += v_contour_R[i].y;
                    }

                    for(uint i = 0; i < v_contour_G.size(); i++){
                        this->v_position[Position::GREEN_P].x += v_contour_G[i].x;
                        this->v_position[Position::GREEN_P].y += v_contour_G[i].y;
                    }

                    this->v_position[Position::RED_P].x /= static_cast<int>(v_contour_R.size());
                    this->v_position[Position::RED_P].y /= static_cast<int>(v_contour_R.size());

                    this->v_position[Position::GREEN_P].x /= static_cast<int>(v_contour_G.size());
                    this->v_position[Position::GREEN_P].y /= static_cast<int>(v_contour_G.size());

                    /*cv::circle(img,
                               cv::Point(this->m_position[Position::GREEN_P].x, this->m_position[Position::GREEN_P].y),
                               radius,
                               cv::Scalar(0, 255, 0),
                               cv::FILLED);

                    cv::circle(img,
                               cv::Point(this->m_position[Position::RED_P].x, this->m_position[Position::RED_P].y),
                               radius,
                               cv::Scalar(0, 0, 255),
                               cv::FILLED);*/

                    //cv::imshow("leds sur image  " + m_cam->getCamPosition(), img); pause();
                }
                else{
                    this->v_position[Position::RED_P].x = POSITION_NOT_FOUND;
                    this->v_position[Position::RED_P].y = POSITION_NOT_FOUND;

                    this->v_position[Position::GREEN_P].x = POSITION_NOT_FOUND;
                    this->v_position[Position::GREEN_P].y = POSITION_NOT_FOUND;
                }
            }
            else{
                this->v_position[Position::RED_P].x = POSITION_NOT_FOUND;
                this->v_position[Position::RED_P].y = POSITION_NOT_FOUND;

                this->v_position[Position::GREEN_P].x = POSITION_NOT_FOUND;
                this->v_position[Position::GREEN_P].y = POSITION_NOT_FOUND;
            }

            //position du robot:
            if(this->v_position[Position::GREEN_P].x != POSITION_NOT_FOUND && this->v_position[Position::GREEN_P].y != POSITION_NOT_FOUND &&
               this->v_position[Position::RED_P].x != POSITION_NOT_FOUND && this->v_position[Position::RED_P].y != POSITION_NOT_FOUND){
                this->v_position[Position::CENTER].x = static_cast<int>((this->v_position[Position::GREEN_P].x +
                                                                        this->v_position[Position::RED_P].x)/2);
                this->v_position[Position::CENTER].y = static_cast<int>((this->v_position[Position::GREEN_P].y +
                                                                        this->v_position[Position::RED_P].y)/2);
            }else{
                this->v_position[Position::CENTER].x = POSITION_NOT_FOUND;
                this->v_position[Position::CENTER].y = POSITION_NOT_FOUND;
            }

            v_previousPosition.x = v_position[Position::CENTER].x;
            v_previousPosition.y = v_position[Position::CENTER].y;

            /*cv::circle(result,
                       cv::Point(this->m_position[Position::GREEN_P].x, this->m_position[Position::GREEN_P].y),
                       radius,
                       cv::Scalar(0, 255, 0),
                       cv::FILLED);

            cv::circle(result,
                       cv::Point(this->m_position[Position::RED_P].x, this->m_position[Position::RED_P].y),
                       radius,
                       cv::Scalar(0, 0, 255),
                       cv::FILLED);

            cv::circle(result,
                       cv::Point(this->m_position[Position::CENTER].x, this->m_position[Position::CENTER].y),
                       radius,
                       cv::Scalar(0, 255, 255),
                       cv::FILLED);*/

            //cv::imshow("result " + m_cam->getCamPosition(), result); pause();
            //cv::destroyAllWindows();

            v_mask_leds.release();
            v_contours.clear();
            //contour.clear();
            v_contour_R.clear();
            v_contour_G.clear();
        }
    }
    catch (...) {

    }
}

cv::Point *RobotPositionThread::getPosition(cv::Mat capture)
{
    this->v_snap = capture.clone();

    this->start();
    while(this->isRunning());
    return this->v_position;
}

cv::Mat RobotPositionThread::showRobotPosition(){
    cv::Mat result = cv::Mat::zeros(this->v_mask.size(), CV_8UC3);
    int radius = 3;

    cv::circle(result, this->v_position[Position::RED_P], radius, cv::Scalar(0, 0, 255), cv::FILLED);
    cv::circle(result, this->v_position[Position::GREEN_P], radius, cv::Scalar(0, 255, 0), cv::FILLED);
    cv::circle(result, this->v_position[Position::CENTER], radius, cv::Scalar(255, 0, 0), cv::FILLED);

    return result;
}
