#include "workingarea.h"

using namespace cv;

WorkingArea::WorkingArea(int camera, cv::Mat img)
{
    std::string hsvFile;
    std::string hsvOriginFile;

    //cv::Mat wa = cv::imread(pathEmptyArea[camera]);
    cv::Mat wa = img.clone();
    cv::Mat waHSV;
    cv::Mat mask3P;
    cv::Mat maskOri;
    cv::Mat mask_zone;
    cv::FileStorage fs;
    std::vector<std::vector<cv::Point>> contours[2];
    std::vector<std::vector<cv::Point>> temp_contours[2];
    std::vector<cv::Point> bary;
    std::vector<int> sum_x[2];
    std::vector<int> sum_y[2];

    int whereIsOrigin = -1;

    QFile file(QString::fromStdString(maskArea[camera]));

#ifdef SHOW_IMG
    std::string win = "window";

    namedWindow(win);
    imshow(win, wa); pause();
#endif
    cvtColor(wa, waHSV, cv::COLOR_BGR2HSV);

    fs.open(HSVConfigFile[camera], cv::FileStorage::READ);
    cv::inRange(waHSV,
                cv::Scalar(fs["h_min"], fs["s_min"], fs["v_min"]),
                cv::Scalar(fs["h_max"], fs["s_max"], fs["v_max"]),
                mask3P);
    fs.release();

    fs.open(HSVConfigOriginFile[camera], cv::FileStorage::READ);
    cv::inRange(waHSV,
                cv::Scalar(fs["h_min"], fs["s_min"], fs["v_min"]),
                cv::Scalar(fs["h_max"], fs["s_max"], fs["v_max"]),
                maskOri);
    fs.release();

#ifdef  SHOW_IMG
    imshow(win, waHSV); pause();
    imshow(win, mask3P); pause();
    imshow(win, maskOri); pause();
#endif

    cv::findContours(mask3P, contours[0], cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    cv::findContours(maskOri, contours[1], cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    for (uint i = 0; i < contours[0].size(); i++) {
        if (contourArea(contours[0][i]) > 100) temp_contours[0].push_back(contours[0][i]);
    }

    for (uint i = 0; i < contours[1].size(); i++) {
        if (contourArea(contours[1][i]) > 100) temp_contours[1].push_back(contours[1][i]);
    }

#ifdef SHOW_IMG
    cv::Mat img = mask3P.clone();
    cv::Mat imgO = maskOri.clone();
    cv::Mat imgGlob;

    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    cv::cvtColor(imgO, imgO, cv::COLOR_GRAY2BGR);

    cv::drawContours(img, temp_contours[0], -1, cv::Scalar(0, 0, 255), 2);
    cv::drawContours(imgO, temp_contours[1], -1, cv::Scalar(255, 0, 0), 2);
    imshow(win, img); pause();
    imshow(win, imgO); pause();

    imgGlob = img + imgO;
    imshow(win, imgGlob); pause();
#endif

    for (uint i = 0; i < temp_contours[0].size(); i++) {
        int sumx = 0;
        int sumy = 0;
        for (uint j = 0; j < temp_contours[0][i].size(); j++) {
            sumx += temp_contours[0][i][j].x;
            sumy += temp_contours[0][i][j].y;
        }

        sumx /= static_cast<int>(temp_contours[0][i].size());
        sumy /= static_cast<int>(temp_contours[0][i].size());
        bary.emplace_back(cv::Point(sumx, sumy));
    }

    for (uint i = 0; i < temp_contours[1].size(); i++) {
        int sumx = 0;
        int sumy = 0;
        for (uint j = 0; j < temp_contours[1][i].size(); j++) {
            sumx += temp_contours[1][i][j].x;
            sumy += temp_contours[1][i][j].y;
        }

        sumx /= static_cast<int>(temp_contours[1][i].size());
        sumy /= static_cast<int>(temp_contours[1][i].size());
        bary.emplace_back(cv::Point(sumx, sumy));
    }




    bary = sortCorners(bary);

    if(camera == Camera::DROITE) whereIsOrigin = Corner::TOP_LEFT;
    else if(camera == Camera::GAUCHE) whereIsOrigin = Corner::BOTTOM_RIGHT;
    else if(camera == Camera::MILIEU) whereIsOrigin = Corner::TOP_RIGHT;

#ifdef SHOW_IMG
    cv::Scalar color;
    int radius = 3;

    for (int i = 0; i < 4; i++) {
        if (i == whereIsOrigin) {
            color = cv::Scalar(0, 0, 255);
        }
        else {
            color = cv::Scalar(0, 0, 0);
        }

        circle(imgGlob, bary[static_cast<uint>(i)], radius, color, cv::FILLED);

    }
    imshow(win, imgGlob); pause();
#endif

    mask_zone = workingAreaDelimiter(IMG_H, IMG_W, bary);

    this->v_mask = mask_zone.clone();
    this->origin = bary[static_cast<uint>(whereIsOrigin)];
    this->v_corners = bary;

    file.setPermissions(QFileDevice::WriteUser);
    fs.open(maskArea[camera], FileStorage::WRITE);
    fs << "mask" << this->v_mask;
    fs.release();
    file.setPermissions(QFileDevice::ReadUser);

#ifdef SHOW_IMG
    cv::destroyAllWindows();
#endif
}

std::vector<cv::Point> WorkingArea::getCorners(){
    return this->v_corners;
}

cv::Point WorkingArea::getOrigin(){
    return this->origin;
}

cv::Mat WorkingArea::getMask(){
    return this->v_mask.clone();
}

void WorkingArea::setMask(const cv::Mat &mask){
    this->v_mask = mask.clone();
}
