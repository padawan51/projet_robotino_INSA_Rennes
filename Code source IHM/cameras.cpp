#include "cameras.h"

using namespace cv;
using namespace std;

Cameras::Cameras(int idCam, std::string camPosition, bool useCamera){
    cv::VideoCapture cap;
    cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);

    this->v_id = idCam;
    this->v_useCamera = useCamera;
    this->wa = nullptr;
    this->rp = nullptr;

    if(this->v_useCamera && (v_id != -1)){
        this->acq = new AcquisitionThread(&v_id);
        this->acq->setIsSnapshot(true);
        this->v_camIsPresent = isCameraPresent();
    }
    else if(!this->v_useCamera && (v_id!= -1)) this->v_camIsPresent = isCameraPresent();
    else v_camIsPresent = false;

    this->v_initOk = false;
    this->v_camPosition = camPosition;
    v_codeCam = (camPosition == "gauche") ? Camera::GAUCHE :
              ((camPosition == "droite") ? Camera::DROITE :
              ((camPosition == "milieu") ? Camera::MILIEU : -1));

    //Récupération de la distance des leds par rapport au sol
    fs["leds_height"] >> v_height_led;
    fs.release();
}

bool Cameras::hasCalibFile(){
    bool exist = false;

    if(v_codeCam >= 0){
        QDir dir = QDir::current();
        string msg;

        this->v_hasIntrinsic = dir.exists(QString(intrinsicsCameraParametersFile[v_codeCam].c_str()));

        if(!this->v_hasIntrinsic){
            msg = "Le fichier des parametres intrinseques de la camera " + this->v_camPosition + " n'existe pas";
            sendMessage(QString::fromStdString(msg));
        }

        exist = this->v_hasIntrinsic;
        if(exist) this->v_initOk = true;
    }
    else sendMessage("[CAMERA " + QString::fromStdString(v_camPosition).toUpper() + "] Camera introuvable (CAM ID : " + QString::number(v_codeCam) + ")");

    return exist;
}

bool Cameras::isInit(){
    return this->v_initOk;
}

void Cameras::calcIntrinsicParams(){

    cv::FileStorage fs(patternConfig, cv::FileStorage::READ);

    if(fs.isOpened()){
        std::vector<cv::Mat> img_list;
        cv::Mat cameraMatrix = (Mat_<float>(3,3) << 1, 0, IMG_W/2, 0, 1, IMG_H/2, 0, 0, 1);
        cv::Mat distCoeffs = Mat::zeros(Size(1,5), CV_32F);
        std::vector<cv::Mat> rvec;
        std::vector<cv::Mat> tvec;
        cv::Mat gray;

        int flags = 0;
        int css = fs["checkerboard_sqare_size"];
        int ccw = fs["checkerboard_corner_width"];
        int cch = fs["checkerboard_corner_height"];

        fs.release();

        // Création d'un vecteur pour stocker les points 3D pour chaque image d'échiquier
        std::vector<std::vector<cv::Point3f>> objpoints;

        // Création d'un vecteur pour stocker les points 2D pour chaque image d'échiquier
        std::vector<std::vector<cv::Point2f>> imgpoints;

        // Vecteur pour stocker les coordonnées en pixel des coins détectés sur l'échiquier
        std::vector<cv::Point2f> corner_pts;

        bool success;
        Size img_sz;
        double error;

        loadImages(imageFiles4Calibration[v_codeCam], img_list);

        // Définition des coordonnées réels des points 3D
        std::vector<cv::Point3f> objp;
        for (int i = 0; i < cch; i++)
        {
            for (int j = 0; j < ccw; j++)
                objp.push_back(cv::Point3f(j * css, i * css, 0));
        }

        flags = cv::CALIB_CB_ADAPTIVE_THRESH |
               cv::CALIB_CB_FAST_CHECK |
               cv::CALIB_CB_NORMALIZE_IMAGE |
               cv::CALIB_CB_FILTER_QUADS;

        for (size_t i = 0; i < img_list.size(); i++)
        {
            cv::cvtColor(img_list.at(i), gray, cv::COLOR_BGR2GRAY);
            success = cv::findChessboardCorners(gray, cv::Size(ccw, cch), corner_pts, flags);

            if (i == 0) img_sz = gray.size();
            if (success)
            {
                cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1);
                cv::cornerSubPix(gray, corner_pts, cv::Size(5, 5), cv::Size(-1, -1), criteria);
                objpoints.push_back(objp);
                imgpoints.push_back(corner_pts);
            }
        }
        //flags = cv::CALIB_FIX_ASPECT_RATIO;

        error = cv::calibrateCamera(objpoints, imgpoints, img_sz, cameraMatrix, distCoeffs, rvec, tvec);

        this->saveCameraParameters(cameraMatrix, distCoeffs, error, this->v_codeCam);
        this->v_initOk = true;
    }
}

int Cameras::calcExtrinsicParams()
{
    QFile file;
    std::vector<cv::Point3d> objectPoints;
    std::vector<cv::Point2d> imagePoints;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat rvec, tvec;
    cv::Mat R, T;
    cv::FileStorage fs;
    std::string msg;
    double AREA_HIGH, AREA_WIDTH;

    fs.open(intrinsicsCameraParametersFile[v_codeCam], cv::FileStorage::READ);

    if(fs.isOpened()){
        fs["intrinsic"] >> cameraMatrix;
        fs["dist"] >> distCoeffs;
        fs.release();

        for(uint i = 0; i < this->wa->getCorners().size(); i++){
            imagePoints.emplace_back(this->wa->getCorners()[i]);
        }

        fs.open(saveParamIHMFile, cv::FileStorage::READ);
        fs["working_area_length"] >> AREA_HIGH;
        fs["working_area_width"] >> AREA_WIDTH;
        fs.release();

        //Configuration des coordonnées des sommets du rectangle qu'est la zone de travail
        if(v_codeCam == Camera::DROITE){
            objectPoints.emplace_back(cv::Point3d(0.0, 0.0, 0.0));
            objectPoints.emplace_back(cv::Point3d(0.0, AREA_HIGH, 0.0));
            objectPoints.emplace_back(cv::Point3d(AREA_WIDTH, AREA_HIGH, 0.0));
            objectPoints.emplace_back(cv::Point3d(AREA_WIDTH, 0.0, 0.0));
        }
        else if(v_codeCam == Camera::GAUCHE){
            objectPoints.emplace_back(cv::Point3d(AREA_WIDTH, AREA_HIGH, 0.0));
            objectPoints.emplace_back(cv::Point3d(AREA_WIDTH, 0.0, 0.0));
            objectPoints.emplace_back(cv::Point3d(0.0, 0.0, 0.0));
            objectPoints.emplace_back(cv::Point3d(0.0, AREA_HIGH, 0.0));
        }
        else if(v_codeCam == Camera::MILIEU){
            objectPoints.emplace_back(cv::Point3d(AREA_WIDTH, 0.0, 0.0));
            objectPoints.emplace_back(cv::Point3d(0.0, 0.0, 0.0));
            objectPoints.emplace_back(cv::Point3d(0.0, AREA_HIGH, 0.0));
            objectPoints.emplace_back(cv::Point3d(AREA_WIDTH, AREA_HIGH, 0.0));
        }

        //Calcul de la matrice de ratation et de la matrice de translation de la caméra
        //en utilisant la fonction solvePnP() de OpenCV
        if(v_codeCam == Camera::MILIEU) cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_IPPE);
        else cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        cv::Rodrigues(rvec, R);
        T = tvec;

        //Sauvegarde
        file.setFileName(QString::fromStdString(extrinsicsCameraParametersFile[v_codeCam]));
        file.setPermissions(QFileDevice::WriteUser);
        fs.open(extrinsicsCameraParametersFile[v_codeCam], cv::FileStorage::WRITE);

        if(fs.isOpened()){
            fs << "R" << R;
            fs << "T" << T;

            fs.release();
            file.setPermissions(QFileDevice::ReadUser);
        }
        else{
            msg = "Impossible d'ouvrir le fichier des parametres extrinseques de la camera " + this->v_camPosition;
            sendMessage(QString::fromStdString(msg));
            return FILE_NOT_OPEN;
        }
    }
    else{
        msg = "Impossible d'ouvrir le fichier des parametres intrinseques de la camera " + this->v_camPosition;
        sendMessage(QString::fromStdString(msg));
        return FILE_NOT_OPEN;
    }

    return SUCCESSFULL;
}

void Cameras::saveCameraParameters(cv::Mat cameraMatrix, cv::Mat dist, double err_reprojection, int code1,
                                   int code2,
                                   bool stereo,
                                   cv::Mat R,
                                   cv::Mat T,
                                   cv::Mat E,
                                   cv::Mat F) {

    if (code1 >= 0 && !stereo) {
        QFile file(QString::fromStdString(intrinsicsCameraParametersFile[code1]));
        file.setPermissions(QFileDevice::WriteUser);

        cv::FileStorage saveCamParam(intrinsicsCameraParametersFile[code1], cv::FileStorage::WRITE);

        saveCamParam << "intrinsic" << cameraMatrix;
        saveCamParam << "dist" << dist;
        saveCamParam << "error" << err_reprojection;

        saveCamParam.release();
        file.setPermissions(QFileDevice::ReadUser);
    }
    else if(code1 >= 0 && stereo && code2 >=0){
        int k = (code2 == Camera::DROITE) ? 1 : 0;

        cv::FileStorage saveCamParam(stereocalibParametersFile[k], cv::FileStorage::WRITE);

        saveCamParam << "R" << R;
        saveCamParam << "T" << T;
        saveCamParam << "E" << E;
        saveCamParam << "F" << F;
        saveCamParam << "error" << err_reprojection;

        saveCamParam.release();
    }
    else cout << "\nERROR : invalid camera" << endl;
}

/*int Cameras::stereoCalibration(Cameras cam){
    std::vector<cv::Mat> img_list1, img_list2;
    cv::Mat cameraMatrix1, cameraMatrix2;
    cv::Mat distCoeffs1, distCoeffs2;
    cv::Mat R, T, E, F;
    cv::Mat gray1, gray2;

    int flags = 0;
    bool success1, success2;
    Size img_sz;

    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f>> objPoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f>> imgPoints1, imgPoints2;

    // vector to store the pixel coordinates of detected checker board corners
    std::vector<cv::Point2f> corner_pts1, corner_pts2;

    cv::FileStorage fs;

    fs.open(intrinsicsCameraParametersFile[this->codeCam], cv::FileStorage::READ);

    if(fs.isOpened()){
        fs["intrinsic"] >> cameraMatrix2;
        fs["dist"] >> distCoeffs2;
        fs.release();
    }else{
        cout << "\nUnable to open the file : " <<intrinsicsCameraParametersFile[this->codeCam] << std::endl;
        return -1;
    }

    fs.open(intrinsicsCameraParametersFile[cam.codeCam], cv::FileStorage::READ);

    if(fs.isOpened()){
        fs["intrinsic"] >> cameraMatrix1;
        fs["dist"] >> distCoeffs1;
        fs.release();
    }else{
        cout << "\nUnable to open the file : " <<intrinsicsCameraParametersFile[cam.codeCam] << std::endl;
        return -1;
    }

    loadImages(imageFiles4StereoCalib[cam.codeCam], img_list1);
    loadImages(imageFiles4StereoCalib[this->codeCam], img_list2);

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;

    for (int i = 0; i < CHECKERBOARD_CORNERS_HEIGHT; i++)
    {
        for (int j = 0; j < CHECKERBOARD_CORNERS_WIDTH; j++)
            objp.push_back(cv::Point3f(j * CHECKERBOARD_SQUARE_SIZE, i * CHECKERBOARD_SQUARE_SIZE, 0));
    }

    flags = cv::CALIB_CB_ADAPTIVE_THRESH |
           cv::CALIB_CB_FAST_CHECK |
           cv::CALIB_CB_NORMALIZE_IMAGE |
           cv::CALIB_CB_FILTER_QUADS;

    // Looping over all the images in the directory
    for (size_t i = 0, j = 0; i < img_list1.size() || j < img_list2.size(); i++, j++)
    {
        cv::cvtColor(img_list1.at(i), gray1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(img_list2.at(i), gray2, cv::COLOR_BGR2GRAY);

        success1 = cv::findChessboardCorners(gray1, cv::Size(CHECKERBOARD_CORNERS_WIDTH, CHECKERBOARD_CORNERS_HEIGHT), corner_pts1, flags);
        success2 = cv::findChessboardCorners(gray2, cv::Size(CHECKERBOARD_CORNERS_WIDTH, CHECKERBOARD_CORNERS_HEIGHT), corner_pts2, flags);

        if (i == 0) img_sz = gray1.size();

        if (success1 && success2)
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray1, corner_pts1, cv::Size(5, 5), cv::Size(-1, -1), criteria);
            cv::cornerSubPix(gray2, corner_pts2, cv::Size(5, 5), cv::Size(-1, -1), criteria);

            objPoints.push_back(objp);
            imgPoints1.push_back(corner_pts1);
            imgPoints2.push_back(corner_pts2);
        }
    }

    flags = cv::CALIB_FIX_INTRINSIC;

    double error = cv::stereoCalibrate(objPoints, imgPoints1, imgPoints2,
                                       cameraMatrix1, distCoeffs1,
                                       cameraMatrix2, distCoeffs2,
                                       img_sz, R, T, E, F, flags);

    this->saveCameraParameters(Mat(), Mat(), error, this->codeCam, cam.codeCam, true, R, T, E, F);

    return 0;
}*/

Mat Cameras::getProjectionMatrix()
{
    cv::Mat K, R, T;
    cv::Mat A;
    cv::Mat P;
    cv::Mat temp;

    //Récupération des paramètres intrinsèques et extrinqèques de la caméra
    cv::FileStorage fs(intrinsicsCameraParametersFile[this->v_codeCam], cv::FileStorage::READ);
    fs["intrinsic"] >> K;
    fs.release();

    fs.open(extrinsicsCameraParametersFile[this->v_codeCam], cv::FileStorage::READ);
    fs["R"] >> R;
    fs["T"] >> T;
    fs.release();

    //Mise en place de la matrice de transformation en coordonnées homogènes
    for(int i = 0; i < 3; i++){
        temp = ((cv::Mat_<double>(1, 4) << R.ptr<double>(i)[0], R.ptr<double>(i)[1], R.ptr<double>(i)[2], T.ptr<double>(i)[0]));
        A.push_back(temp);
    }

    //Calcul de la matrice de projection
    P = K*A;
    return P.clone();
}

cv::Mat Cameras::getRobotPosition(){
    cv::Mat position = (cv::Mat_<double>(1,3) << 0.0, 0.0, 0.0);
    cv::Point* pixelPosition;
    cv::Point3d realPosition[3];
    cv::Vec2d vectorPosition;
    double theta;

    //Calcul de la position du robot en pixel
    pixelPosition = this->rp->getPosition(v_snapshot);

    //Calcul de la position du robot en coordonnées réelles
    if(pixelPosition[Position::CENTER].x != POSITION_NOT_FOUND && pixelPosition[Position::CENTER].y != POSITION_NOT_FOUND){
        realPosition[Position::GREEN_P] = this->getRealPosition(pixelPosition[Position::GREEN_P], v_height_led);
        realPosition[Position::RED_P] = this->getRealPosition(pixelPosition[Position::RED_P], v_height_led);
        realPosition[Position::CENTER] = this->getRealPosition(pixelPosition[Position::CENTER], v_height_led);

        vectorPosition.val[0] = realPosition[Position::RED_P].x - realPosition[Position::GREEN_P].x;
        vectorPosition.val[1] = realPosition[Position::RED_P].y - realPosition[Position::GREEN_P].y;

        theta = atan2(vectorPosition.val[1], vectorPosition.val[0]) + (M_PI/2);
        if(theta > M_PI) theta -= M_PI;
        else if (theta <= -M_PI) theta += M_PI;

        position.ptr<double>(0)[0] = realPosition[Position::CENTER].x; //en mm
        position.ptr<double>(0)[1] = realPosition[Position::CENTER].y; //en mm
        position.ptr<double>(0)[2] = theta; //en radians
    }else{
        position.ptr<double>(0)[0] = POSITION_NOT_FOUND;
        position.ptr<double>(0)[1] = POSITION_NOT_FOUND;
        position.ptr<double>(0)[2] = POSITION_NOT_FOUND;
    }

    return position.clone();
}

cv::Point2i Cameras::getPixelPosition(cv::Point3i realPosition)
{
    cv::Mat pixelPosition;
    cv::Mat position = (cv::Mat_<double>(3,1) << realPosition.x, realPosition.y, realPosition.z);
    cv::Point2i pixel;
    cv::Mat P = this->getProjectionMatrix();

    position.push_back(1.0); // passage en coordonnée homogène

    pixelPosition = P*position;

    pixel.x = static_cast<int>(pixelPosition.ptr<double>(0)[0]/pixelPosition.ptr<double>(2)[0]);
    pixel.y = static_cast<int>(pixelPosition.ptr<double>(1)[0]/pixelPosition.ptr<double>(2)[0]);

    return pixel;
}

Point3d Cameras::getRealPosition(cv::Point pixel, int z)
{
    vector<cv::Point2d> impt = { pixel };
    vector<cv::Point2d> u_impt;
    cv::Mat intrinsic_P, distCoeffs;
    cv::Mat P = this->getProjectionMatrix();
    cv::Mat res;
    cv::Mat A = (cv::Mat_<double>(2,2) << 0.0, 0.0, 0.0, 0.0);
    cv::Mat B = (cv::Mat_<double>(2,1) << 0.0, 0.0);
    cv::Mat inv_A;
    cv::Point3d worldCoordinate;
    cv::FileStorage fs(intrinsicsCameraParametersFile[this->v_codeCam], cv::FileStorage::READ);

    fs["dist"] >> distCoeffs;
    fs["intrinsic"] >> intrinsic_P;
    fs.release();

    cv::undistortPoints(impt, u_impt, intrinsic_P, distCoeffs);

    auto u = impt[0].x;
    auto v = impt[0].y;
    auto a = static_cast<double>(z)*( u * P.ptr<double>(2)[2] - P.ptr<double>(0)[2] ) - P.ptr<double>(0)[3] + (u * P.ptr<double>(2)[3]);
    auto b = static_cast<double>(z)*( v * P.ptr<double>(2)[2] - P.ptr<double>(1)[2] ) - P.ptr<double>(1)[3] + (v * P.ptr<double>(2)[3]);

    B.ptr<double>(0)[0] = a;
    B.ptr<double>(1)[0] = b;
    A.ptr<double>(0)[0] = P.ptr<double>(0)[0] - (u * P.ptr<double>(2)[0]);
    A.ptr<double>(0)[1] = P.ptr<double>(0)[1] - (u * P.ptr<double>(2)[1]);
    A.ptr<double>(1)[0] = P.ptr<double>(1)[0] - (v * P.ptr<double>(2)[0]);
    A.ptr<double>(1)[1] = P.ptr<double>(1)[1] - (v * P.ptr<double>(2)[1]);
    inv_A = A.inv();
    res = inv_A * B;

    worldCoordinate.x = res.ptr<double>(0)[0]; //x
    worldCoordinate.y = res.ptr<double>(1)[0]; //y
    worldCoordinate.z = static_cast<double>(z); //z

    return worldCoordinate;
}

cv::Mat Cameras::detectObstacle() {
    Mat img = this->v_useCamera ? this->acq->getFrame() : cv::imread(pathImageObs1[this->v_codeCam]);
    Mat edges;
    Mat edges_color;
    Mat contour_isolated = Mat::zeros(img.size(), CV_8UC3);
    //Mat emptyArea = imread(pathEmptyArea[this->codeCam]);
    Mat mask;
    FileStorage fs(maskArea[this->v_codeCam], FileStorage::READ);

    //string win1 = "Window 1";
    //string win2 = "Window 2";

    fs["mask"] >> mask;
    fs.release();

    std::vector<std::vector<Point>> contours;
    std::vector<std::vector<Point>> temp;
    Rect bounding_rect;

    //Isolation des contours dans l'image
    //imshow(win1, mask); pause();
    //imshow(win1, img); pause();
    Canny(img, edges, 135, 55); //imshow(win1, edges); pause();
    GaussianBlur(edges, edges, Size(3, 3), 1.0); //imshow(win1, edges); pause();
    dilate(edges, edges, Mat(), Point(-1, -1), 2);// imshow(win1, edges); pause();
    //imshow(win1, edges); pause();
    medianBlur(edges, edges, 3);//imshow(win1, edges); pause();

    for (int i = 0; i < edges.rows; i++) {
        uchar* p = edges.ptr(i);

        for (int j = 0; j < edges.cols; j++) {
            if (p[j] != 0) p[j] = 255;
        }
    }

    Mat result;

    //Isolation de la zone de travail
    bitwise_and(mask, edges, result);
    //imshow(win1, result); pause();

    erode(result, result, Mat(), Point(-1,-1),3);
    bitwise_xor(mask, result, result);
    result = -result + mask;
    //imshow(win1, result); pause();

    edges_color = result.clone();

    //Recherche d'obstacle dans la zone de travail du robot
    findContours(result, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    cvtColor(edges_color, edges_color, COLOR_GRAY2BGR);

    for (uint i = 0; i < contours.size(); i++) {
        if (contourArea(contours[i]) > 2500 && contourArea(contours[i]) < 60000) {
            temp.push_back(contours[i]);
        }
    }

    for (uint i = 0; i < temp.size()- 1; i++) {
        for (uint j = i + 1; j < temp.size(); j++) {
            if (contourArea(temp[i]) < contourArea(temp[j])) {
                std::vector<Point> t;

                t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }

    if(!temp.empty()){
        std::vector<std::vector<Point>> con = {temp[0]};
        cv::drawContours(edges_color, con, -1, Scalar(0, 0, 255), 2);
        //imshow("edges color", edges_color); //pause();
        bounding_rect = boundingRect(temp[0]);
        rectangle(contour_isolated, bounding_rect, Scalar(0, 0, 255), FILLED);
        //imshow(win2, contour_isolated); pause();
    }

    destroyAllWindows();

    return contour_isolated.clone();
}

bool Cameras::isCameraPresent()
{
    cv::VideoCapture cap;
    bool present;

    cap.open(v_id, cv::CAP_DSHOW);
    if(cap.isOpened()){
         present = true;
        emit sendMessage("Caméra " + QString::fromStdString(v_camPosition) + " : caméra détectée", GREEN_COLOR);
    }
    else{
        present = false;
        emit sendMessage("Caméra " + QString::fromStdString(v_camPosition) + " : aucune caméra valide à l'index indiqué", RED_COLOR);
    }
    cap.release();

    return present;
}

cv::Mat Cameras::generateTopographicMap(int canalBGR){
    Mat pixel;
    Mat world;
    Point3d worldCoord;
    cv::Mat obs = this->detectObstacle();

    cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);
    int AREA_HIGH, AREA_WIDTH;

    fs["working_area_length"] >> AREA_HIGH;
    fs["working_area_width"] >> AREA_WIDTH;
    fs.release();

    Mat realMap = Mat::zeros(Size(AREA_HIGH, AREA_WIDTH), CV_8UC3);

    for (int i = 0; i < obs.rows; i++) {
        Vec3b* p = obs.ptr<Vec3b>(i);
        for (int j = 0; j < obs.cols; j++) {
            if (p[j].val[2] == 255 && p[j].val[0] == 0 && p[j].val[1] == 0) {

                worldCoord = realCoordinate(Point(j, i), this->v_codeCam, 0);

                int x = static_cast<int>(worldCoord.x);
                int y = static_cast<int>(worldCoord.y);

                if ((x > 0 && x < AREA_WIDTH) && (y > 0 && y < AREA_HIGH)) {
                    if(canalBGR >=0 && canalBGR < 3){
                        realMap.ptr<Vec3b>(x)[y].val[canalBGR] = 255;
                    } else realMap.ptr<Vec3b>(x)[y].val[0] = 255;
                }
            }

        }
    }

    this->v_map = realMap.clone();
    return this->v_map.clone();
}

Point3d Cameras::realCoordinate(Point pix, int codeCam, int z) {
    Point3d wordCoordinate;

    vector<Point2f> impt = { pix };
    vector<Point2f> u_impt;
    Mat R01, R10, T01, T10;
    cv::Mat intrinsic;
    cv::Mat distCoeffs;
    cv::FileStorage fs(intrinsicsCameraParametersFile[codeCam], FileStorage::READ);

    //Récupération des paramètres intrinsèques et extrinqèques de la caméra
    fs["intrinsic"] >> intrinsic;
    fs["dist"] >> distCoeffs;
    fs.release();

    fs.open(extrinsicsCameraParametersFile[codeCam], FileStorage::READ);
    fs["R"] >> R01;
    fs["T"] >> T01;
    fs.release();

    //Calcul du vecteur de translation permettant de passer du repère de la caméra au repère réel
    transpose(R01, R10);
    T10 = -R10 * T01;

    cv::undistortPoints(impt, u_impt, intrinsic, distCoeffs);
    Mat ray_cam = (Mat_<double>(3,1) << u_impt[0].x, u_impt[0].y, 1);
    Mat ray_world = R10 * ray_cam;

    double depth = (z - static_cast<double>(T10.ptr<double>(2)[0])) / static_cast<double>(ray_world.ptr<double>(2)[0]);

    Mat res = T10 + depth * ray_world;

    wordCoordinate.x = res.ptr<double>(0)[0];
    wordCoordinate.y = res.ptr<double>(1)[0];
    wordCoordinate.z = res.ptr<double>(2)[0];

    return wordCoordinate;
}

bool Cameras::run()
{
    if(v_codeCam >= 0){
        cv::Mat waImg = v_useCamera ? this->acq->getFrame() : cv::imread(pathEmptyArea[v_codeCam]);

        if(this->wa != nullptr) delete this->wa;

        this->wa = new WorkingArea(v_codeCam, waImg);

        if(this->rp == nullptr){
            if(this->hasCalibFile()){
                this->rp = new RobotPositionThread(this);
                //emit sendMessage("Camera " + QString::fromStdString(camPosition) + " : Fichier de calibration détecté");

                return true;
            }
            else{
                emit sendMessage("Camera " + QString::fromStdString(v_camPosition) + " : Fichier de calibration absent");
                return false;
            }
        }
        else return true;
    }
    else return false;
}

void Cameras::capture()
{
    if(this->useCam()) v_snapshot = this->acq->getFrame();
    else v_snapshot = cv::imread(pathImageRobotLeds[this->v_codeCam]);
}

void Cameras::setId(int value)
{
    if(v_id != value){
        v_id = value;
        if(v_useCamera && (v_id != -1)) v_camIsPresent = isCameraPresent();
    }
}

bool Cameras::getCamIsPresent() const
{
    return v_camIsPresent;
}
