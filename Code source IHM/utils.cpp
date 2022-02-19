#include "utils.h"

using namespace std;

std::string projectDirectory = "..\\";

std::string pathEmptyArea[3] = {projectDirectory + "images\\Camera_gauche\\zone_vide.png",
                                projectDirectory + "images\\Camera_milieu\\zone_vide.png",
                                projectDirectory + "images\\Camera_droite\\zone_vide.png"};

std::string HSVConfigFile[3] = {projectDirectory + "config\\Camera_gauche\\HSVConfig_camera_gauche.yml",
                                projectDirectory + "config\\Camera_milieu\\HSVConfig_camera_milieu.yml",
                                projectDirectory + "config\\Camera_droite\\HSVConfig_camera_droite.yml"};

std::string HSVConfigOriginFile[3] = {projectDirectory + "config\\Camera_gauche\\HSVConfig_origine_camera_gauche.yml",
                                      projectDirectory + "config\\Camera_milieu\\HSVConfig_origine_camera_milieu.yml",
                                      projectDirectory + "config\\Camera_droite\\HSVConfig_origine_camera_droite.yml"};

std::string HSVRedLed[3] = {projectDirectory + "config\\Camera_gauche\\HSVDetect_red_led.yml",
                            projectDirectory + "config\\Camera_milieu\\HSVDetect_red_led.yml",
                            projectDirectory + "config\\Camera_droite\\HSVDetect_red_led.yml"};

std::string HSVGreenLed[3] = {projectDirectory + "config\\Camera_gauche\\HSVDetect_green_led.yml",
                              projectDirectory + "config\\Camera_milieu\\HSVDetect_green_led.yml",
                              projectDirectory + "config\\Camera_droite\\HSVDetect_green_led.yml"};

std::string HSVLed_R = projectDirectory + "config\\HSVDetect_led_R.yml";

std::string HSVLed_V = projectDirectory + "config\\HSVDetect_led_V.yml";

std::string patternConfig = projectDirectory + "others\\pattern_config.yml";

std::string intrinsicsCameraParametersFile[3] = {projectDirectory + "config\\Camera_gauche\\intrinseques_camera_gauche.yml",
                                                 projectDirectory + "config\\Camera_milieu\\intrinseques_camera_milieu.yml",
                                                 projectDirectory + "config\\Camera_droite\\intrinseques_camera_droite.yml"};

std::string extrinsicsCameraParametersFile[3] = {projectDirectory + "config\\Camera_gauche\\extrinseques_camera_gauche.yml",
                                                 projectDirectory + "config\\Camera_milieu\\extrinseques_camera_milieu.yml",
                                                 projectDirectory + "config\\Camera_droite\\extrinseques_camera_droite.yml"};

std::string stereocalibParametersFile[2] = {projectDirectory + "config\\stereo\\stereo_milieu_gauche.yml",
                                            projectDirectory + "config\\stereo\\stereo_milieu_droite.yml"};

std::string imageFiles4Calibration[3] = {projectDirectory + "images\\Camera_gauche\\Chessboard_Images",
                                         projectDirectory + "images\\Camera_milieu\\Chessboard_Images",
                                         projectDirectory + "images\\Camera_droite\\Chessboard_Images"};

std::string imageFiles4StereoCalib[3] = {projectDirectory + "images\\Camera_gauche\\Stereocalibration_Images",
                                         projectDirectory + "images\\Camera_milieu\\Stereocalibration_Images",
                                         projectDirectory + "images\\Camera_droite\\Stereocalibration_Images"};

std::string maskArea[3] = {projectDirectory + "others\\Camera_gauche\\emptyAreaMask.yml",
                           projectDirectory + "others\\Camera_milieu\\emptyAreaMask.yml",
                           projectDirectory + "others\\Camera_droite\\emptyAreaMask.yml"};

std::string pathTopographicMap = projectDirectory + "images\\topographicMap.png";
std::string pathRealTopographicMap = projectDirectory + "images\\realTopographicMap.png";

std::string pathImageObs1[3] = {projectDirectory + "images\\Camera_gauche\\obs_1_cg.png",
                                projectDirectory + "images\\Camera_milieu\\obs_1_cm.png",
                                projectDirectory + "images\\Camera_droite\\obs_1_cd.png"};

std::string pathImageObs2[3] = {projectDirectory + "images\\Camera_gauche\\obs_2_cg.png",
                                projectDirectory + "images\\Camera_milieu\\obs_2_cm.png",
                                projectDirectory + "images\\Camera_droite\\obs_2_cd.png"};

std::string pathImageRobotLeds[3] = {projectDirectory + "images\\Camera_gauche\\robot_leds.png",
                                     projectDirectory + "images\\Camera_milieu\\robot_leds.png",
                                     projectDirectory + "images\\Camera_droite\\robot_leds.png"};
std::string saveTrajectory = projectDirectory + "others\\path.yml";
std::string saveParamIHMFile = projectDirectory + "config\\paramIHM.yml";
std::string saveProject = projectDirectory + "projects\\";
std::string camerasIDFile = projectDirectory +"others\\cameras_IDs.yml";
std::string saveCorrectorSettings = projectDirectory + "others\\corrector.yml";
std::string saveVVOType = projectDirectory + "others\\vvot.yml";
std::string batchFile = projectDirectory + "scripts\\setup.bat";
std::string batchDirectory = projectDirectory + "scripts";

double scale = 0.25;

namespace Havry
{
    union Float32ConversionHelper
    {
        static_assert(sizeof(float32) == sizeof(uint32), "");
        float32 f;
        uint32 u;
    };

    union Float64ConversionHelper
    {
        static_assert(sizeof(float64) == sizeof(uint64), "");
        float64 f;
        uint64 u;
    };
}


QPixmap  Mat2Pixmap(const cv::Mat &src, bool scaled, int scaleFactor){
    QPixmap qPixm;
    QImage *qImg;

    cv::Mat frame = src.clone();
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

    qImg = new QImage(frame.cols, frame.rows, QImage::Format_RGB32);

    for(int i = 0; i < frame.rows; i++){
        auto p = frame.ptr<cv::Vec3b>(i);
        QRgb* qP = (QRgb*) qImg->scanLine(i);

        for(int j = 0; j < frame.cols; j++){
            qP[j] = qRgb(p[j][0], p[j][1], p[j][2]);
        }
    }

    qPixm.convertFromImage(*qImg);

    if(scaled){
        qPixm = qPixm.scaled(frame.cols/scaleFactor, frame.rows/scaleFactor);
    }

    delete qImg;

    return qPixm;
}

void loadImages(const std::string& dir_name, std::vector<cv::Mat>& img_list) {
    std::vector<std::string> files;

    cv::glob(dir_name, files);

    std::cout << "\nDirectory : " << dir_name;
    std::cout << "\nFile size = " << files.size();
    for(uint i = 0; i < files.size(); i++)
        std::cout << "\nFichier " << i+1 << " : " << files[i];
    for (uint i = 0; i < files.size(); i++) {
        cv::Mat img = cv::imread(files[i]);

        if (img.empty()) continue;
        img_list.push_back(img);
    }
    files.clear();
}

std::vector<cv::Point> sortCorners(std::vector<cv::Point> corners){
    std::vector<cv::Point> res;
    std::vector<int> sum_xy;
    cv::Point pt_tl;
    cv::Point pt_br;
    cv::Point pt_tr;
    cv::Point pt_bl;

    for (uint i = 0; i < corners.size(); i++) {
        int a;
        a = corners[i].x + corners[i].y;
        sum_xy.push_back(a);
    }

    for (uint i = 0; i < sum_xy.size() - 1; i++) {
        for (uint j = i+1; j < sum_xy.size(); j++) {
            int change;
            cv::Point pt;

            if (sum_xy[i] > sum_xy[j]) {
                change = sum_xy[i];
                sum_xy[i] = sum_xy[j];
                sum_xy[j] = change;

                pt = corners[i];
                corners[i] = corners[j];
                corners[j] = pt;
            }
        }
    }

    if(corners[1].y < corners[2].y){
        pt_tl = corners[0]; //plus petit x+y
        pt_tr = corners[1]; //entre 2 et 1, a le y le plus faible
        pt_br = corners[3]; //plus grand x+y
        pt_bl = corners[2];

    }else{
        pt_tl = corners[0]; //plus petit x+y
        pt_tr = corners[2]; //entre 2 et 1, a le y le plus faible
        pt_br = corners[3]; //plus grand x+y
        pt_bl = corners[1];

    }

    res.push_back(pt_tl);
    res.push_back(pt_tr);
    res.push_back(pt_br);
    res.push_back(pt_bl);

    return res;
}

std::vector<std::vector<std::vector<int>>> readHSVConfig(bool readOri){
    //0 = hsv
    //1 = camera
    //2 = values hsv

    std::vector<std::vector<std::vector<int>>> hsv(3);

    for(uint i = 0; i < hsv.size(); i++){
        std::vector<int> temp;
        hsv[i].push_back(temp);
        hsv[i].push_back(temp);
        hsv[i].push_back(temp);

    }

    for(uint i = 0; i < hsv.size(); i++){
        for(uint j = 0; j < hsv[i].size(); j++){
            hsv[i][j].push_back(0);
            hsv[i][j].push_back(0);
        }

    }

   read(Camera::GAUCHE, hsv, readOri);
   read(Camera::DROITE, hsv, readOri);
   read(Camera::MILIEU, hsv, readOri);

    return hsv;
}

void read(uint camera, std::vector<std::vector<std::vector<int>>> &hsv, bool readOri){
    cv::FileStorage fs;
    int h_min, h_max, s_min, s_max, v_min, v_max;
    std::string fileName;

    fileName = (readOri) ? HSVConfigOriginFile[camera] : HSVConfigFile[camera];
    fs.open(fileName, cv::FileStorage::READ);

     if(fs.isOpened()){
         fs["h_min"] >> h_min;
         fs["h_max"] >> h_max;
         fs["s_min"] >> s_min;
         fs["s_max"] >> s_max;
         fs["v_min"] >> v_min;
         fs["v_max"] >> v_max;

         hsv[HSVCode::HUE][camera][0] = h_min;
         hsv[HSVCode::HUE][camera][1] = h_max;
         hsv[HSVCode::SAT][camera][0] = s_min;
         hsv[HSVCode::SAT][camera][1] = s_max;
         hsv[HSVCode::VAL][camera][0] = v_min;
         hsv[HSVCode::VAL][camera][1] = v_max;

         fs.release();
     } else{
         std::cout << "\nCan't open file " << std::endl;
     }
}

bool saveHSVConfig(int camera,
                   int h_min, int h_max,
                   int s_min, int s_max,
                   int v_min, int v_max,
                   bool saveOrigin,
                   std::string msg){

    bool save = false;
    std::string fileName;
    cv::FileStorage fs;
    int cam = 0;
    QFile file;

    if(camera == Camera::DROITE) cam = Camera::DROITE;
    else if(camera == Camera::GAUCHE) cam = Camera::GAUCHE;
    else if(camera == Camera::MILIEU) cam = Camera::MILIEU;

    if(!saveOrigin) fileName = HSVConfigFile[cam];
    else fileName = HSVConfigOriginFile[cam];

    file.setFileName(QString::fromStdString(fileName));
    file.setPermissions(QFileDevice::WriteUser);

    fs.open(fileName, cv::FileStorage::WRITE);

    if(fs.isOpened()){
        fs << "h_min" << h_min;
        fs << "h_max" << h_max;
        fs << "s_min" << s_min;
        fs << "s_max" << s_max;
        fs << "v_min" << v_min;
        fs << "v_max" << v_max;

        fs.release();
        msg = "Sauvegarde réussie.";

        save = true;
    }
    else{
        msg = "Echec de la Sauvegarde. \nLe fichier n'a pas pu être ouvert";
    }

    file.setPermissions(QFileDevice::ReadUser);

    return save;
}

cv::Mat workingAreaDelimiter(int rows, int cols, std::vector<cv::Point> corners, int foregroundColor) {
    cv::Mat bs = cv::Mat::ones(rows, cols, CV_8UC1) * foregroundColor;

    double a; // pente
    double b; //ordonnée à l'origine
    double c; //abscisse à l'origine

    double a_p;
    int lim;
    int lim2;
    int lim3;
    int lim_c;
    int lim_r;

    //Traitement du bord supérieur:

    a = static_cast<double>(corners[Corner::TOP_LEFT].y - corners[Corner::TOP_RIGHT].y) / (corners[Corner::TOP_LEFT].x - corners[Corner::TOP_RIGHT].x);
    b = ((corners[Corner::TOP_LEFT].y + corners[Corner::TOP_RIGHT].y) - a * (corners[Corner::TOP_LEFT].x + corners[Corner::TOP_RIGHT].x)) / 2.0;
    c = -(b / a);

    if (corners[Corner::TOP_LEFT].y > corners[Corner::TOP_RIGHT].y) {
        lim_c = c > cols ? cols : static_cast<int>(c);
        lim_r = b > rows ? rows : static_cast<int>(b);

        for (int i = 0; i < lim_r; i++) {
            auto p = bs.ptr<uchar>(i);

            if (i <= b) {
                for (int j = 0; j < lim_c; j++) {
                    if (j < c) {
                        a_p = (static_cast<double>(i)) / (static_cast<double>(j) - c);
                        if (a_p >= a) p[j] = 0;
                    }
                }
            }
        }
    }

    else {
        if (c < 0){
            lim = static_cast<int>(a * cols + b);
            lim2 = 0;
        }
        else{
            lim = static_cast<int>(a * cols + b) <= rows ? static_cast<int>(a * cols + b) : rows;
            lim2 = static_cast<int>(c) + 1;
        }

        for (int i = 0; i < lim; i++) {
            auto p = bs.ptr<uchar>(i);

            for (int j = lim2; j < cols; j++) {
                a_p = static_cast<double>(i) / (static_cast<double>(j) - c);
                if (a_p <= a) p[j] = 0;
            }
        }
    }

    ////Traitement du bord inférieur:

    a = static_cast<double>(corners[Corner::BOTTOM_LEFT].y - corners[Corner::BOTTOM_RIGHT].y) / (corners[Corner::BOTTOM_LEFT].x - corners[Corner::BOTTOM_RIGHT].x);
    b = ((corners[Corner::BOTTOM_LEFT].y + corners[Corner::BOTTOM_RIGHT].y) - a * (corners[Corner::BOTTOM_LEFT].x + corners[Corner::BOTTOM_RIGHT].x)) / 2.0;
    c = -(b / a);

    if (corners[Corner::BOTTOM_LEFT].y > corners[Corner::BOTTOM_RIGHT].y) {
        //lim_c = c > cols ? cols : (int)c;
        //lim_r = b > rows ? rows : (int)b;

        int var = static_cast<int>(cols * a + b) >= 0 ? static_cast<int>(cols * a + b) : 0;

        for (int i = var; i < rows; i++) {
            auto p = bs.ptr<uchar>(i);

            for (auto j = static_cast<int>((rows - b) / a); j < cols; j++) {
                a_p = static_cast<double>(i) / (static_cast<double>(j) - c);
                if (a_p <= a) p[j] = 0;
                if (j >= c && c < cols) {
                    p[j] = 0;
                }
            }
        }
    }
    else {

        if (c < 0){
            lim = static_cast<int>(b);
            lim2 = lim;
            lim3 = cols;
        }
        else{
            lim = static_cast<int>((rows - b) / a) <= cols ? static_cast<int>((rows - b) / a) : cols;
            lim2 = 0;
            lim3 = lim;
        }

        for (int i = lim2; i < rows; i++) {
            auto p = bs.ptr<uchar>(i);

            for (int j = 0; j < lim3; j++) {
                a_p = static_cast<double>(i) / (static_cast<double>(j) - c);
                if (a_p >= a) p[j] = 0;
            }
        }
    }

    //Traitement du bord gauche:

    a = static_cast<double>(corners[Corner::BOTTOM_LEFT].y - corners[Corner::TOP_LEFT].y) / (corners[Corner::BOTTOM_LEFT].x - corners[Corner::TOP_LEFT].x);
    b = ((corners[Corner::BOTTOM_LEFT].y + corners[Corner::TOP_LEFT].y) - a * (corners[Corner::BOTTOM_LEFT].x + corners[Corner::TOP_LEFT].x)) / 2.0;
    c = -(b / a);

    if (corners[Corner::TOP_LEFT].x > corners[Corner::BOTTOM_LEFT].x) {
        lim_c = c > cols ? cols : static_cast<int>(c);
        lim_r = b > rows ? rows : static_cast<int>(b);

        for (int i = 0; i < lim_r; i++) {
            auto p = bs.ptr<uchar>(i);

            if (i <= b) {
                for (int j = 0; j < lim_c; j++) {
                    if (j < c) {
                        a_p = static_cast<double>(i) / (static_cast<double>(j) - c);
                        if (a_p >= a) p[j] = 0;
                    }
                }
            }
        }
    }
    else {
        auto val = static_cast<int>((rows - b) / a);

        if (c < 0){lim = static_cast<int>(b);
            lim2 = lim;
            lim3 = cols;
        }
        else{
            lim = val < cols ? val : cols;
            lim2 = 0;
            lim3 = lim;
        }

        for (int i = lim2; i < rows; i++) {
            auto p = bs.ptr<uchar>(i);

            for (int j = 0; j < lim3; j++) {
                if (c < 0) {
                    a_p = static_cast<double>(i) / (static_cast<double>(j) - c);
                    if (a_p >= a) p[j] = 0;
                }
                else {
                    a_p = (static_cast<double>(rows - i)) / (static_cast<double>(val) - j);
                    if (a_p <= a) p[j] = 0;
                }

            }
        }
    }

    //Traitement du bord droit:

    a = static_cast<double>(corners[Corner::BOTTOM_RIGHT].y - corners[Corner::TOP_RIGHT].y) / (corners[Corner::BOTTOM_RIGHT].x - corners[Corner::TOP_RIGHT].x);
    b = ((corners[Corner::BOTTOM_RIGHT].y + corners[Corner::TOP_RIGHT].y) - a * (corners[Corner::BOTTOM_RIGHT].x + corners[Corner::TOP_RIGHT].x)) / 2.0;
    c = -(b / a);

    if (corners[Corner::BOTTOM_RIGHT].x > corners[Corner::TOP_RIGHT].x) {
        if (c < 0){
            lim = static_cast<int>(a * cols + b);
            lim2 = 0;
        }
        else{
            lim = static_cast<int>(a * cols + b) <= rows ? static_cast<int>(a * cols + b) : rows;
            lim2 = static_cast<int>(c) + 1;
        }

        for (int i = 0; i < lim; i++) {
            auto p = bs.ptr<uchar>(i);

            for (int j = lim2; j < cols; j++) {
                a_p = static_cast<double>(i) / (static_cast<double>(j) - c);
                if (a_p <= a) p[j] = 0;
            }
        }
    }
    else {
        lim_c = c > cols ? cols : static_cast<int>(c);
        lim_r = b > rows ? rows : static_cast<int>(b);

        for (int i = 0; i < lim_r; i++) {
            auto p = bs.ptr<uchar>(i);

            if (i <= b) {
                for (int j = 0; j < lim_c; j++) {
                    if (j < c) {
                        a_p = static_cast<double>(i) / (static_cast<double>(j) - c);
                        if (a_p >= a) p[j] = 0;
                    }
                }
            }
        }
    }

    return bs;
}

cv::Mat getTopographicMap(){
    return cv::imread(pathTopographicMap, 0);
}

cv::Mat getRealTopographicMap(){
    return cv::imread(pathRealTopographicMap, 0);
}

void pause(){
    while('c' != cv::waitKey(10)){}
}

void elapsedTime(std::chrono::steady_clock::time_point ti, double &time_s, int &min, int &sec)
{
    std::chrono::duration<double> time_span = CLOCK_DURATION(CLOCK_NOW - ti);

    time_s = time_span.count();
    min = static_cast<int>(time_s)/60;
    sec = static_cast<int>(time_s) % 60;
}

void ras(int num)
{
    std::cout << "\nRAS_" << num << " ..." << std::endl;
}

bool delay_s(double seconde)
{
    CLOCK_TIME_POINT ti;
    double duration = 0.0;
    std::chrono::duration<double> time_span = {};

    ti = CLOCK_NOW;
    do{
        time_span = CLOCK_DURATION(CLOCK_NOW - ti);
        duration = time_span.count();
    }while(duration <= seconde);

    return true;
}

void delay_ms(double milliseconde)
{
    CLOCK_TIME_POINT ti;
    double duration = 0.0;
    double seconde = milliseconde/1000.0;
    std::chrono::duration<double> time_span = {};

    ti = CLOCK_NOW;
    do{
        time_span = CLOCK_DURATION(CLOCK_NOW - ti);
        duration = time_span.count();
    }while(duration <= seconde);
}

void runClient(std::string msg)
{
    int conn;
    int response;
    WSADATA WSAData;
    SOCKET sock;
    SOCKADDR_IN serverAddr;
    socklen_t serverAddrLen;

    WSAStartup(MAKEWORD(2,2), &WSAData);

    sock = socket(AF_INET, SOCK_STREAM, 0);
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(50300);
    serverAddrLen = sizeof(serverAddr);

    conn = ::connect(sock, (SOCKADDR *)&serverAddr, serverAddrLen);

    if(conn >= 0){
        response = send(sock, msg.c_str(), sizeof(msg), 0);
        if(response == SOCKET_ERROR){
            std::cout << "L'envoi a echoue" << std::endl;
        }
    }else std::cout << "Connexion au serveur impossible (conn = " << conn << ")" << std::endl;
}

void runClient(const uint8* buff, size_t buffSize)
{
    int conn;
    int response;
    WSADATA WSAData;
    SOCKET sock;
    SOCKADDR_IN serverAddr;
    socklen_t serverAddrLen;

    WSAStartup(MAKEWORD(2,2), &WSAData);

    sock = socket(AF_INET, SOCK_STREAM, 0);
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(50300);
    serverAddrLen = sizeof(serverAddr);

    conn = ::connect(sock, (SOCKADDR *)&serverAddr, serverAddrLen);

    if(conn >= 0){
        response = send(sock, (char*)buff, static_cast<int>(buffSize), 0);
        if(response == SOCKET_ERROR){
            std::cout << "L'envoi a echoue" << std::endl;
        }
    }else std::cout << "Connexion au serveur impossible (conn = " << conn << ")" << std::endl;

    closesocket(sock);
    WSACleanup();
}

bool Serializer::write(uint8 data)
{
    return this->writeBytes(&data, 1);
}

bool Serializer::write(uint16 data)
{
    uint8 byte;
    int nbBit = 16;

    for(int i = 0; i < 2; i++){
        byte = static_cast<uint8>((data >> (nbBit - 8*(i+1))) & 0xFF);
        if(write(byte)) continue;
        else return false;
    }

    return true;
}

bool Serializer::write(uint32 data)
{
    uint8 byte;
    int nbBit = 32;

    for(int i = 0; i < 4; i++){
        byte = static_cast<uint8>((data >> (nbBit - 8*(i+1))) & 0xFF);
        if(write(byte)) continue;
        else return false;
    }

    return true;
}

bool Serializer::write(uint64 data)
{
    uint8 byte;
    int nbBit = 64;

    for(int i = 0; i < 8; i++){
        byte = static_cast<uint8>((data >> (nbBit - 8*(i+1))) & 0xFF);
        if(write(byte)) continue;
        else return false;
    }

    return true;
}

bool Serializer::write(int8 data)
{
    return this->write(*reinterpret_cast<uint8*>(&data));
}

bool Serializer::write(int16 data)
{
    return this->write(*reinterpret_cast<uint16*>(&data));
}

bool Serializer::write(int32 data)
{
    return this->write(*reinterpret_cast<uint32*>(&data));
}

bool Serializer::write(float32 data)
{
    uint32 conv;
    Havry::Float32ConversionHelper helper = {};
    helper.f = data;
    conv = helper.u;
    return write(conv);
}

bool Serializer::write(float64 data)
{
    uint64 conv;
    Havry::Float64ConversionHelper helper = {};
    helper.f = data;
    conv = helper.u;

    return write(conv);
}

bool Serializer::writeBytes(const uint8 *buffer, size_t nbBytes)
{
    this->mBuffer.insert(mBuffer.cend(), buffer, buffer + nbBytes);
    return true;
}

bool Deserializer::read(uint8 &data)
{
    return this->readBytes(1, &data);
}

bool Deserializer::read(uint16 &data)
{
    uint8 bytesRead[2];
    uint16 bytes[2];

    if(!readBytes(2, bytesRead)) return false;

    for(int i = 0; i < 2; i++){
        bytes[i] = static_cast<uint16>(bytesRead[i]);
    }

    data = (bytes[0] << 8u) | (bytes[1] << 0u);

    return true;
}

bool Deserializer::read(uint32 &data)
{
    uint8 bytesRead[4];
    uint32 bytes[4];

    if(!readBytes(4, bytesRead)) return false;

    for(int i = 0; i < 4; i++){
        bytes[i] = static_cast<uint32>(bytesRead[i]);
    }

    data = (bytes[0] << 24) |
           (bytes[1] << 16) |
           (bytes[2] <<  8) |
           (bytes[3] <<  0);

    return true;
}

bool Deserializer::read(uint64 &data)
{
    uint8 bytesRead[8];
    uint64 bytes[8];

    if(!readBytes(8, bytesRead)) return false;

    for(int i = 0; i < 8; i++){
        bytes[i] = static_cast<uint32>(bytesRead[i]);
    }

    data = (bytes[0] << 56) |
           (bytes[1] << 48) |
           (bytes[2] << 40) |
           (bytes[3] << 32) |
           (bytes[4] << 24) |
           (bytes[5] << 16) |
           (bytes[6] <<  8) |
           (bytes[7] <<  0);

    return true;
}

bool Deserializer::read(int8 &data)
{
    return this->read(reinterpret_cast<uint8&>(data));
}

bool Deserializer::read(int16 &data)
{
    return this->read(reinterpret_cast<uint16&>(data));
}

bool Deserializer::read(int32 &data)
{
    return this->read(reinterpret_cast<uint32&>(data));
}

bool Deserializer::read(float32 &data)
{
    uint8 conv[4];
    uint32 temp;
    Havry::Float32ConversionHelper helper = {};

    if(readBytes(4, reinterpret_cast<uint8*>(&conv))){
        temp = (static_cast<uint32>(conv[0]) << 24) | //Octet de poid fort
               (static_cast<uint32>(conv[1]) << 16) |
               (static_cast<uint32>(conv[2]) <<  8) |
               (static_cast<uint32>(conv[3]) <<  0);  //Octet de poid faible

        helper.u = temp;
        data = helper.f;

        return true;
    }
    return false;
}

bool Deserializer::read(float64 &data)
{
    uint8 conv[8];
    uint64 temp;
    Havry::Float64ConversionHelper helper = {};

    if(readBytes(8, reinterpret_cast<uint8*>(&conv))){
        temp = (static_cast<uint64>(conv[0]) << 56) | //Octet de poid fort
               (static_cast<uint64>(conv[1]) << 48) |
               (static_cast<uint64>(conv[2]) << 40) |
               (static_cast<uint64>(conv[3]) << 32) |
               (static_cast<uint64>(conv[4]) << 24) |
               (static_cast<uint64>(conv[5]) << 16) |
               (static_cast<uint64>(conv[6]) <<  8) |
               (static_cast<uint64>(conv[7]) <<  0);  //Octet de poid faible

        helper.u = temp;
        data = helper.f;

        return true;
    }
    return false;
}

void Deserializer::setBuffer(const uint8 *buffer)
{
    mBuffer = buffer;
}

void Deserializer::setBufferSize(const size_t bufferSize)
{
    this->mBufferSize = bufferSize;
}

bool Deserializer::readBytes(size_t nbBytes, uint8 *buffer)
{
    if(remainingBytes() < nbBytes) return false;

    for(size_t i = 0; i < nbBytes; ++i){
        buffer[i] = mBuffer[mBytesRead + i];
    }

    mBytesRead += nbBytes;
    return true;
}

void moveRobot(int lx, int ly, double az)
{
    double vx = lx/1000.0;
    double vy = ly/1000.0;
    uint8 index = Index::VELOCITIES;

    Serializer serial;

    serial.write(index);
    serial.write(vx);
    serial.write(vy);
    serial.write(az);

    runClient(serial.buffer(), serial.bufferSize());
}

void processNameAndID(DWORD &processID, TCHAR *szProcessName)
{
    HANDLE hProcess = OpenProcess( PROCESS_QUERY_INFORMATION |
                                   PROCESS_VM_READ,
                                   FALSE, processID );

    if (nullptr != hProcess )
    {
        HMODULE hMod;
        DWORD cbNeeded;

        if ( EnumProcessModules( hProcess, &hMod, sizeof(hMod),
             &cbNeeded) )
        {
            GetModuleBaseName( hProcess, hMod, szProcessName, 100);
        }
    }
    CloseHandle( hProcess );
}

bool findProcess(const std::string &processName, DWORD &processID)
{
    //Obtention de la liste des PID des processus lancés
    DWORD aProcesses[1024], cbNeeded, cProcesses;
    bool hasProcess = false;

    if ( !EnumProcesses( aProcesses, sizeof(aProcesses), &cbNeeded ) ) return false;

    //Calcul du nombre de processus retournés
    cProcesses = cbNeeded / sizeof(DWORD);

    //Récupération du nom et du PID de chaque processus
    for (unsigned int i = 0; i < cProcesses; i++ )
    {
        if( aProcesses[i] != 0 )
        {
            std::string nameP;
            TCHAR szProcessName[MAX_PATH];
            size_t pReturnValue;
            char newChar[PROCESS_BUFFER_SIZE];

            processNameAndID(aProcesses[i], szProcessName);

            wcstombs_s(&pReturnValue, newChar, static_cast<size_t>(PROCESS_BUFFER_SIZE), szProcessName, static_cast<size_t>(PROCESS_BUFFER_SIZE -1));

            nameP = string(newChar, pReturnValue);

            if(std::strcmp(processName.c_str(), nameP.c_str()) == 0){
                hasProcess = true;
                processID = aProcesses[i];
            }
        }
    }

    return hasProcess;
}

void generateBatchFile(const std::string &IPAddress)
{
    QFile file(QString::fromStdString(batchFile));
    std::string text = "@echo off\n"
                       "\n"
                       "set ROS_MASTER_URI=http://" + IPAddress + ":11311\n"
                       "set ROS_IP=" + IPAddress + "\n"
                       "\n"
                       "C:/opt/ros/noetic/x64/setup.bat && roscore";

    file.open(QFileDevice::WriteOnly);
    file.write(text.c_str());
    file.close();
}

bool validateIPAddress(QString IPAddress)
{
    QStringList strList = IPAddress.split(QChar('.'), Qt::SkipEmptyParts);
    bool ok;

    for(int i = 0; i < strList.size(); i++){
        int val = strList[i].toInt(&ok);

        if(!ok) break;
        else if((val < 0) || (val > 255)){
            ok = false;
            break;
        }
    }

    return ok;
}
