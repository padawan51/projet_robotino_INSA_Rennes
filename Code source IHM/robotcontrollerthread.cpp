#include "robotcontrollerthread.h"

RobotControllerThread::RobotControllerThread()
{
    qRegisterMetaType<cv::Mat>();
    this->v_isInit = false;
}

void RobotControllerThread::run()
{
    moveInPath();
}

void RobotControllerThread::init(const Robot &i_robot,
                                 std::vector<cv::Mat> i_path,
                                 std::vector<Cameras*> i_cam,
                                 std::vector<cv::Point> reachablePoints,
                                 bool takeMeasure,
                                 bool manualControl){
    cv::FileStorage fs(saveCorrectorSettings, cv::FileStorage::READ);

    if(fs.isOpened()){
        v_Kp = fs["Kp_position"];
        v_Kp_phi = fs["Kp_orientation"];
    }

    fs.release();

    this->v_robot = i_robot;
    this->v_path = i_path;
    this->v_cam = i_cam;
    this->v_manualControl = manualControl;
    this->v_takeMeasure = takeMeasure;
    this->v_reachablePoints = reachablePoints;

    this->v_isInit = true;
}

void RobotControllerThread::moveInPath(){
    if(this->v_isInit){
#ifdef MOVE
        double positionError;
        double angularError;
        double Vx_real;
        double Vy_real;
        double Vx;
        double Vy;
        double Vphi;
        double velocityMagnitude;
        int count;
        bool only_linear_vel_control = false;
        CLOCK_TIME_POINT timer_start;
        std::chrono::duration<double> time_span = {};
        double deltaT;
#endif

        size_t pathSize;
        int sz;

        cv::Mat position;
        cv::Mat position_new;
        cv::Mat trajectory;

        double restTime;
        double minAngularVel;
        int toolLen;
        cv::Point2d pm;
        cv::FileStorage fs;
        std::vector<Results> rawData;
        std::vector<Data> reducedData;
        std::vector<cv::Point2d> measurementPoints;

        fs.open(saveParamIHMFile, cv::FileStorage::READ);
        fs["rest_time"] >> restTime;
        fs["tool_length"] >> toolLen;
        fs["min_angular_velocity"] >> minAngularVel;
        fs.release();

#ifdef MOVE
        calcPosition(position);

        if((position.ptr<double>(0)[0] < 0) || (position.ptr<double>(0)[1] < 0)){
            std::cout << "Move robot to a position were cameras can mesure his position!" << std::endl << std::endl;
            return;
        }
#endif

        pathSize = this->v_path.size();

        for(uint i = 0; i < pathSize; i++){
            trajectory = this->v_path[i];
            sz = trajectory.rows;

            if(v_robot.initSensorPosition()){
#ifdef MOVE
                positionError = 100.0;
                angularError = 0.0;
                Vx_real = 0.0;
                Vy_real = 0.0;
                Vx = 0.0;
                Vy = 0.0;
                Vphi = 0.0;
                velocityMagnitude = 0.0;
                count = 0;

                for(int j = 0; j < sz; j++){
                    if(static_cast<int>(trajectory.ptr<double>(j)[3]) == 0) only_linear_vel_control = true;
                    else if(static_cast<int>(trajectory.ptr<double>(j)[3]) == 1) only_linear_vel_control = false;

                    timer_start = CLOCK_NOW;

                    do{
                        calcPosition(position_new);

                        time_span = CLOCK_DURATION(CLOCK_NOW - timer_start);
                        deltaT = time_span.count();

                        if(position_new.ptr<double>(0)[0] > 0 && position_new.ptr<double>(0)[1] > 0){
                            position.ptr<double>(0)[0] =
                                    position.ptr<double>(0)[0] +
                                    deltaT*Vx_real +
                                    0.2*(position_new.ptr<double>(0)[0] - (position.ptr<double>(0)[0] + deltaT*Vx_real));

                            position.ptr<double>(0)[1] =
                                    position.ptr<double>(0)[1] +
                                    deltaT*Vx_real +
                                    0.2*(position_new.ptr<double>(0)[1] - (position.ptr<double>(0)[1] + deltaT*Vy_real));

                            position.ptr<double>(0)[2] =
                                    position.ptr<double>(0)[2] +
                                    deltaT*Vx_real +
                                    0.2*(position_new.ptr<double>(0)[2] - (position.ptr<double>(0)[2] + deltaT*Vphi));
                        }else{
                            position.ptr<double>(0)[0] = position.ptr<double>(0)[0] + deltaT*Vx_real;
                            position.ptr<double>(0)[1] = position.ptr<double>(0)[1] + deltaT*Vy_real;
                            position.ptr<double>(0)[2] = position.ptr<double>(0)[2] + deltaT*Vphi;
                        }

                        positionError = sqrt(
                                    ((trajectory.ptr<double>(j)[0] - position.ptr<double>(0)[0]) * (trajectory.ptr<double>(j)[0] - position.ptr<double>(0)[0]))
                                    +
                                    ((trajectory.ptr<double>(j)[1] - position.ptr<double>(0)[1]) * (trajectory.ptr<double>(j)[1] - position.ptr<double>(0)[1]))
                                    );

                        angularError = trajectory.ptr<double>(j)[2] - position.ptr<double>(0)[2];

                        cv::Mat direction = (cv::Mat_<double>(1,3) <<
                                         trajectory.ptr<double>(j)[0] - position.ptr<double>(0)[0],
                                         trajectory.ptr<double>(j)[1] - position.ptr<double>(0)[1],
                                         trajectory.ptr<double>(j)[2] - position.ptr<double>(0)[2]);

                        double error = sqrt(
                                            (direction.ptr<double>(0)[0] * direction.ptr<double>(0)[0]) +
                                            (direction.ptr<double>(0)[1] * direction.ptr<double>(0)[1]) +
                                            (direction.ptr<double>(0)[2] * direction.ptr<double>(0)[2])
                                           );

                        direction = direction/error;
                        Vx_real = v_Kp*error*(direction.ptr<double>(0)[0]);
                        Vy_real = v_Kp*error*(direction.ptr<double>(0)[1]);

                        Vx = cos(position.ptr<double>(0)[2])*Vx_real + sin(position.ptr<double>(0)[2])*Vy_real;
                        Vy = -sin(position.ptr<double>(0)[2])*Vx_real + cos(position.ptr<double>(0)[2])*Vy_real;

                        velocityMagnitude = sqrt((Vx * Vx) + (Vy * Vy));
                        Vphi = v_Kp_phi*error*(direction.ptr<double>(0)[2]);

                        if(velocityMagnitude > this->v_robot.getMaxLinVel()){
                            Vx_real *= (this->v_robot.getMaxLinVel()/velocityMagnitude);
                            Vy_real *= (this->v_robot.getMaxLinVel()/velocityMagnitude);

                            Vx *= (this->v_robot.getMaxLinVel()/velocityMagnitude);
                            Vy *= (this->v_robot.getMaxLinVel()/velocityMagnitude);
                            Vphi *= (this->v_robot.getMaxLinVel()/velocityMagnitude);
                        }

                        if(abs(Vphi) > this->v_robot.getMaxAngVel()){
                            Vx_real = Vx_real*(this->v_robot.getMaxAngVel()/abs(Vphi));
                            Vy_real = Vy_real*(this->v_robot.getMaxAngVel()/abs(Vphi));

                            Vx = Vx*(this->v_robot.getMaxAngVel()/abs(Vphi));
                            Vy = Vy*(this->v_robot.getMaxAngVel()/abs(Vphi));
                            Vphi = Vphi*(this->v_robot.getMaxAngVel()/abs(Vphi));
                        }

                        Vx /= 1000.0;
                        Vy /= 1000.0;

                        cv::Mat temp = (cv::Mat_<double>(1,3) << Vx, Vy, Vphi);
                        this->v_robot.setVelocity(temp);

                        count++;

                        if(count > 2000) break;

                    }while((positionError > 10 ||
                           angularError > 0.035 ||
                           velocityMagnitude > 50 ||
                           (Vphi > 0.1 && !only_linear_vel_control)) && !isInterruptionRequested());

                    if(isInterruptionRequested()) break;
                }

                cv::Mat temp = (cv::Mat_<double>(1,3) << 0.0, 0.0, 0.0);
                this->v_robot.setVelocity(temp);
#endif

                if(!isInterruptionRequested()){
                    pm.x = v_reachablePoints[i].y;
                    pm.y = v_reachablePoints[i].x;

                    if(v_takeMeasure){
                        if(waitEndOfMeasurements(rawData)){
                            measurementPoints.push_back(pm);
                            data(rawData[i], reducedData);
                        }
                    }
                }
            }

            emit measurementPoint(static_cast<int>(i)+1);

            if(isInterruptionRequested()) break;
        }

        if(!measurementPoints.empty() && v_takeMeasure)
        {
            saveData(rawData, reducedData, measurementPoints);
        }

        emit measureFinish();
    }
}

void RobotControllerThread::calcPosition(cv::Mat &robotPosition)
{
    int _count = 0;
    int areaWidth, areaHigh;
    std::vector<bool> positionIsFound(3);
    std::vector<cv::Mat> robotPositionByCam(3);
    cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);

    fs["working_area_width"] >> areaWidth;
    fs["working_area_length"] >> areaHigh;


    for(uint i = 0; i < 3; i++){
        positionIsFound[i] = false;
    }

    for(uint i = 0; i < 3; i++){
        v_cam[i]->capture();
    }

    for(uint i = 0; i < 3; i++){
        robotPositionByCam[i] = v_cam[i]->getRobotPosition();

        if(robotPositionByCam[i].ptr<double>(0)[0] > 0 &&
           robotPositionByCam[i].ptr<double>(0)[1] > 0 &&
           robotPositionByCam[i].ptr<double>(0)[0] <= areaWidth &&
           robotPositionByCam[i].ptr<double>(0)[1] <= areaHigh){
           _count++;
           positionIsFound[i] = true;
        }
    }

    if(_count > 0){
        double pos_cam_L_x = robotPositionByCam[Camera::GAUCHE].ptr<double>(0)[0];
        double pos_cam_L_y = robotPositionByCam[Camera::GAUCHE].ptr<double>(0)[1];
        double pos_cam_L_t = robotPositionByCam[Camera::GAUCHE].ptr<double>(0)[2];
        double weight_L = positionIsFound[Camera::GAUCHE] ? 1.0 : 0.0;

        double pos_cam_M_x = robotPositionByCam[Camera::MILIEU].ptr<double>(0)[0];
        double pos_cam_M_y = robotPositionByCam[Camera::MILIEU].ptr<double>(0)[1];
        double pos_cam_M_t = robotPositionByCam[Camera::MILIEU].ptr<double>(0)[2];
        double weight_M = positionIsFound[Camera::MILIEU] ? 1.0 : 0.0;

        double pos_cam_R_x = robotPositionByCam[Camera::DROITE].ptr<double>(0)[0];
        double pos_cam_R_y = robotPositionByCam[Camera::DROITE].ptr<double>(0)[1];
        double pos_cam_R_t = robotPositionByCam[Camera::DROITE].ptr<double>(0)[2];
        double weight_R = positionIsFound[Camera::DROITE] ? 1.0 : 0.0;

        cv::Mat weights = (cv::Mat_<double>(1, 3) <<
                          weight_L, weight_M, weight_R);

        cv::Mat pos_cam = (cv::Mat_<double>(3, 3) <<
                           pos_cam_L_x, pos_cam_L_y, pos_cam_L_t,
                           pos_cam_M_x, pos_cam_M_y, pos_cam_M_t,
                           pos_cam_R_x, pos_cam_R_y, pos_cam_R_t);

        robotPosition = weights * pos_cam;
        robotPosition /= _count;
    }
    else{
        robotPosition = (cv::Mat_<double>(1,3) <<
                                POSITION_NOT_FOUND,
                                POSITION_NOT_FOUND,
                                POSITION_NOT_FOUND);
    }

    emit realRobotPosition(robotPosition);
}

void RobotControllerThread::takeMeasures()
{
    Serializer serializer;
    uint8 index = Index::RPI_GO;

    serializer.write(index);
    runClient(serializer.buffer(), serializer.bufferSize());
}

bool RobotControllerThread::waitEndOfMeasurements(std::vector<Results> &rawData)
{
    bool isFinish = false;
    int conn;
    int nbBytes;
    uint8 index;

    WSADATA WSAData;
    SOCKET sock;
    SOCKADDR_IN serverAddr;
    socklen_t serverAddrLen;
    uint8 buff[BUFFER_MAX];

    WSAStartup(MAKEWORD(2,2), &WSAData);

    sock = socket(AF_INET, SOCK_STREAM, 0);
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(50300);
    serverAddrLen = sizeof(serverAddr);

    conn = ::connect(sock, (SOCKADDR *)&serverAddr, serverAddrLen);

    if(conn >= 0){
        Serializer serial;
        index = Index::RPI_GO;
        serial.write(index);
        send(sock, (char*)serial.buffer(), static_cast<int>(serial.bufferSize()), 0);

        do{
            if((nbBytes = ::recv(sock, (char*)buff, BUFFER_MAX, 0)) != SOCKET_ERROR){

                Deserializer deserializer(buff, static_cast<size_t>(nbBytes));
                deserializer.read(index);

                switch (index) {
                case Index::U_IHM:
                {
                    int32 val;
                    Serializer serial2;

                    deserializer.read(val);
                    emit processedMeasure(val);

                    index = Index::NEXT_MEASURE;
                    serial2.write(index);

                    runClient(serial2.buffer(), serial2.bufferSize());
                    break;
                }

                case Index::EOM:
                {
                    Results results;

                    deserializer.read(results.heights);
                    deserializer.read(results.airSpeed);
                    deserializer.read(results.airSpeedTimer);
                    deserializer.read(results.temperature);
                    deserializer.read(results.temperatureTimer);

                    rawData.push_back(results);

                    isFinish = true;
                    break;
                }
                }
            }

            if(isInterruptionRequested()) break;

        }while(!isFinish);
    }

    closesocket(sock);
    WSACleanup();

    return isFinish;
}

void RobotControllerThread::data(Results &rawData, std::vector<Data> &reduceData)
{
    int32 currentHeight = rawData.heights[0];
    size_t sz = rawData.heights.size();
    float restTime;
    float temperatureMean = 0.;
    float temperatureDispersion = 0.;
    float airSpeedMean = 0.;
    float airSpeedDispersion = 0.;
    int count = 0;
    Data result;

    cv::FileStorage fs(saveParamIHMFile, cv::FileStorage::READ);

    fs["rest_time"] >> restTime;
    fs.release();

    for(uint i = 0; i < sz; i++){
        if((rawData.airSpeedTimer[i] >= (restTime + 1)) && (rawData.temperatureTimer[i] >= (restTime + 1))){
            if(currentHeight == rawData.heights[i]){
                temperatureMean += rawData.temperature[i];
                temperatureDispersion += (rawData.temperature[i] * rawData.temperature[i]);
                airSpeedMean += rawData.airSpeed[i];
                airSpeedDispersion += (rawData.airSpeed[i] * rawData.airSpeed[i]);
                count++;
            }else{
                temperatureMean /= count;
                temperatureDispersion /= count;
                temperatureDispersion -= (temperatureMean * temperatureMean);
                temperatureDispersion = sqrt(temperatureDispersion);

                airSpeedMean /= count;
                airSpeedDispersion /= count;
                airSpeedDispersion -= (airSpeedMean * airSpeedMean);
                airSpeedDispersion = sqrt(airSpeedDispersion);

                result.heights.push_back(currentHeight);
                result.temperatureMean.push_back(temperatureMean);
                result.temperatureDispersion.push_back(temperatureDispersion);
                result.airSpeedMean.push_back(airSpeedMean);
                result.airSpeedDispersion.push_back(airSpeedDispersion);

                temperatureMean = 0.;
                temperatureDispersion = 0.;
                airSpeedMean = 0.;
                airSpeedDispersion = 0.;
                currentHeight = rawData.heights[i];
                count = 0;

                temperatureMean += rawData.temperature[i];
                temperatureDispersion += (rawData.temperature[i] * rawData.temperature[i]);
                airSpeedMean += rawData.airSpeed[i];
                airSpeedDispersion += (rawData.airSpeed[i] * rawData.airSpeed[i]);
                count++;
            }

            if(i == (sz-1)){
                temperatureMean /= count;
                temperatureDispersion /= count;
                temperatureDispersion -= (temperatureMean * temperatureMean);
                temperatureDispersion = sqrt(temperatureDispersion);

                airSpeedMean /= count;
                airSpeedDispersion /= count;
                airSpeedDispersion -= (airSpeedMean * airSpeedMean);
                airSpeedDispersion = sqrt(airSpeedDispersion);

                result.heights.push_back(currentHeight);
                result.temperatureMean.push_back(temperatureMean);
                result.temperatureDispersion.push_back(temperatureDispersion);
                result.airSpeedMean.push_back(airSpeedMean);
                result.airSpeedDispersion.push_back(airSpeedDispersion);
            }
        }
    }

    reduceData.push_back(result);
}

void RobotControllerThread::saveData(std::vector<Results> &rawData, std::vector<Data> &reduceData, std::vector<cv::Point2d> &measurementPoints)
{
    QFile file(QString::fromStdString(saveProject) + "current_project.name");
    char data[255];
    bool save = false;
    std::string pfn;

    if(!v_manualControl){
        if(file.open(QIODevice::OpenModeFlag::ReadOnly)){
            file.readLine(data, 255);
            std::string projectFileName(data);

            pfn = projectFileName;
            file.remove();
        }
        save = true;
    }
    else{
        pfn = "free_robot_position";
        save = true;
    }

    if(save){
        QXlsx::Document document(QString::fromStdString(saveProject + pfn + ".xlsx"));
        QXlsx::Format headingFormat;
        QXlsx::Format valueFormat;
        int headingRow = 2;
        int row = headingRow;
        double columnWidth = 22;
        size_t sz = measurementPoints.size();
        bool changeBackgroundColor = false;
        QXlsx::AbstractSheet *sheetName = document.currentSheet();

        document.renameSheet(sheetName->sheetName(), "raw_data");

        headingFormat.setHorizontalAlignment(QXlsx::Format::AlignHCenter);
        headingFormat.setVerticalAlignment(QXlsx::Format::AlignVCenter);
        headingFormat.setBorderStyle(QXlsx::Format::BorderMedium);
        headingFormat.setFontBold(true);
        headingFormat.setPatternBackgroundColor(QColor(0,100,255));

        valueFormat.setHorizontalAlignment(QXlsx::Format::AlignHCenter);
        valueFormat.setVerticalAlignment(QXlsx::Format::AlignVCenter);
        valueFormat.setBorderStyle(QXlsx::Format::BorderMedium);

        //Sauvegarde des données brutes
        document.setColumnWidth(ExcelEntryRawData::ENTRY_POINT, columnWidth);
        document.setColumnWidth(ExcelEntryRawData::ENTRY_HEIGHT, columnWidth);
        document.setColumnWidth(ExcelEntryRawData::ENTRY_TEMP_DELAY, columnWidth);
        document.setColumnWidth(ExcelEntryRawData::ENTRY_TEMP, columnWidth);
        document.setColumnWidth(ExcelEntryRawData::ENTRY_VEL_DELAY, columnWidth);
        document.setColumnWidth(ExcelEntryRawData::ENTRY_VEL, columnWidth);

        document.write(headingRow, ExcelEntryRawData::ENTRY_POINT, "Point (x;y)(mm)", headingFormat);
        document.write(headingRow, ExcelEntryRawData::ENTRY_HEIGHT, "Hauteur (mm)", headingFormat);
        document.write(headingRow, ExcelEntryRawData::ENTRY_TEMP_DELAY, "Temps Température (s)", headingFormat);
        document.write(headingRow, ExcelEntryRawData::ENTRY_TEMP, "Température (°)", headingFormat);
        document.write(headingRow, ExcelEntryRawData::ENTRY_VEL_DELAY, "Temps Vitesse (s)", headingFormat);
        document.write(headingRow, ExcelEntryRawData::ENTRY_VEL, "Vitesse (m/s)", headingFormat);

        for(uint i = 0; i < sz; i++){
            int firstRow = row + 1;
            int firstColumn = ExcelEntryRawData::ENTRY_POINT;
            int lastRow = row + static_cast<int>(rawData[i].heights.size());
            int lastColumn = firstColumn;

            row = lastRow;

            changeBackgroundColor ?
                        valueFormat.setPatternBackgroundColor(QColor(0, 200, 255)) :
                        valueFormat.setPatternBackgroundColor(QColor(255, 255, 255));

            document.write(firstRow,
                           ExcelEntryRawData::ENTRY_POINT,
                           QString::fromStdString("(" +
                                                  std::to_string(static_cast<int>(measurementPoints[i].x)) +
                                                  " ; " +
                                                  std::to_string(static_cast<int>(measurementPoints[i].y)) +
                                                  ")"),
                           valueFormat);
            document.mergeCells(QXlsx::CellRange(firstRow, firstColumn, lastRow, lastColumn), valueFormat);

            for(uint j = 0; j < rawData[i].heights.size(); j++){
                document.write(firstRow + static_cast<int>(j), ExcelEntryRawData::ENTRY_HEIGHT, rawData[i].heights[j], valueFormat);
                document.write(firstRow + static_cast<int>(j), ExcelEntryRawData::ENTRY_TEMP_DELAY, rawData[i].temperatureTimer[j], valueFormat);
                document.write(firstRow + static_cast<int>(j), ExcelEntryRawData::ENTRY_TEMP, rawData[i].temperature[j], valueFormat);
                document.write(firstRow + static_cast<int>(j), ExcelEntryRawData::ENTRY_VEL_DELAY, rawData[i].airSpeedTimer[j], valueFormat);
                document.write(firstRow + static_cast<int>(j), ExcelEntryRawData::ENTRY_VEL, rawData[i].airSpeed[j], valueFormat);
            }

            changeBackgroundColor = 1 - changeBackgroundColor;
        }

        //Sauvegarde des données réduites
        document.addSheet("reduce_data");
        document.selectSheet("reduce_data");

        document.setColumnWidth(ExcelEntryReduceData::ENTRY_RD_POINT, columnWidth);
        document.setColumnWidth(ExcelEntryReduceData::ENTRY_RD_HEIGHT, columnWidth);
        document.setColumnWidth(ExcelEntryReduceData::ENTRY_RD_TEMP_MEAN, columnWidth);
        document.setColumnWidth(ExcelEntryReduceData::ENTRY_RD_TEMP_DISPERSION, columnWidth);
        document.setColumnWidth(ExcelEntryReduceData::ENTRY_RD_VEL_MEAN, columnWidth);
        document.setColumnWidth(ExcelEntryReduceData::ENTRY_RD_VEL_DISPERSION, columnWidth);

        document.write(headingRow, ExcelEntryReduceData::ENTRY_RD_POINT, "Point (x;y)(mm)", headingFormat);
        document.write(headingRow, ExcelEntryReduceData::ENTRY_RD_HEIGHT, "Hauteur (mm)", headingFormat);
        document.write(headingRow, ExcelEntryReduceData::ENTRY_RD_TEMP_MEAN, "Température (°)", headingFormat);
        document.write(headingRow, ExcelEntryReduceData::ENTRY_RD_TEMP_DISPERSION, "écart-type (°)", headingFormat);
        document.write(headingRow, ExcelEntryReduceData::ENTRY_RD_VEL_MEAN, "Vitesse (m/s)", headingFormat);
        document.write(headingRow, ExcelEntryReduceData::ENTRY_RD_VEL_DISPERSION, "écart-type (m/s)", headingFormat);

        changeBackgroundColor = false;
        row = headingRow;

        for(uint i = 0; i < sz; i++){
            int firstRow = row + 1;
            int firstColumn = ExcelEntryReduceData::ENTRY_RD_POINT;
            int lastRow = row + static_cast<int>(reduceData[i].heights.size());
            int lastColumn = firstColumn;

            row = lastRow;

            changeBackgroundColor ?
                        valueFormat.setPatternBackgroundColor(QColor(0, 200, 255)) :
                        valueFormat.setPatternBackgroundColor(QColor(255, 255, 255));

            document.write(firstRow,
                           ExcelEntryReduceData::ENTRY_RD_POINT,
                           QString::fromStdString("(" +
                                                  std::to_string(static_cast<int>(measurementPoints[i].x)) +
                                                  " ; " +
                                                  std::to_string(static_cast<int>(measurementPoints[i].y)) +
                                                  ")"),
                           valueFormat);
            document.mergeCells(QXlsx::CellRange(firstRow, firstColumn, lastRow, lastColumn), valueFormat);

            for(uint j = 0; j < reduceData[i].heights.size(); j++){
                document.write(firstRow + static_cast<int>(j), ExcelEntryReduceData::ENTRY_RD_HEIGHT, reduceData[i].heights[j], valueFormat);
                document.write(firstRow + static_cast<int>(j), ExcelEntryReduceData::ENTRY_RD_TEMP_MEAN, reduceData[i].temperatureMean[j], valueFormat);
                document.write(firstRow + static_cast<int>(j), ExcelEntryReduceData::ENTRY_RD_TEMP_DISPERSION, reduceData[i].temperatureDispersion[j], valueFormat);
                document.write(firstRow + static_cast<int>(j), ExcelEntryReduceData::ENTRY_RD_VEL_MEAN, reduceData[i].airSpeedMean[j], valueFormat);
                document.write(firstRow + static_cast<int>(j), ExcelEntryReduceData::ENTRY_RD_VEL_DISPERSION, reduceData[i].airSpeedDispersion[j], valueFormat);
            }

            changeBackgroundColor = 1 - changeBackgroundColor;
        }

        document.save();
    }
}
