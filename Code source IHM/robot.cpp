#include "robot.h"
#include "cameras.h"


using namespace std;
using namespace cv;

Robot::Robot(int i_diameter, cv::Mat i_toolZone, double i_maxLinVel, double i_maxAngVel):
    v_diameter(i_diameter),
    v_maxLinVel(i_maxLinVel),
    v_maxAngVel(i_maxAngVel),
    v_toolZone(generateToolZone(i_toolZone))
{}

cv::Mat Robot::generateToolZone(cv::Mat i_toolZone)
{
    int lengthToolZone = i_toolZone.rows;

    cv::Mat mesh;

    if (lengthToolZone > 0) {
        double deltaT = 20.0;
        double deltaHeight = 30.0;
        double heightMax;
        double widthMax;
        double angle;
        cv::Mat directionVector;
        double height;
        double t;
        cv::Mat P0;
        cv::Mat newPoint;
        cv::Mat polarCoordinates;
        cv::Mat invPolarCoordinates;

        for (int i = 0; i < lengthToolZone; i++) {
            heightMax = i_toolZone.ptr<double>(i)[0];
            widthMax = i_toolZone.ptr<double>(i)[1];
            angle = i_toolZone.ptr<double>(i)[2];
            directionVector = (cv::Mat_<double>(1, 2) << sin(angle), -cos(angle));
            height = sqrt(((v_diameter / 2.0) * (v_diameter / 2.0)) - ((widthMax / 2.0) * (widthMax / 2.0)));

            while (height <= heightMax) {
                t = -widthMax / 2.0;

                while (t <= (widthMax / 2.0)) {
                    P0 = height * (cv::Mat_<double>(1, 2) << cos(angle), sin(angle));
                    newPoint = P0 + t * directionVector;

                    if (norm(newPoint, cv::NORM_L2) > (v_diameter / 2.0)) {
                        polarCoordinates = MyMath::getPolarCoordinates(newPoint);
                        invPolarCoordinates = (cv::Mat_<double>(1, 2) << polarCoordinates.ptr<double>(0)[1], polarCoordinates.ptr<double>(0)[0]);
                        mesh.push_back(invPolarCoordinates);
                    }

                    t += deltaT;
                }

                if ((t - deltaT) < (widthMax / 2.0)) {
                    t = widthMax / 2.0;
                    P0 = height * (cv::Mat_<double>(1, 2) << cos(angle), sin(angle));
                    newPoint = P0 + t * directionVector;

                    if (norm(newPoint, cv::NORM_L2) > (v_diameter / 2.0)) {
                        polarCoordinates = MyMath::getPolarCoordinates(newPoint);
                        invPolarCoordinates = (cv::Mat_<double>(1, 2) << polarCoordinates.ptr<double>(0)[1], polarCoordinates.ptr<double>(0)[0]);
                        mesh.push_back(invPolarCoordinates);
                    }
                }
                height += deltaHeight;
            }


            if ((height - deltaHeight) < heightMax) {
                height = heightMax;
                t = -widthMax / 2;

                while (t <= (widthMax / 2.0)) {
                    P0 = height * (cv::Mat_<double>(1, 2) << cos(angle), sin(angle));
                    newPoint = P0 + t * directionVector;

                    if (norm(newPoint, cv::NORM_L2) > (v_diameter / 2.0)) {
                        polarCoordinates = MyMath::getPolarCoordinates(newPoint);
                        invPolarCoordinates = (cv::Mat_<double>(1, 2) << polarCoordinates.ptr<double>(0)[1], polarCoordinates.ptr<double>(0)[0]);
                        mesh.push_back(invPolarCoordinates);
                    }

                    t += deltaT;
                }

                if ((t - deltaT) < (widthMax / 2.0)) {
                    t = widthMax / 2.0;
                    P0 = height * (cv::Mat_<double>(1, 2) << cos(angle), sin(angle));
                    newPoint = P0 + t * directionVector;

                    if (norm(newPoint, cv::NORM_L2) > (v_diameter / 2.0)) {
                        polarCoordinates = MyMath::getPolarCoordinates(newPoint);
                        invPolarCoordinates = (cv::Mat_<double>(1, 2) << polarCoordinates.ptr<double>(0)[1], polarCoordinates.ptr<double>(0)[0]);
                        mesh.push_back(invPolarCoordinates);
                    }

                }
            }
        }
    }

    return mesh;
}

bool Robot::setVelocity(cv::Mat velocity)
{
    Velocities data;
    Serializer serializer;

    uint8 index = Index::VELOCITIES;
    data.vx = velocity.ptr<double>(0)[0];
    data.vy = velocity.ptr<double>(0)[1];
    data.omega_z = velocity.ptr<double>(0)[2];

    serializer.clearBuffer();
    serializer.write(index);
    serializer.write(data.vx);
    serializer.write(data.vy);
    serializer.write(data.omega_z);

    runClient(serializer.buffer(), serializer.bufferSize());

    return true;
}

int Robot::getDiameter() const
{
    return v_diameter;
}

void Robot::setDiameter(int value)
{
    v_diameter = value;
}

double Robot::getMaxLinVel() const
{
    return v_maxLinVel;
}

void Robot::setMaxLinVel(double value)
{
    v_maxLinVel = value;
}

double Robot::getMaxAngVel() const
{
    return v_maxAngVel;
}

void Robot::setMaxAngVel(double value)
{
    v_maxAngVel = value;
}

cv::Mat Robot::getToolZone() const
{
    return v_toolZone;
}

void Robot::setToolZone(const cv::Mat &value)
{
    v_toolZone = value;
}

cv::Mat Robot::getSensorsPosition() const
{
    return v_sensorsPosition;
}

void Robot::setSensorsPosition(const cv::Mat &value)
{
    v_sensorsPosition = value;
}

std::vector<cv::Point> Robot::getPosition4Measure() const
{
    return v_position4Measure;
}

void Robot::setPosition4Measure(const std::vector<cv::Point> &value)
{
    v_position4Measure = value;
}


cv::Mat Robot::getPosition(Cameras &cam)
{
    Mat position;

    position = cam.getRobotPosition();
    //position.ptr<double>(0)[0] = position.ptr<double>(0)[0] + 50*cos(position.ptr<double>(0)[2]); //position x
    //position.ptr<double>(0)[1] = position.ptr<double>(0)[1] + 50*sin(position.ptr<double>(0)[2]); //position y

    return position;
}

bool Robot::initSensorPosition()
{
    int conn;
    int result;
    uint8 index;
    Serializer serializer;

    WSADATA WSAData;
    SOCKET sock;
    SOCKADDR_IN serverAddr;
    socklen_t serverAddrLen;
    uint8 state;

    index = Index::INIT_SENSOR_POS;
    serializer.write(index);

    WSAStartup(MAKEWORD(2,2), &WSAData);

    sock = socket(AF_INET, SOCK_STREAM, 0);
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(50300);
    serverAddrLen = sizeof(serverAddr);

    conn = ::connect(sock, (SOCKADDR *)&serverAddr, serverAddrLen);

    if(conn >= 0){
        result = send(sock, (char*)serializer.buffer(), static_cast<int>(serializer.bufferSize()), 0);
        if(result == SOCKET_ERROR){
            std::cout << "L'envoi a echoue" << std::endl;
            return false;
        }
        result = recv(sock, (char*)&state, sizeof(state), 0);

        if(result == SOCKET_ERROR){
            std::cout << "(InitSensorPosition) : Echec lors de la réception des donnees." << std::endl;
            return false;
        }

        if(state != 1){
            std::cout << "(InitSensorPosition): La position des capteurs n'a pas pu etre initialisee." << std::endl;
            return false;
        }

    }else{
        std::cout << "Connexion au serveur impossible (conn = " << conn << ")" << std::endl;
        return false;
    }

    closesocket(sock);
    WSACleanup();

    return true;
}
