#ifndef UTILS_H
#define UTILS_H

//#include <Windows.h>
#include <WinSock2.h>
#include <tchar.h>
#include <Psapi.h>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <mbstring.h>
#include <winreg.h>
#include <stddef.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Advapi32.lib")

#include <QPixmap>
#include <QImage>
#include <QFile>
#include <QColor>
#include <QDir>
#include <QThread>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "types.h"

#define BUFFER_MAX 20000
#define PROCESS_BUFFER_SIZE 100
#define FILE_NOT_OPEN -1
#define SUCCESSFULL 0

#define RED_COLOR QColor(255, 0, 0)
#define GREEN_COLOR QColor(0, 255, 0)
#define WHITE_COLOR QColor(255, 255, 255)
#define BLACK_COLOR QColor(0, 0, 0)
#define YELLOW_COLOR QColor(255, 255, 0)
#define BLUE_COLOR QColor(0, 0, 255)

#define IMG_H 540
#define IMG_W 960
/*#define CHECKERBOARD_SQUARE_SIZE 23 // dimension du côté d'un carreau sur l'échiqier de calibration (en mm)
#define CHECKERBOARD_CORNERS_WIDTH 9 // nombre de coins sur la longueur de l'échiqier de calibration
#define CHECKERBOARD_CORNERS_HEIGHT 6 // nombre de coins sur la largeur de l'échiqier de calibration*/

//#define IHM_DEBUG

enum Camera {GAUCHE, MILIEU, DROITE};
enum HSVCode {HUE, SAT, VAL};
enum Corner {TOP_LEFT, TOP_RIGHT, BOTTOM_RIGHT, BOTTOM_LEFT};
enum Stereo {MIDDLE_LEFT, MIDDLE_RIGHT};
enum Color {RED, GREEN};
enum Position {RED_P, GREEN_P, CENTER};
enum Direction {LEFT_2_RIGHT = 1, RIGHT_2_LEFT = 0};
enum Index {VELOCITIES = 1, PARAMETERS = 2, HEIGHTS = 3, DELAY = 4,
            INIT_SENSOR_POS = 5, RPI_GO = 6, U_IHM = 7, EOM = 8,
            NEXT_MEASURE = 9, VEL_VOLT_OUT_TYPE = 10, STOP_MEASURE = 11};
enum CanalBGR {B = 0, G = 1, R = 2};
enum ExcelEntryRawData {
    ENTRY_POINT = 2,
    ENTRY_HEIGHT = 3,
    ENTRY_TEMP_DELAY = 4,
    ENTRY_TEMP = 5,
    ENTRY_VEL_DELAY = 6,
    ENTRY_VEL = 7
};
enum ExcelEntryReduceData {
    ENTRY_RD_POINT = 2,
    ENTRY_RD_HEIGHT = 3,
    ENTRY_RD_TEMP_MEAN = 4,
    ENTRY_RD_TEMP_DISPERSION = 5,
    ENTRY_RD_VEL_MEAN = 6,
    ENTRY_RD_VEL_DISPERSION = 7
};
enum CorrectorType {P = 0, PI = 1, PID = 2};
enum VelocityVoltageOutputType {TYPE_0_5_V = 0, TYPE_1_5_V = 1, TYPE_0_10_V = 2,
                                TYPE_2_10_V = 3, TYPE_0_20_mA = 4, TYPE_4_20_mA = 5};
enum RobotMove {FWD, BWD, LFT, RGT, CCW, CW, STOP};
enum Endianness {LITTLE = 10, BIG = 20};
enum ProjectNode {ROS_MASTER, PC_CONTROLLER, INTERMEDIARY, VELOCITY};

struct ParametersIHM
{
    float32 pulleyRadius;
    float32 motorAccuracy;
    uint8 motorAccuracyDivider;
    uint16 linVelMotorMast;
    uint8 dutyCycle;
};

struct Delay
{
    // en secondes
    uint8 restDelay;
    uint16 measureDelay;
    float32 periodDelay;
};

struct Velocities
{
    float64 vx;
    float64 vy;
    float64 omega_z;
};

struct HeightsList
{
    std::vector<uint32> heights;
};

struct Results
{
    std::vector<int32> heights;
    std::vector<float32> airSpeed;
    std::vector<float32> temperature;
    std::vector<float32> airSpeedTimer;
    std::vector<float32> temperatureTimer;
};

typedef struct ParametersIHM ParametersIHM;
typedef struct Delay Delay;
typedef struct Velocities Velocities;
typedef struct HeightsList HeightsList;
typedef struct Results Results;
typedef int socklen_t;

extern std::string projectDirectory;
extern std::string pathEmptyArea[3];
extern std::string HSVConfigFile[3];
extern std::string HSVConfigOriginFile[3];
extern std::string HSVRedLed[3];
extern std::string HSVGreenLed[3];
extern std::string HSVLed_R;
extern std::string HSVLed_V;
extern std::string patternConfig;
extern std::string intrinsicsCameraParametersFile[3];
extern std::string extrinsicsCameraParametersFile[3];
extern std::string stereocalibParametersFile[2];
extern std::string imageFiles4Calibration[3];
extern std::string imageFiles4StereoCalib[3];
extern std::string maskArea[3];
extern std::string pathTopographicMap;
extern std::string pathRealTopographicMap;
extern std::string pathImageObs1[3];
extern std::string pathImageObs2[3];
extern std::string pathImageRobotLeds[3];
extern std::string saveTrajectory;
extern std::string saveParamIHMFile;
extern std::string saveProject;
extern std::string camerasIDFile;
extern std::string saveCorrectorSettings;
extern std::string saveVVOType;
extern std::string batchFile;
extern std::string batchDirectory;
//extern std::string

extern double scale;

namespace Havry{
    union Float32ConversionHelper;
    union Float64ConversionHelper;
}

/*namespace Havry{
    namespace Endian{
        class EndianSERVER : public QThread
        {
        public:
            EndianSERVER() = default;
            ~EndianSERVER() = default;
            void run();
            std::string getLog() const;

        private:
            std::string log;
        };

        class EndianCLIENT : public QThread
        {
        public:
            EndianCLIENT() = default;
            ~EndianCLIENT() = default;
            void run();
            std::string getLog() const;

        private:
            std::string log;
        };
    }
}*/

class Serializer
{
public:
    Serializer() = default;

    bool write(uint8 data);
    bool write(uint16 data);
    bool write(uint32 data);
    bool write(uint64 data);
    bool write(int8 data);
    bool write(int16 data);
    bool write(int32 data);
    bool write(float32 data);
    bool write(float64 data);

    template<class T>
    bool write(const std::vector<T> &data)
    {
        this->mBuffer.reserve(this->mBuffer.size() + data.size() + 1);

        if(!write(static_cast<uint8>(data.size())))
            return false;

        for(T entry : data){
            if(!write(entry))
                return false;
        }
        return true;
    }

    inline const uint8* buffer() const {return mBuffer.data();}
    inline size_t bufferSize() const {return mBuffer.size();}
    inline void clearBuffer() {mBuffer.clear();}
private:
    std::vector<uint8> mBuffer;
private:
    bool writeBytes(const uint8* buffer, size_t nbBytes);
};

class Deserializer
{
public:
    Deserializer() = default;
    Deserializer(const uint8* buffer, const size_t bufferSize):
        mBuffer(buffer), mBufferSize(bufferSize)
    {}

    bool read(uint8& data);
    bool read(uint16& data);
    bool read(uint32& data);
    bool read(uint64& data);
    bool read(int8& data);
    bool read(int16& data);
    bool read(int32& data);
    bool read(float32& data);
    bool read(float64& data);

    template<typename T>
    bool read(std::vector<T>& data){
        uint8 nbElts;

        if(!read(nbElts))
            return false;

        data.reserve(nbElts);

        for(uint8 i = 0; i < nbElts; ++i){
            std::vector<T>::value_type element;

            if(!read(element))
                return false;
            data.push_back(element);
        }
        return true;
    }

    inline size_t remainingBytes() const {return mBufferSize - mBytesRead;}
    void setBuffer(const uint8 *buffer);
    void setBufferSize(const size_t bufferSize);

private:
    const uint8* mBuffer;
    size_t mBufferSize;
    size_t mBytesRead{0};
    int endianness;
private:
    bool readBytes(size_t nbBytes, uint8* buffer);
};

QPixmap  Mat2Pixmap(const cv::Mat &src, bool scaled = false, int scaleFactor = 1);

void loadImages(const std::string& dir_name, std::vector<cv::Mat>& img_list);

std::vector<cv::Point> sortCorners(std::vector<cv::Point> corners);

std::vector<std::vector<std::vector<int>>> readHSVConfig(bool readOri = false);

void read(uint camera, std::vector<std::vector<std::vector<int>>> &hsv, bool readOri = false);

bool saveHSVConfig(int camera,
                   int h_min, int h_max,
                   int s_min, int s_max,
                   int v_min, int v_max,
                   bool saveOrigin = false,
                   std::string msg = "");

cv::Mat workingAreaDelimiter(int rows, int cols, std::vector<cv::Point> corners, int foregroundColor = 255);

cv::Mat getTopographicMap();

cv::Mat getRealTpographicMap();

void pause();

void elapsedTime(CLOCK_TIME_POINT ti, double &time_s, int &min, int &sec);

void ras(int num = 1);

bool delay_s(double seconde = 1.0);
void delay_ms(double milliseconde = 1.0);

void runClient(std::string msg);
void runClient(const uint8* buff, size_t buffSize);

void moveRobot(int lx = 0, int ly = 0, double az = 0.0);

void processNameAndID(DWORD &processID, TCHAR *szProcessName);

bool findProcess(const std::string &processName, DWORD &processID);

void generateBatchFile(const std::string &IPAddress);

bool validateIPAddress(QString IPAddress);

#endif // UTILS_H
