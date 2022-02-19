#ifndef CAMERAS_H
#define CAMERAS_H

#include <QObject>
#include <QDir>
#include <QtMath>

#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <math.h>

#include "acquisitionthread.h"
#include "utils.h"
#include "workingarea.h"
#include "robot.h"
#include "robotpositionthread.h"

/**
 * @brief Cette classe permet d'instancier un objet de type Camera
 * @version 0.1
 * @date 2021
 * @author Vaneck MVONDO (vaneck.mvondo@hotmail.com)
 */

class Cameras: public QObject
{
    Q_OBJECT

public:
    /**
     * @brief Constructeur par défaut de la classe Camera
     */
    Cameras() = default;

    /**
     * @brief Constructeur surchargé de la classe Camera
     * @param id Identifiant de la caméra à instancier
     * @param camPosition Position de la caméra dans la chambre climatique (les valeurs prises sont : gauche, droite ou milieu)
     * @param useCamera Si true, alors la caméras est utilisée, sinon c'est une image enregistrée qui est utilisée
     */
    Cameras(int id, std::string camPosition, bool useCamera = true);

    ~Cameras() = default;

    friend class RobotPositionThread;

    /**
     * @brief Accesseur permettant d'obtenir l'identifiant de la caméra
     * @return Retourne un entier correspondant à l'identifiant de la caméra
     */
    inline int getId(){return this->v_id;}

    /**
     * @brief Accesseur permettant d'obtenir le code attribué à la caméra selon sa position
     * @return Retourne un entier correspondant au code de la caméra
     */
    inline int getCodeCam(){return this->v_codeCam;}

    /**
     * @brief Accesseur permettant d'obtenir la position de la caméra
     * @return Retourne une chaîne de caractère correspondant à la position de la caméra dans la pièce
     */
    inline std::string getCamPosition(){return this->v_camPosition;}

    /**
     * @brief Permet d'obtenir la matrix de projection de la caméra à partir de ses paramètres intrinsèques et extrinsèques
     * @return Retourne un objet de type cv::Mat correspondant à la matrice de projection de la caméra
     */
    cv::Mat getProjectionMatrix();

    /**
     * @brief Permet d'obtenir des coordonnées réels d'un point dont les coordonnées sont fournis en pixels
     * @param pixel Point dont les coordonnées sont données en pixel et dont-on souhaite retrouvé les coordonnées réelles
     * @param z Hauteur à laquelle se trouve le point en coordonnée réelle
     * @return Retourne un objet de type cv::Point3d contenant les coordonnées réelles du point
     */
    cv::Point3d getRealPosition(cv::Point pixel, int z);

    /**
     * @brief Permet d'obtenir les coordonnées en pixel d'un point dont on a les coordonnées réelles
     * @param realPosition Objet de type cv::Mat contenant les coordonnées réelles
     * @return Retourne un objet de type cv::Point2i contenant les coordonnées en pixel du point 3D
     */
    cv::Point2i getPixelPosition(cv::Point3i realPosition);

    /**
     * @brief Permet d'obtenir la position du robot dans la zone de travail
     * @return Retourne un objet de type cv::Mat contenant la pose du robot
     */
    cv::Mat getRobotPosition();

    /**
     * @brief Permet de vérifier si le fichier de calibration de la caméra est disponible
     * @return Retourne un booléen pour indiquer si le fichier est disponible (true = disponible)
     */
    bool hasCalibFile();

    /**
     * @brief Permet de savoir si la caméra a été initialisée
     * @return Retourne un booléen pour indiquer si la caméra a été initialisée (true = initialisée)
     */
    bool isInit();

    /**
     * @brief Permet de générer la carte représentant la zone de travail du robot
     * @param canal Correspond au canal du code BGR (exemple: canal = 0 pour B)
     * @return Retourne un objet de type cv::Mat correspondant au mapping de la zone de travail
     */
    cv::Mat generateTopographicMap(int canal);

    /**
     * @brief Permet de calculer les paramètres intrinsèques de la caméra
     */
    void calcIntrinsicParams();

    /**
     * @brief Permet de calculer les paramètres extrinsèques de la caméra
     * @return Retourne un entier pour indiquer si tout s'esst bien passé (0 : ok, -1 : en cas de problèmes)
     */
    int calcExtrinsicParams();
    //int stereoCalibration(Cameras cam);

    /**
     * @brief Permet d'obtenir des coordonnées réels d'un point dont les coordonnées sont fournis en pixels
     * @param pix Point dont les coordonnées sont données en pixel et dont-on souhaite retrouvé les coordonnées réelles
     * @param codeCam code de la camera à partir de laquelle on souhaite déterminer les coordonnées réelles du point
     * @param z Hauteur à laquelle se trouve le point en coordonnée réelle qui doit être déterminé
     * @return Retourne un objet de type cv::Point3d contenant les coordonnées réelles du point
     */
    cv::Point3d realCoordinate(cv::Point pix, int codeCam = -1, int z = 0);

    /**
     * @brief Permet de sauvegarder les paramètres de la caméra
     * @param cameraMatrix Matrice des paramètres intrinsèques de la caméra
     * @param dist Matrice des coefficients de distorsion de la caméra
     * @param error
     * @param code1 code de la caméra 1
     * @param code2 code de la caméra 2 (utile si les paramètres à sauvegarder sont ceux de la stéréocalibration)
     * @param stereo booléen indiquant si les données à sauvegarder sont celles de la calibration ou de la stéréocalibration
     * @param R Matrice de rotation
     * @param T Matrice de translation
     * @param E Matrice essentielle
     * @param F Matrice fondamentale
     */
    void saveCameraParameters(cv::Mat cameraMatrix, cv::Mat dist, double error, int code1,
                              int code2 = -1,
                              bool stereo = false,
                              cv::Mat R = cv::Mat(),
                              cv::Mat T = cv::Mat(),
                              cv::Mat E = cv::Mat(),
                              cv::Mat F = cv::Mat());

    /**
     * @brief permet de démarrer l'objet de type Camera
     */
    bool run();

    /**
     * @brief Permet de savoir s'il existe un fichier contenant les paramètres intrinsèques de la caméra
     * @return Retourne un booléen: true si le fichier existe.
     */
    inline bool hasIntrinsicP(){return v_hasIntrinsic;}

    /**
     * @brief Permet de savoir s'il existe un fichier contenant les paramètres extrinsèques de la caméra
     * @return Retourne un booléen: true si le fichier existe.
     */
    inline bool hasExtrinsicP(){return v_hasExtrinsic;}

    /**
     * @brief Permet d'obtenir la map vue de la caméra
     * @return Retourne un objet de type cv::Mat correspondant à la map
     */
    inline cv::Mat getMap(){return this->v_map;}

    /**
     * @brief Permet de savoir si la caméra est utilisée
     * @return Retourne true si la caméra est utilisée
     */
    inline bool useCam(){return this->v_useCamera;}

    /**
     * @brief Capture d'une image à partir de la caméra
     */
    void capture();

    /**
     * @brief Configure un nouvel identifiant pour la caméra
     * @param value Nouvel identifiant
     */
    void setId(int value);

    /**
     * @brief Permet de savoir si la caméra est présente
     * @return Retourne true si la caméra est présente
     */
    bool getCamIsPresent() const;
public:
    /**
     * @var acq Définie le thread d'acquisition des images venant la caméra
     */
    AcquisitionThread *acq;

    /**
     * @var wa Objet pour le calcul de la zone de travail vue de la caméra
     */
    WorkingArea *wa;

    /**
     * @var rp Définie le thread de calcul de la position du robot
     */
    RobotPositionThread *rp;

private:
    int v_id;
    int v_codeCam;
    std::string v_camPosition;
    bool v_initOk;
    bool v_hasIntrinsic;
    bool v_hasExtrinsic;
    bool v_useCamera;
    bool v_camIsPresent;
    cv::Mat v_map;
    cv::Mat v_snapshot;
    int v_height_led;
private:
    cv::Mat detectObstacle();
    bool isCameraPresent();
public:
signals:
    /**
     * @brief Le signal est émit pour l'envoi d'un message vers l'IHM
     * @param msg Message à envoyer
     */
    void sendMessage(QString msg, QColor color = WHITE_COLOR);
};

#endif // CAMERAS_H
