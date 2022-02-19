#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <Qt>
#include <QFlags>
#include <QMainWindow>
#include <QImage>
#include <QPixmap>
#include <QVector>
#include <QPalette>
#include <QColor>
#include <QApplication>
#include <QDesktopWidget>
#include <QDebug>
#include <QThread>
#include <QCloseEvent>
#include <QDialog>
#include <QActionGroup>
#include <QDoubleSpinBox>
#include <QWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QRandomGenerator64>
#include <QMessageBox>

#include <xlsxdocument.h>

#include <thread>
#include <random>
#include <WinSock2.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/fast_math.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <utility>

#pragma comment(lib, "ws2_32.lib")

#include "acquisitionthread.h"
#include "utils.h"
#include "thresholddialog.h"
#include "correctordialog.h"
#include "workingarea.h"
#include "cameras.h"
#include "robot.h"
#include "measurement.h"
#include "path.h"
#include "robotcontrollerthread.h"
#include "cameraviewbutton.h"
#include "css.h"
#include "moverobotthread.h"
#include "chronothread.h"
#include "checknodesthread.h"

//#define SHOW_IMG_MAP
#define CHECK_CAMERAS

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


/**
 * @class MainWindow
 * @brief Cette classe permet permet de gérer l'IHM dans sa globalité
 * @version 0.1
 * @date 2021
 * @author Vaneck MVONDO (vaneck.mvondo@hotmail.com)
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief Constructeur de la classe MainWindow
     * @param parent Prent auquel se refère ce widget
     */
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
private:
    /**
     * @brief Permet d'initialiser l'IHM
     */
    void init();

    /**
     * @brief Permet d'initialiser les caméras
     */
    void initCameras();

    /**
     * @brief Permet de gérer les connects
     */
    void connects();
    void createActions();
    void addActionsToMenu();
    /**
     * @brief Permet de charger, à partir d'un fichier, les paramètres de l'IHM
     */
    void loadParamIHM();

    /**
     * @brief Redéfinition de la méthode closeEvent() de la classe QWidget afin de géner le bouton de fermeture de la fenêtre (croix)
     * @param event
     */
    void closeEvent(QCloseEvent *event);

    /**
     * @brief Calcul des points de mesures atteignables par le robot
     * @param map Carte de déplacement du robot
     * @param mesPoint Liste des points de mesures
     * @param reachablePoints Liste des points de mesures atteignables
     */
    void extractMeasurePoints(cv::Mat map, std::vector<cv::Point> mesPoint, std::vector<cv::Point> &reachablePoints);

    /**
     * @brief Permet d'ordonner les points de mesures selon l'ordre de traitement que l'on souhaite faire faire au robot
     * @param reachablePoints Liste des points de mesures atteignables
     */
    void orderingMeasurePoints(std::vector<cv::Point> &reachablePoints);

    /**
     * @brief Calcule des points de positionnement du robot vis à vis des points de mesures
     * @param map4RobotPosition Carte de déplacement du robot
     * @param reachablePoints Contient les points de mesures atteignables par le robot
     * @param length_tool Distance du centre du robot à l'extrémité du capteur
     * @param areaW Largeur de la zone de travail
     * @param areaL Longeur de la zone de travail
     */
    void computeRobotPositions(cv::Mat map4RobotPosition, std::vector<cv::Point> reachablePoints,
                               int length_tool, int areaW, int areaL);

    /**
     * @brief Permet de calculer la position du robot à partir des LEDs
     * @param initialRobotPosition Donne la position du robot
     * @param robotPositionByCam Vecteur contenant la positon du robot obtenu par chaque caméra
     * @param areaW Largeur de la zone de travail
     * @param areaL Longueur de la zone de travail
     */
    void calcPosition(cv::Mat &initialRobotPosition, std::vector<cv::Mat> &robotPositionByCam, int areaW, int areaL);

    /**
     * @brief Permet de générer la trajectoire du robot
     * @param initialRobotPosition
     * @param robot
     */
    void generatePath(cv::Mat initialRobotPosition, Robot &robot);

    void computeRobotOrientation();

    /**
     * @brief Permet d'afficher la carte et la trajectoire qui a été générée
     * @param map carte
     * @param reachablePoints Points atteignables
     * @param areaW Largeur de la zone de travail
     */
    void generateFinalMap(cv::Mat &map, std::vector<cv::Point> reachablePoints, int areaW);

    /**
     * @brief Mise à jour de l'IHM
     * @param map carte + trajectoire + position du robot
     * @param nbPoint Nombre de points de mesures
     */
    void updateIHM(cv::Mat &map, int nbPoint);

    /**
     * @brief Permet d'envoyer, par socket, les paramètres du moteur au noeud ROS qui gère le moteur du mât vertical
     */
    void sendMotorParamToNode();

    /**
     * @brief Permet d'envoyer la liste des hauteurs à atteindre aux noeuds ROS
     */
    void sendHeightListToNode();

    /**
     * @brief Permet d'envoyer la temporisation aux noeuds ROS
     */
    void sendDelaysToNode();

    /**
     * @brief Envoi du type de sortie du capteur de vitesse aérodynamique
     */
    void sendVoltOutputTypeToNode();

    /**
     * @brief Permet d'arrêter l'acquisition des mesures
     */
    void sendStopToNode();

    /**
     * @brief Permet d'aciver les widgets de l'IHM
     * @param enabled True pour activer.
     */
    void enableWidgets(bool enabled);

    /**
     * @brief Sauvegarde du type de sortie du capteur de vitesse aérodynamique
     * @param type Type de sortie à sauvegarder
     */
    void saveVelVoltOutputType(int type);

    /**
     * @brief Capture et enregistrement des images d'une mire de calibration de la caméra
     */
    void calibrationImage();

    /**
     * @brief Mise en place d'info-bulles
     */
    void setToolTip();

    /**
     * @brief Positionnement des widgets sur l'IHM
     */
    void configWidgets();

    /**
     * @brief Mise en place de raccourcis clavier
     */
    void keyboardShortcuts();

    /**
     * @brief Initialisation des l'IHM
     */
    void initViews();

    /**
     * @brief Gère l'activation/désactivation des caméras
     * @param launch Booléen indiquant si la caméra doit être lancée (true) ou arrêtée (false)
     */
    void manageCameras(bool launch);

    /**
     * @brief Lance l'acquisition d'images par la caméra 1, et arrête la caméra 2 (ou 3)
     * @param acqT_1 Thread d'acquisition de la camera 1
     * @param acqT_2 Thread d'acquisition de la camera 2
     * @param acqT_3 Thread d'acquisition de la camera 3
     */
    void runCamera(AcquisitionThread *acqT_1,
                   AcquisitionThread *acqT_2,
                   AcquisitionThread *acqT_3);

    /**
     * @brief Gère l'état d'activation du joystick
     * @param enable Boolée indiquant l'activation (true) ou la désactivation (false)
     */
    void enableJoystick(bool enable);

    /**
     * @brief Vérifie si les noeuds du côté du Raspberry Pi ont été lancés
     * @param motorNode Booléen indiquant la présence du noeud motor_mast_node (true : présent)
     * @param sensorNode Booléen indiquant la présence du noeud sensors_node (true : présent)
     */
    void checkRPiNodes(bool &motorNode, bool &sensorNode);

    /**
     * @brief Vérifie si les noeuds du côté du PC ont été lancés
     * @param msg Message de sortie pour informer l'utilisateur de l'analyse qui a été faite
     * @return Retourne un booléen indiquant si tous les noeuds ont été lancés (true).
     */
    bool checkPCNodes(QString &msg);

    /**
     * @brief Vérifie qu'il n'y a pas d'erreurs dans la liste des hauteurs saisies par l'utilisateur
     * @return Retourne un booléen: true d'il n'y a pas d'erreurs
     */
    bool checkHeights();

    /**
     * @brief Vérifies qu'il y a bien des caméras aux IDs renseignées par l'utilisateur, et que des
     * fichiers de calibration sont disponibles
     * @param msg Message de sortie pour informer l'utilisateur de l'analyse qui a été faite
     * @return
     */
    bool checkCameras(QString &msg);

    /**
     * @brief Permet de gérer le joystick
     * @param robotDirection Direction désirée pour le robot
     * @param pressedBtnState Etat avant appui du bouton sélectionné
     * @param unPressedBtnState Etat avant appui du bouton opposé au bouton sélectionné
     * @param pressedBtn Bouton sélectionné
     * @param unpressedBtn Bouton opposé au bouton sélectionné
     */
    void manageJoystick(int robotDirection, bool &pressedBtnState, bool &unPressedBtnState,
                        QPushButton *pressedBtn = nullptr, QPushButton *unpressedBtn = nullptr);

    /**
     * @brief Permet de démarrer un noeud ROS côté PC
     * @param nodeExe Exécutable du noeud à lancer
     * @param msg Message de sortie
     * @return Retourne true si le noeud a été lancé
     */
    bool launchNode(QString nodeExe, QString &msg, void (*function)());

    /**
     * @brief Permet de démarrer un noeud ROS côté PC
     * @param nodeExe Exécutable du noeud à lancer
     * @param msg Message de sortie
     * @param nodeStatus Permet de savoir si le noeud a déjà été lancé pour éviter un lancement multiple
     * @return Retourne true si le noeud a été lancé
     */
    bool launchNode(QString nodeExe, QString *msg, bool nodeStatus, void (*function)());

    /**
     * @brief Permet d'arrêter un noeud ROS côté PC
     * @param nodeExe Exécutable du noeud à arrêter
     * @param nodeStatus Permet de savoir si le noeud a été lancé
     */
    void stopNode(std::string nodeExe, bool nodeStatus);
public:
    ThresholdDialog *v_threshDial;
    CorrectorDialog *v_corrector;
private:
    Ui::MainWindow *ui;
    QDialog *v_projectName;
    QDialog *v_viewCameras;
    QPixmap *v_qPixWA;
    QAction *v_correctorP_action;
    QAction *v_correctorPI_action;
    QAction *v_correctorPID_action;
    QAction *v_0_5_V_action;
    QAction *v_1_5_V_action;
    QAction *v_0_10_V_action;
    QAction *v_2_10_V_action;
    QAction *v_0_20_mA_action;
    QAction *v_4_20_mA_action;
    QActionGroup *v_correctorGroup;
    QActionGroup *v_voltageGroup;
    QString projectFileName;
    QString v_joystickEnabledStyle;
    QString v_joystickDesabledStyle;
    QString v_joystickDesabledStyle_2;
    QString v_joystickRotEnabledStyle;
    QString v_joystickRotDesabledStyle;
    QString v_joystickRotDesabledStyle_2;
    cv::Mat v_map;
    std::vector<cv::Mat> v_generatedPath;
    cv::Mat v_initialRobotPosition;
    std::vector<cv::Point> v_reachablePoints;
    std::vector<cv::Point> v_unReachablePoints;
    std::vector<cv::Point> v_robotPositionPoints;
    int v_nb_of_measurement_points;
    int nb_of_process_points;
    int v_nb_of_height;
    int v_windowWidth;
    int v_windowHeight;
    int v_maxWinH;
    int v_maxWinW;
    int v_countImage;
    int v_cursorPos;
    const int v_nbCamera = 10;
    double v_resizeWidthImage;
    double v_resizeHeightImage;
    double v_fx;
    double v_fy;
    bool v_IHMStatus;
    bool v_hasProjectnamePointer;
    bool v_camerasDialog;
    bool v_left_btn_joy_status;
    bool v_right_btn_joy_status;
    bool v_fw_btn_joy_status;
    bool v_bw_btn_joy_status;
    bool v_cw_btn_joy_status;
    bool v_ccw_btn_joy_status;
    bool v_rosMasterNodeStatus;
    bool v_pcControllerNodeStatus;
    bool v_intermediaryNodeStatus;
    bool v_velNodeStatus;
    std::string v_pushButtonStyle_2;
    std::string v_pushButtonStyle_3;
    std::string v_pushButtonStyle_4;
    RobotControllerThread *v_rc;
    std::vector<Cameras*> v_cam;
    CameraViewButton *v_cameraBtn;
    AcquisitionThread *v_cam_G_acq;
    AcquisitionThread *v_cam_M_acq;
    AcquisitionThread *v_cam_D_acq;
    MoveRobotThread *v_moveRobotThread;
    ChronoThread *v_chronoImg;
    CheckNodesThread *v_checkNodes_T;
private slots:
    void dispFrame(const cv::Mat &frame);

    //void displayCameraView

    /**
     * @brief Permet de lancer le noeud maître ROS
     */
    void rosNodeMaster();

    /**
     * @brief Permet de quitter l'IHM
     */
    void quitIHM();

    /**
     * @brief Permet de lancer le calcul de la carte de déplacement du robot et détection d'obstacles
     */
    void generateMap();
    //void showImg();

    /**
     * @brief Permet de lister les noeuds ROS connectés
     */
    void nodeList();

    /**
     * @brief Permet de lancer la fenêtre de seuillage
     */
    void setTresh();

    /**
     * @brief Permet de lancer les mesures
     */
    void measure();

    /**
     * @brief Gestion des adresses IP du robot et du PC
     */
    void changeIP();

    /**
     * @brief Permet de gérer les hauteurs à atteindre
     */
    void heightList();

    /**
     * @brief Permet de d'arrêter le robot
     */
    void stop();

    /**
     * @brief Permet de sauvegarder les paramètres de l'IHM
     */
    void saveParamIHM(bool force = false);

    /**
     * @brief Permet de gérer le nombre de point de mesure traiter
     * @param val nombre de point traiter
     */
    void processedPoint(int val);

    /**
     * @brief Permet de gérer le nombre de mesures effectuées
     * @param val nombre de mesures effectuées
     */
    void processedMeasure(int val);

    /**
     * @brief Permet de lancer un nouveau projet
     */
    void newProject();

    /**
     * @brief Permet de sauvegarder le nom du projet en cours
     */
    void saveNewProjectName();

    /**
     * @brief Permet de gérer le nom du projet en création
     * @param text Nom du projet
     */
    void newProjectName(const QString &text);

    /**
     * @brief Permet d'afficher la fenêtre de visualisation des caméras connectées au PC
     */
    void camerasView();

    /**
     * @brief Sauvegarde les identifiants des caméras
     */
    void saveIDsCam();

    /**
     * @brief Gère l'affichage du panneau de contrôle manuel
     */
    void manualControlView();

    /**
     * @brief gère l'affichage du panneau de calibration
     */
    void calibrationView();

    void targetHeight();

    /**
     * @brief Ajoute du texte dans la zone de message de l'IHM
     * @param msg Texte à ajouter
     * @param color Couleur du texte
     */
    void myAppend(QString msg, QColor color);

    /**
     * @brief Gère la disponibilité de la zone de message de l'IHM
     * @param checked zone de message activée si true
     */
    void messageView(bool checked);

    /**
     * @brief Gère la direction du robot vers la droite en utilisant le joystick
     */
    void yVelPlusManualControl();

    /**
     * @brief Gère la direction du robot vers la gauche en utilisant le joystick
     */
    void yVelMinusManualControl();

    /**
     * @brief Gère la direction du robot vers l'avant en utilisant le joystick
     */
    void xVelPlusManualControl();

    /**
     * @brief Gère la direction du robot vers l'arrière en utilisant le joystick
     */
    void xVelMinusManualControl();

    /**
     * @brief Gère la rotation du robot dans le sens inverse des aguilles d'une montre en utilisant le joystick
     */
    void zAngVelPlusManualControl();

    /**
     * @brief Gère la rotation du robot dans le sens des aguilles d'une montre en utilisant le joystick
     */
    void zAngVelMinusManualControl();

    /**
     * @brief Arrête le contrôle manuel du robot
     */
    void stopManualControl();

    /**
     * @brief Vérifie la présence d'une caméra
     * @param camIndex Index de la caméra
     */
    void checkCameras(int camIndex);

    /**
     * @brief Sauvegarde du type de sortie 0-5V du capteur de vitesse aérodynamique
     */
    void _0_5_V();

    /**
     * @brief Sauvegarde du type de sortie 1-5V du capteur de vitesse aérodynamique
     */
    void _1_5_V();

    /**
     * @todo
     */
    void camLeftView();

    /**
     * @todo
     */
    void camMiddleView();

    /**
     * @todo
     */
    void camRightView();

    /**
     * @brief Lance la calibration des caméras sélectionnées au sein de l'IHM
     */
    void calibrateCameras();

    /**
     * @brief Lance le chrono pour la capture des images pour la calibration des caméras
     */
    void runChrono();

    /**
     * @brief Mise à jour du chrono de capture des images pour la calibration des caméras
     * @param chrono Nouvelle valeur du chrono
     */
    void updateChrono(int chrono);

    /**
     * @todo
     */
    void configView(bool status = false);

    /**
     * @brief Contrôle l'édition de la liste des hauteurs à atteindre
     * @param text Liste des hauteurs à atteindre
     */
    void changeHeightList(QString text);

    /**
     * @brief Affichage d'un message dans la zone des messages
     * @param msg Message à afficher
     * @param textColor Couleur du texte
     * @param textBackgroundColor Couleur d'arrière plan
     */
    void displayMsgSlot(const QString &msg, const QColor &textColor = WHITE_COLOR, const QColor &textBackgroundColor = BLACK_COLOR);

    /**
     * @todo
     */
    void robotFreePos(bool checked);

    /**
     * @todo
     */
    void robotFPXMax(int val);

    /**
     * @todo
     */
    void robotFPYMax(int val);

    /**
     * @todo
     */
    void robotFPLaunchBtn();

    /**
     * @brief Lance la prise des mesures en contrôle manuel
     */
    void takeMeasureManualControl();

    /**
     * @brief Affiche la fenêtre de paramétrage des coefficients du correcteur
     */
    void showCorrectorDial();

    /**
     * @brief Lance l'acquisition des mesures
     */
    void startMeasure();

    /**
     * @brief Lance le noeud pc_controller_node
     * @param msg Message de sortie
     * @return Retourne true si le noeud a été lancé
     */
    bool runControllerNode(QString *msg = nullptr);

    /**
     * @brief Lance le noeud intermediary_node
     * @param msg Message de sortie
     * @return Retourne true si le noeud a été lancé
     */
    bool runIntermediaryNode(QString *msg = nullptr);

    /**
     * @brief Lance le noeud vel_node
     * @param msg Message de sortie
     * @return Retourne true si le noeud a été lancé
     */
    bool runVelocityNode(QString *msg = nullptr);

    /**
     * @brief Lance tous les noeuds côté PC
     * @warning Le noeud maître doit être lancé avant
     */
    void runAllNodes();

    /**
     * @brief Arrête le noeud pc_controller_node
     */
    void stopControllerNode();

    /**
     * @brief Arrête le noeud intermediary_node
     */
    void stopIntermediaryNode();

    /**
     * @brief Arrête le noeud vel_node
     */
    void stopVelocityNode();

    /**
     * @brief Arrête le noeud maître ROS
     */
    void stopRosMasterNode();

    /**
     * @brief Arrête tous les noeuds, sauf le noeud maître
     */
    void stopAllNodes();

    /**
     * @brief Mise à jour de l'état des noeuds
     * @param node Noeud à mettre à jour
     * @param status Etat du noeud : en cours de fonctionnement (true) ou arrêté (false)
     */
    void updateNodesStatus(int node, bool status);
private: signals:
    /**
     * @brief Le signal est envoyé lorsqu'un message doit être affiché dans l'IHM
     * @param msg Message à afficher
     */
    void sendMessage(QString msg);
    void displayMsgSignal(const QString &msg, const QColor &textColor = WHITE_COLOR, const QColor &textBackgroundColor = BLACK_COLOR);
};
#endif // MAINWINDOW_H
