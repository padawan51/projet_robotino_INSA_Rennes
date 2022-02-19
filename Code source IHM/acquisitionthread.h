#ifndef ACQUISITION_H
#define ACQUISITION_H

#include <QThread>
#include <QImage>
#include <QPixmap>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utils.h"

/**
 * @class AcquisitionThread
 * @brief Cette classe permet permet d'acquérir, dans un thread séparé, le flux de la caméra.
 * @version 0.1
 * @date 2021
 * @author Vaneck MVONDO (vaneck.mvondo@hotmail.com)
 */

Q_DECLARE_METATYPE(cv::Mat);

class AcquisitionThread : public QThread
{
    Q_OBJECT
public:
    /**
     * @brief Permet la construction d'un nouvel objet de type Acquisition avec un parent donné
     * @param parent
     */
    explicit AcquisitionThread(QObject *parent = nullptr);

    /**
     * @brief Permet la construction d'un nouvel objet de type Acquisition
     * @param num_cam indique le numéro de la caméra dont il y a besoin d'acquérir le flux vidéo
     */
    AcquisitionThread(int *numCam);
    ~AcquisitionThread();

    /**
     * @brief redéfinition de la méthode run() de QThread pour l'acquisition en temps réel
     */
    void run();

    /**
     * @brief accesseur qui permet de récupérer, sous forme d'image, ce que voit la caméra
     * @return Retourne l'image sous forme d'un objet de type cv::Mat
     */
    cv::Mat getFrame();

    /**
     * @brief Configure l'identifiant de la caméra
     * @param numCam Nouvel identifiant de la caméra
     */
    void setIDCam(int numCam);

    /**
     * @brief Permet à la caméra d'acquérir soit une unique image, soit un flux d'image en continu
     * @param value Si true, alors la caméra capture une unique image
     */
    void setIsSnapshot(bool value);

    /**
     * @brief Permet d'obtenir l'identifiant de la caméra
     * @return Retourne l'identiant de la caméra
     */
    int getIdCam();

    /**
     * @brief Permet de savoir si la caméra est ouverte
     * @return Retourne true si la caméra est ouverte
     */
    bool getCamIsOpen() const;

    /**
     * @brief Permet de démarrer la caméra
     * @return Retourne true si la caméra a été démarrée
     */
    bool launchCamera();

    /**
     * @brief Permet d'arrêter la caméra
     * @return Retourne true si la caméra s'est arrêtée
     */
    bool stopCamera();

private:
    /**
     * @brief Initialise la caméra
     */
    void init();

signals:
        /**
         * @brief Ce signal est émit lorsqu'une nouvelle acquisition a été faite
         * @param frame objet de type QPixmap qui représente l'image qui a été capturée
         */
        void newFrame(cv::Mat frame);

private:
    cv::Mat v_frame;
    cv::VideoCapture v_cap;
    int *v_IdCam;
    bool v_camIsOpen;
    bool v_isSnapshot;
    bool v_isInit;
};

#endif // ACQUISITION_H
