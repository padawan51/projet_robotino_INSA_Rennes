#ifndef CAMERAVIEWBUTTON_H
#define CAMERAVIEWBUTTON_H

#include <QObject>
#include <QWidget>
#include <QPushButton>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>

#include "utils.h"
#include "css.h"

/**
 * @class CameraViewButton
 * @brief Cette classe permet permet de créer un bouton personnalisé lié à une caméra.
 * @version 0.1
 * @date 2021
 * @author Vaneck MVONDO (vaneck.mvondo@hotmail.com)
 */
class CameraViewButton : public QPushButton
{
    Q_OBJECT
public:
    /**
     * @brief Constructeur par défaut de la classe
     */
    CameraViewButton();

    /**
     * @brief Surcharge du constructeur de la classe
     * @param index Identifiant de la caméra
     * @param winW Largeur (en pixel) de l'écran sur lequel est exécuté l'IHM
     * @param winH Hauteur (en pixel) de l'écran sur lequel est exécuté l'IHM
     */
    CameraViewButton(int index, int winW, int winH);
    ~CameraViewButton() = default;

    /**
     * @brief Permet d'envoyer l'image capturée par la caméra vers l'IHM
     */
    void sendPixmap();

    /**
     * @brief Permet de configurer l'identifiant du bouton
     * @param index
     */
    void setIndex(int index);

    /**
     * @brief Permet de savoir si la caméra qui est lié au bouton est disponible
     * @return Retourne true si la caméra est présente
     */
    bool getIsPresent() const;

    /**
     * @brief Permet de savoir si le bouton a été sélectionné par l'utilisateur
     * @return Retourne true si le bouton a été sélectionné
     */
    bool getIsSelected() const;

    /**
     * @brief Configure l'aspect visuel du bouton selon qu'il a été sélectionné ou pas
     * @param value Est à true si le bouton a étté sélectionné
     */
    void setIsSelected(bool value);

    /**
     * @brief Enregistre les nouvelles dimensions de l'écran sur lequel est exécuté l'IHM
     * @param w Nouvelle largeur
     * @param h Nouvelle hauteur
     */
    void setWinParam(const int w, const int h);

private:
    /**
     * @brief Permet d'ouvrir la caméra
     */
    void openCamera();

public slots:
    /**
     * @brief Permet la capture d'une image
     */
    void captureImage();

signals:
    void image(QPixmap frame);
    void checkRunningCameras(int camIndex);
private:
    int v_index;
    int v_winW;
    int v_winH;
    bool v_isPresent;
    bool v_isSelected;
    QPixmap v_pixmap;
};

#endif // CAMERAVIEWBUTTON_H
