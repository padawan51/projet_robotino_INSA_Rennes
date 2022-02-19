#ifndef CSS_H
#define CSS_H

#include <QString>

const QString checkBoxStyle_1 = "color: rgb(255, 255, 255);";

const QString checkBoxStyle_2 = "color: rgb(0, 100, 0);";

const QString checkBoxStyle_3 = "QCheckBox "
                                "{ color: rgb(0, 100, 0); }";

const QString checkBoxStyle_4 = "QCheckBox "
                                "{ color: rgb(255, 255, 255); }"
                                "QCheckBox:checked "
                                "{ color: rgb(0, 200, 0); }";

const QString dialogStyle_1 = "background-color: rgb(15, 15, 15);";

const QString doubleSpinBoxStyle_1 = "QDoubleSpinBox "
                                     "{ border-style: outset;"
                                     "border-width: 2px;"
                                     "border-radius: 6px;"
                                     "border-color: rgb(255, 0, 0);"
                                     "background-color: rgb(240, 240, 240);"
                                     "color: rgb(0, 0, 0); }"
                                     "QDoubleSpinBox:hover"
                                     "{ background-color: rgb(255, 100, 100); }";

const QString doubleSpinBoxStyle_2 = "QDoubleSpinBox "
                                     "{ border-style: outset;"
                                     "border-width: 2px;"
                                     "border-radius: 6px;"
                                     "border-color: rgb(100, 100, 100);"
                                     "background-color: rgb(240, 240, 240); "
                                     "color: rgb(100, 100, 100); }";

const QString frameStyle_1 = "QFrame "
                             "{ border-style: outset;"
                             "border-radius: 8px;"
                             "border-width: 2px;"
                             "border-color: rgb(100, 100, 100); }";

const QString groupBoxStyle_1 = "QGroupBox "
                                "{ border-style: outset;"
                                "border-radius: 8px;"
                                "border-width: 1px;"
                                "border-color: rgb(100, 100, 100); }";

const QString groupBoxStyle_2 = "QGroupBox "
                                "{ color: rgb(0, 255, 0);"
                                "border-style: outset;"
                                "border-radius: 8px;"
                                "border-width: 2px;"
                                "border-color: rgb(100, 100, 100); }"
                                "QRadioButton "
                                "{ color: rgb(255, 255, 255); }"
                                "QGroupBox#highToAchieve QRadioButton "
                                "{ color: rgb(100, 100, 100); }";

const QString groupBoxStyle_3 = "QGroupBox "
                                "{ color: rgb(100, 100, 100);"
                                "border-style: outset;"
                                "border-radius: 8px;"
                                "border-width: 2px;"
                                "border-color: rgb(100, 100, 100); }"
                                "QRadioButton "
                                "{ color: rgb(100, 100, 100); }"
                                "QCheckBox "
                                "{ color: rgb(100, 100, 100); }";

const QString groupBoxStyle_4 = "QGroupBox "
                                "{ border-style: outset;"
                                "border-radius: 8px;"
                                "border-width: 2px;"
                                "border-color: rgb(100, 100, 100); }";

const QString groupBoxStyle_5 = "QGroupBox "
                                "{ color: rgb(0, 255, 0);"
                                "border-style: outset;"
                                "border-radius: 8px;"
                                "border-width: 2px;"
                                "border-color: rgb(100, 100, 100); }"
                                "QRadioButton "
                                "{ color: rgb(255, 255, 255); "
                                "background-color: rgb(15, 15, 15); }"
                                "QRadioButton:checked "
                                "{ color: rgb(0, 100, 0); "
                                "background-color: rgb(0, 0, 0); }";

const QString groupBoxStyle_6 = "QGroupBox "
                                "{ color: rgb(100, 100, 100);"
                                "border-style: outset;"
                                "border-radius: 8px;"
                                "border-width: 2px;"
                                "border-color: rgb(100, 100, 100); }"
                                "QLabel "
                                "{ color: rgb(150, 150, 150); }";

const QString groupBoxStyle_7 = "QGroupBox "
                                "{ color: rgb(0, 255, 0);"
                                "border-style: outset;"
                                "border-radius: 8px;"
                                "border-width: 2px;"
                                "border-color: rgb(100, 100, 100); }"
                                "QLabel "
                                "{ color: rgb(255, 255, 255); }"
                                "QCheckBox "
                                "{ color: rgb(255, 255, 255); }";

const QString groupBoxStyle_8 = "QGroupBox "
                                "{ border-style: outset;"
                                "border-radius: 8px;"
                                "border-width: 2px;"
                                "border-color: rgb(100, 100, 100); }"
                                "QGroupBox:enabled "
                                "{ color: rgb(0, 255, 0); }"
                                /*"QGroupBox:enabled*/ "QLabel "
                                "{ color: rgb(255, 255, 255); }"
                                "QGroupBox:!enabled "
                                "{ color: rgb(100, 100, 100); }"
                                "QGroupBox:!enabled QLabel"
                                "{ color: rgb(150, 150, 150); }";

const QString groupBoxStyle_9 = "QGroupBox "
                                "{ color: rgb(0, 255, 0);"
                                "border-style: outset;"
                                "border-radius: 8px;"
                                "border-width: 2px;"
                                "border-color: rgb(100, 100, 100); }"
                                "QLabel "
                                "{ color: rgb(255, 255, 255); }"
                                "QSpinBox "
                                "{ border-style: outset;"
                                "border-width: 2px;"
                                "border-radius: 6px;"
                                "border-color: rgb(100, 100, 100);"
                                "background-color: rgb(240, 240, 240); }"
                                "QSpinBox:hover"
                                "{ border-color: rgb(255, 100, 100); }"
                                "QLineEdit, QProgressBar "
                                "{ background-color: rgb(240, 240, 240);"
                                "border-style: outset;"
                                "border-color: rgb(100, 100, 100);"
                                "border-radius: 6px;"
                                "border-width: 2px; }"
                                "QLineEdit:hover "
                                "{ border-color: rgb(255, 100, 100); }"
                                "QRadioButton "
                                "{ color: rgb(255, 255, 255); }";

const QString groupBoxStyle_10 = "QGroupBox "
                                 "{ color: rgb(100, 100, 100);"
                                 "border-style: outset;"
                                 "border-radius: 8px;"
                                 "border-width: 2px;"
                                 "border-color: rgb(100, 100, 100); }"
                                 "QLabel "
                                 "{ color: rgb(100, 100, 100); }"
                                 "QSpinBox "
                                 "{ border-style: outset;"
                                 "border-width: 2px;"
                                 "border-radius: 6px;"
                                 "border-color: rgb(100, 100, 100);"
                                 "background-color: rgb(240, 240, 240); }"
                                 "QLineEdit, QProgressBar "
                                 "{ background-color: rgb(240, 240, 240);"
                                 "border-style: outset;"
                                 "border-color: rgb(100, 100, 100);"
                                 "border-radius: 6px;"
                                 "border-width: 2px; }";

const QString groupBoxStyle_12 = "QGroupBox "
                                "{ color: rgb(0, 255, 0);"
                                "border-style: outset;"
                                "border-radius: 8px;"
                                "border-width: 2px;"
                                "border-color: rgb(100, 100, 100); }"
                                "QLabel "
                                "{ color: rgb(255, 255, 255); }"
                                "QSpinBox "
                                "{ border-style: outset;"
                                "border-width: 2px;"
                                "border-radius: 6px;"
                                "border-color: rgb(100, 100, 100);"
                                "background-color: rgb(240, 240, 240); }"
                                "QSpinBox:hover"
                                "{ border-color: rgb(255, 100, 100); }"
                                "QRadioButton "
                                "{ color: rgb(255, 255, 255); }";

const QString groupBoxStyle_11 = "QGroupBox "
                                "{ color: rgb(100, 100, 100);"
                                "border-style: outset;"
                                "border-radius: 8px;"
                                "border-width: 2px;"
                                "border-color: rgb(100, 100, 100); }"
                                "QLabel "
                                "{ color: rgb(100, 100, 100); }"
                                "QSpinBox "
                                "{ border-style: outset;"
                                "border-width: 2px;"
                                "border-radius: 6px;"
                                "border-color: rgb(100, 100, 100);"
                                "background-color: rgb(240, 240, 240); }"
                                "QRadioButton "
                                "{ color: rgb(100, 100, 100); }";

const QString labelStyle_1 = "QLabel "
                             "{ color: rgb(0, 0, 0); }";

const QString labelStyle_2 = "QLabel "
                             "{ color: rgb(255, 255, 255); }";

const QString labelStyle_3 = "QLabel "
                             "{ border-style: outset;"
                             "border-width: 3px;"
                             "border-color: rgb(100, 100, 100); }";
const QString labelStyle_4 = "QLabel "
                             "{ color: rgb(150, 150, 150); }";

const QString lcdStyle_1 = "background-color: rgb(0, 0, 0);"
                           "border-radius: 8px;"
                           "border-width: 2px;"
                           "border-color: rgb(0, 255, 0);"
                           "border-style: outset;"
                           "color: rgb(0, 255, 0);";

const QString lcdStyle_2 = "background-color: rgb(0, 255, 0);"
                           "border-radius: 8px;"
                           "border-width: 2px;"
                           "border-color: rgb(0, 0, 0);"
                           "border-style: outset;";

const QString lcdStyle_3 = "background-color: rgb(0, 0, 0);"
                           "border-radius: 8px;"
                           "border-width: 2px;"
                           "border-color: rgb(255, 255, 0);"
                           "border-style: outset;"
                           "color: rgb(255, 255, 0);";

const QString lcdStyle_4 = "background-color: rgb(255, 255, 0);"
                           "border-radius: 8px;"
                           "border-width: 2px;"
                           "border-color: rgb(0, 0, 0);"
                           "border-style: outset;";

const QString lcdStyle_5 = "background-color: rgb(230, 230, 230);"
                           "border-radius: 8px;"
                           "border-width: 2px;"
                           "border-color: rgb(200, 200, 200);"
                           "border-style: outset;";

const QString lcdStyle_6 = "background-color: rgb(0, 0, 0);"
                           "border-radius: 8px;"
                           "border-width: 2px;"
                           "border-color: rgb(255, 0, 0);"
                           "border-style: outset;"
                           "color: rgb(255, 255, 255);";

const QString lineEditStyle_1 = "background-color: rgb(230, 230, 230);"
                                "border-style: outset;"
                                "border-width: 2px;"
                                "border-color: rgb(200, 200, 200);"
                                "border-radius: 6px;";

const QString lineEditStyle_2 = "border-style: outset;"
                                "border-width: 2px;"
                                "border-radius: 10px;"
                                "border-color: rgb(100, 100, 100);"
                                "background-color: rgb(240, 240, 240);";

const QString lineEditStyle_3 = "border-style: outset;"
                                "border-width: 2px;"
                                "border-radius: 10px;"
                                "border-color: rgb(0, 150, 0);"
                                "background-color: rgb(255, 255, 255);";

const QString lineEditStyle_4 = "background-color: rgb(255, 255, 255);"
                                "border-style: outset;"
                                "border-width: 2px;"
                                "border-color: rgb(255, 100, 100);"
                                "border-radius: 6px;";

const QString lineEditStyle_5 = "background-color: rgb(230, 230, 230);"
                                "border-style: outset;"
                                "border-width: 2px;"
                                "border-color: rgb(200, 200, 200);"
                                "border-radius: 6px;";

const QString pushButtonStyle_1 = "QPushButton "
                                  "{ background-color: rgb(0, 255, 0);"
                                  "border-style: outset;"
                                  "border-radius: 8px;"
                                  "border-width: 2px;"
                                  "border-color: rgb(0, 255, 0); }"
                                  "QPushButton:pressed "
                                  "{ background-color: rgb(120, 255, 120);"
                                  "border-style: inset;"
                                  "border-color: rgb(0, 255, 0); }"
                                  "QPushButton:hover:!pressed "
                                  "{ background-color: rgb(120, 255, 120); }";

const QString pushButtonDesabledStyle = "QPushButton "
                                        "{ background-color: rgb(230, 230, 230);"
                                        "border-style: outset;"
                                        "border-radius: 8px;"
                                        "border-width: 2px;"
                                        "border-color: rgb(200, 200, 200); }";

const QString pushButtonEnabledStyle = "QPushButton "
                                       "{ background-color: rgb(255, 255, 255);"
                                       "border-style: outset;"
                                       "border-radius: 8px;"
                                       "border-width: 2px;"
                                       "border-color: rgb(255, 0, 0); }"
                                       "QPushButton:pressed "
                                       "{ background-color: rgb(255, 0, 0);"
                                       "border-style: inset;"
                                       "border-color: rgb(255, 0, 0); }"
                                       "QPushButton:hover:!pressed "
                                       "{ background-color: rgb(255, 100, 100); }";

const QString pushButtonEnabledStyle_2 = "QPushButton "
                                       "{ background-color: rgb(100, 255, 100);"
                                       "border-style: outset;"
                                       "border-radius: 8px;"
                                       "border-width: 2px;"
                                       "border-color: rgb(0, 255, 0); }"
                                       "QPushButton:pressed "
                                       "{ background-color: rgb(0, 255, 0);"
                                       "border-style: inset;"
                                       "border-color: rgb(0, 255, 0); }"
                                       "QPushButton:hover:!pressed "
                                       "{ background-color: rgb(0, 200, 0); }";

const QString qAppStyle_1 = "QLabel, QCheckBox, QRadioButton "
                            "{ color: rgb(255, 255, 255); }"
                            "QFrame#frame_2 QLabel "
                            "{ color: rgb(100, 100, 100); }"
                            "QGroupBox "
                            "{ color: rgb(0, 255, 0); }"
                            "QLabel#camView "
                            "{ border-style: outset;"
                            "border-width: 2px;"
                            "border-color: rgb(100, 100, 100); }"
                            "QGroupBox#IDs_cams "
                            "{ color: rgb(0, 255, 0); }"
                            "QGroupBox#log "
                            "{ background-color: rgb(100, 100, 100); "
                            "color: rgb(0, 255, 0); }"
                            "QGroupBox#highToAchieve QRadioButton "
                            "{ color: rgb(100, 100, 100); }"
                            "QMessageBox "
                            "{background-color: rgb(15, 15, 15); }";

const QString qAppStyle_2 = "QLabel, QCheckBox, QRadioButton "
                            "{ color: rgb(100, 100, 100); }"
                            "QCheckBox#calib_left_checkbox, QCheckBox#calib_right_checkbox, QCheckBox#calib_middle_checkbox"
                            "{ color: rgb(255, 255, 255); }"
                            "QGroupBox "
                            "{ color: rgb(100, 100, 100); }"
                            "QLabel#camView "
                            "{ border-style: outset;"
                            "border-width: 2px;"
                            "border-color: rgb(100, 100, 100); }"
                            "QGroupBox#manualControl "
                            "{ color: rgb(0, 255, 0); }"
                            "QGroupBox#IDs_cams "
                            "{ color: rgb(0, 255, 0); }"
                            "QGroupBox#IDs_cams QLabel"
                            "{ color: rgb(255, 255, 255); }"
                            "QGroupBox#IPAdress "
                            "{ color: rgb(0, 255, 0); }"
                            "QGroupBox#calibration "
                            "{ color: rgb(0, 255, 0); }"
                            "QGroupBox#IPAdress QLabel "
                            "{ color: rgb(255, 255, 255); }"
                            "QCheckBox#manualControl_checkbox "
                            "{ color: rgb(255, 255, 255); }"
                            "QCheckBox#calibration_checkbox "
                            "{ color: rgb(255, 255, 255); }"
                            "QGroupBox#log "
                            "{ background-color: rgb(100, 100, 100);"
                            "color: rgb(0, 255, 0); }"
                            "QFrame#frame QLabel "
                            "{ color: rgb(255, 255, 255); }"
                            "QMessageBox "
                            "{background-color: rgb(15, 15, 15); }"
                            "QMessageBox QLabel "
                            "{ color: rgb(255, 255, 255); }";

const QString radioStyle_1 = "color: rgb(0, 100, 0);"
                             "background-color: rgb(0, 0, 0);";

const QString radioStyle_2 = "color: rgb(100, 100, 100);";

const QString radioStyle_3 = "color: rgb(100, 100, 100);";

const QString validatePushButtonStyle = "QPushButton "
                                        "{ background-color: rgb(255, 100, 100);"
                                        "border-style: outset;"
                                        "border-radius: 8px;"
                                        "border-width: 2px;"
                                        "border-color: rgb(255, 0, 0); }"
                                        "QPushButton:pressed "
                                        "{ background-color: rgb(255, 0, 0);"
                                        "border-style: inset;"
                                        "border-color: rgb(255, 0, 0); }"
                                        "QPushButton:hover "
                                        "{ background-color: rgb(255, 200, 200); }";
#endif // CSS_H
