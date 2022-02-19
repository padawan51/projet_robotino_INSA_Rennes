#ifndef CORRECTORDIALOG_H
#define CORRECTORDIALOG_H

#include <QDialog>

#include "utils.h"
#include "css.h"

namespace Ui {
class CorrectorDialog;
}

class CorrectorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CorrectorDialog(int winW, int winH, int type = -1, QWidget *parent = nullptr);
    ~CorrectorDialog();

private:
    Ui::CorrectorDialog *ui;

    int v_winW;
    int v_winH;
    int v_type;
private:
    void configWidgets();
    void connects();
    void loadValues();
private slots:
    void save();
};

#endif // CORRECTORDIALOG_H
