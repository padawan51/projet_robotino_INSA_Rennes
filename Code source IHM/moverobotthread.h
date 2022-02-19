#ifndef MOVEROBOTTHREAD_H
#define MOVEROBOTTHREAD_H

#include <QObject>
#include <QThread>
#include <QPushButton>

#include "css.h"
#include "utils.h"

class MoveRobotThread : public QThread
{
    Q_OBJECT

public:
    MoveRobotThread(bool *forward, bool *backward, bool *left,
                    bool *right, bool *ccw, bool *cw//, QPushButton *stop
                    );
    ~MoveRobotThread();
    void joystick(int direction, int lx = 0, int ly = 0, double az = 0.0);
    void run();
private:
    bool *v_forward;
    bool *v_backward;
    bool *v_left;
    bool *v_right;
    bool *v_ccw;
    bool *v_cw;

    int v_lx;
    int v_ly;
    int v_direction;
    double v_az;
};

#endif // MOVEROBOTTHREAD_H
