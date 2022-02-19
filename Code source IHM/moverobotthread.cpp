#include "moverobotthread.h"

MoveRobotThread::MoveRobotThread(bool *forward, bool *backward, bool *left,
                                 bool *right, bool *ccw, bool *cw)
{
    v_forward = forward;
    v_backward = backward;
    v_left = left;
    v_right = right;
    v_ccw = ccw;
    v_cw = cw;

    v_lx = 0;
    v_ly = 0;
    v_direction = 0;
    v_az = 0.0;
}

MoveRobotThread::~MoveRobotThread()
{
    v_forward = nullptr;
    v_backward = nullptr;
    v_left = nullptr;
    v_right = nullptr;
    v_ccw = nullptr;
    v_cw = nullptr;
}

void MoveRobotThread::joystick(int direction, int lx, int ly, double az)
{
    v_direction = direction;
    v_lx = lx;
    v_ly = ly;
    v_az = az;
}

void MoveRobotThread::run()
{
    int lx = 0;
    int ly = 0;
    double az = 0.0;

    if(v_direction != RobotMove::STOP){
        if(*v_forward && !*v_backward) lx = (v_lx);
        else if(!*v_forward && *v_backward) lx = -(v_lx);

        if(*v_right && !*v_left) ly = (v_ly);
        else if(!*v_right && *v_left) ly = -(v_ly);

        if(*v_ccw && !*v_cw) az = (v_az);
        else if(!*v_ccw && *v_cw) az = -(v_az);
    }
    else{
        lx = 0;
        ly = 0;
        az = 0.0;
    }

    moveRobot(lx, ly, az);
}
