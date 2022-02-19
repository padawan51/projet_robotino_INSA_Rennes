#include "checknodesthread.h"

CheckNodesThread::CheckNodesThread(QObject *parent)
    : QThread{parent}
{

}

void CheckNodesThread::run()
{
    std::string processName;
    DWORD processID = 0;

    while(!isInterruptionRequested()){
        processName = "rosmaster.exe";
        emit nodeStatus(Node::ROS_MASTER, findProcess(processName, processID));

        processName = "pc_controller_node.exe";
        emit nodeStatus(Node::PC_CONTROLLER, findProcess(processName, processID));

        processName = "intermediary_node.exe";
        emit nodeStatus(Node::INTERMEDIARY, findProcess(processName, processID));

        processName = "vel_node.exe";
        emit nodeStatus(Node::VELOCITY, findProcess(processName, processID));
    }

}
