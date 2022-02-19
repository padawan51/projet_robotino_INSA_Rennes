#ifndef CHECKNODESTHREAD_H
#define CHECKNODESTHREAD_H

#include <QThread>
#include <QObject>

#include "utils.h"

typedef ProjectNode Node;

class CheckNodesThread : public QThread
{
    Q_OBJECT
public:
    explicit CheckNodesThread(QObject *parent = nullptr);
    void run();
public:
signals:
    void nodeStatus(int node, bool status);
};

#endif // CHECKNODESTHREAD_H
