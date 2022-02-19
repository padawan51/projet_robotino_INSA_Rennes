#ifndef CHRONOTHREAD_H
#define CHRONOTHREAD_H

#include <QObject>
#include <QThread>

#include <iostream>

class ChronoThread: public QThread
{
    Q_OBJECT
public:
    ChronoThread(int tempo_ = 0);
    void run();
    void setTempo(int value);
private:
    int v_tempo; //unité: seconde
public: signals:
    void chrono(int time);

};

#endif // CHRONOTHREAD_H
