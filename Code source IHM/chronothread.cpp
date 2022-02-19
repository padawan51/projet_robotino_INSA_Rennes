#include "chronothread.h"

ChronoThread::ChronoThread(int tempo_)
{
    if(tempo_ > 0) this->v_tempo = tempo_;
    else v_tempo = 5;
}

void ChronoThread::run()
{
    int val = 0;
    CLOCK_TIME_POINT t_start;
    double duration = 0.0;
    std::chrono::duration<double> t_span = {};

    t_start = CLOCK_NOW;

    do{
        t_span = CLOCK_DURATION(CLOCK_NOW - t_start);
        duration = t_span.count();
        if(static_cast<int>(duration) != val){
            val++;
            emit chrono(val);
        }
    }while(val <= v_tempo && !isInterruptionRequested());

    emit chrono(0);
}

void ChronoThread::setTempo(int value)
{
    if(value > 0) v_tempo = value, std::cout << "\ntempo = " << v_tempo << std::endl;
    else v_tempo = 5;
}
