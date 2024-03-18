#include "MiniSegway.h"


MiniSegway::MiniSegway() : _Thread(osPriorityHigh, 4096)
{
    _Thread.start(callback(this, &MiniSegway::threadTask));
    _Ticker.attach(callback(this, &MiniSegway::sendThreadFlag), std::chrono::microseconds{100000});
}

MiniSegway::~MiniSegway()
{
    _Ticker.detach();
    _Thread.terminate();
}

void MiniSegway::threadTask()
{   
    DigitalOut led1(LED1);
    printf("MiniSegay starting...\n");

    while (true) {
        ThisThread::flags_wait_any(_ThreadFlag);
        printf("_i: %d\n", _i++);
        led1 = !led1;
    }
}

void MiniSegway::sendThreadFlag()
{
    _Thread.flags_set(_ThreadFlag);
}