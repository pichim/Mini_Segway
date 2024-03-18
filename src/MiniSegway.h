#ifndef MINI_SEGWAY_H_
#define MINI_SEGWAY_H_

#include "ThreadFlag.h"

class MiniSegway
{
public:
    explicit MiniSegway();
    virtual ~MiniSegway();

    // int getCntr() const { return _i; }

private:
    Thread _Thread;
    Ticker _Ticker;
    ThreadFlag _ThreadFlag;

    int _i{0};

    void threadTask();
    void sendThreadFlag();
};
#endif /* MINI_SEGWAY_H_ */
