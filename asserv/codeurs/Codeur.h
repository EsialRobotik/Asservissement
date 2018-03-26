#ifndef CODEUR
#define CODEUR

#include "mbed.h"
#include "../config/config.h"

//Un codeur branché directement sur la Mbed

class Codeur
{

public:

    Codeur(PinName pinChanA, PinName pinChanB);
    ~Codeur();
    int32_t getCount();
    void reset();

private:

    void handleInterruptA();
    void handleInterruptB();

    volatile int32_t count;
    InterruptIn pinChannelA;
    InterruptIn pinChannelB;

};

#endif
