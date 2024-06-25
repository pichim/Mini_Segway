#pragma once

#include "EncoderCounter.h"
#include "IIRFilter.h"

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f /* pi */
#endif

class Encoder
{
public:
    explicit Encoder(PinName a,
                     PinName b,
                     uint16_t counts_per_turn,
                     float fcut,
                     float D,
                     float Ts);
    virtual ~Encoder() = default;

    typedef struct encoder_signals_s {
        long counts{0};
        float velocity{0.0f};
        // TODO: should rotations be a double?
        float rotations{0.0f};
    } encoder_signals_t;

    void reset();
    encoder_signals_t read();

private:
    EncoderCounter _EncoderCounter;
    IIRFilter _lowPass2;
    encoder_signals_t _encoder_signals;

    short _counts_previous;
    float _counts_per_turn;
    float _Ts;

    float updateEncoderAndReturnDeltaCounts();
};