#ifndef ENCODER_H_
#define ENCODER_H_

#include "EncoderCounter.h"
#include "IIR_Filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Encoder
{
public:
    explicit Encoder(PinName a,
                     PinName b,
                     uint16_t counts_per_turn,
                     float f_cut,
                     float Ts);
    virtual ~Encoder() {};

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
    IIR_Filter _IIR_Velocity_Filter;
    encoder_signals_t _encoder_signals;

    short _counts_previous;
    float _counts_per_turn;
    float _Ts;

    float updateEncoderAndReturnDeltaCounts();
};

#endif /* ENCODER_H_ */