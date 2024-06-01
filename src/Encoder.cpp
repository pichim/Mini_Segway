#include "Encoder.h"

Encoder::Encoder(PinName a,
                 PinName b,
                 uint16_t counts_per_turn,
                 float w_cut,
                 float D,
                 float Ts) : _EncoderCounter(a, b)
                           , _IIR_Velocity_Filter(w_cut,
                                                  D,
                                                  Ts,
                                                  1.0f)
                           , _counts_per_turn(static_cast<float>(counts_per_turn))
                           , _Ts(Ts)
{
    reset();
}

void Encoder::reset()
{
    _EncoderCounter.reset();
    _IIR_Velocity_Filter.reset();
    _encoder_signals.counts = _counts_previous = _EncoderCounter.read();
    _encoder_signals.velocity = 0.0f;
    _encoder_signals.rotations = 0.0f;
}

Encoder::encoder_signals_t Encoder::read()
{
    const float rotation_increment = updateEncoderAndReturnDeltaCounts();
    _encoder_signals.velocity = _IIR_Velocity_Filter.filter(rotation_increment / _Ts);
    _encoder_signals.rotations = static_cast<float>(_encoder_signals.counts) / _counts_per_turn;
    return _encoder_signals;
}

float Encoder::updateEncoderAndReturnDeltaCounts()
{  
    // avoid overflow
    const short counts = _EncoderCounter.read();
    const short counts_delta = counts - _counts_previous;
    _counts_previous = counts;
    _encoder_signals.counts += counts_delta;
    return static_cast<float>(counts_delta) / _counts_per_turn;
}
