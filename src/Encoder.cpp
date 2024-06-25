#include "Encoder.h"

Encoder::Encoder(PinName a,
                 PinName b,
                 uint16_t counts_per_turn,
                 float fcut,
                 float D,
                 float Ts) : _EncoderCounter(a, b)
                           , _counts_per_turn(static_cast<float>(counts_per_turn))
                           , _Ts(Ts)
{
    _lowPass2.lowPass2Init(fcut, D, Ts);
    reset();
}

void Encoder::reset()
{
    _EncoderCounter.reset();
    _lowPass2.reset(0.0f);
    _encoder_signals.counts = _counts_previous = _EncoderCounter.read();
    _encoder_signals.velocity = 0.0f;
    _encoder_signals.rotations = 0.0f;
}

Encoder::encoder_signals_t Encoder::read()
{
    static float velocity_gain = 2.0f * M_PIf / _Ts;
    static float rotation_gain = 2.0f * M_PIf / _counts_per_turn;

    const float rotation_increment = updateEncoderAndReturnDeltaCounts();
    _encoder_signals.velocity = _lowPass2.apply(velocity_gain * rotation_increment);
    _encoder_signals.rotations = rotation_gain * static_cast<float>(_encoder_signals.counts);
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
