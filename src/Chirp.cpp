#include "Chirp.h"

Chirp::Chirp(const float f0, const float f1, const float t1, const float Ts)
{
    init(f0, f1, t1, Ts);
}

// initialize the chirp signal generator
// f0: start frequency in Hz
// f1: end frequency in Hz
// t1: signal length in seconds
// Ts: sampling time in seconds
void Chirp::init(const float f0, const float f1, const float t1, const float Ts)
{
    chirp.f0 = f0;
    chirp.Ts = Ts;
    chirp.N = static_cast<uint32_t>(t1 / Ts);
    chirp.beta = powf(f1 / f0, 1.0f / t1);
    chirp.k0 = 2.0f * M_PIf / logf(chirp.beta);
    chirp.k1 = chirp.k0 * f0;
    reset();
}

// reset the chirp signal generator fully
void Chirp::reset()
{
    chirp.count = 0;
    chirp.isFinished = false;
    resetSignals();
}

// update the chirp signal generator
bool Chirp::update()
{
    if (chirp.isFinished) {

        return false;

    } else if (chirp.count == chirp.N) {

        chirp.isFinished = true;
        resetSignals();
        return false;

    } else {

        chirp.fchirp = chirp.f0 * powf(chirp.beta, static_cast<float>(chirp.count) * chirp.Ts);
        chirp.sinarg = chirp.k0 * chirp.fchirp - chirp.k1;
        chirp.sinarg = fmodf(chirp.sinarg, 2.0f * M_PIf);
        chirp.exc = sinf(chirp.sinarg);
        chirp.count++;

        return true;
    }
}

float Chirp::getFreq() const
{
    return chirp.fchirp;
}

float Chirp::getSinarg() const
{
    return chirp.sinarg;
}

float Chirp::getExc() const
{
    return chirp.exc;
}

// reset the chirp signal generator signals
void Chirp::resetSignals()
{
    chirp.exc = 0.0f;
    chirp.fchirp = 0.0f;
    chirp.sinarg = 0.0f;
}