// Halden VT-4 — physical noise sources.
//
// Each source is specified by its white spectral density e_w [V/√Hz] and a
// flicker corner f_c: S(f) = e_w² · (1 + f_c / f). The flicker part is a
// sum of one-pole low-passes with geometrically spaced corners and weights
// 1/√f_k, which is 1/f within ±0.4 dB from 3 Hz to 30 kHz.

#pragma once

#include "Rng.h"

#include <array>
#include <cmath>

namespace vt4
{

class NoiseSource
{
public:
    static constexpr int kPoles = 8;

    void prepare (double sampleRate, double ew, double fc, uint32_t seed)
    {
        fs = sampleRate;
        white.seed (seed * 2654435761u + 1u);
        flick.seed (seed * 2246822519u + 7u);
        double f = 2.0;
        double sUnit = 0.0;                     // PSD at 1 kHz for unit-variance input
        const double f0 = 1000.0;
        for (int k = 0; k < kPoles; ++k)
        {
            a[(size_t) k] = std::exp (-2.0 * 3.141592653589793 * f / fs);
            g[(size_t) k] = 1.0 / std::sqrt (f);
            // exact digital one-pole magnitude² at f0: (1−a)² / |1 − a e^{-jw}|²
            const double w = 2.0 * 3.141592653589793 * f0 / fs;
            const double ak = a[(size_t) k];
            const double m2 = (1.0 - ak) * (1.0 - ak) / (1.0 - 2.0 * ak * std::cos (w) + ak * ak);
            sUnit += g[(size_t) k] * g[(size_t) k] * m2;
            f *= 4.0;
        }
        sUnit /= (0.5 * fs);
        const double sWhite = ew * ew;                 // V²/Hz
        whiteGain = std::sqrt (sWhite * 0.5 * fs);     // σ for a flat PSD of e_w²
        flickGain = fc > 0.0 ? std::sqrt (sWhite * (fc / f0) / sUnit) : 0.0;
        z.fill (0.0);
    }

    inline double next() noexcept
    {
        double y = whiteGain * (double) white.next();
        if (flickGain > 0.0)
        {
            const double x = (double) flick.next();
            double acc = 0.0;
            for (int k = 0; k < kPoles; ++k)
            {
                z[(size_t) k] = a[(size_t) k] * z[(size_t) k] + (1.0 - a[(size_t) k]) * x;
                acc += g[(size_t) k] * z[(size_t) k];
            }
            y += flickGain * acc;
        }
        return y;
    }

private:
    double fs = 176400.0, whiteGain = 0.0, flickGain = 0.0;
    NoiseGen white, flick;
    std::array<double, kPoles> a {}, g {}, z {};
};

} // namespace vt4
