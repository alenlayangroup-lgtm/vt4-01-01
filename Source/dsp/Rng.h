// Deterministic random numbers (identical on every compiler / OS), used for
// per-unit component tolerances and for noise.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace vt4
{

class Rng
{
public:
    explicit Rng (uint64_t seed = 1) : s (seed != 0 ? seed : 0x9E3779B97F4A7C15ull) {}

    uint64_t next() noexcept
    {
        uint64_t z = (s += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }

    double uniform() noexcept { return (double) (next() >> 11) * (1.0 / 9007199254740992.0); }

    double gauss() noexcept
    {
        const double u1 = std::max (uniform(), 1.0e-12);
        const double u2 = uniform();
        return std::sqrt (-2.0 * std::log (u1)) * std::cos (6.283185307179586 * u2);
    }

    double clippedGauss (double limit = 2.5) noexcept { return std::clamp (gauss(), -limit, limit); }

private:
    uint64_t s;
};

// Fast Gaussian-ish noise for the audio thread: one xorshift64* step gives
// four 16-bit uniforms, their sum (Irwin–Hall, n = 4) is scaled to σ = 1.
class NoiseGen
{
public:
    NoiseGen() noexcept : s (0x9E3779B97F4A7C15ull) {}
    explicit NoiseGen (uint32_t seed) noexcept { this->seed (seed); }
    void seed (uint32_t v) noexcept { s = ((uint64_t) v << 32 | (uint64_t) (v ^ 0x5bd1e995u)) * 0x9E3779B97F4A7C15ull; if (s == 0) s = 1; }

    inline float next() noexcept
    {
        s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
        const uint64_t v = s * 2685821657736338717ull;
        const float a = (float) (uint32_t) (v & 0xffffu) + (float) (uint32_t) ((v >> 16) & 0xffffu)
                      + (float) (uint32_t) ((v >> 32) & 0xffffu) + (float) (uint32_t) (v >> 48);
        return (a * (1.0f / 65536.0f) - 2.0f) * 1.7320508f;    // var of the sum of 4 U(0,1) = 1/3
    }

private:
    uint64_t s;
};

// A table of component multipliers for one circuit board, drawn once per
// unit serial number and channel. Each slot is one physical part.
template <size_t N>
struct PartTolerances
{
    double m[N];

    void generate (uint64_t seed, const double (&sigma)[N], double amount)
    {
        Rng rng (seed);
        for (size_t i = 0; i < N; ++i)
            m[i] = 1.0 + sigma[i] * amount * rng.clippedGauss();
    }

    void nominal() { for (size_t i = 0; i < N; ++i) m[i] = 1.0; }
    double operator[] (int i) const { return m[(size_t) i]; }
};

inline uint64_t boardSeed (uint32_t serial, int channel, uint32_t board)
{
    return (uint64_t) serial * (uint64_t) 0x100000001B3ull
         + (uint64_t) (channel + 1) * (uint64_t) 0x9E3779B97F4A7C15ull
         + (uint64_t) board * (uint64_t) 0xD6E8FEB86659FD93ull;
}

} // namespace vt4
