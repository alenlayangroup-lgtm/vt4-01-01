// Halden VT-4 — the passive LC equaliser.
//
// Topology ("bridged injection" network, original to this design):
//
//   driver ──┬── R1 8k2 ──┬── N_mid ──► make-up stage grid
//            │            R2 2k2
//            │            ⏚
//            ├── pad (constant-impedance, 10 k in) ── tap x·v ── Rt ── band branch ──► N_mid
//            ├── pad … (one per band section)
//
// The flat network sits at −13.5 dB. Every band is a constant-impedance
// stepped pad whose output (x·v_in behind a fixed Rt) feeds an LC branch into
// the mid node. With x above the null x0 (= flat ratio) the branch lifts the
// mid node around its resonance (boost), below x0 it pulls it down (cut).
// Because the pad's source impedance does not change with x, the branch Q
// stays almost constant over the gain range; boost and cut use their own
// inductor taps and capacitors (centre-detent changeover), as in classic
// passive designs. All bands share the mid node, so they interact like the
// parts of a real passive network do.
//
//   LOW      L shelf (25 … 140 Hz, ±10 dB) + WEIGHT: two resonators fed from
//            extra pad sections, a bump at 0.8·f (in phase) and a dip at
//            2.2·f (mirrored tap) — a resonant shelf, mirrored for cuts.
//   BODY     series LC bell, very wide (≈ 2.5 octaves), 180 … 2700 Hz, ±6 dB
//   PRESENCE series LC bell, FOCUS wide/narrow taps, 1.2 … 8.2 kHz, ±8 dB
//   AIR      C shelf, 10 … 40 kHz, ±6 dB
//
// Inductors carry their winding resistance and core loss. Component values
// scale exactly with frequency (L, C ∝ 1/f), so the gain law of every band
// is independent of the selected frequency.

#pragma once

#include "Circuit.h"
#include "Rng.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace vt4
{

enum EqBand : int { kLow = 0, kBody, kPres, kAir, kNumBands };

namespace eq
{
    // network
    constexpr double R1 = 8200.0, R2 = 2200.0, RGL = 1.0e6, RSRC = 0.5;
    constexpr double PAD_IN = 33.0e3, RT = 600.0, RT_LOW = 400.0;
    constexpr double QCORE = 80.0;

    // bells: (Rq, Z0) for boost / cut
    constexpr double BODY_B_RQ = 34.7,  BODY_B_Z0 = 1467.4,  BODY_C_RQ = 53.9, BODY_C_Z0 = 1041.6;
    constexpr double PRW_B_RQ  = 78.2,  PRW_B_Z0  = 2692.8,  PRW_C_RQ  = 29.7, PRW_C_Z0  = 1650.3;
    constexpr double PRN_B_RQ  = 10.6,  PRN_B_Z0  = 5619.4,  PRN_C_RQ  = 16.3, PRN_C_Z0  = 3484.8;
    constexpr double BELL_DCR  = 0.015;           // winding resistance as fraction of Z0
    // Discretisation of the bell resonators: prewarped at the band frequency,
    // the L and C constants skewed apart (L: c·(1 − kL·θ²), C: c·(1 + kC·θ²),
    // θ = πf/fs). Fitted minimax against the analog network over 20 Hz–20 kHz
    // at 176.4 and 192 kHz: worst error 0.17 → 0.07 dB (PRESENCE wide at 8.2 kHz).
    constexpr double SKEW_BODY_L = 4.000, SKEW_BODY_C = -1.318;
    constexpr double SKEW_PRW_L  = 1.512, SKEW_PRW_C  =  1.264;
    constexpr double SKEW_PRN_L  = 1.125, SKEW_PRN_C  =  1.078;
    constexpr double AIR_WARP    = 16000.0;       // AIR shelf capacitor prewarp
    // Centre trim of the bells: the branch resonates slightly above the dial
    // value so that the peak (or dip) of the whole network lands on it — the
    // loading of the mid node pulls the extremum 0.1–0.6 % low otherwise.
    // Mean over 25 … 100 % of the range; residual ≤ ±0.1 %.
    constexpr double TRIM_BODY_B = 1.00588, TRIM_BODY_C = 1.00540;
    constexpr double TRIM_PRW_B  = 1.00339, TRIM_PRW_C  = 1.00302;
    constexpr double TRIM_PRN_B  = 1.00150, TRIM_PRN_C  = 1.00132;
    // LOW shelf: L = k/f
    constexpr double LOW_B_RQ = 100.0, LOW_B_LK = 524.0, LOW_C_RQ = 100.0, LOW_C_LK = 282.7;
    constexpr double LOW_QDC  = 15.0;             // ωL / DCR at the band frequency
    // WEIGHT resonators
    constexpr double W_FB = 0.8, W_ZB = 5000.0, W_RMIN = 1500.0, W_KB = 2.5;
    constexpr double W_FD = 2.2, W_ZD = 3000.0, W_DR = 0.5, W_KD = 0.6;
    constexpr double W_DCR = 0.02;
    // AIR shelf: C = k/f
    constexpr double AIR_B_RQ = 100.0, AIR_B_CK = 5.081e-5, AIR_C_RQ = 100.0, AIR_C_CK = 7.346e-5;

    // ranges
    constexpr double kMaxGain[kNumBands] = { 10.0, 6.0, 8.0, 6.0 };
    constexpr double kFMin[kNumBands] = { 25.0, 180.0, 1200.0, 10000.0 };
    constexpr double kFMax[kNumBands] = { 140.0, 2700.0, 8200.0, 40000.0 };
    constexpr int kNumSteps[kNumBands] = { 6, 8, 6, 5 };
    constexpr double kSteps[kNumBands][8] = {
        { 25, 35, 50, 70, 100, 140, 0, 0 },
        { 180, 270, 390, 560, 820, 1200, 1800, 2700 },
        { 1200, 1800, 2700, 3900, 5600, 8200, 0, 0 },
        { 10000, 14000, 20000, 28000, 40000, 0, 0, 0 } };
    constexpr double kHpfSteps[6] = { 18, 25, 35, 50, 70, 100 };

    // reference frequency (relative to the band frequency) of the gain law
    constexpr double kRefRatio[kNumBands] = { 0.1, 1.0, 1.0, 4.0 };

    inline double nullRatio()
    {
        const double r2 = R2 * RGL / (R2 + RGL);
        return r2 / (R1 + r2);
    }

    inline double snapFreq (int band, double f)
    {
        int best = 0;
        double bd = 1e30;
        for (int k = 0; k < kNumSteps[band]; ++k)
        {
            const double d = std::fabs (std::log (f / kSteps[band][k]));
            if (d < bd) { bd = d; best = k; }
        }
        return kSteps[band][best];
    }
}

struct BandState
{
    int set = 0;            // 0 = branch out, +1 boost parts, −1 cut parts
    double x = 0.2111;      // pad tap ratio
    double f = 1000.0;      // band frequency
    bool narrow = false;    // PRESENCE focus
    double weight = 0.0;    // LOW: 0 … 10
};

struct EqState
{
    std::array<BandState, kNumBands> band {};
};

// Per-unit component tolerances, drawn deterministically from the serial.
struct EqTolerance
{
    uint64_t seed = 1;
    double amount = 0.0;    // 0 = nominal, 1 = typical spread

    static double draw (uint64_t seed, uint64_t key, double sigma, double amount)
    {
        if (amount <= 0.0) return 1.0;
        Rng r (seed * 0x9E3779B97F4A7C15ull + key * 0xD1B54A32D192ED03ull + 0x632BE59BD9B4E019ull);
        r.next();
        return 1.0 + sigma * amount * r.clippedGauss();
    }

    // multiplier for one part; per switch position, interpolated in log f
    double part (int band, int set, int focus, int comp, double f, double sigma) const
    {
        if (amount <= 0.0) return 1.0;
        const int n = eq::kNumSteps[band];
        const double lf = std::log (std::clamp (f, eq::kSteps[band][0], eq::kSteps[band][n - 1]));
        int k = 0;
        while (k < n - 2 && lf > std::log (eq::kSteps[band][k + 1])) ++k;
        const double a = std::log (eq::kSteps[band][k]), b = std::log (eq::kSteps[band][k + 1]);
        const double t = std::clamp ((lf - a) / (b - a), 0.0, 1.0);
        auto key = [&] (int pos) { return (uint64_t) (((band * 4 + (set + 1)) * 2 + focus) * 16 + pos) * 32 + (uint64_t) comp; };
        const double m0 = draw (seed, key (k), sigma, amount), m1 = draw (seed, key (k + 1), sigma, amount);
        return m0 + (m1 - m0) * t;
    }
    double fixed (int comp, double sigma) const { return draw (seed, 100000u + (uint64_t) comp, sigma, amount); }
};

namespace eq
{
    // adds an inductor with winding resistance and core loss between a and b
    inline void inductor (Circuit& c, int a, int b, double L, double dcr, double fBand, double warp, int tag = -1, double skew = 0.0)
    {
        const int m = c.addNode();
        c.R (a, m, dcr);
        c.L (m, b, L, warp, tag, skew);
        c.R (a, b, 2.0 * kPi * fBand * L * QCORE);
    }

    inline void pad (Circuit& c, int NI, int& W, double x, double rt)
    {
        const int T = c.addNode();
        W = c.addNode();
        c.R (NI, Circuit::kGnd, PAD_IN);
        c.E (T, Circuit::kGnd, NI, Circuit::kGnd, x);
        c.R (T, W, rt);
    }

    // Builds the network. `warp` enables per-band prewarping (digital use).
    inline void build (Circuit& c, const EqState& s, const EqTolerance& tol, bool warp = true)
    {
        c.clear();
        const int G = Circuit::kGnd;
        const int IN = c.addNode(), NI = c.addNode(), OUT = c.addNode();
        c.V (IN, G, 0);
        c.R (IN, NI, RSRC);
        c.R (NI, OUT, R1 * tol.fixed (1, 0.005));
        c.R (OUT, G, R2 * tol.fixed (2, 0.005));
        c.R (OUT, G, RGL);
        c.output (OUT);

        for (int b = 0; b < kNumBands; ++b)
        {
            const BandState& st = s.band[(size_t) b];
            if (st.set == 0) continue;
            const double f = st.f;
            const int si = st.set > 0 ? 1 : 0;           // 1 boost, 0 cut
            const int fo = (b == kPres && st.narrow) ? 1 : 0;
            auto tp = [&] (int comp, double sigma) { return tol.part (b, st.set, fo, comp, f, sigma); };
            const int tagBase = b * 32 + si * 16;       // part identity: band, boost / cut set
            int W = 0;
            pad (c, NI, W, st.x, (b == kLow ? RT_LOW : RT) * tp (0, 0.01));

            if (b == kBody || b == kPres)
            {
                double rq, z0;
                if (b == kBody) { rq = si ? BODY_B_RQ : BODY_C_RQ; z0 = si ? BODY_B_Z0 : BODY_C_Z0; }
                else if (fo)    { rq = si ? PRN_B_RQ : PRN_C_RQ;   z0 = si ? PRN_B_Z0 : PRN_C_Z0; }
                else            { rq = si ? PRW_B_RQ : PRW_C_RQ;   z0 = si ? PRW_B_Z0 : PRW_C_Z0; }
                const double trim = b == kBody ? (si ? TRIM_BODY_B : TRIM_BODY_C)
                                  : (fo ? (si ? TRIM_PRN_B : TRIM_PRN_C) : (si ? TRIM_PRW_B : TRIM_PRW_C));
                const double fr = f * trim;                     // resonance of the branch
                const double w = 2.0 * kPi * fr;
                const double L = z0 / w * tp (1, 0.03), C = 1.0 / (z0 * w) * tp (2, 0.02);
                const double kL = b == kBody ? SKEW_BODY_L : (fo ? SKEW_PRN_L : SKEW_PRW_L);
                const double kC = b == kBody ? SKEW_BODY_C : (fo ? SKEW_PRN_C : SKEW_PRW_C);
                const int a = c.addNode(), bb = c.addNode();
                c.R (W, a, rq * tp (3, 0.01));
                inductor (c, a, bb, L, BELL_DCR * z0 * tp (4, 0.05), fr, warp ? fr : 0.0, tagBase + 0, warp ? kL : 0.0);
                c.C (bb, OUT, C, warp ? fr : 0.0, tagBase + 1, warp ? -kC : 0.0);
            }
            else if (b == kLow)
            {
                const double L = (si ? LOW_B_LK : LOW_C_LK) / f * tp (1, 0.03);
                const int a = c.addNode();
                c.R (W, a, (si ? LOW_B_RQ : LOW_C_RQ) * tp (3, 0.01));
                inductor (c, a, OUT, L, 2.0 * kPi * f * L / LOW_QDC * tp (4, 0.05), f, 0.0, tagBase + 0);

                if (st.weight > 1.0e-3)
                {
                    const double x0 = nullRatio();
                    const double wR = W_RMIN * 10.0 / st.weight;
                    // bump resonator (in phase, amplified tap)
                    {
                        const double xb = std::clamp (x0 + W_KB * (st.x - x0), 0.0, 1.0);
                        int Wb = 0; pad (c, NI, Wb, xb, RT * tp (5, 0.01));
                        const double fb = W_FB * f, w = 2.0 * kPi * fb;
                        const double Lb = W_ZB / w * tp (6, 0.03), Cb = 1.0 / (W_ZB * w) * tp (7, 0.02);
                        const int a2 = c.addNode(), b2 = c.addNode();
                        c.R (Wb, a2, wR);
                        inductor (c, a2, b2, Lb, W_DCR * W_ZB, fb, warp ? fb : 0.0, tagBase + 2);
                        c.C (b2, OUT, Cb, warp ? fb : 0.0, tagBase + 3);
                    }
                    // dip resonator (mirrored tap)
                    {
                        const double xd = std::clamp (x0 - W_KD * (st.x - x0), 0.0, 1.0);
                        int Wd = 0; pad (c, NI, Wd, xd, RT * tp (8, 0.01));
                        const double fd = W_FD * f, w = 2.0 * kPi * fd;
                        const double Ld = W_ZD / w * tp (9, 0.03), Cd = 1.0 / (W_ZD * w) * tp (10, 0.02);
                        const int a3 = c.addNode(), b3 = c.addNode();
                        c.R (Wd, a3, wR * W_DR);
                        inductor (c, a3, b3, Ld, W_DCR * W_ZD, fd, warp ? fd : 0.0, tagBase + 4);
                        c.C (b3, OUT, Cd, warp ? fd : 0.0, tagBase + 5);
                    }
                }
            }
            else // AIR
            {
                const double C = (si ? AIR_B_CK : AIR_C_CK) / f * tp (2, 0.02);
                const int a = c.addNode();
                c.R (W, a, (si ? AIR_B_RQ : AIR_C_RQ) * tp (3, 0.01));
                c.C (a, OUT, C, warp ? AIR_WARP : 0.0, tagBase + 0);
            }
        }
    }

    // Gain of a single band (others out) at its reference frequency, nominal
    // parts, analog domain — the calibration of the gain law.
    inline double bandGainDb (int band, int set, double x, bool narrow, double weight = 0.0)
    {
        EqState s;
        const double f = 1000.0;
        BandState& b = s.band[(size_t) band];
        b.set = set; b.x = x; b.f = f; b.narrow = narrow; b.weight = weight;
        Circuit c, flat;
        EqTolerance nominal;
        build (c, s, nominal, false);
        EqState none;
        build (flat, none, nominal, false);
        const double fr = f * kRefRatio[band];
        return 20.0 * std::log10 (std::abs (c.analogResponse (fr)) / std::abs (flat.analogResponse (fr)));
    }

    // Inverse gain law by bisection (used by the table generator).
    inline double solveTap (int band, int set, bool narrow, double gainDb)
    {
        const double x0 = nullRatio();
        double lo = set > 0 ? x0 : 1.0e-9, hi = set > 0 ? 1.0 - 1.0e-9 : x0;
        for (int it = 0; it < 80; ++it)
        {
            const double mid = 0.5 * (lo + hi);
            const double g = bandGainDb (band, set, mid, narrow);
            if (g < gainDb) lo = mid; else hi = mid;    // gain rises with x on both sides
        }
        return 0.5 * (lo + hi);
    }
}

} // namespace vt4
