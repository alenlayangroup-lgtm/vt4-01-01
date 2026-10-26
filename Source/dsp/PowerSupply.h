// Halden VT-4 — shared stereo power supply.
//
//   mains ── HV winding (centre-tapped, R_w) ── GZ34 / silicon ──┬── C1 47µ ── choke 10 H ──┬── C2 220µ = B+ main
//                                                                                        │
//          per channel:  8k2 / 100µ ─ 6k8 / 100µ → B+ V1a (two-stage: the input stage
//                        has the highest gain after it)
//                        6k8 / 100µ → B+ V3a        470 / 100µ → B+ driver
//          White cathode followers draw from B+ main
//
// The rectifier is the only nonlinearity (Child–Langmuir 3/2 law for the
// GZ34, Shockley diode for silicon). Every stage's actual plate current is
// fed back as a load, so B+ sags with the music, both channels share the
// same reservoir (a tiny, physical crosstalk) and the 100/120 Hz ripple that
// is left after the choke and the RC filters reaches each stage's plate.

#pragma once

#include "Circuit.h"

#include <cmath>

namespace vt4
{

struct PsuLoads
{
    double v1a[2] { 1.0e-3, 1.0e-3 };
    double v3a[2] { 1.1e-3, 1.1e-3 };
    double drv[2] { 10.0e-3, 10.0e-3 };
    double white = 2.0 * 30.0e-3;
};

struct PsuRails
{
    double main = 300.0;
    double v1a[2] { 285.0, 285.0 };
    double v3a[2] { 292.0, 292.0 };
    double drv[2] { 295.0, 295.0 };
};

class PowerSupply
{
public:
    enum Rect { kTube = 0, kSilicon = 1 };

    struct Values
    {
        double vhv  = 362.0;    // peak of each HV half winding
        double rw   = 60.0;
        double c1   = 47.0e-6, esr1 = 0.5;
        double lch  = 10.0, rch = 150.0;
        double c2   = 220.0e-6;
        double rd1  = 8.2e3,  cd1 = 100.0e-6;
        double rd1b = 6.8e3,  cd1b = 100.0e-6;
        double rd2  = 6.8e3,  cd2 = 100.0e-6;
        double rd3  = 470.0,  cd3 = 100.0e-6;
        double rsil = 60.0;     // series resistor in silicon mode (matches B+)
    };

    void prepare (double sampleRate, CircuitWorkspace& ws)
    {
        fs = sampleRate;
        build (ws);
        phase = 0.0;
        settle (PsuLoads {}, ws);
    }

    void setMains (double hz) { mainsHz = hz; }
    void setRectifier (int r, CircuitWorkspace& ws) { if (r != rect) { rect = r; build (ws); } }
    int rectifier() const { return rect; }
    // warm-up of the rectifier (GZ34 is indirectly heated): 0 … 1
    void setRectifierEmission (double e) { emission = e; }

    inline const PsuRails& step (const PsuLoads& l) noexcept
    {
        phase += mainsHz / fs;
        if (phase >= 1.0) phase -= 1.0;
        const double e = std::fabs (val.vhv * std::sin (2.0 * kPi * phase));
        u[0] = e;
        u[1] = l.v1a[0]; u[2] = l.v1a[1]; u[3] = l.v3a[0]; u[4] = l.v3a[1];
        u[5] = l.drv[0]; u[6] = l.drv[1]; u[7] = l.white;
        double p[1];
        c.controls (u, p);
        solveDiode (p[0]);
        const double i[1] { id };
        rails.main   = c.outputNL (0, u, i);
        rails.v1a[0] = c.outputNL (1, u, i); rails.v1a[1] = c.outputNL (2, u, i);
        rails.v3a[0] = c.outputNL (3, u, i); rails.v3a[1] = c.outputNL (4, u, i);
        rails.drv[0] = c.outputNL (5, u, i); rails.drv[1] = c.outputNL (6, u, i);
        vC1 = c.outputNL (7, u, i);
        vMid[0] = c.outputNL (8, u, i); vMid[1] = c.outputNL (9, u, i);
        c.advanceNL (u, i);
        iChoke = c.state (1);
        return rails;
    }

    const PsuRails& current() const noexcept { return rails; }

    // Runs the supply to its steady state for the given loads (start-up).
    // The rectifier, reservoir and choke settle in a few seconds, but the RC
    // rail filters have time constants of up to ~2 s, so instead of running
    // for tens of seconds their capacitors are set to the exact DC solution
    // for the mean B+ (R·I drops); a coarse copy at fs/8 runs the rest.
    void settle (const PsuLoads& l, CircuitWorkspace& ws)
    {
        PowerSupply coarse;
        coarse.val = val; coarse.rect = rect; coarse.mainsHz = mainsHz; coarse.emission = emission;
        coarse.fs = fs > 20000.0 ? fs / 4.0 : fs;
        coarse.build (ws);
        double mainMean = 0.83 * val.vhv;
        for (int pass = 0; pass < 3; ++pass)
        {
            coarse.seedRails (l, mainMean);
            const int n = (int) (coarse.fs * (pass == 0 ? 4.0 : 2.0));
            const int nAvg = (int) (coarse.fs / mainsHz) * 4;           // last four mains cycles
            double acc = 0.0;
            for (int k = 0; k < n; ++k)
            {
                coarse.step (l);
                if (k >= n - nAvg) acc += coarse.rails.main;
            }
            mainMean = acc / (double) nAvg;
        }
        if (&coarse != this && coarse.fs != fs)
        {
            const double T = 1.0 / fs;
            c.setState (0, 2.0 * val.c1 / T * coarse.vC1);
            c.setState (1, coarse.iChoke);
            c.setState (2, 2.0 * val.c2 / T * coarse.rails.main);
            phase = coarse.phase;
            vd = coarse.vd; id = coarse.id;
        }
        else
            *this = coarse;
        seedRails (l, mainMean);
        for (int k = 0; k < (int) (fs * 0.25); ++k) step (l);
    }

    // Puts the RC rail capacitors at their DC voltages for a given B+ and loads.
    void seedRails (const PsuLoads& l, double mainV)
    {
        const double T = 1.0 / fs;
        for (int ch = 0; ch < 2; ++ch)
        {
            const double vm = mainV - l.v1a[ch] * val.rd1;
            const double v1 = vm - l.v1a[ch] * val.rd1b;
            const double v3 = mainV - l.v3a[ch] * val.rd2;
            const double vdv = mainV - l.drv[ch] * val.rd3;
            c.setState (3 + 4 * ch, 2.0 * val.cd1 / T * vm);
            c.setState (4 + 4 * ch, 2.0 * val.cd1b / T * v1);
            c.setState (5 + 4 * ch, 2.0 * val.cd2 / T * v3);
            c.setState (6 + 4 * ch, 2.0 * val.cd3 / T * vdv);
            vMid[ch] = vm; rails.v1a[ch] = v1; rails.v3a[ch] = v3; rails.drv[ch] = vdv;
        }
    }

private:
    void build (CircuitWorkspace& ws)
    {
        c.clear();
        const int G = Circuit::kGnd;
        const int H = c.addNode(), A = c.addNode(), K = c.addNode(), C1 = c.addNode(), Ch = c.addNode(), M = c.addNode();
        c.V (H, G, 0);
        c.R (H, A, val.rw + (rect == kSilicon ? val.rsil : 0.0));
        c.N (A, K);
        c.control (A, K);
        c.R (K, C1, val.esr1); c.C (C1, G, val.c1);
        c.R (K, Ch, val.rch); c.L (Ch, M, val.lch);
        c.C (M, G, val.c2);
        c.I (M, G, 7);                                   // White cathode followers
        int rails1[2], rails3[2], railsD[2], mid1[2];
        for (int ch = 0; ch < 2; ++ch)
        {
            mid1[ch] = c.addNode(); c.R (M, mid1[ch], val.rd1); c.C (mid1[ch], G, val.cd1);
            rails1[ch] = c.addNode(); c.R (mid1[ch], rails1[ch], val.rd1b); c.C (rails1[ch], G, val.cd1b); c.I (rails1[ch], G, 1 + ch);
            rails3[ch] = c.addNode(); c.R (M, rails3[ch], val.rd2); c.C (rails3[ch], G, val.cd2); c.I (rails3[ch], G, 3 + ch);
            railsD[ch] = c.addNode(); c.R (M, railsD[ch], val.rd3); c.C (railsD[ch], G, val.cd3); c.I (railsD[ch], G, 5 + ch);
        }
        c.output (M);
        c.output (rails1[0]); c.output (rails1[1]);
        c.output (rails3[0]); c.output (rails3[1]);
        c.output (railsD[0]); c.output (railsD[1]);
        c.output (C1);
        c.output (mid1[0]); c.output (mid1[1]);
        c.compile (fs, ws);
    }

    inline double diode (double v, double& d) const noexcept
    {
        if (rect == kTube)
        {
            // GZ34: I = K·v^1.5 with a soft onset (≈ 17 V drop at 250 mA)
            const double kgz = 3.6e-3 * emission, vs = 0.5;
            const double x = v / vs;
            const double sp = x > 30.0 ? x : (x < -30.0 ? std::exp (x) : std::log1p (std::exp (x)));
            const double sg = x > 30.0 ? 1.0 : (x < -30.0 ? std::exp (x) : 1.0 / (1.0 + std::exp (-x)));
            const double vv = vs * sp, sq = std::sqrt (vv);
            d = 1.5 * kgz * sq * sg;
            return kgz * vv * sq;
        }
        // silicon: Shockley, n·Vt = 50 mV
        const double is = 1.0e-9, nvt = 0.05;
        const double ex = std::exp (std::min (v / nvt, 60.0));
        d = is * ex / nvt;
        return is * (ex - 1.0);
    }

    // v = p + F·i(v), F < 0: g(v) = v − p − F·i(v) is monotonic increasing.
    // Safeguarded Newton: the root lies in [min(p, 0) − 1, max(p, 0)], every
    // iterate is kept inside the shrinking bracket (bisection fallback), so a
    // change of rectifier or a cold start can never overshoot.
    inline void solveDiode (double p) noexcept
    {
        const double F = c.feedback (0, 0);
        double lo = std::min (p, 0.0) - 1.0, hi = std::max (p, 0.0) + 1.0e-9;
        double v = std::clamp (vd, lo, hi);
        if (rect == kSilicon) v = std::min (v, 0.75);
        double d = 0.0, i = 0.0, gPrev = 1.0e300;
        for (int it = 0; it < 100; ++it)
        {
            i = diode (v, d);
            const double g = v - p - F * i;
            if (g > 0.0) hi = v; else lo = v;
            if (std::fabs (g) < 1.0e-9 || hi - lo < 1.0e-10) break;
            const double gd = 1.0 - F * d;
            double nv = v - g / gd;
            // bisect when Newton leaves the bracket or stalls (clamped exponential)
            if (! (nv > lo && nv < hi) || std::fabs (g) > 0.5 * gPrev) nv = 0.5 * (lo + hi);
            gPrev = std::fabs (g);
            v = nv;
        }
        vd = v;
        id = diode (v, d);
    }

    Values val;
    Circuit c;
    double u[8] {};
    double fs = 176400.0, mainsHz = 50.0, phase = 0.0, id = 0.0, vd = 0.0, emission = 1.0;
    double vC1 = 0.0, iChoke = 0.0, vMid[2] { 0.0, 0.0 };
    int rect = kTube;
    PsuRails rails;
};

} // namespace vt4
