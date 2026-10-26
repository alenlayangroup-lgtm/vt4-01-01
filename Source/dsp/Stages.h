// Halden VT-4 — tube stages.
//
// Each stage is a small circuit solved as a whole with its tube (nodal DK +
// damped Newton on the plate and grid currents), so loading, cathode
// degeneration, coupling capacitors and feedback are exact rather than
// approximated by block diagrams.
//
//   InputStage  V1a  5751 common cathode, AC-switchable cathode bypass
//                    (CLEAN / WARM / BOLD), followed by the passive RC-RC
//                    high-pass (18 … 100 Hz, OFF) that feeds the driver.
//   DriverStage V2a  6922 cathode follower that drives the passive EQ.
//   OutputStage V3a  5751 make-up gain inside a feedback loop with the 6922
//                    White cathode follower (TIGHT / MEDIUM / OPEN). The loop
//                    is closed from the output transformer's feedback winding
//                    (an inverted copy of the follower output) into V3a's
//                    unbypassed cathode resistor; the iron itself stays
//                    outside the loop, so feedback never cancels its
//                    character.
//
// All stages take the stage supply (B+) as a second input, so sag and
// ripple from the power supply model reach the tubes physically.

#pragma once

#include "Circuit.h"
#include "Triode.h"

namespace vt4
{

// A tube stage = circuit + tube + solver state.
struct TubeStage
{
    Circuit c;
    TriodeParams tube;
    TriodeSolver sol;
    double u[2] { 0.0, 0.0 };    // [grid source incl. bias, B+]
    double lastIp = 0.0;          // plate current (for the supply model)
    double qJ[4] { 0.0, 0.0, 0.0, 0.0 };   // tube Jacobian at the quiescent point

    template <int NS>
    inline double stepT (double vin, double bplus, int out) noexcept
    {
        u[0] = vin; u[1] = bplus;
        double p[2];
        c.template controlsT<NS, 2, 2, 2> (u, p);
        sol.solve (tube, c, p);
        const double i[2] { sol.ip, sol.ig };
        const double y = c.template outputT<NS, 2, 2> (out, u, i);
        c.template advanceT<NS, 2, 2> (u, i);
        lastIp = sol.ip + sol.ig;
        return y;
    }

    inline double step (double vin, double bplus, int out = 0) noexcept
    {
        switch (c.states())
        {
            case 0: return stepT<0> (vin, bplus, out);
            case 1: return stepT<1> (vin, bplus, out);
            case 2: return stepT<2> (vin, bplus, out);
            case 3: return stepT<3> (vin, bplus, out);
            case 4: return stepT<4> (vin, bplus, out);
            default: break;
        }
        u[0] = vin; u[1] = bplus;
        double p[2];
        c.controls (u, p);
        sol.solve (tube, c, p);
        const double i[2] { sol.ip, sol.ig };
        const double y = c.outputNL (out, u, i);
        c.advanceNL (u, i);
        lastIp = sol.ip + sol.ig;
        return y;
    }

    // DC operating point for constant inputs: solve the static circuit
    // (capacitors open), then put the dynamic states at that point.
    void settle (double vin, double bplus, CircuitWorkspace& ws)
    {
        u[0] = vin; u[1] = bplus;
        Circuit::StaticModel sm;
        c.compileStatic (ws, sm);
        double p[2];
        sm.controls (u, p);
        sol.solveRobust (tube, p, sm.feedback (0, 0), sm.feedback (0, 1), sm.feedback (1, 0), sm.feedback (1, 1));
        sol.restart (p);
        const double i[2] { sol.ip, sol.ig };
        c.steadyState (u, i);
        lastIp = sol.ip + sol.ig;
        const double vg = p[0] + sm.feedback (0, 0) * sol.ip + sm.feedback (0, 1) * sol.ig;
        const double vp = p[1] + sm.feedback (1, 0) * sol.ip + sm.feedback (1, 1) * sol.ig;
        const TriodeEval e = triode (tube, vg, vp);
        qJ[0] = e.dIpdVg; qJ[1] = e.dIpdVp; qJ[2] = e.dIgdVg; qJ[3] = 0.0;
    }

    // Recomputes the quiescent Jacobian for the given bias and supply without
    // touching the running state (used while audio is playing).
    //
    // The quiescent point moves only slowly (supply sag, warm-up, drift), so
    // it is found by Newton from the previous one, converged to machine
    // precision; the bracketed solver is the fallback. The static model is
    // recompiled only when the stage circuit was rebuilt.
    void updateQuiescent (double vin, double bplus, CircuitWorkspace& ws)
    {
        if (smVersion != c.version()) { c.compileStatic (ws, smCache); smVersion = c.version(); }
        const Circuit::StaticModel& sm = smCache;
        const double uu[2] { vin, bplus };
        double p[2];
        sm.controls (uu, p);
        const double F00 = sm.feedback (0, 0), F01 = sm.feedback (0, 1), F10 = sm.feedback (1, 0), F11 = sm.feedback (1, 1);
        double i0 = qIp, i1 = qIg;
        bool ok = false;
        if (haveQ)
        {
            for (int it = 0; it < 30; ++it)
            {
                const double vg = p[0] + F00 * i0 + F01 * i1;
                const double vp = p[1] + F10 * i0 + F11 * i1;
                const TriodeEval e = triode (tube, vg, vp);
                const double r0 = i0 - e.ip, r1 = i1 - e.ig;
                const double j00 = 1.0 - (e.dIpdVg * F00 + e.dIpdVp * F10);
                const double j01 = -(e.dIpdVg * F01 + e.dIpdVp * F11);
                const double j10 = -(e.dIgdVg * F00);
                const double j11 = 1.0 - (e.dIgdVg * F01);
                const double det = j00 * j11 - j01 * j10;
                if (! (std::fabs (det) > 1.0e-30)) break;
                const double d0 = ( j11 * r0 - j01 * r1) / det;
                const double d1 = (-j10 * r0 + j00 * r1) / det;
                i0 -= d0; i1 -= d1;
                if (! (i0 > 0.0) || i1 < -1.0e-12 || ! std::isfinite (i1)) break;
                if (std::fabs (d0) <= 1.0e-14 * i0 && std::fabs (d1) <= 1.0e-20 + 1.0e-14 * std::fabs (i1)) { ok = true; break; }
            }
        }
        if (! ok)
        {
            TriodeSolver tmp = sol;
            tmp.solveRobust (tube, p, F00, F01, F10, F11);
            i0 = tmp.ip; i1 = tmp.ig;
        }
        i1 = std::max (i1, 0.0);
        qIp = i0; qIg = i1; haveQ = true;
        const double vg = p[0] + F00 * i0 + F01 * i1;
        const double vp = p[1] + F10 * i0 + F11 * i1;
        const TriodeEval e = triode (tube, vg, vp);
        qJ[0] = e.dIpdVg; qJ[1] = e.dIpdVp; qJ[2] = e.dIgdVg; qJ[3] = 0.0;
    }

    // cache for updateQuiescent
    Circuit::StaticModel smCache;
    uint32_t smVersion = 0xffffffffu;
    double qIp = 0.0, qIg = 0.0;
    bool haveQ = false;

    // Small-signal Jacobian of the tube at the present solution (for the
    // display response): rows Ip, Ig; columns Vgk, Vpk.
    void jacobian (double* Jf) const
    {
        double p[2];
        c.controls (u, p);
        const double vg = p[0] + c.feedback (0, 0) * sol.ip + c.feedback (0, 1) * sol.ig;
        const double vp = p[1] + c.feedback (1, 0) * sol.ip + c.feedback (1, 1) * sol.ig;
        const TriodeEval e = triode (tube, vg, vp);
        Jf[0] = e.dIpdVg; Jf[1] = e.dIpdVp; Jf[2] = e.dIgdVg; Jf[3] = 0.0;
    }

    double vgk() const
    {
        double p[2];
        c.controls (u, p);
        return p[0] + c.feedback (0, 0) * sol.ip + c.feedback (0, 1) * sol.ig;
    }
    double vpk() const
    {
        double p[2];
        c.controls (u, p);
        return p[1] + c.feedback (1, 0) * sol.ip + c.feedback (1, 1) * sol.ig;
    }
};

// ---------------------------------------------------------------------------
struct InputStageValues
{
    double rgs   = 600.0;     // grid source (pad Thevenin + stopper)
    double rl    = 100.0e3;   // plate load
    double rk    = 10.0e3;    // cathode resistor (DC)
    double rb    = 1.0e9;     // AC bypass resistance (CLEAN ∞, WARM 2k2, BOLD 0)
    double ck    = 100.0e-6;  // bypass capacitor
    double ch1   = 1.0e-6;    // high-pass capacitors (OFF = 1 µF)
    double ch2   = 1.0e-6;
    double rh    = 470.0e3;
    double rg    = 1.0e6;     // driver grid leak (AC)
};

inline void buildInputStage (Circuit& c, const InputStageValues& v)
{
    c.clear();
    const int G = Circuit::kGnd;
    const int S = c.addNode(), Gn = c.addNode(), P = c.addNode(), K = c.addNode(),
              X = c.addNode(), B = c.addNode(), H = c.addNode(), O = c.addNode();
    c.V (S, G, 0); c.V (B, G, 1);
    c.R (S, Gn, v.rgs);
    c.R (B, P, v.rl);
    c.R (K, G, v.rk);
    c.R (K, X, v.rb); c.C (X, G, v.ck);
    c.N (P, K); c.N (Gn, K);
    c.control (Gn, K); c.control (P, K);
    c.C (P, H, v.ch1); c.R (H, G, v.rh); c.C (H, O, v.ch2); c.R (O, G, v.rg);
    c.output (O); c.output (P);
}

// ---------------------------------------------------------------------------
struct DriverStageValues
{
    double rgs   = 1000.0;    // grid stopper
    double rk    = 10.0e3;    // cathode resistor
    double co    = 47.0e-6;   // output coupling capacitor
    double rload = 4000.0;    // input impedance of the passive EQ
};

inline void buildDriverStage (Circuit& c, const DriverStageValues& v)
{
    c.clear();
    const int G = Circuit::kGnd;
    const int S = c.addNode(), Gn = c.addNode(), B = c.addNode(), K = c.addNode(), O = c.addNode();
    c.V (S, G, 0); c.V (B, G, 1);
    c.R (S, Gn, v.rgs);
    c.N (B, K); c.N (Gn, K);
    c.control (Gn, K); c.control (B, K);
    c.R (K, G, v.rk);
    c.C (K, O, v.co); c.R (O, G, v.rload);
    c.output (O); c.output (K);
}

// ---------------------------------------------------------------------------
struct OutputStageValues
{
    double rgs   = 2200.0;    // EQ output impedance + stopper
    double rl    = 100.0e3;
    double rka   = 330.0;     // unbypassed cathode resistor (feedback point)
    double rkb   = 1200.0;
    double ckb   = 220.0e-6;
    double rfb   = 4700.0;    // feedback resistor (OPEN = 1e9)
    double cfb   = 10.0e-6;
    double acf   = 0.985;     // White cathode follower gain …
    double rocf  = 5.0;       // … and output resistance
};

inline void buildOutputStage (Circuit& c, const OutputStageValues& v)
{
    c.clear();
    const int G = Circuit::kGnd;
    const int S = c.addNode(), Gn = c.addNode(), P = c.addNode(), B = c.addNode(), K = c.addNode(),
              F = c.addNode(), Yi = c.addNode(), Y = c.addNode(), M = c.addNode(), W = c.addNode();
    c.V (S, G, 0); c.V (B, G, 1);
    c.R (S, Gn, v.rgs);
    c.R (B, P, v.rl);
    c.N (P, K); c.N (Gn, K);
    c.control (Gn, K); c.control (P, K);
    c.R (K, F, v.rka); c.R (F, G, v.rkb); c.C (F, G, v.ckb);
    c.E (Yi, G, P, G, v.acf); c.R (Yi, Y, v.rocf);
    c.E (W, G, Y, G, -1.0);                       // feedback winding (inverting)
    c.C (W, M, v.cfb); c.R (M, K, v.rfb);
    c.output (Y); c.output (P);
}

} // namespace vt4
