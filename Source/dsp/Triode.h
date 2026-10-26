// Halden VT-4 — triode model.
//
// Plate current: Koren's improved triode equation
//
//   E1 = (Vpk / kp) · ln(1 + exp(kp · (1/μ + Vgk / √(kvb + Vpk²))))
//   Ip = 2 · E1^ex / kg1            (E1 > 0)
//
// Grid current: Child–Langmuir 3/2-power law with a soft onset, flowing from
// grid to cathode once the grid approaches the cathode potential.
//
// Parameters for the 5751 and the 6922 / E88CC were fitted to the published
// characteristics (5751: 1.0 mA, 1.2 mA/V at 250 V / −3 V; 6922: 15 mA,
// 12.5 mA/V, μ ≈ 33 at 90 V / −1.3 V).

#pragma once

#include <algorithm>
#include <cmath>

namespace vt4
{

struct TriodeParams
{
    double mu = 78.0, ex = 1.4, kg1 = 1070.0, kp = 338.0, kvb = 300.0;
    double kgr = 3.0e-4;   // grid current scale  [A / V^1.5]
    double vgo = -0.35;    // grid current onset  [V]
    double vgs = 0.04;     // onset softness      [V]
    double emission = 1.0; // cathode emission factor (warm-up, ageing)

    static TriodeParams t5751() { return {}; }
    static TriodeParams t6922()
    {
        TriodeParams p;
        p.mu = 32.41; p.ex = 1.3; p.kg1 = 228.3; p.kp = 1489.9; p.kvb = 300.0;
        p.kgr = 1.2e-3; p.vgo = -0.4; p.vgs = 0.04;
        return p;
    }
};

struct TriodeEval
{
    double ip, ig;        // currents
    double dIpdVg, dIpdVp, dIgdVg;
};

inline double softplus (double x) noexcept
{
    if (x > 30.0) return x;
    if (x < -30.0) return std::exp (x);
    return std::log1p (std::exp (x));
}
inline double sigmoid (double x) noexcept
{
    if (x > 30.0) return 1.0;
    if (x < -30.0) return std::exp (x);
    return 1.0 / (1.0 + std::exp (-x));
}

inline TriodeEval triode (const TriodeParams& t, double vgk, double vpk) noexcept
{
    TriodeEval r { 0.0, 0.0, 0.0, 0.0, 0.0 };
    if (vpk > 1.0e-6)
    {
        const double s = std::sqrt (t.kvb + vpk * vpk);
        const double a = t.kp * (1.0 / t.mu + vgk / s);
        double sp, sg;
        if (a > 30.0)       { sp = a; sg = 1.0; }
        else if (a < -30.0) { sp = std::exp (a); sg = sp; }
        else
        {
            const double ea = std::exp (a);
            sp = std::log1p (ea);
            sg = ea / (1.0 + ea);
        }
        const double e1 = vpk * sp / t.kp;
        if (e1 > 1.0e-30)
        {
            const double k = 2.0 * t.emission / t.kg1;
            const double pw = std::exp (t.ex * std::log (e1));       // e1^ex
            r.ip = k * pw;
            const double dIdE = t.ex * r.ip / e1;
            const double dEdVg = vpk * sg / s;
            const double dEdVp = sp / t.kp - vpk * vpk * sg * vgk / (s * s * s);
            r.dIpdVg = dIdE * dEdVg;
            r.dIpdVp = dIdE * dEdVp;
        }
    }
    // grid current: kgr · softplus_s(vgk − vgo)^1.5 (below xg = −20 it is
    // under 1e-18 A: exactly zero is the same thing and saves three functions)
    const double xg = (vgk - t.vgo) / t.vgs;
    if (xg > -20.0)
    {
        double spg, sgg;
        if (xg > 30.0) { spg = xg; sgg = 1.0; }
        else { const double e = std::exp (xg); spg = std::log1p (e); sgg = e / (1.0 + e); }
        const double sp = t.vgs * spg;
        const double sq = std::sqrt (sp);
        r.ig = t.kgr * sp * sq;
        r.dIgdVg = 1.5 * t.kgr * sq * sgg;
    }
    return r;
}

// Solves the two nonlinear port currents of a triode embedded in a compiled
// circuit: i = f(p + F·i) with i = [Ip, Ig], controls v = [Vgk, Vpk].
// Damped Newton, warm-started from the previous sample.
struct TriodeSolver
{
    double ip = 1.0e-3, ig = 0.0;
    double ipPrev = 1.0e-3, igPrev = 0.0;   // previous samples (for the predictor)
    double ipPrev2 = 1.0e-3, igPrev2 = 0.0;
    double pc[3][2] {};                     // controls of the last three samples
    double jg = 0.0, jp = 0.0, jgg = 0.0;   // tube derivatives at the last solution
    bool haveJ = false;
    int lastIterations = 0;

    int robustCalls = 0;
    // Acceptance: Newton converges quadratically, so a correction of
    // tol·|i| leaves an error of order tol²·|i| (≈ −170 dB re the signal).
    double tol = 1.0e-5;

    // Predictor: the trajectory of the last three samples extrapolated
    // quadratically, plus a linear correction for where the controls (input
    // and circuit states) actually went versus their own extrapolation,
    // through the tube's last Jacobian. The first Newton step from there
    // usually lands within tol, so most samples cost one model evaluation.
    // If Newton does not converge (large jumps, start-up, overload) a
    // bracketed solve takes over; with negative feedback the residual is
    // monotonic in each current, so the bracket always holds the solution.
    template <class Circ>
    inline void solve (const TriodeParams& t, const Circ& c, const double* p) noexcept
    {
        const double F00 = c.feedback (0, 0), F01 = c.feedback (0, 1);
        const double F10 = c.feedback (1, 0), F11 = c.feedback (1, 1);
        double i0 = 3.0 * (ip - ipPrev) + ipPrev2;
        double i1 = 3.0 * (ig - igPrev) + igPrev2;
        if (haveJ)
        {
            const double dp0 = p[0] - (3.0 * (pc[0][0] - pc[1][0]) + pc[2][0]);
            const double dp1 = p[1] - (3.0 * (pc[0][1] - pc[1][1]) + pc[2][1]);
            // (I − Jf·F)·δ = Jf·Δp
            const double j00 = 1.0 - (jg * F00 + jp * F10), j01 = -(jg * F01 + jp * F11);
            const double j10 = -(jgg * F00), j11 = 1.0 - (jgg * F01);
            const double det = j00 * j11 - j01 * j10;
            if (std::fabs (det) > 1.0e-30)
            {
                const double b0 = jg * dp0 + jp * dp1, b1 = jgg * dp0;
                i0 += ( j11 * b0 - j01 * b1) / det;
                i1 += (-j10 * b0 + j00 * b1) / det;
            }
        }
        i0 = std::max (i0, 0.0);
        i1 = std::max (i1, 0.0);
        pc[2][0] = pc[1][0]; pc[2][1] = pc[1][1];
        pc[1][0] = pc[0][0]; pc[1][1] = pc[0][1];
        pc[0][0] = p[0];     pc[0][1] = p[1];
        int it = 0;
        bool ok = false;
        for (; it < 10; ++it)
        {
            const double vg = p[0] + F00 * i0 + F01 * i1;
            const double vp = p[1] + F10 * i0 + F11 * i1;
            const TriodeEval e = triode (t, vg, vp);
            const double r0 = i0 - e.ip, r1 = i1 - e.ig;
            // J = I − Jf·F,  Jf = [[dIpdVg, dIpdVp], [dIgdVg, 0]]
            const double j00 = 1.0 - (e.dIpdVg * F00 + e.dIpdVp * F10);
            const double j01 = -(e.dIpdVg * F01 + e.dIpdVp * F11);
            const double j10 = -(e.dIgdVg * F00);
            const double j11 = 1.0 - (e.dIgdVg * F01);
            const double det = j00 * j11 - j01 * j10;
            if (! (std::fabs (det) > 1.0e-30)) break;
            const double d0 = ( j11 * r0 - j01 * r1) / det;
            const double d1 = (-j10 * r0 + j00 * r1) / det;
            const double ni0 = i0 - d0, ni1 = i1 - d1;
            if (ni0 < -1.0e-9 || ni1 < -1.0e-9) break;   // left the physical region
            i0 = ni0; i1 = ni1;
            jg = e.dIpdVg; jp = e.dIpdVp; jgg = e.dIgdVg;
            if (std::fabs (d0) <= 1.0e-16 + tol * std::fabs (i0)
                && std::fabs (d1) <= 1.0e-16 + tol * std::max (std::fabs (i1), 1.0e-7)) { ok = true; break; }
        }
        lastIterations = it + 1;
        ipPrev2 = ipPrev; igPrev2 = igPrev;
        ipPrev = ip; igPrev = ig;
        if (ok && std::isfinite (i0) && std::isfinite (i1)) { ip = std::max (i0, 0.0); ig = std::max (i1, 0.0); haveJ = true; return; }
        solveRobust (t, p, F00, F01, F10, F11);
        restart (p);
    }

    // Forget the history (after a hard step, a settle or a recompile): the
    // predictor starts again from the present solution.
    void restart (const double* p) noexcept
    {
        ipPrev2 = ipPrev = ip; igPrev2 = igPrev = ig;
        for (auto& r : pc) { r[0] = p[0]; r[1] = p[1]; }
        haveJ = false;
    }

    void solveRobust (const TriodeParams& t, const double* p, double F00, double F01, double F10, double F11) noexcept
    {
        ++robustCalls;
        // grid current for a given plate current: ig = g(p0 + F00·ip + F01·ig), bracket [0, g(p0 + F00·ip)]
        auto gridFor = [&] (double i0) -> double
        {
            const double v0 = p[0] + F00 * i0;
            double hi = triode (t, v0, 1.0).ig;
            if (hi <= 1.0e-18) return 0.0;
            double lo = 0.0;
            for (int k = 0; k < 80; ++k)
            {
                const double m = 0.5 * (lo + hi);
                const double r = m - triode (t, v0 + F01 * m, 1.0).ig;
                if (r < 0.0) lo = m; else hi = m;
                if (hi - lo < 1.0e-16 + 1.0e-12 * hi) break;
            }
            return 0.5 * (lo + hi);
        };
        auto resid = [&] (double i0, double& i1) -> double
        {
            i1 = gridFor (i0);
            const double vg = p[0] + F00 * i0 + F01 * i1;
            const double vp = p[1] + F10 * i0 + F11 * i1;
            return i0 - triode (t, vg, vp).ip;
        };
        double gLo = 0.0, gHi = 0.0;
        double a = 0.0, fa = resid (a, gLo);
        double b = std::max (-fa, 1.0e-9), fb = resid (b, gHi);
        int guard = 0;
        while (fb < 0.0 && guard++ < 60) { a = b; fa = fb; gLo = gHi; b *= 2.0; fb = resid (b, gHi); }
        // Illinois false position
        int side = 0;
        double x = b, gx = gHi;
        for (int k = 0; k < 100; ++k)
        {
            x = (a * fb - b * fa) / (fb - fa);
            if (! (x > a && x < b)) x = 0.5 * (a + b);
            double gxx = 0.0;
            const double fx = resid (x, gxx);
            gx = gxx;
            if (std::fabs (fx) < 1.0e-15 || (b - a) < 1.0e-15 + 1.0e-12 * b) break;
            if ((fx < 0.0) == (fa < 0.0)) { a = x; fa = fx; if (side == -1) fb *= 0.5; side = -1; }
            else                          { b = x; fb = fx; if (side == +1) fa *= 0.5; side = +1; }
        }
        ip = x; ig = gx;
    }
};

} // namespace vt4
