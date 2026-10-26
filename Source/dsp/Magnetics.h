// Halden VT-4 — audio transformer model.
//
//   source u ── Rs (source + primary DCR) ──┬── primary v_p ──► × ratio → HF
//                                           │                    (leakage, C)
//                                  magnetising branch  i_m(φ)
//                                  load + eddy loss    v_p / Rpar
//
// Core flux φ = ∫ v_p dt. The magnetising current follows the core's
// constitutive law in normalised flux x = φ / φ_sat:
//
//   h(x) = (1 − W)·x + a·x³ + b·(sinh(gx) − gx − (gx)³/6)   anhysteretic
//        + Σ w_k · S_k[x]                                   hysteresis
//
// S_k are stop operators (a Preisach-type model) with log-spaced thresholds
// r_k and weights w_k ∝ r_k, i.e. a uniform switching density: below r_max
// this reproduces the Rayleigh law exactly — permeability rising with the
// amplitude, parabolic loop branches, a loop area ∝ amplitude³ and a third
// harmonic whose share falls only in proportion to level. Minor loops close
// (return-point memory), so the core has memory but no drift.
//
// The small-signal slope of h is 1 at every amplitude → IRON OFF (h = x)
// has exactly the same frequency response; only the nonlinearity is gone.
//
// Each sample is solved implicitly (Newton on the flux, with the exact
// piecewise derivative of the stop operators).

#pragma once

#include "Circuit.h"

#include <algorithm>
#include <array>
#include <cmath>

#if defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
 #include <emmintrin.h>
 #define VT4_STOPS_SSE2 1
#elif defined(__aarch64__) || defined(_M_ARM64)
 #include <arm_neon.h>
 #define VT4_STOPS_NEON 1
#endif

namespace vt4
{

struct CoreMaterial
{
    double hyst  = 0.02;    // W: total weight of the hysteresis (Rayleigh) part
    double rmax  = 0.25;    // upper end of the Rayleigh region (normalised flux)
    double a     = 0.01;    // cubic curvature of the anhysteretic curve
    double b     = 0.03;    // saturation knee weight
    double g     = 3.5;     // knee sharpness
};

struct TransformerParams
{
    double ratio  = 1.0;     // secondary / primary voltage ratio (unloaded)
    double lm     = 30.0;    // small-signal magnetising inductance [H] (primary)
    double vsat   = 20.0;    // peak primary voltage at fsat that reaches x = 1
    double fsat   = 20.0;
    double rs     = 50.0;    // source resistance incl. primary DCR [Ω]
    double rpar   = 10.0e3;  // reflected load ∥ eddy-current loss [Ω]
    double kout   = 1.0;     // secondary DCR / load divider
    double fhf    = 80.0e3;  // leakage-inductance / winding-capacitance resonance
    double qhf    = 0.7;
    CoreMaterial core;
    bool linear   = false;   // IRON OFF
};

class Transformer
{
public:
    static constexpr int kStops = 24;      // even (the loop runs in pairs)

    void prepare (double sampleRate, const TransformerParams& p)
    {
        fs = sampleRate;
        setParams (p);
        reset();
    }

    void setParams (const TransformerParams& p)
    {
        prm = p;
        const double T = 1.0 / fs;
        phiSat = p.vsat / (2.0 * kPi * p.fsat);
        alpha = p.rpar / (p.rs + p.rpar);
        rho = p.rs * p.rpar / (p.rs + p.rpar);
        i0 = phiSat / p.lm;                      // current scale: slope 1 ↔ Lm
        cst = 0.5 * T / phiSat;
        k = cst * rho * i0;
        // Rayleigh stops: log-spaced thresholds, weights ∝ r (uniform density)
        const double rmin = p.core.rmax * 1.0e-5;
        double wsum = 0.0;
        for (int i = 0; i < kStops; ++i)
        {
            const double t = (double) i / (double) (kStops - 1);
            r[(size_t) i] = rmin * std::pow (p.core.rmax / rmin, t);
            w[(size_t) i] = r[(size_t) i];
            wsum += w[(size_t) i];
        }
        for (auto& v : w) v *= p.core.hyst / wsum;
        designHF();
    }

    void reset()
    {
        x = 0.0; xs = 0.0; vPrev = 0.0; hCur = 0.0; dSlope = 1.0;
        for (auto& b : stopBuf) b.fill (0.0);
        cur = 0;
        z1 = z2 = 0.0;
    }

    // A linear core can carry any flux; a real one cannot. Called when the
    // core becomes nonlinear (IRON on) so the state starts inside the B-H range.
    void sanitiseCore() noexcept
    {
        auto& st = stopBuf[(size_t) cur];
        for (int i = 0; i < kStops; ++i)
            st[(size_t) i] = std::clamp (st[(size_t) i] + (x - xs), -r[(size_t) i], r[(size_t) i]);
        x = std::clamp (x, -1.2, 1.2);
        xs = x;
        hCur = hNow();
        dSlope = 1.0;
    }

    inline double process (double u) noexcept
    {
        const double rhs = x + cst * (alpha * u + vPrev);   // x_new = rhs − k·h(x_new)
        double xn, h;
        if (prm.linear)
        {
            xn = rhs / (1.0 + k);
            h = xn;
        }
        else
        {
            // Predictor on the last incremental slope of h, then one pass over
            // the stop operators at the predicted flux. k is tiny (the
            // magnetising current barely loads the source within a sample),
            // so the Newton step that follows is of order k·Δslope·Δx
            // (≈ 1e-10): it is applied to the flux and to h linearly, and
            // the stops catch up on the next sample (they remember the flux
            // they were moved to, xs). The error left is of order k·h''·step².
            // A large step (deep saturation, overload) runs full Newton on the
            // untouched stop state instead (double-buffered).
            xn = (rhs + k * (dSlope * x - hCur)) / (1.0 + k * dSlope);
            double d = 1.0;
            h = commit (xn, d);
            const double step = (xn + k * h - rhs) / (1.0 + k * d);
            if (std::fabs (step) < 1.0e-9)
            {
                cur ^= 1;
                xs = xn;
                xn -= step;
                h -= d * step;
            }
            else
            {
                double st = step;
                for (int it = 0; it < 8; ++it)
                {
                    xn -= st;
                    h = hEval (xn, d);
                    st = (xn + k * h - rhs) / (1.0 + k * d);
                    if (std::fabs (st) < 1.0e-13) break;
                }
                xn -= st;
                h = commit (xn, d);
                cur ^= 1;
                xs = xn;
            }
            hCur = h;
            dSlope = d;
        }
        const double vp = alpha * u - rho * i0 * h;
        x = xn; vPrev = vp;

        const double y = vp * prm.ratio * prm.kout;
        const double out = b0 * y + z1;
        z1 = b1 * y - a1 * out + z2;
        z2 = b2 * y - a2 * out;
        return out;
    }

    // small-signal response (identical for IRON ON and OFF)
    std::complex<double> response (double f) const
    {
        using cd = std::complex<double>;
        const cd s (0.0, 2.0 * kPi * f);
        // trapezoidal integrator: the flux path in the discrete domain
        const cd z = std::polar (1.0, 2.0 * kPi * f / fs);
        const cd integ = 0.5 / fs * (z + 1.0) / (z - 1.0);          // T/2 (1+z⁻¹)/(1−z⁻¹)
        // v_p = α u − ρ i0 x,  x = integ · v_p / φsat  → v_p = α u / (1 + ρ i0 integ / φsat)
        const cd vp = alpha / (1.0 + rho * i0 * integ / phiSat);
        const cd zi = 1.0 / z;
        const cd hf = (b0 + b1 * zi + b2 * zi * zi) / (1.0 + a1 * zi + a2 * zi * zi);
        (void) s;
        return vp * prm.ratio * prm.kout * hf;
    }

    double flux() const noexcept { return x; }
    const TransformerParams& params() const noexcept { return prm; }

private:
    inline double hNow() const noexcept
    {
        const auto& st = stopBuf[(size_t) cur];
        const double dx = x - xs;
        double sum = 0.0;
        for (int i = 0; i < kStops; ++i) sum += w[(size_t) i] * std::min (std::max (st[(size_t) i] + dx, -r[(size_t) i]), r[(size_t) i]);
        return anh (x) + sum;
    }

    // Moves the stops from the current buffer to the flux xn, writing the
    // other buffer; returns h(xn) and its slope. Two stops per instruction
    // (SSE2 / NEON), a scalar pair loop elsewhere.
    inline double commit (double xn, double& d) noexcept
    {
        const double* src = stopBuf[(size_t) cur].data();
        double* dst = stopBuf[(size_t) (cur ^ 1)].data();
        const double* rr = r.data();
        const double* ww = w.data();
        const double dx = xn - xs;
        double hs, ds;
       #if VT4_STOPS_SSE2
        const __m128d vdx = _mm_set1_pd (dx), zero = _mm_setzero_pd();
        __m128d hv = zero, dv = zero;
        for (int i = 0; i < kStops; i += 2)
        {
            const __m128d ri = _mm_loadu_pd (rr + i), wi = _mm_loadu_pd (ww + i);
            const __m128d sv = _mm_add_pd (_mm_loadu_pd (src + i), vdx);
            const __m128d cl = _mm_min_pd (_mm_max_pd (sv, _mm_sub_pd (zero, ri)), ri);
            _mm_storeu_pd (dst + i, cl);
            hv = _mm_add_pd (hv, _mm_mul_pd (wi, cl));
            dv = _mm_add_pd (dv, _mm_and_pd (_mm_cmpeq_pd (cl, sv), wi));
        }
        hs = _mm_cvtsd_f64 (_mm_add_sd (hv, _mm_unpackhi_pd (hv, hv)));
        ds = _mm_cvtsd_f64 (_mm_add_sd (dv, _mm_unpackhi_pd (dv, dv)));
       #elif VT4_STOPS_NEON
        const float64x2_t vdx = vdupq_n_f64 (dx);
        float64x2_t hv = vdupq_n_f64 (0.0), dv = vdupq_n_f64 (0.0);
        for (int i = 0; i < kStops; i += 2)
        {
            const float64x2_t ri = vld1q_f64 (rr + i), wi = vld1q_f64 (ww + i);
            const float64x2_t sv = vaddq_f64 (vld1q_f64 (src + i), vdx);
            const float64x2_t cl = vminq_f64 (vmaxq_f64 (sv, vnegq_f64 (ri)), ri);
            vst1q_f64 (dst + i, cl);
            hv = vaddq_f64 (hv, vmulq_f64 (wi, cl));
            dv = vaddq_f64 (dv, vreinterpretq_f64_u64 (vandq_u64 (vceqq_f64 (cl, sv), vreinterpretq_u64_f64 (wi))));
        }
        hs = vaddvq_f64 (hv);
        ds = vaddvq_f64 (dv);
       #else
        double hs0 = 0.0, hs1 = 0.0, ds0 = 0.0, ds1 = 0.0;
        for (int i = 0; i < kStops; i += 2)
        {
            const double s0 = src[i] + dx, s1 = src[i + 1] + dx;
            const double c0 = std::min (std::max (s0, -rr[i]), rr[i]), c1 = std::min (std::max (s1, -rr[i + 1]), rr[i + 1]);
            dst[i] = c0; dst[i + 1] = c1;
            hs0 += ww[i] * c0; hs1 += ww[i + 1] * c1;
            ds0 += c0 == s0 ? ww[i] : 0.0;
            ds1 += c1 == s1 ? ww[i + 1] : 0.0;
        }
        hs = hs0 + hs1; ds = ds0 + ds1;
       #endif
        const auto& c = prm.core;
        const double gx = std::min (std::max (c.g * xn, -30.0), 30.0);
        d = (1.0 - c.hyst) + 3.0 * c.a * xn * xn + c.b * c.g * kneeD (gx) + ds;
        return (1.0 - c.hyst) * xn + c.a * xn * xn * xn + c.b * knee (gx) + hs;
    }

    inline double anh (double v) const noexcept
    {
        const auto& c = prm.core;
        const double gx = std::min (std::max (c.g * v, -30.0), 30.0);
        return (1.0 - c.hyst) * v + c.a * v * v * v + c.b * knee (gx);
    }

    static inline double knee (double gx) noexcept
    {
        const double g2 = gx * gx;
        if (g2 < 1.0)
        {
            const double g4 = g2 * g2;
            return gx * g4 * (1.0 / 120.0 + g2 * (1.0 / 5040.0 + g2 * (1.0 / 362880.0)));
        }
        return std::sinh (gx) - gx - gx * g2 / 6.0;
    }
    static inline double kneeD (double gx) noexcept   // d/d(gx)
    {
        const double g2 = gx * gx;
        if (g2 < 1.0)
        {
            const double g4 = g2 * g2;
            return g4 * (1.0 / 24.0 + g2 * (1.0 / 720.0 + g2 * (1.0 / 40320.0)));
        }
        return std::cosh (gx) - 1.0 - 0.5 * g2;
    }

    // h(x_new) and dh/dx with the stops moved to x_new (current buffer untouched)
    inline double hEval (double xn, double& d) const noexcept
    {
        const auto& c = prm.core;
        const auto& st = stopBuf[(size_t) cur];
        const double gx = std::min (std::max (c.g * xn, -30.0), 30.0);
        double h = (1.0 - c.hyst) * xn + c.a * xn * xn * xn + c.b * knee (gx);
        double dd = (1.0 - c.hyst) + 3.0 * c.a * xn * xn + c.b * c.g * kneeD (gx);
        const double dx = xn - xs;
        double hs = 0.0, ds = 0.0;
        for (int i = 0; i < kStops; ++i)
        {
            const double ri = r[(size_t) i];
            const double s = st[(size_t) i] + dx;
            const double cl = std::min (std::max (s, -ri), ri);
            hs += w[(size_t) i] * cl;
            ds += cl == s ? w[(size_t) i] : 0.0;
        }
        d = dd + ds;
        return h + hs;
    }

    void designHF()
    {
        // 2nd-order low-pass, bilinear prewarped at 20 kHz (top of the audio
        // band) so the audible part of the roll-off is exact
        const double w0 = 2.0 * kPi * prm.fhf;
        const double wp = 2.0 * kPi * 20000.0;
        const double K = wp / std::tan (wp / (2.0 * fs));
        const double a0 = K * K + K * w0 / prm.qhf + w0 * w0;
        b0 = w0 * w0 / a0; b1 = 2.0 * b0; b2 = b0;
        a1 = (2.0 * w0 * w0 - 2.0 * K * K) / a0;
        a2 = (K * K - K * w0 / prm.qhf + w0 * w0) / a0;
    }

    TransformerParams prm;
    double fs = 176400.0, phiSat = 1.0, alpha = 1.0, rho = 0.0, i0 = 1.0, cst = 0.0, k = 0.0;
    double x = 0.0, xs = 0.0, vPrev = 0.0, hCur = 0.0, dSlope = 1.0;
    std::array<std::array<double, kStops>, 2> stopBuf {};   // stop states (double-buffered)
    int cur = 0;
    std::array<double, kStops> r {}, w {};
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
};

} // namespace vt4
