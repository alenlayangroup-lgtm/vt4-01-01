// Halden VT-4 — band audition: SOLO windows.
//
// SOLO plays the output through a listening window placed on the band:
//
//   E(f)    = 20·lg |H(f; θ) / H(f; θ₀)|      effect of band b, in context
//                                             (θ₀: same settings, band b flat)
//   f₁, f₂  : |E(f)| = ½·max|E| over 20 Hz … 20 kHz (outermost crossings)
//
// At 0 dB gain E vanishes, so the window uses the shape of a tiny boost,
// i.e. the derivative ∂E/∂g (the window does not jump when the knob crosses
// zero). The high-pass uses its −3 dB point (it plays what is cut away).
//
// The window is a Butterworth filter of 8th order (24 dB/oct per slope),
// built from the 4th-order low-pass prototype with s → (s² + ω₀²)/(B·s),
// ω₀ = √(Ω₁Ω₂), B = Ω₂ − Ω₁, both edges prewarped Ω = 2fs·tan(πf/fs), so the
// digital filter is exactly −3.01 dB at f₁ and f₂ and exactly 0 dB at the
// centre. One-sided windows are 4th-order low- or high-passes. Sections are
// topology-preserving state-variable filters (exact bilinear transform of
// each analog section, robust while the edges move).

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>

namespace vt4
{

struct SoloWindow
{
    enum Type : int { kBandPass = 0, kLowPass = 1, kHighPass = 2 };
    int type = kBandPass;
    double f1 = 100.0, f2 = 1000.0;      // edges (Hz); LP uses f2, HP uses f1
};

// One TPT state-variable section with a given analog cutoff (already in the
// prewarped domain) and damping k = 1/Q.
struct SvfSection
{
    double g = 0.1, k = 1.4142135623730951, a1 = 0.0, a2 = 0.0, a3 = 0.0;
    double ic1 = 0.0, ic2 = 0.0;

    void set (double omegaA, double damping, double fs) noexcept
    {
        g = omegaA / (2.0 * fs);
        k = damping;
        a1 = 1.0 / (1.0 + g * (g + k));
        a2 = g * a1;
        a3 = g * a2;
    }
    void reset() noexcept { ic1 = ic2 = 0.0; }

    // returns low, band, high
    inline void tick (double v0, double& low, double& band, double& high) noexcept
    {
        const double v3 = v0 - ic2;
        const double v1 = a1 * ic1 + a2 * v3;
        const double v2 = ic2 + a2 * ic1 + a3 * v3;
        ic1 = 2.0 * v1 - ic1;
        ic2 = 2.0 * v2 - ic2;
        low = v2; band = v1; high = v0 - k * v1 - v2;
    }

    // exact discrete response of the outputs (for tests and the display)
    std::complex<double> response (double f, double fs, int out) const
    {
        using cd = std::complex<double>;
        const cd z = std::polar (1.0, 2.0 * 3.14159265358979323846 * f / fs);
        // bilinear: s/ωa = (1/g)·(z − 1)/(z + 1)
        const cd sn = (1.0 / g) * (z - 1.0) / (z + 1.0);
        const cd den = sn * sn + k * sn + 1.0;
        if (out == 0) return 1.0 / den;
        if (out == 1) return sn / den;
        return sn * sn / den;
    }
};

class WindowFilter
{
public:
    static constexpr int kSections = 4;

    void prepare (double sampleRate)
    {
        fs = sampleRate;
        reset();
    }

    void reset() noexcept { for (auto& s : sec) s.reset(); }

    // Designs the filter for a window (no smoothing).
    void design (const SoloWindow& w) noexcept
    {
        win = w;
        const double fmax = 0.49 * fs;
        auto warp = [&] (double f) { return 2.0 * fs * std::tan (3.14159265358979323846 * std::clamp (f, 1.0, fmax) / fs); };
        if (w.type == SoloWindow::kBandPass)
        {
            const double O1 = warp (std::min (w.f1, w.f2)), O2 = warp (std::max (w.f1, w.f2));
            const double B = std::max (O2 - O1, 1.0e-6 * O2), w0 = std::sqrt (O1 * O2);
            int n = 0;
            for (int kk = 1; kk <= 2; ++kk)
            {
                // upper-half-plane poles of the 4th-order Butterworth prototype
                const double th = 3.14159265358979323846 * (0.5 + (2.0 * kk - 1.0) / 8.0);
                const std::complex<double> p (std::cos (th), std::sin (th));
                const std::complex<double> pb = p * B;
                const std::complex<double> disc = std::sqrt (pb * pb - 4.0 * w0 * w0);
                for (const std::complex<double> r : { 0.5 * (pb + disc), 0.5 * (pb - disc) })
                {
                    const double wc = std::abs (r);              // s² − 2Re(r)s + |r|²
                    const double damp = -2.0 * r.real() / wc;
                    sec[(size_t) n].set (wc, damp, fs);
                    gain[(size_t) n] = B / wc;                   // section = (B s)/(s² + a s + b)
                    ++n;
                }
            }
            nSec = 4;
        }
        else
        {
            const double O = warp (w.type == SoloWindow::kLowPass ? w.f2 : w.f1);
            sec[0].set (O, 2.0 * std::sin (3.14159265358979323846 / 8.0), fs);         // Q 1.3066
            sec[1].set (O, 2.0 * std::sin (3.0 * 3.14159265358979323846 / 8.0), fs);   // Q 0.5412
            gain[0] = gain[1] = 1.0;
            nSec = 2;
        }
    }

    inline double process (double x) noexcept
    {
        double y = x, lo, bp, hi;
        if (win.type == SoloWindow::kBandPass)
            for (int i = 0; i < nSec; ++i) { sec[(size_t) i].tick (y, lo, bp, hi); y = gain[(size_t) i] * bp; }
        else if (win.type == SoloWindow::kLowPass)
            for (int i = 0; i < nSec; ++i) { sec[(size_t) i].tick (y, lo, bp, hi); y = lo; }
        else
            for (int i = 0; i < nSec; ++i) { sec[(size_t) i].tick (y, lo, bp, hi); y = hi; }
        return y;
    }

    std::complex<double> response (double f) const
    {
        std::complex<double> h = 1.0;
        const int out = win.type == SoloWindow::kBandPass ? 1 : (win.type == SoloWindow::kLowPass ? 0 : 2);
        for (int i = 0; i < nSec; ++i) h *= gain[(size_t) i] * sec[(size_t) i].response (f, fs, out);
        return h;
    }

    const SoloWindow& window() const noexcept { return win; }

private:
    double fs = 48000.0;
    SoloWindow win;
    std::array<SvfSection, kSections> sec {};
    std::array<double, kSections> gain { 1.0, 1.0, 1.0, 1.0 };
    int nSec = 4;
};

// Finds the window edges from a band-effect curve E(f) (dB) sampled by a
// callable effect(f). Grid search on 1/48 octave, then bisection.
template <class Effect>
inline SoloWindow findSoloWindow (Effect&& effect, double fLo = 20.0, double fHi = 20000.0)
{
    constexpr int N = 481;                            // 10 octaves at 1/48 octave
    double F[N], E[N];
    const double r = std::log (fHi / fLo);
    int ipk = 0;
    for (int i = 0; i < N; ++i)
    {
        F[i] = fLo * std::exp (r * i / (N - 1));
        E[i] = std::fabs (effect (F[i]));
        if (E[i] > E[ipk]) ipk = i;
    }
    SoloWindow w;
    const double half = 0.5 * E[ipk];
    if (! (half > 1.0e-12)) { w.type = SoloWindow::kBandPass; w.f1 = 0.7 * F[ipk]; w.f2 = 1.4 * F[ipk]; return w; }
    auto refine = [&] (double a, double b)       // |E| − half changes sign between a and b
    {
        double ea = std::fabs (effect (a)) - half;
        for (int it = 0; it < 48; ++it)
        {
            const double m = std::sqrt (a * b);
            const double em = std::fabs (effect (m)) - half;
            if ((em >= 0.0) == (ea >= 0.0)) { a = m; ea = em; } else b = m;
            if (b / a < 1.0 + 1.0e-7) break;
        }
        return std::sqrt (a * b);
    };
    // outermost crossings on each side of the peak
    int lo = -1, hi = -1;
    for (int i = 0; i < ipk; ++i) if (E[i] < half && E[i + 1] >= half) { lo = i; break; }
    for (int i = N - 1; i > ipk; --i) if (E[i] < half && E[i - 1] >= half) { hi = i; break; }
    const bool hasLo = lo >= 0, hasHi = hi >= 0;
    if (hasLo && hasHi)
    {
        w.type = SoloWindow::kBandPass;
        w.f1 = refine (F[lo], F[lo + 1]);
        w.f2 = refine (F[hi - 1], F[hi]);
    }
    else if (hasHi)
    {
        w.type = SoloWindow::kLowPass;
        w.f2 = refine (F[hi - 1], F[hi]);
        w.f1 = fLo;
    }
    else if (hasLo)
    {
        w.type = SoloWindow::kHighPass;
        w.f1 = refine (F[lo], F[lo + 1]);
        w.f2 = fHi;
    }
    else
    {
        w.type = SoloWindow::kBandPass; w.f1 = fLo; w.f2 = fHi;
    }
    return w;
}

} // namespace vt4
