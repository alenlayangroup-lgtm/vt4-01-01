// Halden VT-4 — measurement bench.
//
// Every number quoted for the plugin is measured here, on the same DSP code
// the plugin runs (no JUCE needed). Each test prints its figures and a
// verdict; the exit code is the number of failed tests.
//
//   vt4_bench          full run (≈ 2–4 min)
//   vt4_bench quick    shorter measurement windows

#include "dsp/Audition.h"
#include "dsp/Device.h"
#include "dsp/DisplayModel.h"
#include "dsp/Engine.h"
#include "dsp/Spectrum.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace vt4;

namespace
{
constexpr double FS = 176400.0;          // internal rate at 44.1 kHz (4× oversampling)
bool quick = false;
int failures = 0, passes = 0;

double db (double x) { return 20.0 * std::log10 (std::max (x, 1.0e-300)); }

void verdict (bool ok, const char* what)
{
    std::printf ("  => %s  %s\n\n", ok ? "PASS" : "FAIL", what);
    (ok ? passes : failures)++;
}

void header (const char* t) { std::printf ("---- %s ", t); for (int i = (int) std::strlen (t); i < 70; ++i) std::putchar ('-'); std::printf ("\n"); }

// Deterministic pink-ish noise (Kellet filter on white noise).
struct Pink
{
    std::mt19937 gen;
    std::uniform_real_distribution<double> u { -1.0, 1.0 };
    double b[7] {};
    explicit Pink (unsigned seed) : gen (seed) {}
    double next()
    {
        const double w = u (gen);
        b[0] = 0.99886 * b[0] + w * 0.0555179; b[1] = 0.99332 * b[1] + w * 0.0750759;
        b[2] = 0.96900 * b[2] + w * 0.1538520; b[3] = 0.86650 * b[3] + w * 0.3104856;
        b[4] = 0.55000 * b[4] + w * 0.5329522; b[5] = -0.7616 * b[5] - w * 0.0168980;
        const double y = b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6] + w * 0.5362;
        b[6] = w * 0.115926;
        return y * 0.11;
    }
};

// Blackman–Harris windowed single-frequency DFT: amplitude of the component
// at frequency f (exact frequency, so no scalloping).
struct Bh
{
    std::vector<double> w; double sum = 0.0;
    explicit Bh (int n) : w ((size_t) n)
    {
        for (int i = 0; i < n; ++i)
        {
            const double t = 2.0 * kPi * i / (n - 1);
            w[(size_t) i] = 0.35875 - 0.48829 * std::cos (t) + 0.14128 * std::cos (2 * t) - 0.01168 * std::cos (3 * t);
            sum += w[(size_t) i];
        }
    }
    std::complex<double> at (const std::vector<double>& y, double f, double fs) const
    {
        double re = 0.0, im = 0.0;
        const double dw = 2.0 * kPi * f / fs;
        // recurrence for cos / sin (stable for these lengths in double)
        double c = 1.0, s = 0.0;
        const double cd = std::cos (dw), sd = std::sin (dw);
        for (size_t n = 0; n < y.size(); ++n)
        {
            const double v = w[n] * y[n];
            re += v * c; im -= v * s;
            const double c2 = c * cd - s * sd;
            s = s * cd + c * sd; c = c2;
        }
        return std::complex<double> (re, im) * (2.0 / sum);
    }
};

struct Tone { double gainDb = 0.0, h[10] {}, thd = 0.0; };

// Drives the device with sines (amplitudes aL, aR, peak, full scale = 1) and
// measures output channel ch. The first `settle` seconds are discarded.
Tone tone (Device& d, double f, double aL, double aR, int ch, double settle, double meas)
{
    const int ns = (int) (settle * FS), nm = (int) (meas * FS);
    std::vector<double> y ((size_t) nm);
    double ph = 0.0, l, r;
    const double dph = 2.0 * kPi * f / FS;
    for (int n = 0; n < ns + nm; ++n)
    {
        const double x = std::sin (ph);
        ph += dph; if (ph > 2.0 * kPi) ph -= 2.0 * kPi;
        d.process (aL * x, aR * x, l, r);
        if (n >= ns) y[(size_t) (n - ns)] = ch == 0 ? l : r;
    }
    Bh bh (nm);
    Tone t;
    for (int k = 1; k <= 9; ++k)
    {
        if (f * k > 0.45 * FS) break;
        t.h[k] = std::abs (bh.at (y, f * k, FS));
    }
    double s = 0.0;
    for (int k = 2; k <= 9; ++k) s += t.h[k] * t.h[k];
    const double a = ch == 0 ? aL : aR;
    t.thd = 100.0 * std::sqrt (s) / t.h[1];
    t.gainDb = db (t.h[1] / (a != 0.0 ? a : std::max (aL, aR)));
    return t;
}

double settleSilence (Device& d, double secs)
{
    double l, r;
    for (int n = 0; n < (int) (secs * FS); ++n) d.process (0.0, 0.0, l, r);
    return l;
}

// 20 Hz – 20 kHz band limit (4th-order Butterworth HP + LP) for noise figures
struct Biquad
{
    double b0, b1, b2, a1, a2, z1 = 0, z2 = 0;
    double run (double x) { const double y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; }
};
Biquad rbj (bool hp, double fc, double q)
{
    const double w = 2 * kPi * fc / FS, c = std::cos (w), al = std::sin (w) / (2 * q);
    Biquad b; const double a0 = 1 + al;
    if (hp) { b.b0 = (1 + c) / 2 / a0; b.b1 = -(1 + c) / a0; b.b2 = b.b0; }
    else    { b.b0 = (1 - c) / 2 / a0; b.b1 = (1 - c) / a0;  b.b2 = b.b0; }
    b.a1 = -2 * c / a0; b.a2 = (1 - al) / a0;
    return b;
}
// noise re a full-scale sine (rms 1/√2), 20 Hz – 20 kHz
double noiseDbfs (Device& d, double secs)
{
    Biquad f[4] = { rbj (true, 20, 0.5412), rbj (true, 20, 1.3066), rbj (false, 20000, 0.5412), rbj (false, 20000, 1.3066) };
    double l, r, acc = 0.0;
    for (int n = 0; n < (int) (0.5 * FS); ++n) { d.process (0, 0, l, r); double y = l; for (auto& b : f) y = b.run (y); }
    const int N = (int) (secs * FS);
    for (int n = 0; n < N; ++n) { d.process (0, 0, l, r); double y = l; for (auto& b : f) y = b.run (y); acc += y * y; }
    return 10.0 * std::log10 (acc / N * 2.0);
}

DeviceParams musical()
{
    DeviceParams p;
    p.noise = false;
    auto& c = p.ch[0];
    c.hpfHz = 25.0;
    c.band[kLow]  = { 4.0, 50.0, false, 5.0 };
    c.band[kBody] = { -2.0, 560.0, false, 0.0 };
    c.band[kPres] = { 3.0, 3900.0, false, 0.0 };
    c.band[kAir]  = { 2.5, 20000.0, false, 0.0 };
    p.ch[1] = c;
    return p;
}

std::vector<double> logGrid (double f0, double f1, int n)
{
    std::vector<double> v ((size_t) n);
    for (int i = 0; i < n; ++i) v[(size_t) i] = f0 * std::pow (f1 / f0, (double) i / (n - 1));
    return v;
}
} // namespace

// ============================================================================
static void testEqDigitalVsAnalog()
{
    header ("1. EQ network: digital (prewarped trapezoidal) vs analog, 20 Hz-20 kHz");
    const auto grid = logGrid (20.0, 20000.0, 240);
    for (double fsx : { 176400.0, 192000.0 })
    {
        Device d;
        DeviceParams p; p.noise = false; p.tolerance = 0.0;
        d.prepare (fsx, p);
        double worst = 0.0; std::string where;
        int cases = 0;
        for (int b = 0; b < kNumBands; ++b)
            for (int k = 0; k < eq::kNumSteps[b]; ++k)
                for (double g : { -1.0, -0.5, 0.5, 1.0 })
                    for (int variant = 0; variant < (b == kLow ? 3 : (b == kPres ? 2 : 1)); ++variant)
                    {
                        DeviceParams q = p;
                        auto& bp = q.ch[0].band[(size_t) b];
                        bp.freq = eq::kSteps[b][k];
                        bp.gainDb = g * eq::kMaxGain[b];
                        if (b == kLow) bp.weight = variant * 5.0;
                        if (b == kPres) bp.narrow = variant == 1;
                        q.ch[1] = q.ch[0];
                        d.setParams (q);
                        d.jumpToTargets();
                        const Circuit& c = d.eqCircuit (0);
                        for (double f : grid)
                        {
                            const double e = std::fabs (db (std::abs (c.digitalResponse (f))) - db (std::abs (c.analogResponse (f))));
                            if (e > worst)
                            {
                                worst = e;
                                char buf[160];
                                std::snprintf (buf, sizeof buf, "band %d at %g Hz, %+.1f dB%s, measured at %.0f Hz", b, bp.freq, bp.gainDb,
                                               b == kLow ? (variant ? (variant == 1 ? " W5" : " W10") : "") : (b == kPres && variant ? " narrow" : ""), f);
                                where = buf;
                            }
                        }
                        ++cases;
                    }
        std::printf ("  internal rate %.1f kHz: %d settings, worst |digital - analog| = %.4f dB (%s)\n", fsx / 1000.0, cases, worst, where.c_str());
        verdict (worst < 0.1, "digital EQ within 0.1 dB of the analog network");
    }
}

// ============================================================================
static void testBandCentres()
{
    header ("2. Band centres (BODY / PRESENCE bells, tolerance 0, analog network)");
    double worst = 0.0;
    for (int b : { kBody, kPres })
        for (int narrow = 0; narrow < (b == kPres ? 2 : 1); ++narrow)
        {
            std::printf ("  %s%s:", b == kBody ? "BODY" : "PRESENCE", narrow ? " narrow" : "");
            for (int k = 0; k < eq::kNumSteps[b]; ++k)
            {
                EqState s;
                auto& st = s.band[(size_t) b];
                st.set = 1; st.f = eq::kSteps[b][k]; st.narrow = narrow == 1;
                st.x = tapFromTable (b, 1, st.narrow, eq::kMaxGain[b]);
                Circuit c, flat; EqTolerance nominal;
                eq::build (c, s, nominal, false);
                eq::build (flat, EqState {}, nominal, false);
                // golden-section search of the peak of |H / H_flat| in log f
                double lo = std::log (st.f / 1.5), hi = std::log (st.f * 1.5);
                auto g = [&] (double lf) { const double f = std::exp (lf); return std::abs (c.analogResponse (f)) / std::abs (flat.analogResponse (f)); };
                for (int it = 0; it < 80; ++it)
                {
                    const double m1 = hi - 0.618 * (hi - lo), m2 = lo + 0.618 * (hi - lo);
                    if (g (m1) < g (m2)) lo = m1; else hi = m2;
                }
                const double fp = std::exp (0.5 * (lo + hi));
                const double dev = 100.0 * (fp / st.f - 1.0);
                worst = std::max (worst, std::fabs (dev));
                std::printf (" %g:%+.2f%%", st.f, dev);
            }
            std::printf ("\n");
        }
    std::printf ("  worst deviation of the peak from the engraved frequency: %.2f %%\n", worst);
    verdict (worst < 0.25, "bell peaks on the engraved frequency within 0.25 % (nominal parts)");

    // gain law: the dial reads the band's gain (bells at their centre, shelves on
    // their plateau: LOW at f/10, AIR at 4f), in the running digital network
    Device d;
    DeviceParams p; p.noise = false; p.tolerance = 0.0;
    d.prepare (FS, p);
    Circuit flatC; { EqState none; EqTolerance nominal; eq::build (flatC, none, nominal, true); CircuitWorkspace w; flatC.compile (FS, w); }
    double worstG = 0.0; std::string where;
    for (int b = 0; b < kNumBands; ++b)
        for (int k = 0; k < eq::kNumSteps[b]; ++k)
        {
            const double f = eq::kSteps[b][k];
            const double fref = f * eq::kRefRatio[b];
            if (fref > 20000.0) continue;     // AIR's plateau (4f ≥ 40 kHz) lies above the audio band
            for (double g : { -1.0, -0.5, -0.25, 0.25, 0.5, 1.0 })
            {
                DeviceParams q = p;
                q.ch[0].band[(size_t) b].freq = f;
                q.ch[0].band[(size_t) b].gainDb = g * eq::kMaxGain[b];
                q.ch[1] = q.ch[0];
                d.setParams (q);
                d.jumpToTargets();
                const double meas = db (std::abs (d.eqCircuit (0).digitalResponse (fref)) / std::abs (flatC.digitalResponse (fref)));
                const double e = std::fabs (meas - g * eq::kMaxGain[b]);
                if (e > worstG)
                {
                    worstG = e;
                    char buf[120]; std::snprintf (buf, sizeof buf, "band %d at %g Hz, dial %+.1f dB, measured %+.3f dB", b, f, g * eq::kMaxGain[b], meas);
                    where = buf;
                }
            }
        }
    std::printf ("  gain law, digital network: worst |measured - dial| = %.3f dB (%s)\n", worstG, where.c_str());
    std::printf ("  (AIR is a shelf whose plateau lies above 20 kHz: its dial is exact for the analog network,\n"
                 "   and the digital shelf follows that network within test 1's limit inside 20 Hz-20 kHz)\n");
    verdict (worstG < 0.05, "every band gives the dialled gain within 0.05 dB");
}

// ============================================================================
static void testSoloWindows()
{
    header ("3. SOLO listening windows: edges -3.0103 dB, centre 0 dB");
    double worstEdge = 0.0, worstCentre = 0.0, worstTime = 0.0;
    for (double fs : { 44100.0, 48000.0, 96000.0, 192000.0 })
    {
        std::vector<SoloWindow> ws;
        ws.push_back ({ SoloWindow::kBandPass, 120.0, 900.0 });
        ws.push_back ({ SoloWindow::kBandPass, 2900.0, 4600.0 });
        ws.push_back ({ SoloWindow::kBandPass, 7000.0, 19000.0 });
        ws.push_back ({ SoloWindow::kLowPass, 20.0, 95.0 });
        ws.push_back ({ SoloWindow::kHighPass, 9000.0, 20000.0 });
        for (const auto& w : ws)
        {
            WindowFilter wf; wf.prepare (fs); wf.design (w);
            auto mag = [&] (double f) { return db (std::abs (wf.response (f))); };
            if (w.type == SoloWindow::kBandPass)
            {
                const double fc = fs / kPi * std::atan (std::sqrt (std::tan (kPi * w.f1 / fs) * std::tan (kPi * w.f2 / fs)));
                worstEdge = std::max ({ worstEdge, std::fabs (mag (w.f1) + 3.0103), std::fabs (mag (w.f2) + 3.0103) });
                worstCentre = std::max (worstCentre, std::fabs (mag (fc)));
            }
            else
                worstEdge = std::max (worstEdge, std::fabs (mag (w.type == SoloWindow::kLowPass ? w.f2 : w.f1) + 3.0103));
            // the running filter (TPT state-variable sections) against its formula
            for (double f : { w.f1, w.f2, std::sqrt (w.f1 * w.f2) })
            {
                if (f >= 0.45 * fs) continue;
                WindowFilter run; run.prepare (fs); run.design (w);
                const int n0 = (int) (0.5 * fs), n1 = (int) (0.5 * fs);
                std::vector<double> y ((size_t) n1);
                for (int n = 0; n < n0 + n1; ++n)
                {
                    const double v = run.process (std::sin (2.0 * kPi * f * n / fs));
                    if (n >= n0) y[(size_t) (n - n0)] = v;
                }
                Bh bh (n1);
                const double meas = db (std::abs (bh.at (y, f, fs)));
                worstTime = std::max (worstTime, std::fabs (meas - mag (f)));
            }
        }
    }
    std::printf ("  edges: worst |H(f_edge) + 3.0103 dB| = %.2e dB\n", worstEdge);
    std::printf ("  centre: worst |H(f0)| = %.2e dB\n", worstCentre);
    std::printf ("  running filter vs its exact response: worst %.2e dB (sines at the edges and centre)\n", worstTime);
    verdict (worstEdge < 1.0e-6 && worstCentre < 1.0e-6 && worstTime < 1.0e-3, "SOLO windows are exact (44.1 / 48 / 96 / 192 kHz)");

    // The window finder on real band effects: |E| at both edges = ½·max|E|
    DisplayModel dm;
    DeviceParams p = musical();
    p.ch[0].band[kPres] = { 6.0, 3900.0, true, 0.0 };
    dm.prepare (FS, 44100.0, p);
    double worstHalf = 0.0;
    for (int band = 1; band <= 4; ++band)
    {
        const auto w = dm.soloWindow (0, band);
        // effect on the model grid, interpolated in log f
        auto E = [&] (double f)
        {
            int i = 0;
            while (i < DisplayModel::kN - 2 && dm.freq (i + 1) < f) ++i;
            const double t = std::log (f / dm.freq (i)) / std::log (dm.freq (i + 1) / dm.freq (i));
            return std::fabs ((1.0 - t) * dm.bandDb (0, band, i) + t * dm.bandDb (0, band, i + 1));
        };
        double mx = 0.0;
        for (int i = 0; i < DisplayModel::kN; ++i) if (dm.freq (i) >= 20.0 && dm.freq (i) <= 20000.0) mx = std::max (mx, std::fabs (dm.bandDb (0, band, i)));
        std::printf ("  band %d: window %s %.1f-%.1f Hz, max|E| %.2f dB", band, w.type == 0 ? "BP" : (w.type == 1 ? "LP" : "HP"), w.f1, w.f2, mx);
        for (double fe : { w.f1, w.f2 })
        {
            if ((w.type == SoloWindow::kLowPass && fe == w.f1) || (w.type == SoloWindow::kHighPass && fe == w.f2)) continue;
            const double r = E (fe) / mx;
            worstHalf = std::max (worstHalf, std::fabs (r - 0.5));
            std::printf ("  |E(%.0f)|/max = %.3f", fe, r);
        }
        std::printf ("\n");
    }
    verdict (worstHalf < 0.01, "window edges sit where the band's effect is half its maximum");
}

// ============================================================================
static void testDeltaNull()
{
    header ("4. DELTA null: y(theta0) + yDelta - y(theta), engine at 176.4 kHz");
    DeviceParams th = musical();
    th.drift = false; th.warmup = false;
    const int set = 0, band = 3;                       // PRESENCE +3 dB
    auto e1 = std::make_unique<DeltaEngine>();
    auto e2 = std::make_unique<DeltaEngine>();
    e1->prepare (FS, th); e2->prepare (FS, th);
    Pink pink (11), pink2 (11);
    const int nPre = (int) (0.4 * FS), nPost = (int) ((quick ? 0.4 : 0.8) * FS), nSkip = (int) (0.1 * FS);
    double a, b, c, d2;
    for (int n = 0; n < nPre; ++n)
    {
        const double x = 0.5 * pink.next(), x2 = 0.5 * pink2.next();
        e1->process (x, 0.8 * x, a, b);
        e2->process (x2, 0.8 * x2, c, d2);
    }
    // reference for theta0: the same unit with the same history, band brought to flat
    Device d3 = e2->device();
    d3.setParams (flattenBand (e2->device().params(), set, band));
    e1->setDelta (true, set, band);
    // and a fresh unit that ran theta0 from power-up (physical comparison)
    Device fresh; fresh.prepare (FS, flattenBand (th, set, band));
    Pink pink3 (11);
    for (int n = 0; n < nPre; ++n) { const double x = 0.5 * pink3.next(); fresh.process (x, 0.8 * x, a, b); }

    double sr = 0.0, sy = 0.0, sd = 0.0, sf = 0.0;
    for (int n = 0; n < nPost; ++n)
    {
        const double x = 0.5 * pink.next();
        double dl, dr, tl, tr, zl, zr, fl, fr;
        e1->process (x, 0.8 * x, dl, dr);     // DELTA output
        e2->process (x, 0.8 * x, tl, tr);     // y(theta)
        d3.process (x, 0.8 * x, zl, zr);      // y(theta0)
        fresh.process (x, 0.8 * x, fl, fr);
        if (n < nSkip) continue;              // 30 ms fade + band glide
        const double r = zl + dl - tl;
        sr += r * r; sy += tl * tl; sd += dl * dl;
        sf += (fl - zl) * (fl - zl);
    }
    const double resid = sr > 0.0 ? 10.0 * std::log10 (sr / sy) : -400.0;
    std::printf ("  DELTA level (PRESENCE +3 dB on pink noise): %.1f dB re the output\n", 10.0 * std::log10 (sd / sy));
    if (sr == 0.0) std::printf ("  residual: exactly zero (bit-identical)\n");
    else std::printf ("  residual: %.1f dB re the output\n", resid);
    std::printf ("  (info) clone-and-glide vs a unit that ran flat from power-up: %.1f dB re the output\n", 10.0 * std::log10 (sf / sy + 1e-300));
    verdict (resid < -140.0, "y(theta0) + yDelta = y(theta) below -140 dB");

    // a flat band gives an exactly silent DELTA
    DeviceParams fl = th; fl.ch[0].band[kBody].gainDb = 0.0; fl.ch[1] = fl.ch[0];
    DeltaEngine e3; e3.prepare (FS, fl);
    Pink p4 (5);
    for (int n = 0; n < (int) (0.2 * FS); ++n) { const double x = 0.5 * p4.next(); e3.process (x, x, a, b); }
    e3.setDelta (true, 0, 2);
    double mx = 0.0;
    for (int n = 0; n < (int) (0.3 * FS); ++n)
    {
        const double x = 0.5 * p4.next();
        e3.process (x, x, a, b);
        if (n > (int) (0.031 * FS)) mx = std::max ({ mx, std::fabs (a), std::fabs (b) });   // after the 30 ms fade-in
    }
    std::printf ("  DELTA of a flat band: peak %.3g\n", mx);
    verdict (mx == 0.0, "DELTA of a band at 0 dB is exactly silent");
}

// ============================================================================
static void testMidSide()
{
    header ("5. M/S matrix: exact factors 0.5, identical cores");
    DeviceParams p; p.noise = false; p.tolerance = 0.0; p.matched = true; p.stereo = kMidSide;
    Device d; d.prepare (FS, p);
    settleSilence (d, 0.2);
    const double a = std::pow (10.0, -18.0 / 20.0);
    // L only: R must stay silent when both cores are identical
    double l, r, mxR = 0.0, sL = 0.0;
    for (int n = 0; n < (int) (0.6 * FS); ++n)
    {
        d.process (a * std::sin (2.0 * kPi * 1000.0 * n / FS), 0.0, l, r);
        if (n > (int) (0.2 * FS)) { mxR = std::max (mxR, std::fabs (r)); sL += l * l; }
    }
    std::printf ("  L-only 1 kHz -18 dBFS: R output peak %.3g (%s)\n", mxR, mxR == 0.0 ? "exactly zero" : "non-zero");
    verdict (mxR == 0.0 || db (mxR / a) < -140.0, "M/S encode-decode leaves no L->R leakage with identical cores");

    // M/S flat has the response of LINK flat
    DeviceParams q = p; q.stereo = kLink;
    Device dl; dl.prepare (FS, q);
    double worst = 0.0;
    for (double f : { 30.0, 100.0, 1000.0, 10000.0 })
    {
        const auto t1 = tone (d, f, std::pow (10.0, -30.0 / 20.0), 0.0, 0, 0.15, 0.25);
        const auto t2 = tone (dl, f, std::pow (10.0, -30.0 / 20.0), 0.0, 0, 0.15, 0.25);
        worst = std::max (worst, std::fabs (t1.gainDb - t2.gainDb));
    }
    std::printf ("  M/S vs LINK, flat, -30 dBFS, 30 Hz-10 kHz: worst gain difference %.5f dB\n", worst);
    verdict (worst < 0.001, "M/S with flat settings sounds like LINK");

    // a real unit (tolerance 1): the L->R leakage is the channel mismatch of the hardware
    DeviceParams u = p; u.tolerance = 1.0;
    Device du; du.prepare (FS, u);
    settleSilence (du, 0.2);
    const auto tl = tone (du, 1000.0, a, 0.0, 0, 0.2, 0.3);
    const auto tr = tone (du, 1000.0, a, 0.0, 1, 0.0, 0.3);
    std::printf ("  (info) serial 1, tolerance 100 %%: L->R leakage at 1 kHz %.1f dB (core mismatch, as in hardware)\n\n", db (tr.h[1] / tl.h[1]));
}

// ============================================================================
static void testIron()
{
    header ("6. Transformers: IRON OFF / NICKEL / STEEL");
    DeviceParams p; p.noise = false; p.mode = kClean; p.tolerance = 0.0;
    const double a40 = std::pow (10.0, -40.0 / 20.0), a18 = std::pow (10.0, -18.0 / 20.0);
    const double fr[] = { 20.0, 30.0, 50.0, 100.0, 1000.0, 10000.0, 20000.0 };
    double g[3][7] {};
    for (int iron : { kIronOff, kNickel, kSteel })
    {
        DeviceParams q = p; q.iron = iron;
        Device d; d.prepare (FS, q);
        settleSilence (d, 0.2);
        for (int i = 0; i < 7; ++i) g[iron][i] = tone (d, fr[i], a40, a40, 0, fr[i] < 60 ? 0.5 : 0.15, fr[i] < 60 ? 0.6 : 0.2).gainDb;
    }
    std::printf ("  small signal (-40 dBFS), gain re 1 kHz of IRON OFF:\n         ");
    for (double f : fr) std::printf ("%9g", f);
    std::printf ("\n");
    const char* nm[3] = { "OFF   ", "NICKEL", "STEEL " };
    for (int iron = 0; iron < 3; ++iron)
    {
        std::printf ("  %s ", nm[iron]);
        for (int i = 0; i < 7; ++i) std::printf ("%+9.4f", g[iron][i] - g[0][4]);
        std::printf ("\n");
    }
    double worst = 0.0;
    for (int i = 0; i < 7; ++i) worst = std::max (worst, std::fabs (g[kNickel][i] - g[kIronOff][i]));
    double worst40 = 0.0;
    for (int i = 2; i < 7; ++i) worst40 = std::max (worst40, std::fabs (g[kNickel][i] - g[kIronOff][i]));
    std::printf ("  NICKEL vs OFF: worst %.4f dB (20 Hz-20 kHz), %.4f dB (50 Hz-20 kHz)\n", worst, worst40);
    verdict (worst < 0.05 && worst40 < 0.01, "IRON OFF keeps the frequency response (only the nonlinearity goes)");

    std::printf ("  harmonics, CLEAN, -18 dBFS (H2 / H3 re fundamental):\n");
    double h3n30 = 0.0, h3s30 = 0.0, h3off30 = 0.0, h3n100 = 0.0;
    for (int iron : { kIronOff, kNickel, kSteel })
    {
        DeviceParams q = p; q.iron = iron;
        Device d; d.prepare (FS, q);
        settleSilence (d, 0.2);
        std::printf ("  %s", nm[iron]);
        for (double f : { 30.0, 100.0, 1000.0 })
        {
            const auto t = tone (d, f, a18, a18, 0, f < 60 ? 0.8 : 0.3, f < 60 ? 1.0 : 0.4);
            std::printf ("   %4g Hz: H2 %6.1f H3 %6.1f dB", f, db (t.h[2] / t.h[1]), db (t.h[3] / t.h[1]));
            if (f == 30.0) (iron == kNickel ? h3n30 : (iron == kSteel ? h3s30 : h3off30)) = db (t.h[3] / t.h[1]);
            if (f == 100.0 && iron == kNickel) h3n100 = db (t.h[3] / t.h[1]);
        }
        std::printf ("\n");
    }
    std::printf ("  H3 at 30 Hz: NICKEL %.1f dB, STEEL %.1f dB, OFF %.1f dB; NICKEL at 100 Hz %.1f dB\n", h3n30, h3s30, h3off30, h3n100);
    verdict (h3n30 < -78.0 && h3n30 > -92.0 && h3s30 < -73.0 && h3s30 > -86.0 && h3s30 > h3n30 && h3off30 < h3n30 - 20.0 && h3n100 < -92.0,
             "iron H3 at 30 Hz about -85 (nickel) / -80 (steel) dB, ~-100 dB at 100 Hz, gone with IRON OFF");

    // switching IRON while a 30 Hz tone plays: no step, no click
    {
        DeviceParams q = p; q.iron = kNickel;
        Device d; d.prepare (FS, q);
        double l, r, worstJump = 0.0, prev = 0.0, prev2 = 0.0;
        int n = 0;
        auto run = [&] (int count)
        {
            for (int i = 0; i < count; ++i, ++n)
            {
                d.process (a18 * std::sin (2.0 * kPi * 30.0 * n / FS), a18 * std::sin (2.0 * kPi * 30.0 * n / FS), l, r);
                // second difference: a click shows up as a spike
                if (n > 2) worstJump = std::max (worstJump, std::fabs (l - 2.0 * prev + prev2));
                prev2 = prev; prev = l;
            }
        };
        run ((int) (0.5 * FS));
        const double base = worstJump; worstJump = 0.0;
        for (int iron : { kSteel, kIronOff, kNickel, kIronOff, kSteel })
        {
            q.iron = iron; d.setParams (q);
            run ((int) (0.12 * FS));
        }
        std::printf ("  IRON switched 5x under a 30 Hz tone: worst 2nd difference %.3g (steady tone %.3g)\n", worstJump, base);
        verdict (worstJump < 3.0 * base, "IRON switching is click-free");
    }
}

// ============================================================================
static void testModes()
{
    header ("7. Modes: gain, THD at 1 kHz, DRIVE with loudness compensation");
    DeviceParams p; p.noise = false;
    const char* nm[3] = { "CLEAN", "WARM ", "BOLD " };
    double worstGain = 0.0, worstComp = 0.0;
    for (int m : { kClean, kWarm, kBold })
    {
        DeviceParams q = p; q.mode = m;
        Device d; d.prepare (FS, q);
        settleSilence (d, 0.2);
        std::printf ("  %s", nm[m]);
        double g18 = 0.0;
        for (double lev : { -18.0, -6.0, 0.0 })
        {
            const auto t = tone (d, 1000.0, std::pow (10.0, lev / 20.0), std::pow (10.0, lev / 20.0), 0, 0.2, 0.3);
            if (lev == -18.0) { g18 = t.gainDb; worstGain = std::max (worstGain, std::fabs (t.gainDb)); }
            std::printf ("  %+3.0f dBFS: gain %+.3f THD %.4f%% (H2 %.0f, H3 %.0f)", lev, t.gainDb, t.thd, db (t.h[2] / t.h[1]), db (t.h[3] / t.h[1]));
        }
        std::printf ("\n");
        std::printf ("        DRIVE at -18 dBFS:");
        for (double dr : { -6.0, 0.0, 6.0, 12.0 })
        {
            q.driveDb = dr; d.setParams (q);
            settleSilence (d, 0.15);
            const auto t = tone (d, 1000.0, std::pow (10.0, -18.0 / 20.0), std::pow (10.0, -18.0 / 20.0), 0, 0.2, 0.3);
            worstComp = std::max (worstComp, std::fabs (t.gainDb - g18));
            std::printf ("  %+3.0f dB: %+.3f dB, THD %.3f%%", dr, t.gainDb - g18, t.thd);
        }
        std::printf ("\n");
        if (m == kBold)
        {
            q.driveDb = 12.0; d.setParams (q); settleSilence (d, 0.15);
            const auto t = tone (d, 1000.0, 1.0, 1.0, 0, 0.2, 0.3);
            std::printf ("        BOLD, DRIVE +12, 0 dBFS: THD %.2f%%, gain %+.2f dB\n", t.thd, t.gainDb);
        }
    }
    std::printf ("  unity gain: worst |gain| at 1 kHz -18 dBFS %.3f dB; DRIVE compensation: worst level change %.3f dB\n", worstGain, worstComp);
    verdict (worstGain < 0.1 && worstComp < 0.25, "unity gain in every mode; DRIVE changes colour, not level");
}

// ============================================================================
static void testNoise()
{
    header ("8. Noise and hum (20 Hz-20 kHz, re a full-scale sine)");
    DeviceParams p; p.noise = true;
    double clean = 0.0, off = 0.0;
    for (int m : { kClean, kWarm, kBold })
    {
        DeviceParams q = p; q.mode = m;
        Device d; d.prepare (FS, q);
        const double n = noiseDbfs (d, quick ? 0.5 : 1.0);
        if (m == kClean) clean = n;
        std::printf ("  %s: %.1f dBFS\n", m == kClean ? "CLEAN" : (m == kWarm ? "WARM " : "BOLD "), n);
    }
    {
        DeviceParams q = p; q.mode = kWarm; q.acHeaters = true;
        Device d; d.prepare (FS, q);
        std::printf ("  WARM, AC heaters: %.1f dBFS", noiseDbfs (d, quick ? 0.5 : 1.0));
        const int nm = (int) FS;
        std::vector<double> y ((size_t) nm);
        double l, r;
        for (int n = 0; n < nm; ++n) { d.process (0.0, 0.0, l, r); y[(size_t) n] = l; }
        Bh bh (nm);
        std::printf ("  (hum: 50 Hz %.1f dBFS, 150 Hz %.1f dBFS)\n", db (std::abs (bh.at (y, 50.0, FS))), db (std::abs (bh.at (y, 150.0, FS))));
    }
    {
        DeviceParams q = p; q.mode = kWarm; q.noise = false;
        Device d; d.prepare (FS, q);
        off = noiseDbfs (d, quick ? 0.5 : 1.0);
        std::printf ("  WARM, NOISE off (supply ripple only): %.1f dBFS\n", off);
    }
    verdict (clean < -95.0 && off < -115.0, "noise floor about -100 dBFS; with NOISE off the residual ripple stays below -115 dBFS");
}

// ============================================================================
static void testAnalyser()
{
    header ("9. Analyser calibration (0 dBFS sine reads 0 dB)");
    DisplayModel dm;               // only for its grid
    std::vector<double> grid ((size_t) DisplayModel::kN);
    for (int i = 0; i < DisplayModel::kN; ++i) grid[(size_t) i] = dm.freq (i);
    double worstPeak = 0.0, worstGrid = 0.0;
    for (double fs : { 44100.0, 48000.0, 96000.0 })
        for (double f : { 31.5, 97.0, 440.0, 1000.0, 5113.0, 15000.0 })
            for (double lev : { 0.0, -20.0, -60.0 })
            {
                SpectrumAnalyser an; an.prepare (grid, fs);
                std::vector<float> x ((size_t) SpectrumAnalyser::kLong);
                const double a = std::pow (10.0, lev / 20.0);
                for (size_t n = 0; n < x.size(); ++n) x[n] = (float) (a * std::sin (2.0 * kPi * f * (double) n / fs + 0.3));
                an.push (x.data(), (int) x.size());
                an.compute();
                double pk, hz;
                an.strongest (pk, hz);
                worstPeak = std::max (worstPeak, std::fabs (pk - lev));
                double mx = -300.0;
                for (double v : an.values()) mx = std::max (mx, v);
                worstGrid = std::max (worstGrid, std::fabs (mx - lev));
            }
    std::printf ("  sines 31.5 Hz-15 kHz, 0 / -20 / -60 dBFS, 44.1 / 48 / 96 kHz:\n");
    std::printf ("  peak reading worst error %.3f dB, displayed trace maximum worst error %.3f dB\n", worstPeak, worstGrid);
    verdict (worstPeak < 0.1 && worstGrid < 0.1, "analyser calibrated within 0.1 dB");
}

// ============================================================================
static void testDisplayVsSweep()
{
    header ("10. Screen curve vs measured response (-40 dBFS sines, device at 176.4 kHz)");
    struct Case { const char* name; DeviceParams p; int set; };
    std::vector<Case> cases;
    cases.push_back ({ "musical setting", musical(), 0 });
    {
        DeviceParams p; p.noise = false;
        auto& c = p.ch[0];
        c.hpfHz = 35.0;
        c.band[kLow]  = { 10.0, 25.0, false, 10.0 };
        c.band[kBody] = { -6.0, 270.0, false, 0.0 };
        c.band[kPres] = { 8.0, 5600.0, true, 0.0 };
        c.band[kAir]  = { 6.0, 10000.0, false, 0.0 };
        p.ch[1] = c;
        p.mode = kBold; p.iron = kSteel;
        cases.push_back ({ "extremes, BOLD, STEEL", p, 0 });
    }
    {
        DeviceParams p = musical();
        p.stereo = kMidSide;
        p.ch[1].band[kLow].gainDb = -3.0; p.ch[1].band[kAir].gainDb = 4.0; p.ch[1].band[kPres].gainDb = -2.0;
        cases.push_back ({ "M/S, side channel", p, 1 });
    }
    const auto freqs = logGrid (20.0, 20000.0, quick ? 13 : 25);
    double worst = 0.0;
    for (auto& cs : cases)
    {
        DisplayModel dm;
        dm.prepare (FS, 44100.0, cs.p);
        Device d; d.prepare (FS, cs.p);
        settleSilence (d, 0.3);
        const double a = std::pow (10.0, -40.0 / 20.0);
        double w = 0.0, wf = 0.0;
        for (double f : freqs)
        {
            // M/S: excite the side only (L = x, R = −x), else both channels in phase
            const double aR = cs.set == 1 ? -a : a;
            const auto t = tone (d, f, a, aR, 0, f < 60 ? 0.35 : 0.12, f < 60 ? 0.5 : 0.15);
            const double model = db (std::abs (dm.response (cs.set, f)));
            const double e = std::fabs (t.gainDb - model);
            if (e > w) { w = e; wf = f; }
        }
        worst = std::max (worst, w);
        std::printf ("  %-24s worst |measured - screen| = %.4f dB (at %.0f Hz)\n", cs.name, w, wf);
    }
    verdict (worst < 0.05, "the curve on the screen is the measured response within 0.05 dB");
}

// ============================================================================
static void testCpu()
{
    header ("11. CPU");
    DeviceParams p = musical(); p.noise = true;
    Device d; d.prepare (FS, p);
    Pink pk (3);
    const int N = (int) (FS * (quick ? 1.0 : 3.0));
    std::vector<double> x ((size_t) N);
    for (auto& v : x) v = 0.6 * pk.next();
    double l, r, sink = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int n = 0; n < N; ++n) { d.process (x[(size_t) n], 0.9 * x[(size_t) n], l, r); sink += l; }
    const auto t1 = std::chrono::steady_clock::now();
    const double ns = std::chrono::duration<double, std::nano> (t1 - t0).count() / N;
    std::printf ("  %.0f ns per stereo sample at the internal rate\n", ns);
    std::printf ("  host 44.1 kHz (x4 -> 176.4 kHz): %.1f %% of one core; host 96 kHz (x2 -> 192 kHz): %.1f %%\n", ns * 176400.0 / 1.0e7, ns * 192000.0 / 1.0e7);
    std::printf ("  DELTA engaged runs a second device: about twice that while auditioning\n");
    std::printf ("  (sink %.3g)\n\n", sink);
}

// ============================================================================
static void testTorture()
{
    header ("12. Torture: every parameter at random, DELTA toggling, extreme signals");
    std::mt19937 rng (20261025);
    std::uniform_real_distribution<double> U (0.0, 1.0);
    DeviceParams p = musical();
    p.noise = true;
    DeltaEngine e; e.prepare (FS, p);
    const int block = 64;
    const int blocks = (int) ((quick ? 8.0 : 20.0) * FS / block);
    bool bad = false;
    double peak = 0.0;
    int changes = 0, units = 0;
    double ph = 0.0;
    int sigKind = 0, sigLeft = 0;
    double sigF = 100.0, sigA = 0.5;
    Pink pk (9);
    bool delta = false;
    for (int k = 0; k < blocks; ++k)
    {
        if (U (rng) < 0.35)
        {
            ++changes;
            const int what = (int) (U (rng) * 22.0);
            auto& c = p.ch[(size_t) (U (rng) < 0.5 ? 0 : 1)];
            const int b = (int) (U (rng) * 4.0);
            switch (what)
            {
                case 0: p.mode = (int) (U (rng) * 3.0); break;
                case 1: p.driveDb = -6.0 + 18.0 * U (rng); break;
                case 2: p.iron = (int) (U (rng) * 3.0); break;
                case 3: p.stereo = (int) (U (rng) * 3.0); break;
                case 4: c.hpfHz = U (rng) < 0.3 ? 0.0 : eq::kHpfSteps[(int) (U (rng) * 6.0)]; break;
                case 5: case 6: case 7: c.band[(size_t) b].gainDb = (2.0 * U (rng) - 1.0) * eq::kMaxGain[b]; break;
                case 8: case 9: c.band[(size_t) b].freq = eq::kFMin[b] * std::pow (eq::kFMax[b] / eq::kFMin[b], U (rng)); break;
                case 10: c.band[kLow].weight = 10.0 * U (rng); break;
                case 11: c.band[kPres].narrow = ! c.band[kPres].narrow; break;
                case 12: p.mainsHz = U (rng) < 0.5 ? 50.0 : 60.0; break;
                case 13: p.rectifier = (int) (U (rng) * 3.0); break;
                case 14: p.acHeaters = ! p.acHeaters; break;
                case 15: p.sourceOhms = U (rng) < 0.33 ? 50.0 : (U (rng) < 0.5 ? 150.0 : 600.0); break;
                case 16: p.noise = ! p.noise; break;
                case 17: if (U (rng) < 0.25) { p.serial = 1 + (uint32_t) (U (rng) * 998.0); ++units; } break;
                case 18: if (U (rng) < 0.25) { p.tolerance = 2.0 * U (rng); ++units; } break;
                case 19: if (U (rng) < 0.25) { p.matched = ! p.matched; ++units; } break;
                case 20: delta = ! delta; break;
                default: p.warmup = ! p.warmup; break;
            }
            e.setParams (p);
        }
        e.poll();
        e.setDelta (delta, U (rng) < 0.5 ? 0 : 1, (int) (U (rng) * 5.0));
        if (--sigLeft <= 0)
        {
            sigKind = (int) (U (rng) * 5.0);
            sigLeft = 50 + (int) (U (rng) * 400.0);
            sigF = 20.0 * std::pow (1000.0, U (rng));
            sigA = std::pow (10.0, -3.0 * U (rng));
        }
        for (int i = 0; i < block; ++i)
        {
            double x = 0.0, y = 0.0;
            switch (sigKind)
            {
                case 0: x = y = 0.0; break;
                case 1: x = sigA * std::sin (ph); y = -x; break;
                case 2: x = sigA * 4.0 * pk.next(); y = 0.7 * x; break;
                case 3: x = y = std::sin (ph) >= 0.0 ? 1.0 : -1.0; break;   // full-scale square
                default: x = sigA * std::sin (ph); y = sigA * std::sin (1.37 * ph); break;
            }
            ph += 2.0 * kPi * sigF / FS; if (ph > 2.0 * kPi) ph -= 2.0 * kPi;
            double l, r;
            e.process (x, y, l, r);
            if (! std::isfinite (l) || ! std::isfinite (r)) bad = true;
            peak = std::max ({ peak, std::fabs (l), std::fabs (r) });
        }
    }
    std::printf ("  %.0f s of audio, %d parameter changes (%d new units), peak output %.2f (%.1f dBFS), NaN/Inf: %s\n",
                 blocks * (double) block / FS, changes, units, peak, db (peak), bad ? "YES" : "none");
    // The circuit's own swing limits the output to about +12 dBFS (tubes
    // clipping on full-scale squares with every band boosted); a numerical
    // blow-up would show far beyond.
    verdict (! bad && peak < 8.0, "stable under any automation (finite, peaks within the circuit's swing)");
}

// ============================================================================
int main (int argc, char** argv)
{
    std::setvbuf (stdout, nullptr, _IONBF, 0);
    quick = argc > 1 && std::strcmp (argv[1], "quick") == 0;
    const auto t0 = std::chrono::steady_clock::now();
    std::printf ("Halden VT-4 measurement bench%s\n\n", quick ? " (quick)" : "");
    testEqDigitalVsAnalog();
    testBandCentres();
    testSoloWindows();
    testDeltaNull();
    testMidSide();
    testIron();
    testModes();
    testNoise();
    testAnalyser();
    testDisplayVsSweep();
    testCpu();
    testTorture();
    const double secs = std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();
    std::printf ("==== %d passed, %d failed (%.0f s) ====\n", passes, failures, secs);
    return failures;
}
