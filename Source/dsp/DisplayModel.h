// Halden VT-4 — the curve on the screen.
//
// The display runs its own copy of the device (same unit, same tolerances,
// same operating point) and asks it for the small-signal transfer function
// of the actual discrete-time system that processes the sound:
//
//   H(f) = T1 · pad · V1a(+HPF) · driver · EQ network · V3a/White CF · T2 · trim
//
// multiplied by the measured response of the host-rate oversampling filters.
// So the curve is what you hear (at levels where the circuit is linear),
// including band interaction, transformer roll-offs and tolerances.
// Everything here runs on the message thread.

#pragma once

#include "Audition.h"
#include "Device.h"

#include <array>
#include <complex>
#include <memory>
#include <vector>

namespace vt4
{

class DisplayModel
{
public:
    static constexpr int kN = 320;          // 10 Hz … 40 kHz, log-spaced
    static constexpr int kBands = 5;        // 0 HPF, 1 LOW, 2 BODY, 3 PRESENCE, 4 AIR

    DisplayModel()
    {
        for (int i = 0; i < kN; ++i)
            fr[(size_t) i] = 10.0 * std::pow (4000.0, (double) i / (kN - 1));
        osMag.fill (1.0);
    }

    double freq (int i) const noexcept { return fr[(size_t) i]; }
    bool ready() const noexcept { return dev != nullptr; }

    void prepare (double oversampledRate, double hostRate, const DeviceParams& p)
    {
        fs = oversampledRate; hostFs = hostRate;
        dev = std::make_unique<Device>();
        DeviceParams q = p; q.noise = false; q.warmup = false; q.drift = false;
        dev->prepare (fs, q);
        dev->jumpToTargets();
        prm = q;
        computeFixed();
        computeEq();
    }

    // magnitude of the oversampling filters (up + down) at the grid, host rate
    void setOversamplingResponse (const std::vector<double>& mag)
    {
        for (int i = 0; i < kN && i < (int) mag.size(); ++i) osMag[(size_t) i] = mag[(size_t) i];
        computeTotals();
    }

    // same unit (serial, tolerance, matching) as the model was built for
    bool sameUnit (const DeviceParams& p) const noexcept
    {
        return p.serial == prm.serial && p.tolerance == prm.tolerance && p.matched == prm.matched;
    }

    // Returns true if the curves changed. A different unit needs prepare()
    // (≈ 0.1 s: the editor runs it on a background thread).
    bool update (const DeviceParams& pIn)
    {
        if (! dev) return false;
        DeviceParams p = pIn; p.noise = false; p.warmup = false; p.drift = false;
        if (p.serial != prm.serial || p.tolerance != prm.tolerance || p.matched != prm.matched)
        {
            prepare (fs, hostFs, p);
            return true;
        }
        const bool fixedChanged = p.mode != prm.mode || p.driveDb != prm.driveDb || p.iron != prm.iron
                               || p.stereo != prm.stereo || p.sourceOhms != prm.sourceOhms
                               || p.rectifier != prm.rectifier || p.mainsHz != prm.mainsHz
                               || p.ch[0].hpfHz != prm.ch[0].hpfHz || p.ch[1].hpfHz != prm.ch[1].hpfHz;
        bool eqChanged = fixedChanged;
        for (int c = 0; c < 2; ++c)
            for (int b = 0; b < kNumBands; ++b)
            {
                const auto& x = p.ch[(size_t) c].band[(size_t) b];
                const auto& y = prm.ch[(size_t) c].band[(size_t) b];
                eqChanged |= x.gainDb != y.gainDb || x.freq != y.freq || x.narrow != y.narrow || x.weight != y.weight;
            }
        if (! fixedChanged && ! eqChanged) return false;
        dev->setParams (p);
        dev->jumpToTargets();
        prm = p;
        if (fixedChanged) computeFixed();
        computeEq();
        return true;
    }

    // core index used for channel set s (A = L / M, B = R / S)
    int coreFor (int set) const noexcept { return prm.stereo == kLink ? 0 : set; }

    double totalDb (int set, int i) const noexcept { return totDb[(size_t) coreFor (set)][(size_t) i]; }
    double bandDb (int set, int band, int i) const noexcept { return effDb[(size_t) coreFor (set)][(size_t) band][(size_t) i]; }
    bool bandActive (int set, int band) const noexcept { return active[(size_t) coreFor (set)][(size_t) band]; }

    // complex response of channel set s at any frequency (tests)
    std::complex<double> response (int set, double f) const
    {
        const int c = coreFor (set);
        return dev->response (c, f, true);
    }

    // Listening window for SOLO (see Audition.h for the definition).
    SoloWindow soloWindow (int set, int band)
    {
        SoloWindow w;
        if (! dev) return w;
        const int c = coreFor (set);
        if (band == 0)
        {
            const double f = prm.ch[(size_t) (prm.stereo == kLink ? 0 : set)].hpfHz;
            w.type = SoloWindow::kLowPass;
            w.f1 = 20.0;
            w.f2 = f > 0.0 ? f : eq::kHpfSteps[0];
            return w;
        }
        const int b = band - 1;
        const Core& k = dev->core[(size_t) c];
        EqState s;
        for (int j = 0; j < kNumBands; ++j) s.band[(size_t) j] = k.bands[(size_t) j].cur;
        EqState s0 = s;
        s0.band[(size_t) b].set = 0;
        s0.band[(size_t) b].x = eq::nullRatio();
        EqState s1 = s;
        if (s1.band[(size_t) b].set == 0 || std::fabs (prm.ch[(size_t) (prm.stereo == kLink ? 0 : set)].band[(size_t) b].gainDb) < 0.05)
        {
            // flat band: the window follows the shape of a tiny boost (∂E/∂g)
            s1.band[(size_t) b].set = 1;
            s1.band[(size_t) b].x = tapFromTable (b, 1, s1.band[(size_t) b].narrow, 0.1);
        }
        Circuit cw, c0;
        eq::build (cw, s1, k.eqTol, true); cw.compile (fs, ws);
        eq::build (c0, s0, k.eqTol, true); c0.compile (fs, ws);
        auto effect = [&] (double f)
        {
            return 20.0 * std::log10 (std::abs (cw.digitalResponse (f)) / std::abs (c0.digitalResponse (f)));
        };
        return findSoloWindow (effect, 20.0, std::min (20000.0, 0.45 * hostFs));
    }

    const DeviceParams& params() const noexcept { return prm; }
    double hostRate() const noexcept { return hostFs; }
    Device* device() noexcept { return dev.get(); }

private:
    void computeFixed()
    {
        const double x0 = eq::nullRatio();
        for (int c = 0; c < 2; ++c)
        {
            for (int i = 0; i < kN; ++i)
                fixedResp[(size_t) c][(size_t) i] = dev->response (c, fr[(size_t) i], false) / x0;
            // HPF effect: input stage with the high-pass caps vs. the coupling caps alone
            Core& k = dev->core[(size_t) c];
            InputStageValues v = k.v1; v.ch1 = v.ch2 = 1.0e-6; v.rb = k.rb;
            Circuit off; buildInputStage (off, v); off.compile (fs, ws);
            const bool on = prm.ch[(size_t) (prm.stereo == kLink ? 0 : c)].hpfHz > 0.0;
            active[(size_t) c][0] = on;
            for (int i = 0; i < kN; ++i)
            {
                if (! on) { effDb[(size_t) c][0][(size_t) i] = 0.0; continue; }
                const double f = fr[(size_t) i];
                const double a = std::abs (k.s1.c.digitalResponse (f, 0, 0, k.s1.qJ));
                const double b = std::abs (off.digitalResponse (f, 0, 0, k.s1.qJ));
                effDb[(size_t) c][0][(size_t) i] = 20.0 * std::log10 (a / b);
            }
        }
    }

    void computeEq()
    {
        for (int c = 0; c < 2; ++c)
        {
            const Core& k = dev->core[(size_t) c];
            for (int i = 0; i < kN; ++i)
                eqResp[(size_t) c][(size_t) i] = k.eqc.digitalResponse (fr[(size_t) i]);
            EqState s;
            for (int j = 0; j < kNumBands; ++j) s.band[(size_t) j] = k.bands[(size_t) j].cur;
            for (int b = 0; b < kNumBands; ++b)
            {
                auto& e = effDb[(size_t) c][(size_t) (b + 1)];
                if (s.band[(size_t) b].set == 0) { e.fill (0.0); active[(size_t) c][(size_t) (b + 1)] = false; continue; }
                active[(size_t) c][(size_t) (b + 1)] = true;
                EqState s0 = s;
                s0.band[(size_t) b].set = 0;
                Circuit c0; eq::build (c0, s0, k.eqTol, true); c0.compile (fs, ws);
                for (int i = 0; i < kN; i += 2)
                {
                    const double v = 20.0 * std::log10 (std::abs (eqResp[(size_t) c][(size_t) i]) / std::abs (c0.digitalResponse (fr[(size_t) i])));
                    e[(size_t) i] = v;
                    if (i > 0) e[(size_t) (i - 1)] = 0.5 * (v + e[(size_t) (i - 2)]);
                }
                if ((kN & 1) == 0) e[(size_t) (kN - 1)] = e[(size_t) (kN - 2)];
            }
        }
        computeTotals();
    }

    void computeTotals()
    {
        if (! dev) return;
        for (int c = 0; c < 2; ++c)
            for (int i = 0; i < kN; ++i)
            {
                double m = std::abs (fixedResp[(size_t) c][(size_t) i] * eqResp[(size_t) c][(size_t) i]);
                if (fr[(size_t) i] < 0.5 * hostFs) m *= osMag[(size_t) i];
                totDb[(size_t) c][(size_t) i] = 20.0 * std::log10 (std::max (m, 1.0e-12));
            }
    }

    std::unique_ptr<Device> dev;
    DeviceParams prm;
    double fs = 176400.0, hostFs = 44100.0;
    CircuitWorkspace ws;
    std::array<double, kN> fr {}, osMag {};
    std::array<std::array<std::complex<double>, kN>, 2> fixedResp {}, eqResp {};
    std::array<std::array<double, kN>, 2> totDb {};
    std::array<std::array<std::array<double, kN>, kBands>, 2> effDb {};
    std::array<std::array<bool, kBands>, 2> active {};
};

} // namespace vt4
