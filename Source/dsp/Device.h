// Halden VT-4 — the complete stereo device.
//
//   L/R in ─ T1 (nickel) ─┬─ [M/S matrix] ─ core A ─┬─ [M/S matrix] ─ T2 (nickel/steel) ─ out
//                         └───────────────── core B ─┘
//
//   core: pad ─ V1a 5751 ─ RC-RC high-pass ─ V2a 6922 driver ─ passive LC EQ
//         ─ V3a 5751 + White cathode follower (6922) with switchable feedback
//
// One power supply feeds both channels. Everything runs at the oversampled
// rate; parameter changes are smoothed and applied at a control rate by
// recompiling the affected circuits (states are kept, so nothing jumps).

#pragma once

#include "Circuit.h"
#include "EqNetwork.h"
#include "EqTables.h"
#include "Magnetics.h"
#include "Noise.h"
#include "PowerSupply.h"
#include "Rng.h"
#include "Stages.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>

namespace vt4
{

enum Mode : int { kClean = 0, kWarm, kBold };
enum Iron : int { kIronOff = 0, kNickel, kSteel };
enum StereoMode : int { kLink = 0, kDual, kMidSide };

struct BandParams
{
    double gainDb = 0.0;
    double freq = 1000.0;
    bool narrow = false;
    double weight = 0.0;
};

struct ChannelParams
{
    double hpfHz = 0.0;                    // 0 = off
    std::array<BandParams, kNumBands> band {};
};

struct DeviceParams
{
    int mode = kWarm;
    double driveDb = 0.0;
    int iron = kNickel;
    int stereo = kLink;
    std::array<ChannelParams, 2> ch {};
    double sourceOhms = 150.0;
    double calDbu = 18.0;                  // 0 dBFS = +18 dBu
    double mainsHz = 50.0;
    bool acHeaters = false;
    int rectifier = 0;                     // 0 auto, 1 tube, 2 silicon
    bool warmup = false;
    bool drift = false;
    bool matched = true;
    double tolerance = 1.0;                // 0 … 2
    uint32_t serial = 1;
    bool noise = true;

    DeviceParams()
    {
        for (auto& c : ch)
        {
            c.band[kLow].freq = 50.0; c.band[kBody].freq = 560.0;
            c.band[kPres].freq = 3900.0; c.band[kAir].freq = 20000.0;
        }
    }
};

// ---------------------------------------------------------------------------
namespace design
{
    // CLEAN: V1a fully degenerated (10 k unbypassed), tight feedback, silicon rectifier
    // WARM:  V1a partly bypassed (≈ 2k5 AC), run hotter, tight feedback, GZ34
    // BOLD:  V1a mostly bypassed (≈ 900 Ω AC), hotter still, loose feedback, GZ34
    // Levels and bypass were chosen for THD at −18 dBFS of ≈ 0.004 / 0.035 /
    // 0.11 % with a noise floor near −100 / −100 / −97 dBFS (20 Hz–20 kHz).
    inline double kV1aTarget[3] = { 4.0, 7.0, 12.0 };        // V1a output, V pk at 0 dBFS
    constexpr double kV3Target = 4.36;                          // T2 primary, V pk at 0 dBFS
    inline double kRb[3]  = { 1.0e9, 3300.0, 1000.0 };         // V1a AC bypass resistance
    inline double kRfb[3] = { 4700.0, 4700.0, 15000.0 };       // V3a feedback
    constexpr double kV1aIp = 1.0e-3;                           // V1a bias target
    constexpr double kDrvBias = 0.339;                          // driver grid divider (× B+)
    constexpr double kWhiteCurrent = 30.0e-3;                   // per channel

    // Transformer cores. lm: small-signal magnetising inductance [H];
    // vsat: peak primary volts at 20 Hz that reach the knee; hyst: weight of
    // the Rayleigh (hysteresis) part; rmax: top of the Rayleigh region in
    // normalised flux; a, b, g: anhysteretic curvature and saturation knee.
    struct CoreDesign { double lm, vsat, dcr, fhf, hyst, rmax, a, b, g; };
    // Calibrated for H3 at 30 Hz, −18 dBFS: NICKEL ≈ −85 dB, STEEL ≈ −80 dB
    // (150 Ω source); at 0 dBFS / 20 Hz ≈ 0.1 % and 0.2 % THD.
    inline CoreDesign kT1       { 14.0, 30.0, 35.0, 90.0e3, 0.15, 0.15, 0.008, 0.03, 3.5 };   // input, 80 % nickel
    inline CoreDesign kT2Nickel { 30.0, 14.0, 80.0, 85.0e3, 0.05, 0.15, 0.006, 0.04, 4.5 };   // output, nickel
    inline CoreDesign kT2Steel  {  8.0, 25.0, 60.0, 70.0e3, 0.50, 0.20, 0.020, 0.02, 3.0 };   // output, grain-oriented steel
}

inline double tapFromTable (int band, int set, bool narrow, double gainAbs)
{
    const double* t = nullptr; int n = 0;
    switch (band)
    {
        case kLow:  t = set > 0 ? eqtab::LOW_B : eqtab::LOW_C;   n = set > 0 ? eqtab::LOW_B_N : eqtab::LOW_C_N; break;
        case kBody: t = set > 0 ? eqtab::BODY_B : eqtab::BODY_C; n = set > 0 ? eqtab::BODY_B_N : eqtab::BODY_C_N; break;
        case kPres:
            if (narrow) { t = set > 0 ? eqtab::PRN_B : eqtab::PRN_C; n = set > 0 ? eqtab::PRN_B_N : eqtab::PRN_C_N; }
            else        { t = set > 0 ? eqtab::PRW_B : eqtab::PRW_C; n = set > 0 ? eqtab::PRW_B_N : eqtab::PRW_C_N; }
            break;
        default:    t = set > 0 ? eqtab::AIR_B : eqtab::AIR_C;   n = set > 0 ? eqtab::AIR_B_N : eqtab::AIR_C_N; break;
    }
    const double pos = std::clamp (gainAbs / eqtab::kStep, 0.0, (double) (n - 1) - 1.0e-9);
    const int i = (int) pos;
    const double fr = pos - i;
    // Catmull-Rom
    const double p0 = t[std::max (i - 1, 0)], p1 = t[i], p2 = t[std::min (i + 1, n - 1)], p3 = t[std::min (i + 2, n - 1)];
    return p1 + 0.5 * fr * (p2 - p0 + fr * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 + fr * (3.0 * (p1 - p2) + p3 - p0)));
}

// One band of the passive EQ with smoothing and the boost/cut changeover.
struct BandControl
{
    BandState cur;            // what the circuit is built with
    double targetGain = 0.0, targetF = 1000.0, targetW = 0.0;
    bool targetNarrow = false;
    double focus = 0.0;       // 0 wide … 1 narrow (smoothed)

    void jump (int band, const BandParams& p)
    {
        targetGain = p.gainDb; targetF = p.freq; targetW = p.weight; targetNarrow = p.narrow;
        focus = p.narrow ? 1.0 : 0.0;
        cur.f = p.freq; cur.weight = p.weight; cur.narrow = p.narrow;
        cur.set = p.gainDb > 0.0 ? 1 : (p.gainDb < 0.0 ? -1 : 0);
        cur.x = cur.set == 0 ? eq::nullRatio() : tapFromTable (band, cur.set, p.narrow, std::fabs (p.gainDb));
    }

    // returns true if the circuit has to be rebuilt
    bool tick (int band, double a)
    {
        const BandState old = cur;
        const double x0 = eq::nullRatio();
        const int tset = targetGain > 0.0 ? 1 : (targetGain < 0.0 ? -1 : 0);
        // frequency (log), weight, focus
        cur.f = std::exp (std::log (cur.f) + a * (std::log (targetF) - std::log (cur.f)));
        if (std::fabs (std::log (cur.f / targetF)) < 1.0e-7) cur.f = targetF;
        cur.weight += a * (targetW - cur.weight);
        if (std::fabs (cur.weight - targetW) < 1.0e-6) cur.weight = targetW;
        focus += a * ((targetNarrow ? 1.0 : 0.0) - focus);
        if (std::fabs (focus - (targetNarrow ? 1.0 : 0.0)) < 1.0e-6) focus = targetNarrow ? 1.0 : 0.0;
        cur.narrow = focus > 0.5;

        if (cur.set != tset && cur.set != 0)
        {
            // glide to the null on the old parts, then change over
            cur.x += 2.0 * a * (x0 - cur.x);
            if (std::fabs (cur.x - x0) < 2.0e-5) { cur.x = x0; cur.set = tset; }
        }
        else
        {
            if (cur.set == 0 && tset != 0) { cur.set = tset; cur.x = x0; }
            if (cur.set != 0)
            {
                // tap for the target gain, blended across the focus morph
                double xt;
                if (band == kPres && focus > 0.0 && focus < 1.0)
                    xt = (1.0 - focus) * tapFromTable (band, cur.set, false, std::fabs (targetGain))
                       + focus * tapFromTable (band, cur.set, true, std::fabs (targetGain));
                else
                    xt = tapFromTable (band, cur.set, cur.narrow, std::fabs (targetGain));
                cur.x += a * (xt - cur.x);
                if (std::fabs (cur.x - xt) < 1.0e-9) cur.x = xt;
                if (tset == 0 && std::fabs (cur.x - x0) < 2.0e-5) { cur.x = x0; cur.set = 0; }
            }
        }
        return cur.set != old.set || cur.x != old.x || cur.f != old.f || cur.weight != old.weight || cur.narrow != old.narrow;
    }
};

// first-order high-pass for the parafeed coupling (DC block before T2)
struct DcBlock
{
    double a = 0.99998, x1 = 0.0, y1 = 0.0;
    void prepare (double fs, double fc, double x0) { a = std::exp (-2.0 * kPi * fc / fs); x1 = x0; y1 = 0.0; }
    inline double process (double x) noexcept { const double y = a * (y1 + x - x1); x1 = x; y1 = y; return y; }
    std::complex<double> response (double f, double fs) const
    {
        const auto zi = std::polar (1.0, -2.0 * kPi * f / fs);
        return a * (1.0 - zi) / (1.0 - a * zi);
    }
};

// ---------------------------------------------------------------------------
class Core
{
public:
    TubeStage s1, s2, s3;
    Circuit eqc;
    std::array<BandControl, kNumBands> bands {};
    InputStageValues v1;
    DriverStageValues v2;
    OutputStageValues v3;
    NoiseSource n1, n3;
    DcBlock dcb;
    EqTolerance eqTol;

    double rPad = 0.03, r3 = 1.0, trim = 1.0;
    double bias1Ratio = 0.03;
    double rb = design::kRb[1], rfb = design::kRfb[1];
    double hpfC = 1.0e-6;
    double noiseGain = 1.0, humAmp = 0.0;
    double y3dc = 0.0;
    // targets for smoothing
    double tRPad = 0.03, tR3 = 1.0, tTrim = 1.0, tRb = 1.0e9, tRfb = 4700.0, tHpfC = 1.0e-6, tNoise = 1.0;
    double stress = 0.0;

    // grid-bias dividers are decoupled by capacitors (≈ 0.3 Hz), so they
    // follow the slow sag of their rail but not the ripple
    double bias1 = 0.0, bias2 = 0.0, aBias = 1.0e-5;

    inline double process (double vin, const PsuRails& r, int ch, double hum) noexcept
    {
        bias1 += aBias * (bias1Ratio * r.v1a[ch] - bias1);
        bias2 += aBias * (design::kDrvBias * r.drv[ch] - bias2);
        const bool noisy = noiseGain != 0.0;
        const double g1 = vin * rPad + (noisy ? noiseGain * n1.next() : 0.0) + hum + bias1;
        const double o1 = s1.step (g1, r.v1a[ch]);
        const double o2 = s2.step (o1 + bias2, r.drv[ch]);
        const double e = eqc.stepLinear (&o2);
        const double y = s3.step (e * r3 + (noisy ? noiseGain * n3.next() : 0.0), r.v3a[ch]);
        stress = std::max (stress, std::max (std::fabs (s1.sol.ig), std::fabs (s3.sol.ig)) * 2.0e5);
        return dcb.process (y) * trim;
    }
};

class Device
{
public:
    static constexpr int kCtrl = 32;          // control-rate divider

    void prepare (double oversampledRate, const DeviceParams& p)
    {
        fs = oversampledRate;
        prm = p;
        rebuildEverything (true);
    }

    void setParams (const DeviceParams& p)
    {
        const bool unitChanged = p.serial != prm.serial || p.tolerance != prm.tolerance || p.matched != prm.matched;
        const bool supplyChanged = p.mainsHz != prm.mainsHz || rectifierFor (p) != rectifierFor (prm);
        const bool ironChanged = p.iron != prm.iron;
        const bool sourceChanged = p.sourceOhms != prm.sourceOhms;
        prm = p;
        if (unitChanged) { rebuildEverything (false); return; }
        if (supplyChanged)
        {
            psu.setMains (p.mainsHz);
            psu.setRectifier (rectifierFor (p), ws);
        }
        if (ironChanged || sourceChanged) startIronFade();
        updateTargets();
    }

    const DeviceParams& params() const noexcept { return prm; }

    inline void process (double inL, double inR, double& outL, double& outR) noexcept
    {
        if (--ctrl <= 0) { ctrl = kCtrl; controlTick(); }

        // The supply runs at fs / psuDiv (≈ 22 kHz: ripple and sag are slow);
        // its rails are interpolated linearly to the audio rate.
        if (psuCount == 0)
        {
            PsuLoads avg;
            const double inv = 1.0 / (double) psuDiv;
            for (int c = 0; c < 2; ++c)
            {
                avg.v1a[c] = loadSum.v1a[c] * inv; avg.v3a[c] = loadSum.v3a[c] * inv; avg.drv[c] = loadSum.drv[c] * inv;
                loadSum.v1a[c] = loadSum.v3a[c] = loadSum.drv[c] = 0.0;
            }
            avg.white = loads.white;
            railPrev = railNext;
            railNext = psu.step (avg);
        }
        {
            const double t = (double) (psuCount + 1) / (double) psuDiv;
            railNow.main = railPrev.main + t * (railNext.main - railPrev.main);
            for (int c = 0; c < 2; ++c)
            {
                railNow.v1a[c] = railPrev.v1a[c] + t * (railNext.v1a[c] - railPrev.v1a[c]);
                railNow.v3a[c] = railPrev.v3a[c] + t * (railNext.v3a[c] - railPrev.v3a[c]);
                railNow.drv[c] = railPrev.drv[c] + t * (railNext.drv[c] - railPrev.drv[c]);
            }
            if (++psuCount >= psuDiv) psuCount = 0;
        }
        const PsuRails& rails = railNow;
        const double kin = kcal * gDrive;
        double tL = t1[0].process (inL * kin), tR = t1[1].process (inR * kin);
        if (fade > 0)
        {
            const double w = 1.0 - (double) fade / (double) kFade;
            tL = t1Old[0].process (inL * kin) * (1.0 - w) + tL * w;
            tR = t1Old[1].process (inR * kin) * (1.0 - w) + tR * w;
        }
        double a = tL, b = tR;
        const bool ms = prm.stereo == kMidSide;
        if (ms) { a = 0.5 * (tL + tR); b = 0.5 * (tL - tR); }

        double hA = 0.0, hB = 0.0;
        if (humOn > 0.0)
        {
            humPh += prm.mainsHz / fs; if (humPh >= 1.0) humPh -= 1.0;
            // heater–cathode coupling of V1a: the mains fundamental plus a
            // little third harmonic (the heater current is not a pure sine
            // under the transformer's magnetising load); ≈ −95 dBFS at the
            // output in WARM, i.e. just above the noise floor
            const double ph = 2.0 * kPi * humPh;
            hA = humOn * 4.7e-6 * (std::sin (ph) + 0.25 * std::sin (3.0 * ph));
            hB = humOn * 4.7e-6 * (std::sin (ph + 2.1) + 0.25 * std::sin (3.0 * ph + 0.7));
        }
        const double ya = core[0].process (a, rails, 0, hA);
        const double yb = core[1].process (b, rails, 1, hB);
        double oL = ya, oR = yb;
        if (ms) { oL = ya + yb; oR = ya - yb; }

        double zL = t2[0].process (oL), zR = t2[1].process (oR);
        if (fade > 0)
        {
            const double w = 1.0 - (double) fade / (double) kFade;
            zL = t2Old[0].process (oL) * (1.0 - w) + zL * w;
            zR = t2Old[1].process (oR) * (1.0 - w) + zR * w;
            --fade;
        }
        const double ko = 1.0 / kin;
        outL = zL * ko; outR = zR * ko;

        // loads for the next supply step
        for (int c = 0; c < 2; ++c)
        {
            loads.v1a[c] = core[c].s1.lastIp;
            loads.drv[c] = core[c].s2.lastIp;
            loads.v3a[c] = core[c].s3.lastIp;
            loadSum.v1a[c] += loads.v1a[c]; loadSum.drv[c] += loads.drv[c]; loadSum.v3a[c] += loads.v3a[c];
        }
    }

    // ---- monitoring --------------------------------------------------------
    double bplus() const noexcept { return railNow.main; }
    double takeStress() noexcept { const double s = std::max (core[0].stress, core[1].stress); core[0].stress = core[1].stress = 0.0; return s; }
    double warmth() const noexcept { return emission; }
    bool tubeRectifier() const noexcept { return psu.rectifier() == PowerSupply::kTube; }

    // ---- small-signal response (display, calibration) ---------------------
    // Transfer function of core channel c (incl. T1/T2 and trim), at the
    // oversampled rate, at the current operating point.
    std::complex<double> response (int c, double f, bool withEq = true) const
    {
        const Core& k = core[(size_t) c];
        const auto h1 = k.s1.c.digitalResponse (f, 0, 0, k.s1.qJ);
        const auto h2 = k.s2.c.digitalResponse (f, 0, 0, k.s2.qJ);
        const auto h3 = k.s3.c.digitalResponse (f, 0, 0, k.s3.qJ);
        const auto he = withEq ? k.eqc.digitalResponse (f) : std::complex<double> (eq::nullRatio(), 0.0);
        const auto ht = t1[(size_t) c].response (f) * t2[(size_t) c].response (f);
        return ht * k.rPad * h1 * h2 * he * k.r3 * h3 * k.dcb.response (f, fs) * k.trim;
    }

    const Circuit& eqCircuit (int c) const noexcept { return core[(size_t) c].eqc; }
    const BandState& bandState (int c, int b) const noexcept { return core[(size_t) c].bands[(size_t) b].cur; }
    double sampleRate() const noexcept { return fs; }

    // Brings every smoothed value to its target immediately (display model,
    // offline rendering).
    void jumpToTargets()
    {
        updateTargets();
        for (int c = 0; c < 2; ++c)
        {
            auto& k = core[(size_t) c];
            const auto& cp = prm.ch[(size_t) channelSet (c)];
            for (int b = 0; b < kNumBands; ++b) k.bands[(size_t) b].jump (b, cp.band[(size_t) b]);
            k.rPad = k.tRPad; k.r3 = k.tR3; k.rb = k.tRb; k.rfb = k.tRfb; k.hpfC = k.tHpfC; k.noiseGain = k.tNoise;
            rebuildStages (c);
            rebuildEq (c);
        }
        gDrive = tDrive;
        for (int c = 0; c < 2; ++c)
        {
            auto& k = core[(size_t) c];
            k.s1.settle (k.bias1Ratio * psu.current().v1a[c], psu.current().v1a[c], ws);
            k.s2.settle (k.s2.u[0], psu.current().drv[c], ws);
            k.s3.settle (0.0, psu.current().v3a[c], ws);
        }
        seedBias();
        computeTrims (true);
        for (auto& k : core) k.trim = k.tTrim;
    }

    void copyStateFrom (const Device& o) { *this = o; }

private:
    static int rectifierFor (const DeviceParams& p)
    {
        if (p.rectifier == 1) return PowerSupply::kTube;
        if (p.rectifier == 2) return PowerSupply::kSilicon;
        return p.mode == kClean ? PowerSupply::kSilicon : PowerSupply::kTube;
    }

    int channelSet (int c) const noexcept { return prm.stereo == kLink ? 0 : c; }

    // ---- construction -------------------------------------------------------
    void rebuildEverything (bool hard)
    {
        kcal = std::sqrt (2.0) * 0.775 * std::pow (10.0, prm.calDbu / 20.0);
        // tubes and parts for this unit
        const double tol = prm.tolerance;
        for (int c = 0; c < 2; ++c)
        {
            auto& k = core[(size_t) c];
            const uint64_t seed = (uint64_t) prm.serial * 0x100000001B3ull + (uint64_t) (c + 1) * 0x9E3779B97F4A7C15ull;
            Rng r (seed);
            auto part = [&] (double sigma) { return 1.0 + sigma * tol * r.clippedGauss(); };
            const double tubeSig = prm.matched ? 0.015 : 0.12, muSig = prm.matched ? 0.007 : 0.04;
            auto tube = [&] (TriodeParams t)
            {
                t.kg1 *= part (tubeSig); t.mu *= part (muSig); t.kp *= part (tubeSig * 0.6);
                return t;
            };
            k.s1.tube = tube (TriodeParams::t5751());
            k.s2.tube = tube (TriodeParams::t6922());
            k.s3.tube = tube (TriodeParams::t5751());
            nominalTube[c][0] = k.s1.tube; nominalTube[c][1] = k.s2.tube; nominalTube[c][2] = k.s3.tube;
            k.v1 = InputStageValues {};
            k.v1.rl *= part (0.01); k.v1.rk *= part (0.01); k.v1.ck *= part (0.2); k.v1.rh *= part (0.01); k.v1.rg *= part (0.01);
            k.v2 = DriverStageValues {};
            k.v2.rk *= part (0.01); k.v2.co *= part (0.1);
            k.v3 = OutputStageValues {};
            k.v3.rl *= part (0.01); k.v3.rka *= part (0.01); k.v3.rkb *= part (0.01); k.v3.ckb *= part (0.2);
            k.v3.cfb = 47.0e-6 * part (0.1); acfNom[c] = 0.985 * (1.0 + 0.002 * tol * r.clippedGauss()); k.v3.acf = acfNom[c];
            hpfK[c] = 0.0;
            k.eqTol.seed = seed ^ 0xA5A5A5A5ull; k.eqTol.amount = tol;
            tParts[c] = { part (0.08), part (0.05), part (0.15), part (0.15), part (0.08), part (0.05), part (0.15), part (0.15) };
            biasDividerTol[c] = part (0.01);
            noiseSeed[c] = (uint32_t) (seed >> 17) | 1u;
        }
        // supply
        psuDiv = std::max (1, (int) std::lround (fs / 22050.0));   // ≈ 22 kHz: 45 µs resolves the rectifier pulses
        psu.setMains (prm.mainsHz);
        psu.setRectifier (rectifierFor (prm), ws);
        psu.prepare (fs / (double) psuDiv, ws);
        // stages at the design operating point
        for (int c = 0; c < 2; ++c)
        {
            auto& k = core[(size_t) c];
            k.rb = k.tRb = design::kRb[prm.mode];
            k.rfb = k.tRfb = design::kRfb[prm.mode];
            k.hpfC = k.tHpfC = 1.0e-6;
            rebuildStages (c);
        }
        solveOperatingPoint();
        // per-mode design constants (pads, HPF law) from the nominal unit
        designModes();
        // transformers
        setupTransformers (true);
        // EQ
        for (int c = 0; c < 2; ++c)
        {
            auto& k = core[(size_t) c];
            const auto& cp = prm.ch[(size_t) channelSet (c)];
            for (int b = 0; b < kNumBands; ++b) k.bands[(size_t) b].jump (b, cp.band[(size_t) b]);
            rebuildEq (c);
            k.n1.prepare (fs, 1.0e-9, 1500.0, noiseSeed[c] * 3u + 1u);
            k.n3.prepare (fs, 1.0e-9, 1500.0, noiseSeed[c] * 5u + 2u);
            k.y3dc = k.s3.c.outputNL (0, k.s3.u, std::array<double, 2> { k.s3.sol.ip, k.s3.sol.ig }.data());
            k.dcb.prepare (fs, 0.5, k.y3dc);
        }
        updateTargets();
        for (int c = 0; c < 2; ++c)
        {
            auto& k = core[(size_t) c];
            k.rPad = k.tRPad; k.r3 = k.tR3; k.hpfC = k.tHpfC; k.noiseGain = k.tNoise;
            rebuildStages (c);
        }
        gDrive = tDrive;
        computeTrims (true);
        for (auto& k : core) k.trim = k.tTrim;
        if (hard)
        {
            warmTime = 1.0e9; emission = 1.0;
            if (prm.warmup)
            {
                // cold start: the bias network was designed for hot tubes (above);
                // now put every stage and the supply at the operating point of
                // cold cathodes, so the warm-up starts from a consistent state
                warmTime = 0.0; emission = 0.02;
                psu.setRectifierEmission (0.02);
                for (int c = 0; c < 2; ++c)
                {
                    auto& k = core[(size_t) c];
                    k.s1.tube.emission = k.s2.tube.emission = k.s3.tube.emission = emission;
                    k.v3.acf = acfNom[c] * std::clamp (emission * 1.02, 0.0, 1.0);   // as the control tick will set it
                }
                for (int c = 0; c < 2; ++c) rebuildStages (c);
                settleAll();
            }
        }
        ctrl = 1;
        fade = 0;
        (void) hard;
    }

    void rebuildStages (int c)
    {
        auto& k = core[(size_t) c];
        k.v1.rb = k.rb;
        k.v1.ch1 = k.v1.ch2 = k.hpfC;
        k.v3.rfb = k.rfb;
        buildInputStage (k.s1.c, k.v1);   k.s1.c.compile (fs, ws);
        buildDriverStage (k.s2.c, k.v2);  k.s2.c.compile (fs, ws);
        buildOutputStage (k.s3.c, k.v3);  k.s3.c.compile (fs, ws);
    }

    void rebuildEq (int c)
    {
        auto& k = core[(size_t) c];
        EqState s;
        for (int b = 0; b < kNumBands; ++b) s.band[(size_t) b] = k.bands[(size_t) b].cur;
        eq::build (k.eqc, s, k.eqTol, true);
        k.eqc.compile (fs, ws);
    }

    void solveOperatingPoint()
    {
        PsuLoads l;
        for (int pass = 0; pass < 4; ++pass)
        {
            psu.settle (l, ws);
            const PsuRails r = psu.current();
            for (int c = 0; c < 2; ++c)
            {
                auto& k = core[(size_t) c];
                // V1a grid bias: divider set for the design plate current
                if (pass == 0) k.bias1Ratio = findBias (k, r.v1a[c]) / r.v1a[c] * biasDividerTol[c];
                k.s1.settle (k.bias1Ratio * r.v1a[c], r.v1a[c], ws);
                k.s2.settle (design::kDrvBias * r.drv[c], r.drv[c], ws);
                k.s3.settle (0.0, r.v3a[c], ws);
                l.v1a[c] = k.s1.lastIp; l.drv[c] = k.s2.lastIp; l.v3a[c] = k.s3.lastIp;
            }
            l.white = 2.0 * design::kWhiteCurrent;
        }
        loads = l;
        seedSupplyInterpolation();
        seedBias();
        for (int c = 0; c < 2; ++c)
        {
            for (auto* st : { &core[(size_t) c].s1, &core[(size_t) c].s2, &core[(size_t) c].s3 })
                st->c.steadyState (st->u, std::array<double, 2> { st->sol.ip, st->sol.ig }.data());
            auto& k = core[(size_t) c];
            k.y3dc = k.s3.c.outputNL (0, k.s3.u, std::array<double, 2> { k.s3.sol.ip, k.s3.sol.ig }.data());
            k.dcb.prepare (fs, 0.5, k.y3dc);
        }
    }

    // operating point of every stage and the supply for the present tubes
    // and bias networks (no bias search)
    void settleAll()
    {
        PsuLoads l = loads;
        for (int pass = 0; pass < 3; ++pass)
        {
            psu.settle (l, ws);
            const PsuRails r = psu.current();
            for (int c = 0; c < 2; ++c)
            {
                auto& k = core[(size_t) c];
                k.s1.settle (k.bias1Ratio * r.v1a[c], r.v1a[c], ws);
                k.s2.settle (design::kDrvBias * r.drv[c], r.drv[c], ws);
                k.s3.settle (0.0, r.v3a[c], ws);
                l.v1a[c] = k.s1.lastIp; l.drv[c] = k.s2.lastIp; l.v3a[c] = k.s3.lastIp;
            }
        }
        loads = l;
        seedSupplyInterpolation();
        seedBias();
        for (int c = 0; c < 2; ++c)
        {
            for (auto* st : { &core[(size_t) c].s1, &core[(size_t) c].s2, &core[(size_t) c].s3 })
                st->c.steadyState (st->u, std::array<double, 2> { st->sol.ip, st->sol.ig }.data());
            auto& k = core[(size_t) c];
            k.y3dc = k.s3.c.outputNL (0, k.s3.u, std::array<double, 2> { k.s3.sol.ip, k.s3.sol.ig }.data());
            k.dcb.prepare (fs, 0.5, k.y3dc);
        }
    }

    void seedBias()
    {
        const PsuRails r = psu.current();
        for (int c = 0; c < 2; ++c)
        {
            auto& k = core[(size_t) c];
            k.aBias = 1.0 - std::exp (-2.0 * kPi * 0.3 / fs);
            k.bias1 = k.bias1Ratio * r.v1a[c];
            k.bias2 = design::kDrvBias * r.drv[c];
        }
    }

    void seedSupplyInterpolation()
    {
        railPrev = railNext = railNow = psu.current();
        psuCount = 0;
        for (int c = 0; c < 2; ++c)
        {
            loadSum.v1a[c] = loads.v1a[c] * psuDiv;
            loadSum.v3a[c] = loads.v3a[c] * psuDiv;
            loadSum.drv[c] = loads.drv[c] * psuDiv;
        }
    }

    double findBias (Core& k, double bplus)
    {
        double lo = 0.0, hi = 30.0;
        for (int it = 0; it < 50; ++it)
        {
            const double mid = 0.5 * (lo + hi);
            k.s1.settle (mid, bplus, ws);
            if (k.s1.sol.ip < design::kV1aIp) lo = mid; else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    // Pads and the high-pass law, from the nominal (tolerance-free) unit.
    void designModes()
    {
        Core& k = core[0];
        const InputStageValues keep1 = k.v1; const OutputStageValues keep3 = k.v3;
        const double keepRb = k.rb, keepRfb = k.rfb, keepC = k.hpfC;
        const double f0 = 1000.0;
        for (int m = 0; m < 3; ++m)
        {
            k.rb = design::kRb[m]; k.rfb = design::kRfb[m]; k.hpfC = 1.0e-6;
            rebuildStages (0);
            settleCore (0);
            const double g1 = std::abs (k.s1.c.digitalResponse (f0, 0, 0, k.s1.qJ));
            const double g2 = std::abs (k.s2.c.digitalResponse (f0, 0, 0, k.s2.qJ));
            const double g3 = std::abs (k.s3.c.digitalResponse (f0, 0, 0, k.s3.qJ));
            gain1[m] = g1; gain3[m] = g3;
            padNom[m] = design::kV1aTarget[m] / (kcal * g1);
            r3Nom[m]  = design::kV3Target / (design::kV1aTarget[m] * g2 * eq::nullRatio() * g3);
            // high-pass: C·f constant for −3.01 dB at f (RC-RC is frequency-scalable)
            const double fr = 50.0;
            double lo = 1.0e-10, hi = 1.0e-5;
            for (int it = 0; it < 60; ++it)
            {
                const double C = std::sqrt (lo * hi);
                k.hpfC = C; rebuildStages (0);
                const double r = std::abs (k.s1.c.digitalResponse (fr, 0, 0, k.s1.qJ)) / std::abs (k.s1.c.digitalResponse (2000.0, 0, 0, k.s1.qJ));
                if (r < std::pow (10.0, -3.0103 / 20.0)) lo = C; else hi = C;
            }
            hpfLaw[m] = std::sqrt (lo * hi) * fr;
        }
        k.v1 = keep1; k.v3 = keep3; k.rb = keepRb; k.rfb = keepRfb; k.hpfC = keepC;
        rebuildStages (0);
        settleCore (0);
    }

    void settleCore (int c)
    {
        auto& k = core[(size_t) c];
        const PsuRails r = psu.current();
        k.s1.settle (k.bias1Ratio * r.v1a[c], r.v1a[c], ws);
        k.s2.settle (design::kDrvBias * r.drv[c], r.drv[c], ws);
        k.s3.settle (0.0, r.v3a[c], ws);
    }

    // ---- transformers ------------------------------------------------------
    TransformerParams t1Params (int c) const
    {
        TransformerParams p;
        const auto& tp = tParts[c];
        const auto& m = design::kT1;
        p.ratio = 1.0; p.lm = m.lm * tp[0]; p.vsat = m.vsat * tp[1]; p.fsat = 20.0;
        p.rs = prm.sourceOhms + m.dcr; p.rpar = 9100.0; p.kout = 1.0;
        p.fhf = m.fhf; p.qhf = 0.65;
        p.core = { m.hyst * tp[2], m.rmax, m.a * tp[3], m.b, m.g };
        p.linear = prm.iron == kIronOff;
        return p;
    }
    TransformerParams t2Params (int c) const
    {
        TransformerParams p;
        const auto& tp = tParts[c];
        const auto& m = prm.iron == kSteel ? design::kT2Steel : design::kT2Nickel;
        p.ratio = 2.0; p.rs = 5.0 + m.dcr; p.rpar = 2380.0; p.kout = 0.988;
        p.fsat = 20.0; p.qhf = 0.7;
        p.lm = m.lm * tp[4]; p.vsat = m.vsat * tp[5]; p.fhf = m.fhf;
        p.core = { m.hyst * tp[6], m.rmax, m.a * tp[7], m.b, m.g };
        p.linear = prm.iron == kIronOff;
        return p;
    }

    void setupTransformers (bool reset)
    {
        for (int c = 0; c < 2; ++c)
        {
            if (reset) { t1[(size_t) c].prepare (fs, t1Params (c)); t2[(size_t) c].prepare (fs, t2Params (c)); }
            else       { t1[(size_t) c].setParams (t1Params (c));   t2[(size_t) c].setParams (t2Params (c)); }
        }
    }

    void startIronFade()
    {
        t1Old = t1; t2Old = t2;
        for (int c = 0; c < 2; ++c)
        {
            t1[(size_t) c].setParams (t1Params (c));
            t2[(size_t) c].setParams (t2Params (c));
            t1[(size_t) c].sanitiseCore();
            t2[(size_t) c].sanitiseCore();
        }
        fade = kFade;
    }

    // ---- control rate -------------------------------------------------------
    void updateTargets()
    {
        tDrive = std::pow (10.0, prm.driveDb / 20.0);
        humOnTarget = prm.acHeaters && prm.noise ? 1.0 : 0.0;
        for (int c = 0; c < 2; ++c)
        {
            auto& k = core[(size_t) c];
            const int m = prm.mode;
            k.tRb = design::kRb[m]; k.tRfb = design::kRfb[m];
            k.tRPad = padNom[m];
            k.tR3 = r3Nom[m];
            const auto& cp = prm.ch[(size_t) channelSet (c)];
            k.tHpfC = cp.hpfHz > 0.0 ? hpfLaw[m] / cp.hpfHz : 1.0e-6;
            k.tNoise = prm.noise ? 1.0 : 0.0;
            for (int b = 0; b < kNumBands; ++b)
            {
                auto& bc = k.bands[(size_t) b];
                const auto& bp = cp.band[(size_t) b];
                bc.targetGain = std::clamp (bp.gainDb, -eq::kMaxGain[b], eq::kMaxGain[b]);
                bc.targetF = std::clamp (bp.freq, eq::kFMin[b], eq::kFMax[b]);
                bc.targetNarrow = bp.narrow;
                bc.targetW = b == kLow ? std::clamp (bp.weight, 0.0, 10.0) : 0.0;
            }
            // noise densities for this mode: grid source + tube shot noise + unbypassed cathode
            const double kT4 = 4.0 * 1.380649e-23 * 300.0;
            const double gm1 = 1.25e-3, gm3 = 1.3e-3;
            const double rkAc = k.v1.rk * design::kRb[m] / (k.v1.rk + design::kRb[m]);
            const double e1 = std::sqrt (kT4 * (k.v1.rgs + 2.5 / gm1 + rkAc));
            const double e3 = std::sqrt (kT4 * (1700.0 + 2.5 / gm3 + k.v3.rka));
            if (std::fabs (e1 - noiseE1[c]) > 1.0e-15) { k.n1.prepare (fs, e1, 1000.0, noiseSeed[c] * 3u + 1u); noiseE1[c] = e1; }
            if (std::fabs (e3 - noiseE3[c]) > 1.0e-15) { k.n3.prepare (fs, e3, 1000.0, noiseSeed[c] * 5u + 2u); noiseE3[c] = e3; }
        }
    }

    static inline double approach (double cur, double target, double a, bool logDomain)
    {
        if (cur == target) return cur;
        double n;
        if (logDomain) n = std::exp (std::log (cur) + a * (std::log (target) - std::log (cur)));
        else n = cur + a * (target - cur);
        const double rel = logDomain ? std::fabs (std::log (n / target)) : std::fabs (n - target) / (std::fabs (target) + 1.0e-12);
        return rel < 1.0e-7 ? target : n;
    }

    void controlTick()
    {
        const double dt = (double) kCtrl / fs;
        const double aFast = 1.0 - std::exp (-dt / 0.015);
        const double aMode = 1.0 - std::exp (-dt / 0.040);
        const double aSlow = 1.0 - std::exp (-dt / 0.030);

        // warm-up and drift
        if (warmTime < 1.0e8)
        {
            warmTime += dt;
            const double tt = std::clamp ((warmTime - 2.0) / 22.0, 0.0, 1.0);
            emission = 0.02 + 0.98 * tt * tt * (3.0 - 2.0 * tt);
            const double tr = std::clamp ((warmTime - 1.0) / 9.0, 0.0, 1.0);
            psu.setRectifierEmission (0.02 + 0.98 * tr * tr * (3.0 - 2.0 * tr));
            if (warmTime > 30.0) { warmTime = 1.0e9; emission = 1.0; psu.setRectifierEmission (1.0); }
        }
        if (prm.drift)
        {
            for (auto& d : driftState) d = d * 0.99998 + 2.0e-5 * driftRng.gauss();
        }
        for (int c = 0; c < 2; ++c)
        {
            auto& k = core[(size_t) c];
            const double d0 = prm.drift ? 0.003 * driftState[(size_t) (c * 3 + 0)] / 0.0045 : 0.0;
            k.s1.tube.emission = emission * (1.0 + d0);
            k.s2.tube.emission = emission * (1.0 + (prm.drift ? 0.003 * driftState[(size_t) (c * 3 + 1)] / 0.0045 : 0.0));
            k.s3.tube.emission = emission * (1.0 + (prm.drift ? 0.003 * driftState[(size_t) (c * 3 + 2)] / 0.0045 : 0.0));
        }

        gDrive = approach (gDrive, tDrive, aSlow, true);
        humOn = approach (humOn, humOnTarget, aFast, false);
        bool trimsDirty = false;
        for (int c = 0; c < 2; ++c)
        {
            auto& k = core[(size_t) c];
            const double rb0 = k.rb, rfb0 = k.rfb, c0 = k.hpfC;
            k.rb = approach (k.rb, k.tRb, aMode, true);
            k.rfb = approach (k.rfb, k.tRfb, aMode, true);
            k.hpfC = approach (k.hpfC, k.tHpfC, aFast, true);
            const double p0 = k.rPad, q0 = k.r3;
            k.rPad = approach (k.rPad, k.tRPad, aMode, true);
            k.r3 = approach (k.r3, k.tR3, aMode, true);
            k.noiseGain = approach (k.noiseGain, k.tNoise, aFast, false);
            const double v3acf = k.v3.acf;
            k.v3.acf = acfNom[c] * std::clamp (emission * 1.02, 0.0, 1.0);
            if (k.rb != rb0 || k.rfb != rfb0 || k.hpfC != c0 || k.v3.acf != v3acf)
            {
                rebuildStages (c);
                trimsDirty = true;
            }
            if (k.rPad != p0 || k.r3 != q0) trimsDirty = true;
            bool eqDirty = false;
            for (int b = 0; b < kNumBands; ++b)
                eqDirty |= k.bands[(size_t) b].tick (b, aFast);
            if (eqDirty) rebuildEq (c);
            k.trim = approach (k.trim, k.tTrim, aSlow, false);
        }
        if (trimsDirty || trimCountdown-- <= 0) { computeTrims (false); trimCountdown = 256; }
    }

    // Factory calibration: unity small-signal gain at 1 kHz with a flat EQ.
    void computeTrims (bool force)
    {
        const PsuRails r = psu.current();
        for (int c = 0; c < 2; ++c)
        {
            auto& k = core[(size_t) c];
            k.s1.updateQuiescent (k.bias1Ratio * r.v1a[c], r.v1a[c], ws);
            k.s2.updateQuiescent (design::kDrvBias * r.drv[c], r.drv[c], ws);
            k.s3.updateQuiescent (0.0, r.v3a[c], ws);
            const double saved = k.trim;
            k.trim = 1.0;
            const double g = std::abs (response (c, 1000.0, false));
            k.trim = saved;
            if (g > 1.0e-6) k.tTrim = std::clamp (1.0 / g, 0.25, 4.0);
            if (force) k.trim = k.tTrim;
        }
    }

public:
    std::array<Core, 2> core {};
    std::array<Transformer, 2> t1 {}, t2 {}, t1Old {}, t2Old {};
    PowerSupply psu;
    PsuLoads loads, loadSum;
    PsuRails railPrev, railNext, railNow;
    int psuDiv = 4, psuCount = 0;

private:
    static constexpr int kFade = 4096;
    CircuitWorkspace ws;
    DeviceParams prm;
    double fs = 176400.0, kcal = 8.71, gDrive = 1.0, tDrive = 1.0;
    double humOn = 0.0, humOnTarget = 0.0, humPh = 0.0;
    double emission = 1.0, warmTime = 1.0e9;
    int ctrl = 1, fade = 0, trimCountdown = 0;
    double padNom[3] { 0.03, 0.01, 0.005 }, r3Nom[3] { 1.0, 0.5, 0.3 }, hpfLaw[3] { 5.0e-6, 5.0e-6, 5.0e-6 };
    double gain1[3] {}, gain3[3] {};
    double hpfK[2] {};
    double acfNom[2] { 0.985, 0.985 };
    double biasDividerTol[2] { 1.0, 1.0 };
    std::array<std::array<double, 8>, 2> tParts {};
    TriodeParams nominalTube[2][3];
    uint32_t noiseSeed[2] { 1u, 2u };
    double noiseE1[2] { -1.0, -1.0 }, noiseE3[2] { -1.0, -1.0 };
    std::array<double, 6> driftState {};
    Rng driftRng { 12345 };
};

} // namespace vt4
