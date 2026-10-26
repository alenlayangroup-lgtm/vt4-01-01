// Halden VT-4 — the audio engine around the device: DELTA (a second, cloned
// device) at the oversampled rate and SOLO windows at the host rate.
//
// DELTA:  yΔ = y(θ) − y(θ₀). On engage the second device is an exact copy of
//         the running one (tubes, cores, capacitors, supply, noise
//         generators), then the auditioned band glides to flat inside the
//         copy, so the difference grows from zero without a click. The
//         output is y − w·y₀ with w(t) = sin²(πt / 2T), T = 30 ms.
// SOLO:   out = (1 − w)·y + w·W(y), W = listening window on the band's
//         channel (L / R, or M / S in mid-side mode). A new window starts
//         warmed up on the last 0.5 s of signal, so it begins in steady state.

#pragma once

#include "Audition.h"
#include "Device.h"

#include <array>
#include <atomic>
#include <chrono>
#include <new>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace vt4
{

enum class Audition : int { kNone = 0, kSolo = 1, kDelta = 2 };

struct AuditionRequest
{
    Audition mode = Audition::kNone;
    int set = 0;               // 0 = channel A (L / M), 1 = channel B (R / S)
    int band = 0;              // 0 HPF, 1 LOW, 2 BODY, 3 PRESENCE, 4 AIR
    SoloWindow window;         // for SOLO
};

// Parameters with one band of one channel set brought to flat.
inline DeviceParams flattenBand (DeviceParams p, int set, int band)
{
    const int s = p.stereo == kLink ? 0 : set;
    auto& ch = p.ch[(size_t) s];
    if (band == 0) ch.hpfHz = 0.0;
    else
    {
        auto& b = ch.band[(size_t) (band - 1)];
        b.gainDb = 0.0;
        b.weight = 0.0;
    }
    return p;
}

inline double sin2Fade (double t) noexcept   // t in [0, 1]
{
    const double s = std::sin (0.5 * 3.14159265358979323846 * std::clamp (t, 0.0, 1.0));
    return s * s;
}

// --------------------------------------------------------------------------
// Builds a new device off the audio thread (a different unit: serial,
// tolerance or tube matching changes every part, the operating points and
// the supply, which takes ~0.1 s of computation; also the first build after
// the host prepares a new sample rate).
//
// Every product carries a second, untouched copy of the freshly built
// device ("pristine"): when the host deactivates and reactivates the plugin
// the engine restarts from that copy in microseconds instead of rebuilding.
//
// Requests from knob movements are debounced: while a knob keeps turning
// only the last position is built, so a drag does not queue up builds.
class DeviceBuilder
{
public:
    struct Product
    {
        std::unique_ptr<Device> dev, pristine;
    };

    DeviceBuilder() : worker ([this] { run(); }) {}
    ~DeviceBuilder()
    {
        { std::lock_guard<std::mutex> l (m); quit = true; }
        cv.notify_all();
        worker.join();
        delete ready.exchange (nullptr);
        emptyTrash();
    }

    // immediate: build at once (start-up); otherwise wait for the knob to rest
    void request (double fs, const DeviceParams& p, bool immediate = false)
    {
        { std::lock_guard<std::mutex> l (m); want = p; wantFs = fs; pending = true; wantNow = wantNow || immediate; ++wantSeq; }
        cv.notify_all();
    }
    // audio thread: a finished build, or nullptr
    Product* take() noexcept { return ready.exchange (nullptr); }
    // audio thread: hand back objects for deletion off the audio thread
    void retire (Product* p) noexcept
    {
        if (p == nullptr) return;
        for (auto& t : trash)
        {
            Product* expected = nullptr;
            if (t.compare_exchange_strong (expected, p)) return;
        }
        delete p;   // every slot full (cannot happen in practice)
    }
    void retire (std::unique_ptr<Device> d) noexcept
    {
        if (d == nullptr) return;
        Device* raw = d.release();
        for (auto& t : trashDev)
        {
            Device* expected = nullptr;
            if (t.compare_exchange_strong (expected, raw)) return;
        }
        delete raw;   // every slot full (cannot happen in practice)
    }
    bool busy() const noexcept { return building.load() || pendingFlag.load(); }

private:
    void emptyTrash()
    {
        for (auto& t : trash) delete t.exchange (nullptr);
        for (auto& t : trashDev) delete t.exchange (nullptr);
    }

    void run()
    {
        for (;;)
        {
            DeviceParams p; double fs = 0.0;
            {
                std::unique_lock<std::mutex> l (m);
                // retired devices are collected every quarter second
                if (! cv.wait_for (l, std::chrono::milliseconds (250), [this] { return quit || pending; }))
                {
                    l.unlock();
                    emptyTrash();
                    continue;
                }
                if (quit) return;
                pendingFlag.store (true);
                seenSeq = wantSeq;
                // debounce: while newer requests keep arriving, take the latest
                while (! wantNow && ! quit
                       && cv.wait_for (l, std::chrono::milliseconds (90), [this] { return quit || (pending && wantSeq != seenSeq); }))
                    seenSeq = wantSeq;
                if (quit) return;
                p = want; fs = wantFs; pending = false; wantNow = false;
                building.store (true);
                pendingFlag.store (false);
            }
            emptyTrash();
            auto* prod = new Product();
            prod->dev = std::make_unique<Device>();
            prod->dev->prepare (fs, p);
            prod->pristine = std::make_unique<Device> (*prod->dev);
            delete ready.exchange (prod);       // a newer build replaces an unused one
            building.store (false);
        }
    }

    std::mutex m;
    std::condition_variable cv;
    DeviceParams want;
    double wantFs = 0.0;
    bool pending = false, quit = false, wantNow = false;
    unsigned wantSeq = 0, seenSeq = 0;
    std::atomic<bool> building { false }, pendingFlag { false };
    std::atomic<Product*> ready { nullptr };
    std::array<std::atomic<Product*>, 8> trash {};
    std::array<std::atomic<Device*>, 8> trashDev {};
    std::thread worker;
};

// Oversampled part: main device plus the DELTA copy.
class DeltaEngine
{
public:
    // Prepares for a sample rate. Three ways, fastest first:
    //   1. the pristine copy of the last build fits (same rate, same unit and
    //      operating point): copy it and set the knobs (microseconds);
    //   2. synchronous: build here (offline rendering, tests);
    //   3. otherwise the build runs on the builder thread and ready() turns
    //      true when it is done (the host's thread never waits).
    void prepare (double oversampledRate, const DeviceParams& p, bool synchronous = true)
    {
        fs = oversampledRate;
        fadeLen = std::max (1, (int) std::lround (0.030 * fs));
        state = kOff; fadePos = 0;
        swapPos = -1;
        if (incoming) builder.retire (std::move (incoming));
        if (! alt) alt = std::make_unique<Device>();
        params = p;
        unit = p;
        // results of earlier requests are of no use now
        while (auto* d = builder.take())
        {
            if (fitsPristine (d->pristine.get(), p) && ! fitsPristine (pristine.get(), p))
                std::swap (pristine, d->pristine);
            builder.retire (d);
        }

        if (fitsPristine (pristine.get(), p))
        {
            if (! main) main = std::make_unique<Device>();
            *main = *pristine;
            main->setParams (p);
            main->jumpToTargets();
            return;
        }
        if (synchronous)
        {
            main = std::make_unique<Device>();
            main->prepare (fs, p);
            pristine = std::make_unique<Device> (*main);
            return;
        }
        main.reset();
        builder.request (fs, p, true);
    }

    // Offline: wait for a pending build (the host allows the wait).
    bool waitUntilReady (int timeoutMs)
    {
        for (int t = 0; t < timeoutMs && ! main; ++t)
        {
            poll();
            if (! main) std::this_thread::sleep_for (std::chrono::milliseconds (1));
        }
        return main != nullptr;
    }

    bool ready() const noexcept { return main != nullptr; }

    // After a pause (plugin bypassed, circuit not computed): finish a pending
    // unit change, put every stage back at its operating point.
    void restartAtRest() noexcept
    {
        if (! main) return;
        if (swapPos >= 0)
        {
            builder.retire (std::move (main));
            main = std::move (incoming);
            swapPos = -1;
        }
        state = kOff; fadePos = 0;
        main->jumpToTargets();
    }

    static bool sameUnit (const DeviceParams& a, const DeviceParams& b)
    {
        return a.serial == b.serial && std::fabs (a.tolerance - b.tolerance) < 1.0e-9 && a.matched == b.matched;
    }

    void setParams (const DeviceParams& p)
    {
        params = p;
        if (! main)
        {
            if (! sameUnit (p, unit)) { unit = p; builder.request (fs, p, true); }
            return;
        }
        if (! sameUnit (p, unit))
        {
            // keep the running unit (with everything else updated) until the new one is built
            DeviceParams q = p; q.serial = unit.serial; q.tolerance = unit.tolerance; q.matched = unit.matched;
            main->setParams (q);
            if (state != kOff) alt->setParams (flattenBand (q, set, band));
            unit = p;
            DeviceParams b = p; b.warmup = false;      // a new unit arrives warm
            builder.request (fs, b);
            return;
        }
        main->setParams (p);
        if (incoming) incoming->setParams (p);
        if (state != kOff) alt->setParams (flattenBand (p, set, band));
    }

    // Request DELTA on/off for a band. Safe to call every block.
    void setDelta (bool on, int newSet, int newBand)
    {
        if (! main) return;
        if (on)
        {
            if (state == kOff)
            {
                *alt = *main;                       // exact copy of every state
                set = newSet; band = newBand;
                alt->setParams (flattenBand (main->params(), set, band));
                state = kIn; fadePos = 0;
            }
            else
            {
                if (newSet != set || newBand != band)
                {
                    set = newSet; band = newBand;
                    alt->setParams (flattenBand (main->params(), set, band));
                }
                if (state == kOut) { state = kIn; fadePos = fadeLen - fadePos; }
            }
        }
        else if (state == kIn || state == kOn)
        {
            fadePos = state == kIn ? fadeLen - fadePos : 0;
            state = kOut;
        }
    }

    // called once per block by the audio thread
    void poll()
    {
        if (swapPos >= 0) return;
        DeviceBuilder::Product* d = builder.take();
        if (d == nullptr) return;
        if (d->dev->sampleRate() != fs || ! sameUnit (d->dev->params(), params))
        {
            builder.retire (d);                    // a newer build is on its way
            return;
        }
        DeviceParams q = params; q.warmup = d->dev->params().warmup;
        std::swap (pristine, d->pristine);
        if (! main)
        {
            // first build after prepare: start right away
            main = std::move (d->dev);
            main->setParams (q);
            builder.retire (d);
            return;
        }
        incoming = std::move (d->dev);
        builder.retire (d);                        // carries the previous pristine copy away
        incoming->setParams (q);
        swapPos = 0;
    }

    inline void process (double inL, double inR, double& outL, double& outR) noexcept
    {
        main->process (inL, inR, outL, outR);
        if (swapPos >= 0)
        {
            // cross-fade to the newly built unit
            double nl, nr;
            incoming->process (inL, inR, nl, nr);
            const double w = sin2Fade ((double) swapPos / fadeLen);
            outL = (1.0 - w) * outL + w * nl;
            outR = (1.0 - w) * outR + w * nr;
            if (++swapPos >= fadeLen)
            {
                builder.retire (std::move (main));
                main = std::move (incoming);
                swapPos = -1;
                if (state != kOff) { *alt = *main; alt->setParams (flattenBand (main->params(), set, band)); }
            }
        }
        if (state == kOff) return;
        double aL, aR;
        alt->process (inL, inR, aL, aR);
        double w = 1.0;
        if (state == kIn)
        {
            w = sin2Fade ((double) fadePos / fadeLen);
            if (++fadePos >= fadeLen) state = kOn;
        }
        else if (state == kOut)
        {
            w = 1.0 - sin2Fade ((double) fadePos / fadeLen);
            if (++fadePos >= fadeLen) { state = kOff; return; }
        }
        outL -= w * aL;
        outR -= w * aR;
    }

    bool deltaRunning() const noexcept { return state != kOff; }
    Device& device() noexcept { return *main; }
    const Device& device() const noexcept { return *main; }

private:
    // can the pristine device stand in for a fresh build with these parameters?
    // Same rate, same unit and the same operating point (the knobs that set
    // the bias and the supply); everything else is set by setParams.
    bool fitsPristine (const Device* d, const DeviceParams& p) const
    {
        if (d == nullptr || d->sampleRate() != fs) return false;
        const DeviceParams& q = d->params();
        return sameUnit (q, p) && q.mode == p.mode && q.rectifier == p.rectifier && q.mainsHz == p.mainsHz
            && q.warmup == p.warmup && q.calDbu == p.calDbu;
    }

    enum State { kOff, kIn, kOn, kOut };
    std::unique_ptr<Device> main, alt, incoming, pristine;
    DeviceParams params, unit;
    DeviceBuilder builder;
    double fs = 176400.0;
    int state = kOff, fadePos = 0, fadeLen = 1, swapPos = -1;
    int set = 0, band = 0;
};

// --------------------------------------------------------------------------
// Host-rate part: SOLO listening windows.
class SoloEngine
{
public:
    void prepare (double sampleRate)
    {
        fs = sampleRate;
        histLen = std::max (1024, (int) (0.5 * fs));
        hist[0].assign ((size_t) histLen, 0.0);
        hist[1].assign ((size_t) histLen, 0.0);
        histPos = 0;
        fadeLen = std::max (1, (int) std::lround (0.030 * fs));
        for (auto& inst : inst2) for (auto& f : inst.f) f.prepare (fs);
        active = 0; xfadePos = -1; soloW = 0.0; soloPhase = 0.0; soloOn = false;
        have = false;
    }

    // stereo: kLink / kDual / kMidSide
    void setRequest (bool on, int set, int band, int stereo, const SoloWindow& w)
    {
        soloOn = on;
        if (! on) return;
        const Route r { set, band, stereo };
        Instance& a = inst2[(size_t) active];
        if (have && a.route == r && w.type == a.win.type)
        {
            goal = w;                                   // same band: glide the edges
            return;
        }
        // new window: start the idle instance warmed up on the history
        Instance& n = inst2[(size_t) (active ^ 1)];
        n.win = w; n.route = r; goal = w;
        for (auto& f : n.f) { f.design (w); f.reset(); }
        warm (n);
        if (have && soloW > 0.0) xfadePos = 0;          // cross-fade between windows
        else { active ^= 1; xfadePos = -1; }
        have = true;
    }

    void process (float* L, float* R, int n) noexcept
    {
        const double step = 1.0 / fadeLen;
        for (int i = 0; i < n; ++i)
        {
            const double l = L[i], r = R[i];
            hist[0][(size_t) histPos] = l; hist[1][(size_t) histPos] = r;
            if (++histPos >= histLen) histPos = 0;

            // solo on / off: w = sin²(π/2 · phase)
            if (soloOn && soloPhase < 1.0)        soloPhase = std::min (1.0, soloPhase + step);
            else if (! soloOn && soloPhase > 0.0) soloPhase = std::max (0.0, soloPhase - step);
            soloW = sin2Fade (soloPhase);
            if (! have) continue;
            if (soloPhase <= 0.0 && ! soloOn) { have = false; xfadePos = -1; continue; }   // fully off: forget

            if ((i & 15) == 0) glide();

            double sl, sr;
            run (inst2[(size_t) active], l, r, sl, sr);
            if (xfadePos >= 0)
            {
                double nl, nr;
                run (inst2[(size_t) (active ^ 1)], l, r, nl, nr);
                const double w = sin2Fade ((double) xfadePos / fadeLen);
                sl = (1.0 - w) * sl + w * nl; sr = (1.0 - w) * sr + w * nr;
                if (++xfadePos >= fadeLen) { active ^= 1; xfadePos = -1; }
            }
            L[i] = (float) ((1.0 - soloW) * l + soloW * sl);
            R[i] = (float) ((1.0 - soloW) * r + soloW * sr);
        }
    }

    bool soloing() const noexcept { return soloOn || soloPhase > 0.0; }

    // exact response of the active window (tests, display)
    std::complex<double> windowResponse (double f) const { return inst2[(size_t) active].f[0].response (f); }

private:
    struct Route
    {
        int set = 0, band = 0, stereo = 0;
        bool operator== (const Route& o) const noexcept { return set == o.set && band == o.band && stereo == o.stereo; }
    };
    struct Instance { std::array<WindowFilter, 2> f {}; SoloWindow win; Route route; };

    // signal routing: what is filtered, and how it goes back to L / R
    static inline void run (Instance& in, double l, double r, double& ol, double& orr) noexcept
    {
        if (in.route.stereo == kMidSide)
        {
            const double m = 0.5 * (l + r), s = 0.5 * (l - r);
            if (in.route.set == 0) { const double y = in.f[0].process (m); ol = y; orr = y; }
            else                   { const double y = in.f[0].process (s); ol = y; orr = -y; }
        }
        else if (in.route.stereo == kLink)
        {
            ol = in.f[0].process (l); orr = in.f[1].process (r);
        }
        else
        {
            if (in.route.set == 0) { ol = in.f[0].process (l); orr = 0.0; }
            else                   { ol = 0.0; orr = in.f[1].process (r); }
        }
    }

    void warm (Instance& in)
    {
        double a, b;
        for (int k = 0; k < histLen; ++k)
        {
            const size_t idx = (size_t) ((histPos + k) % histLen);
            run (in, hist[0][idx], hist[1][idx], a, b);
        }
    }

    void glide() noexcept
    {
        Instance& a = inst2[(size_t) active];
        if (a.win.type != goal.type) return;
        if (a.win.f1 == goal.f1 && a.win.f2 == goal.f2) return;
        const double k = 1.0 - std::exp (-16.0 / (0.020 * fs));
        auto mv = [&] (double& x, double t)
        {
            x = std::exp (std::log (x) + k * (std::log (t) - std::log (x)));
            if (std::fabs (std::log (x / t)) < 1.0e-6) x = t;
        };
        mv (a.win.f1, goal.f1); mv (a.win.f2, goal.f2);
        for (auto& f : a.f) f.design (a.win);
    }

    double fs = 48000.0;
    std::array<std::vector<double>, 2> hist;
    int histLen = 1024, histPos = 0, fadeLen = 1;
    std::array<Instance, 2> inst2 {};
    int active = 0, xfadePos = -1;
    double soloW = 0.0, soloPhase = 0.0;
    bool soloOn = false, have = false;
    SoloWindow goal;
};

} // namespace vt4
