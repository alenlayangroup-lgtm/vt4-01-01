// Halden VT-4 — plugin-level bench: the real AudioProcessor (oversampling,
// SOLO, bypass, parameters, state), as a host drives it.
//
//   vt4_plugin_bench [quick]

#include <juce_audio_processors/juce_audio_processors.h>
#include "PluginProcessor.h"
#include "PluginEditor.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <random>
#include <thread>
#include <cstdlib>

using namespace juce;

namespace
{
int failures = 0, passes = 0;
bool quick = false;
constexpr double kPi = 3.14159265358979323846;

void verdict (bool ok, const char* what)
{
    std::printf ("  => %s  %s\n\n", ok ? "PASS" : "FAIL", what);
    (ok ? passes : failures)++;
}
void header (const char* t) { std::printf ("---- %s ", t); for (int i = (int) std::strlen (t); i < 70; ++i) std::putchar ('-'); std::printf ("\n"); }
double db (double x) { return 20.0 * std::log10 (std::max (x, 1.0e-300)); }

void setValue (VT4AudioProcessor& p, const String& id, float value)
{
    if (auto* q = p.apvts.getParameter (id))
        q->setValueNotifyingHost (q->convertTo0to1 (value));
}

void musical (VT4AudioProcessor& p)
{
    using namespace vt4p;
    setValue (p, ids::noise, 0.0f);
    setValue (p, ids::hpf (0), (float) posFromHpf (25.0));
    setValue (p, ids::gain (0, 0), 4.0f); setValue (p, ids::weight (0), 5.0f);
    setValue (p, ids::gain (0, 1), -2.0f);
    setValue (p, ids::gain (0, 2), 3.0f);
    setValue (p, ids::gain (0, 3), 2.5f);
}

std::unique_ptr<VT4AudioProcessor> make (double sr, int block)
{
    // offline, like a bounce: the circuit is built inside prepareToPlay
    auto p = std::make_unique<VT4AudioProcessor>();
    p->setNonRealtime (true);
    p->setPlayConfigDetails (2, 2, sr, block);
    p->prepareToPlay (sr, block);
    return p;
}

// runs `blocks` blocks of a generator through the processor; collects channel 0
std::vector<float> run (VT4AudioProcessor& p, int blocks, int block, const std::function<void (int, float&, float&)>& gen,
                        const std::function<void (int)>& perBlock = {})
{
    std::vector<float> out ((size_t) (blocks * block));
    AudioBuffer<float> b (2, block);
    MidiBuffer midi;
    int n = 0;
    for (int k = 0; k < blocks; ++k)
    {
        if (perBlock) perBlock (k);
        for (int i = 0; i < block; ++i, ++n)
        {
            float l = 0.0f, r = 0.0f;
            gen (n, l, r);
            b.setSample (0, i, l); b.setSample (1, i, r);
        }
        p.processBlock (b, midi);
        for (int i = 0; i < block; ++i) out[(size_t) (k * block + i)] = b.getSample (0, i);
    }
    return out;
}

// amplitude of the component at f (Blackman–Harris, exact frequency)
double amplitude (const std::vector<float>& y, size_t start, size_t len, double f, double fs)
{
    double re = 0.0, im = 0.0, ws = 0.0;
    for (size_t n = 0; n < len; ++n)
    {
        const double t = 2.0 * kPi * (double) n / (double) (len - 1);
        const double w = 0.35875 - 0.48829 * std::cos (t) + 0.14128 * std::cos (2 * t) - 0.01168 * std::cos (3 * t);
        const double ph = 2.0 * kPi * f * (double) n / fs;
        re += w * y[start + n] * std::cos (ph);
        im -= w * y[start + n] * std::sin (ph);
        ws += w;
    }
    return 2.0 * std::sqrt (re * re + im * im) / ws;
}
} // namespace

// ============================================================================
static void testBypassAndLatency()
{
    header ("P1. Latency and bypass");
    for (double sr : { 44100.0, 48000.0, 96000.0, 192000.0 })
    {
        auto p = make (sr, 512);
        const int lat = p->getLatencySamples();
        setValue (*p, vt4p::ids::bypass, 1.0f);
        const double a = 0.25;
        auto y = run (*p, 120, 512, [&] (int n, float& l, float& r) { l = r = (float) (a * std::sin (2.0 * kPi * 997.0 * n / sr)); });
        double maxErr = 0.0;
        for (int i = 8192; i < (int) y.size(); ++i)
        {
            const float ref = (float) (a * std::sin (2.0 * kPi * 997.0 * (i - lat) / sr));
            maxErr = std::max (maxErr, (double) std::abs (y[(size_t) i] - ref));
        }
        std::printf ("  %6.0f Hz: latency %d samples (%.3f ms); bypass max error vs aligned input %.3g%s\n", sr, lat, 1000.0 * lat / sr, maxErr, maxErr == 0.0 ? " (bit-exact)" : "");
        verdict (maxErr == 0.0, "bypass is bit-exact and latency-aligned");
    }
}

// ============================================================================
static void testScreenVsPlugin()
{
    header ("P2. Screen curve vs the plugin's output, oversampling included (-40 dBFS)");
    for (double sr : { 44100.0, 96000.0 })
    {
        auto p = make (sr, 512);
        musical (*p);
        // let the parameters glide in
        run (*p, 40, 512, [] (int, float& l, float& r) { l = r = 0.0f; });
        vt4::DisplayModel model;
        model.prepare (p->getOversampledRate(), p->getHostRate(), p->currentParams());
        const auto os = p->oversamplingResponse (model);
        model.setOversamplingResponse (os);
        {
            std::printf ("  %6.0f Hz host, oversampling filters (up + down):", sr);
            for (int i = 0; i < vt4::DisplayModel::kN; ++i)
            {
                const double f = model.freq (i), fn = model.freq (std::min (i + 1, vt4::DisplayModel::kN - 1));
                for (double mark : { 15000.0, 18000.0, 19000.0, 20000.0, 21000.0 })
                    if (f <= mark && fn > mark && mark < 0.5 * sr)
                    {
                        const double t = std::log (mark / f) / std::log (fn / f);
                        std::printf (" %.0f k %+.2f dB", mark / 1000.0, (1.0 - t) * db (os[(size_t) i]) + t * db (os[(size_t) std::min (i + 1, vt4::DisplayModel::kN - 1)]));
                    }
            }
            std::printf ("\n");
        }
        double worst = 0.0, at = 0.0;
        int count = 0;
        const double a = std::pow (10.0, -40.0 / 20.0);
        for (int i = 0; i < vt4::DisplayModel::kN; i += quick ? 16 : 8)
        {
            const double f = model.freq (i);
            if (f < 20.0 || f > 20000.0 || f > 0.45 * sr) continue;
            const int blocks = (int) std::ceil ((f < 60.0 ? 1.0 : 0.45) * sr / 512.0);
            auto y = run (*p, blocks, 512, [&] (int n, float& l, float& r) { l = r = (float) (a * std::sin (2.0 * kPi * f * n / sr)); });
            const size_t len = y.size() / 2;
            const double g = db (amplitude (y, y.size() - len, len, f, sr) / a);
            const double e = std::fabs (g - model.totalDb (0, i));
            if (e > worst) { worst = e; at = f; }
            ++count;
        }
        std::printf ("  %6.0f Hz host: %d frequencies 20 Hz-20 kHz, worst |measured - screen| = %.4f dB (at %.0f Hz)\n", sr, count, worst, at);
        verdict (worst < 0.05, "the screen shows the plugin's real response within 0.05 dB");
    }
}

// ============================================================================
static void testSolo()
{
    header ("P3. SOLO through the plugin: window edges and centre, latency unchanged");
    const double sr = 44100.0;
    auto p = make (sr, 512);
    musical (*p);
    run (*p, 40, 512, [] (int, float& l, float& r) { l = r = 0.0f; });
    vt4::DisplayModel model;
    model.prepare (p->getOversampledRate(), p->getHostRate(), p->currentParams());
    const auto w = model.soloWindow (0, 3);          // PRESENCE
    const int lat0 = p->getLatencySamples();
    auto measure = [&] (double f, bool solo)
    {
        p->soloType.store (w.type); p->soloF1.store ((float) w.f1); p->soloF2.store ((float) w.f2);
        p->auditionSet.store (0); p->auditionBand.store (3);
        p->auditionMode.store (solo ? 1 : 0);
        const double a = 0.05;
        auto y = run (*p, (int) (0.5 * sr / 512), 512, [&] (int n, float& l, float& r) { l = r = (float) (a * std::sin (2.0 * kPi * f * n / sr)); });
        const size_t len = y.size() / 2;
        return amplitude (y, y.size() - len, len, f, sr);
    };
    const double fc = sr / kPi * std::atan (std::sqrt (std::tan (kPi * w.f1 / sr) * std::tan (kPi * w.f2 / sr)));
    std::printf ("  PRESENCE window: %.1f-%.1f Hz (centre %.1f Hz)\n", w.f1, w.f2, fc);
    double worst = 0.0;
    for (auto pr : { std::make_pair (w.f1, -3.0103), std::make_pair (w.f2, -3.0103), std::make_pair (fc, 0.0) })
    {
        const double ratio = db (measure (pr.first, true) / measure (pr.first, false));
        worst = std::max (worst, std::fabs (ratio - pr.second));
        std::printf ("  %8.1f Hz: solo/normal %+.4f dB (expected %+.4f)\n", pr.first, ratio, pr.second);
    }
    p->auditionMode.store (0);
    const int lat1 = p->getLatencySamples();
    std::printf ("  latency before %d, during SOLO %d samples\n", lat0, lat1);
    verdict (worst < 0.01 && lat0 == lat1, "SOLO plays exactly the window, latency constant");

    // DELTA of a flat band through the plugin: silence
    setValue (*p, vt4p::ids::gain (0, 1), 0.0f);
    run (*p, 20, 512, [] (int, float& l, float& r) { l = r = 0.0f; });
    p->auditionBand.store (2); p->auditionMode.store (2);
    std::mt19937 g (3); std::normal_distribution<float> nd (0.0f, 0.05f);
    auto y = run (*p, 60, 512, [&] (int, float& l, float& r) { l = r = nd (g); });
    float mx = 0.0f;
    for (size_t i = 8192; i < y.size(); ++i) mx = std::max (mx, std::abs (y[i]));
    p->auditionMode.store (0);
    std::printf ("  DELTA on a flat band: peak %.3g\n", mx);
    verdict (mx == 0.0f, "DELTA of a flat band is silent through the whole plugin");
}

// ============================================================================
static void testCpuAndTorture()
{
    header ("P4. CPU in real time, and a torture run with host automation");
    for (double sr : { 44100.0, 96000.0 })
    {
        auto p = make (sr, 512);
        musical (*p);
        setValue (*p, vt4p::ids::noise, 1.0f);
        // pink noise, band-limited by the host rate, about -14 dBFS RMS
        std::mt19937 g (1); std::normal_distribution<double> nd (0.0, 1.0);
        double b[7] {};
        auto pink = [&] (int, float& l, float& r)
        {
            const double w = nd (g);
            b[0] = 0.99886 * b[0] + w * 0.0555179; b[1] = 0.99332 * b[1] + w * 0.0750759; b[2] = 0.96900 * b[2] + w * 0.1538520;
            b[3] = 0.86650 * b[3] + w * 0.3104856; b[4] = 0.55000 * b[4] + w * 0.5329522; b[5] = -0.7616 * b[5] - w * 0.0168980;
            const double y = (b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6] + w * 0.5362) * 0.045;
            b[6] = w * 0.115926;
            l = (float) y; r = (float) (0.8 * y);
        };
        const double secs = quick ? 3.0 : 8.0;
        const int blocks = (int) (secs * sr / 512);
        const auto t0 = std::chrono::steady_clock::now();
        run (*p, blocks, 512, pink);
        const double el = std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();
        std::printf ("  %6.0f Hz: %.1f %% of one core (pink noise, musical setting, NOISE on)\n", sr, 100.0 * el / secs);

        // torture: random host automation of every parameter, audition toggles
        std::mt19937 rng (7);
        std::uniform_real_distribution<float> U (0.0f, 1.0f);
        auto& params = p->getParameters();
        bool bad = false;
        float peak = 0.0f;
        AudioBuffer<float> buf (2, 512);
        MidiBuffer midi;
        double ph = 0.0;
        const int tb = (int) ((quick ? 4.0 : 10.0) * sr / 512);
        for (int k = 0; k < tb; ++k)
        {
            for (int c = 0; c < 3; ++c)
            {
                auto* q = params[(int) (U (rng) * (float) params.size()) % params.size()];
                if (q == p->getBypassParameter()) continue;
                q->setValueNotifyingHost (U (rng));
            }
            if (U (rng) < 0.05f) { p->auditionMode.store ((int) (U (rng) * 3.0f)); p->auditionBand.store ((int) (U (rng) * 5.0f)); p->auditionSet.store (U (rng) < 0.5f ? 0 : 1); }
            const int bsz = 1 + (int) (U (rng) * 511.0f);            // hosts vary the block size
            buf.setSize (2, bsz, false, false, true);
            const float amp = U (rng) < 0.1f ? 1.0f : 0.3f;
            for (int i = 0; i < bsz; ++i)
            {
                const float v = (float) (amp * std::sin (ph));
                ph += 2.0 * kPi * (30.0 + 3000.0 * U (rng) * U (rng)) / sr;
                buf.setSample (0, i, v); buf.setSample (1, i, -v);
            }
            p->processBlock (buf, midi);
            for (int c = 0; c < 2; ++c)
                for (int i = 0; i < bsz; ++i)
                {
                    const float v = buf.getSample (c, i);
                    if (! std::isfinite (v)) bad = true;
                    peak = std::max (peak, std::abs (v));
                }
        }
        p->auditionMode.store (0);
        std::printf ("  %6.0f Hz torture: %d blocks of random size, peak %.2f, NaN/Inf: %s\n", sr, tb, peak, bad ? "YES" : "none");
        verdict (! bad && peak < 8.0f, "stable under host automation, any block size");
    }
}

// ============================================================================
static void testState()
{
    header ("P5. State save / restore");
    auto a = make (48000.0, 256);
    std::mt19937 rng (11);
    std::uniform_real_distribution<float> U (0.0f, 1.0f);
    for (auto* q : a->getParameters()) q->setValueNotifyingHost (U (rng));
    MemoryBlock mb;
    a->getStateInformation (mb);
    auto b = make (48000.0, 256);
    b->setStateInformation (mb.getData(), (int) mb.getSize());
    // compare what each parameter means (a bool set to 0.66 is "on", and is stored as 1)
    float worst = 0.0f;
    for (int i = 0; i < a->getParameters().size(); ++i)
    {
        auto* qa = dynamic_cast<RangedAudioParameter*> (a->getParameters()[i]);
        auto* qb = dynamic_cast<RangedAudioParameter*> (b->getParameters()[i]);
        if (qa == nullptr || qb == nullptr) continue;
        const float va = qa->convertFrom0to1 (qa->getValue()), vb = qb->convertFrom0to1 (qb->getValue());
        const float d = std::abs (va - vb);
        if (d > 0.0f)
            std::printf ("    %s: %.6f -> %.6f\n", qa->getName (64).toRawUTF8(), va, vb);
        worst = std::max (worst, d);
    }
    std::printf ("  %d parameters, %d bytes, worst difference after restore %.3g\n", a->getParameters().size(), (int) mb.getSize(), worst);
    verdict (worst == 0.0f, "every parameter survives a save / restore");
}

// ============================================================================
static void testActivation()
{
    header ("P6. Host activation: prepareToPlay never makes the host wait");
    using clock = std::chrono::steady_clock;
    auto ms = [] (clock::time_point a) { return std::chrono::duration<double, std::milli> (clock::now() - a).count(); };
    const double sr = 48000.0;
    const int block = 256;
    const double a = 0.25;
    auto sine = [&] (int n, float& l, float& r) { l = r = (float) (a * std::sin (2.0 * kPi * 440.0 * n / sr)); };

    // 1. a fresh instance in real time: the circuit is built in the background
    auto p = std::make_unique<VT4AudioProcessor>();
    p->setNonRealtime (false);
    p->setPlayConfigDetails (2, 2, sr, block);
    musical (*p);
    auto t0 = clock::now();
    p->prepareToPlay (sr, block);
    const double tFirst = ms (t0);
    const int lat = p->getLatencySamples();
    int blocksDry = 0, n = 0;
    double dryErr = 0.0;
    AudioBuffer<float> b (2, block);
    MidiBuffer midi;
    t0 = clock::now();
    while (! p->circuitRunning() && ms (t0) < 5000.0)
    {
        for (int i = 0; i < block; ++i) { float l, r; sine (n + i, l, r); b.setSample (0, i, l); b.setSample (1, i, r); }
        p->processBlock (b, midi);
        for (int i = 0; i < block && ! p->circuitRunning(); ++i)
            if (n + i >= lat + 64)
                dryErr = std::max (dryErr, (double) std::abs (b.getSample (0, i) - (float) (a * std::sin (2.0 * kPi * 440.0 * (n + i - lat) / sr))));
        n += block;
        ++blocksDry;
        std::this_thread::sleep_for (std::chrono::milliseconds (5));     // the audio thread's pace
    }
    const double tBuild = ms (t0);
    std::printf ("  first prepareToPlay (real time): %.2f ms; circuit running after %.0f ms (%d blocks passed the input, max error vs aligned input %.3g)\n",
                 tFirst, tBuild, blocksDry, dryErr);
    verdict (tFirst < 20.0 && p->circuitRunning() && dryErr == 0.0, "the first activation returns at once; the input passes until the circuit is ready");

    // 2. deactivate / reactivate at the same rate, knobs moved in between
    run (*p, 40, block, sine);
    setValue (*p, vt4p::ids::gain (0, 2), -4.0f);
    setValue (*p, vt4p::ids::drive, 3.0f);
    p->releaseResources();
    t0 = clock::now();
    p->prepareToPlay (sr, block);
    const double tRe = ms (t0);
    const bool readyAtOnce = p->circuitRunning();
    // reference: an instance built from scratch with the same settings
    auto ref = std::make_unique<VT4AudioProcessor>();
    ref->setNonRealtime (true);
    ref->setPlayConfigDetails (2, 2, sr, block);
    MemoryBlock mb; p->getStateInformation (mb); ref->setStateInformation (mb.getData(), (int) mb.getSize());
    ref->prepareToPlay (sr, block);
    auto yA = run (*p, 200, block, sine);
    auto yB = run (*ref, 200, block, sine);
    double diff = 0.0, pk = 0.0;
    for (size_t i = yA.size() / 2; i < yA.size(); ++i) { diff = std::max (diff, (double) std::abs (yA[i] - yB[i])); pk = std::max (pk, (double) std::abs (yB[i])); }
    std::printf ("  reactivation: %.2f ms, running at once: %s; vs a unit built from scratch: max difference %.1f dB below the signal\n",
                 tRe, readyAtOnce ? "yes" : "no", db (pk) - db (diff));
    verdict (tRe < 20.0 && readyAtOnce && db (pk) - db (diff) > 90.0, "disable / enable in the host is instant and sounds the same as a fresh unit");

    // 3. a new sample rate: again in the background
    p->releaseResources();
    t0 = clock::now();
    p->prepareToPlay (96000.0, block);
    const double tRate = ms (t0);
    std::printf ("  change to 96 kHz: prepareToPlay %.2f ms\n", tRate);
    verdict (tRate < 20.0, "a sample-rate change does not block the host either");
}

// ============================================================================
static void testBypassRest()
{
    header ("P7. Bypassed: the circuit rests; switched back in it fades in cleanly");
    const double sr = 44100.0;
    auto p = make (sr, 512);
    musical (*p);
    long long t = 0;      // continuous across runs
    auto sine = [&] (int, float& l, float& r) { l = r = (float) (0.3 * std::sin (2.0 * kPi * 220.0 * (double) (t++) / sr)); };
    const int blocks = (int) (2.0 * sr / 512);
    auto t0 = std::chrono::steady_clock::now();
    run (*p, blocks, 512, sine);
    const double tOn = std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();
    setValue (*p, vt4p::ids::bypass, 1.0f);
    run (*p, 40, 512, sine);
    t0 = std::chrono::steady_clock::now();
    run (*p, blocks, 512, sine);
    const double tOff = std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();
    setValue (*p, vt4p::ids::bypass, 0.0f);
    auto y = run (*p, 200, 512, sine);
    double jump = 0.0, pk = 0.0;
    bool finite = true;
    size_t at = 0;
    for (size_t i = 1; i < y.size(); ++i)
    {
        finite = finite && std::isfinite (y[i]);
        if (std::abs (y[i] - y[i - 1]) > jump) at = i;
        jump = std::max (jump, (double) std::abs (y[i] - y[i - 1]));
        pk = std::max (pk, (double) std::abs (y[i]));
    }
    // a 220 Hz sine of peak A moves at most 2πfA/fs per sample
    const double maxSlope = 2.0 * kPi * 220.0 * pk / sr;
    if (std::getenv ("VT4_DEBUG")) for (size_t i = at > 6 ? at - 6 : 0; i < at + 6 && i < y.size(); ++i) std::printf ("    y[%zu] = %.5f\n", i, y[i]);
    std::printf ("  CPU active %.1f %%, bypassed %.1f %% of one core; back in: largest step %.4f (sine slope %.4f)\n",
                 100.0 * tOn / 2.0, 100.0 * tOff / 2.0, jump, maxSlope);
    verdict (tOff < 0.2 * tOn && finite && jump < 1.5 * maxSlope, "bypassed costs next to nothing; re-engaging is click-free");
}

// ============================================================================
static void testEditorResponsive()
{
    header ("P8. Editor: the message thread never stalls (screen model built in the background)");
    auto p = make (44100.0, 512);
    musical (*p);
    std::unique_ptr<AudioProcessorEditor> ed (p->createEditor());
    auto* mm = MessageManager::getInstance();
    using clock = std::chrono::steady_clock;
    double worst = 0.0;
    auto pump = [&] (double seconds)
    {
        const auto end = clock::now() + std::chrono::duration<double> (seconds);
        while (clock::now() < end)
        {
            const auto t = clock::now();
            mm->runDispatchLoopUntil (5);
            worst = std::max (worst, std::chrono::duration<double, std::milli> (clock::now() - t).count() - 5.0);
        }
    };
    pump (1.0);
    // turn TOLERANCE and SERIAL like a hand would, with audio running
    AudioBuffer<float> b (2, 512);
    MidiBuffer midi;
    for (int k = 0; k < 40; ++k)
    {
        setValue (*p, vt4p::ids::tolerance, 50.0f + 2.0f * (float) k);
        if (k % 10 == 0) setValue (*p, vt4p::ids::serial, (float) (1 + k));
        b.clear();
        p->processBlock (b, midi);
        pump (0.025);
    }
    pump (1.0);
    const bool current = dynamic_cast<VT4AudioProcessorEditor*> (ed.get())->screenModelCurrent();
    ed.reset();
    std::printf ("  longest message-thread stall while the unit changes: %.1f ms; screen follows the final unit: %s\n", worst, current ? "yes" : "no");
    verdict (worst < 30.0 && current, "the editor stays responsive (no 0.1 s rebuilds on the message thread)");
}

int main (int argc, char** argv)
{
    std::setvbuf (stdout, nullptr, _IONBF, 0);
    quick = argc > 1 && std::strcmp (argv[1], "quick") == 0;
    ScopedJuceInitialiser_GUI gui;
    std::printf ("Halden VT-4 plugin bench%s\n\n", quick ? " (quick)" : "");
    const auto t0 = std::chrono::steady_clock::now();
    testBypassAndLatency();
    testScreenVsPlugin();
    testSolo();
    testCpuAndTorture();
    testState();
    testActivation();
    testBypassRest();
    testEditorResponsive();
    std::printf ("==== %d passed, %d failed (%.0f s) ====\n", passes, failures, std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count());
    return failures;
}
