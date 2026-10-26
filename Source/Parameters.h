#pragma once

#include <juce_audio_processors/juce_audio_processors.h>
#include "dsp/Device.h"

#include <atomic>
#include <memory>

namespace vt4p
{
using vt4::kNumBands;

// ---- identifiers ------------------------------------------------------------
namespace ids
{
    inline const juce::String mode      { "mode" };
    inline const juce::String drive     { "drive" };
    inline const juce::String output    { "output" };
    inline const juce::String stepped   { "stepped" };
    inline const juce::String stereo    { "stereo" };
    inline const juce::String iron      { "iron" };
    inline const juce::String bypass    { "bypass" };

    inline const juce::String mains     { "mains" };
    inline const juce::String heaters   { "heaters" };
    inline const juce::String rectifier { "rectifier" };
    inline const juce::String warmup    { "warmup" };
    inline const juce::String tolerance { "tolerance" };
    inline const juce::String serial    { "serial" };
    inline const juce::String tubes     { "tubes" };
    inline const juce::String noise     { "noise" };
    inline const juce::String source    { "source" };

    // per channel set: suffix "_a" (L / M) or "_b" (R / S)
    inline juce::String hpf    (int s) { return s ? "hpf_b"    : "hpf_a"; }
    inline juce::String freq   (int s, int band)
    {
        static const char* n[4] = { "low_f", "body_f", "pres_f", "air_f" };
        return juce::String (n[band]) + (s ? "_b" : "_a");
    }
    inline juce::String gain   (int s, int band)
    {
        static const char* n[4] = { "low_g", "body_g", "pres_g", "air_g" };
        return juce::String (n[band]) + (s ? "_b" : "_a");
    }
    inline juce::String weight (int s) { return s ? "weight_b" : "weight_a"; }
    inline juce::String focus  (int s) { return s ? "focus_b"  : "focus_a"; }
}

// ---- value mapping ------------------------------------------------------------
// Frequency knobs run over the switch positions of the band (evenly spaced
// detents); between detents (SMOOTH) the frequency is interpolated in log f.
inline int numSteps (int band) { return vt4::eq::kNumSteps[band]; }

inline double freqFromPos (int band, double v, bool stepped)
{
    const int n = numSteps (band);
    double p = juce::jlimit (0.0, 1.0, v) * (n - 1);
    if (stepped) p = std::round (p);
    const int i = juce::jlimit (0, n - 2, (int) std::floor (p));
    const double t = p - i;
    const double a = std::log (vt4::eq::kSteps[band][i]), b = std::log (vt4::eq::kSteps[band][i + 1]);
    return std::exp (a + t * (b - a));
}

inline double posFromFreq (int band, double f)
{
    const int n = numSteps (band);
    const double lf = std::log (juce::jlimit (vt4::eq::kSteps[band][0], vt4::eq::kSteps[band][n - 1], f));
    for (int i = 0; i < n - 1; ++i)
    {
        const double a = std::log (vt4::eq::kSteps[band][i]), b = std::log (vt4::eq::kSteps[band][i + 1]);
        if (lf <= b + 1.0e-12) return (i + (lf - a) / (b - a)) / (n - 1);
    }
    return 1.0;
}

// High-pass: 7 positions OFF, 18, 25, 35, 50, 70, 100 Hz. Returns 0 for OFF.
inline double hpfFromPos (double v, bool stepped)
{
    double p = juce::jlimit (0.0, 1.0, v) * 6.0;
    if (stepped) p = std::round (p);
    if (p < 0.5) return 0.0;
    p = juce::jlimit (1.0, 6.0, p) - 1.0;
    const int i = juce::jlimit (0, 4, (int) std::floor (p));
    const double t = p - i;
    const double a = std::log (vt4::eq::kHpfSteps[i]), b = std::log (vt4::eq::kHpfSteps[i + 1]);
    return std::exp (a + t * (b - a));
}

inline double posFromHpf (double f)
{
    if (f <= 0.0) return 0.0;
    const double lf = std::log (juce::jlimit (18.0, 100.0, f));
    for (int i = 0; i < 5; ++i)
    {
        const double a = std::log (vt4::eq::kHpfSteps[i]), b = std::log (vt4::eq::kHpfSteps[i + 1]);
        if (lf <= b + 1.0e-12) return (1.0 + i + (lf - a) / (b - a)) / 6.0;
    }
    return 1.0;
}

inline double quantGain (double db, bool stepped) { return stepped ? std::round (db * 2.0) / 2.0 : db; }
inline double quantWeight (double w, bool stepped) { return stepped ? std::round (w) : w; }

inline juce::String hzText (double f)
{
    if (f >= 1000.0)
    {
        const double k = f / 1000.0;
        return juce::String (k, k < 10.0 ? (std::fabs (k - std::round (k * 10.0) / 10.0) < 0.005 ? 1 : 2) : 1) + " kHz";
    }
    return juce::String ((int) std::lround (f)) + " Hz";
}

inline juce::String dbText (double db, int decimals = 1)
{
    if (std::fabs (db) < 0.5 * std::pow (10.0, -decimals)) return "0.0 dB";
    return (db > 0.0 ? "+" : juce::String::fromUTF8 ("\xe2\x88\x92")) + juce::String (std::fabs (db), decimals) + " dB";
}

// Shared with the parameter text functions (they show the effective value).
struct UiFlags
{
    std::atomic<bool> stepped { true };
};

// ---- layout ---------------------------------------------------------------------
inline juce::AudioProcessorValueTreeState::ParameterLayout createLayout (std::shared_ptr<UiFlags> flags)
{
    using namespace juce;
    AudioProcessorValueTreeState::ParameterLayout layout;
    auto pid = [] (const String& id) { return ParameterID { id, 1 }; };

    layout.add (std::make_unique<AudioParameterChoice> (pid (ids::mode), "Mode", StringArray { "CLEAN", "WARM", "BOLD" }, 1));
    layout.add (std::make_unique<AudioParameterFloat> (pid (ids::drive), "Drive", NormalisableRange<float> (-6.0f, 12.0f, 0.01f), 0.0f,
        AudioParameterFloatAttributes().withStringFromValueFunction ([] (float v, int) { return dbText (v); })));
    layout.add (std::make_unique<AudioParameterFloat> (pid (ids::output), "Output", NormalisableRange<float> (-6.0f, 6.0f, 0.01f), 0.0f,
        AudioParameterFloatAttributes().withStringFromValueFunction ([] (float v, int) { return dbText (v); })));
    layout.add (std::make_unique<AudioParameterBool> (pid (ids::stepped), "Stepped", true));
    layout.add (std::make_unique<AudioParameterChoice> (pid (ids::stereo), "Stereo", StringArray { "LINK", "DUAL", "M/S" }, 0));
    layout.add (std::make_unique<AudioParameterChoice> (pid (ids::iron), "Iron", StringArray { "OFF", "NICKEL", "STEEL" }, 1));
    layout.add (std::make_unique<AudioParameterBool> (pid (ids::bypass), "Bypass", false));

    for (int s = 0; s < 2; ++s)
    {
        const String ch = s ? " B" : " A";
        layout.add (std::make_unique<AudioParameterFloat> (pid (ids::hpf (s)), "HPF" + ch, NormalisableRange<float> (0.0f, 1.0f), 0.0f,
            AudioParameterFloatAttributes().withStringFromValueFunction ([flags] (float v, int)
            {
                const double f = hpfFromPos (v, flags->stepped.load());
                return f <= 0.0 ? String ("OFF") : hzText (f);
            })));
        static const char* names[4] = { "Low", "Body", "Presence", "Air" };
        for (int b = 0; b < kNumBands; ++b)
        {
            const double def = b == 0 ? posFromFreq (0, 50.0) : b == 1 ? posFromFreq (1, 560.0) : b == 2 ? posFromFreq (2, 3900.0) : posFromFreq (3, 20000.0);
            layout.add (std::make_unique<AudioParameterFloat> (pid (ids::freq (s, b)), String (names[b]) + " Freq" + ch, NormalisableRange<float> (0.0f, 1.0f), (float) def,
                AudioParameterFloatAttributes().withStringFromValueFunction ([flags, b] (float v, int) { return hzText (freqFromPos (b, v, flags->stepped.load())); })));
            const float g = (float) vt4::eq::kMaxGain[b];
            layout.add (std::make_unique<AudioParameterFloat> (pid (ids::gain (s, b)), String (names[b]) + " Gain" + ch, NormalisableRange<float> (-g, g, 0.01f), 0.0f,
                AudioParameterFloatAttributes().withStringFromValueFunction ([flags] (float v, int) { return dbText (quantGain (v, flags->stepped.load())); })));
        }
        layout.add (std::make_unique<AudioParameterFloat> (pid (ids::weight (s)), "Weight" + ch, NormalisableRange<float> (0.0f, 10.0f, 0.01f), 0.0f,
            AudioParameterFloatAttributes().withStringFromValueFunction ([flags] (float v, int) { return String (quantWeight (v, flags->stepped.load()), flags->stepped.load() ? 0 : 1); })));
        layout.add (std::make_unique<AudioParameterChoice> (pid (ids::focus (s)), "Focus" + ch, StringArray { "WIDE", "NARROW" }, 0));
    }

    layout.add (std::make_unique<AudioParameterChoice> (pid (ids::mains), "Mains", StringArray { "50 Hz", "60 Hz" }, 0));
    layout.add (std::make_unique<AudioParameterChoice> (pid (ids::heaters), "Heaters", StringArray { "DC", "AC" }, 0));
    layout.add (std::make_unique<AudioParameterChoice> (pid (ids::rectifier), "Rectifier", StringArray { "AUTO", "GZ34", "SILICON" }, 0));
    layout.add (std::make_unique<AudioParameterBool> (pid (ids::warmup), "Warm-up", false));
    layout.add (std::make_unique<AudioParameterFloat> (pid (ids::tolerance), "Tolerance", NormalisableRange<float> (0.0f, 200.0f, 1.0f), 100.0f,
        AudioParameterFloatAttributes().withStringFromValueFunction ([] (float v, int) { return String ((int) v) + " %"; })));
    layout.add (std::make_unique<AudioParameterInt> (pid (ids::serial), "Serial", 1, 999, 1));
    layout.add (std::make_unique<AudioParameterChoice> (pid (ids::tubes), "Tubes", StringArray { "MATCHED", "RANDOM" }, 0));
    layout.add (std::make_unique<AudioParameterBool> (pid (ids::noise), "Noise", true));
    layout.add (std::make_unique<AudioParameterChoice> (pid (ids::source), "Source", StringArray { String::fromUTF8 ("50 \xce\xa9"), String::fromUTF8 ("150 \xce\xa9"), String::fromUTF8 ("600 \xce\xa9") }, 1));
    return layout;
}

// ---- reading -----------------------------------------------------------------------
// Raw pointers to every parameter value (lock-free, any thread).
struct ParamPointers
{
    std::atomic<float>* mode = nullptr; std::atomic<float>* drive = nullptr; std::atomic<float>* output = nullptr;
    std::atomic<float>* stepped = nullptr; std::atomic<float>* stereo = nullptr; std::atomic<float>* iron = nullptr;
    std::atomic<float>* bypass = nullptr;
    std::atomic<float>* mains = nullptr; std::atomic<float>* heaters = nullptr; std::atomic<float>* rectifier = nullptr;
    std::atomic<float>* warmup = nullptr; std::atomic<float>* tolerance = nullptr; std::atomic<float>* serial = nullptr;
    std::atomic<float>* tubes = nullptr; std::atomic<float>* noise = nullptr; std::atomic<float>* source = nullptr;
    std::atomic<float>* hpf[2] {};
    std::atomic<float>* freq[2][4] {};
    std::atomic<float>* gain[2][4] {};
    std::atomic<float>* weight[2] {};
    std::atomic<float>* focus[2] {};

    void attach (juce::AudioProcessorValueTreeState& s)
    {
        auto r = [&] (const juce::String& id) { auto* p = s.getRawParameterValue (id); jassert (p != nullptr); return p; };
        mode = r (ids::mode); drive = r (ids::drive); output = r (ids::output); stepped = r (ids::stepped);
        stereo = r (ids::stereo); iron = r (ids::iron); bypass = r (ids::bypass);
        mains = r (ids::mains); heaters = r (ids::heaters); rectifier = r (ids::rectifier); warmup = r (ids::warmup);
        tolerance = r (ids::tolerance); serial = r (ids::serial); tubes = r (ids::tubes); noise = r (ids::noise); source = r (ids::source);
        for (int c = 0; c < 2; ++c)
        {
            hpf[c] = r (ids::hpf (c)); weight[c] = r (ids::weight (c)); focus[c] = r (ids::focus (c));
            for (int b = 0; b < 4; ++b) { freq[c][b] = r (ids::freq (c, b)); gain[c][b] = r (ids::gain (c, b)); }
        }
    }

    vt4::DeviceParams read() const
    {
        vt4::DeviceParams p;
        auto ld = [] (std::atomic<float>* a) { return (double) a->load (std::memory_order_relaxed); };
        const bool st = ld (stepped) > 0.5;
        p.mode = juce::jlimit (0, 2, (int) std::lround (ld (mode)));
        p.driveDb = ld (drive);
        p.iron = juce::jlimit (0, 2, (int) std::lround (ld (iron)));
        p.stereo = juce::jlimit (0, 2, (int) std::lround (ld (stereo)));
        static const double src[3] = { 50.0, 150.0, 600.0 };
        p.sourceOhms = src[juce::jlimit (0, 2, (int) std::lround (ld (source)))];
        p.mainsHz = ld (mains) > 0.5 ? 60.0 : 50.0;
        p.acHeaters = ld (heaters) > 0.5;
        p.rectifier = juce::jlimit (0, 2, (int) std::lround (ld (rectifier)));
        p.warmup = ld (warmup) > 0.5;
        p.matched = ld (tubes) < 0.5;
        p.tolerance = ld (tolerance) / 100.0;
        p.serial = (uint32_t) juce::jlimit (1, 999, (int) std::lround (ld (serial)));
        p.noise = ld (noise) > 0.5;
        for (int c = 0; c < 2; ++c)
        {
            auto& ch = p.ch[(size_t) c];
            ch.hpfHz = hpfFromPos (ld (hpf[c]), st);
            for (int b = 0; b < kNumBands; ++b)
            {
                auto& bp = ch.band[(size_t) b];
                bp.freq = freqFromPos (b, ld (freq[c][b]), st);
                bp.gainDb = quantGain (ld (gain[c][b]), st);
                bp.narrow = b == vt4::kPres && ld (focus[c]) > 0.5;
                bp.weight = b == vt4::kLow ? quantWeight (ld (weight[c]), st) : 0.0;
            }
        }
        return p;
    }
};

inline bool sameParams (const vt4::DeviceParams& a, const vt4::DeviceParams& b)
{
    if (a.mode != b.mode || a.driveDb != b.driveDb || a.iron != b.iron || a.stereo != b.stereo
        || a.sourceOhms != b.sourceOhms || a.calDbu != b.calDbu || a.mainsHz != b.mainsHz || a.acHeaters != b.acHeaters
        || a.rectifier != b.rectifier || a.warmup != b.warmup || a.drift != b.drift || a.matched != b.matched
        || a.tolerance != b.tolerance || a.serial != b.serial || a.noise != b.noise)
        return false;
    for (int c = 0; c < 2; ++c)
    {
        if (a.ch[(size_t) c].hpfHz != b.ch[(size_t) c].hpfHz) return false;
        for (int k = 0; k < kNumBands; ++k)
        {
            const auto& x = a.ch[(size_t) c].band[(size_t) k];
            const auto& y = b.ch[(size_t) c].band[(size_t) k];
            if (x.gainDb != y.gainDb || x.freq != y.freq || x.narrow != y.narrow || x.weight != y.weight) return false;
        }
    }
    return true;
}

} // namespace vt4p
