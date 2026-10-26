#pragma once

#include <juce_audio_processors/juce_audio_processors.h>
#include "Layout.h"
#include "Meters.h"
#include "Widgets.h"

#include <array>
#include <memory>

namespace vt4ui
{

class Faceplate : public juce::Component
{
public:
    explicit Faceplate (juce::AudioProcessorValueTreeState&);
    ~Faceplate() override;

    void paint (juce::Graphics&) override;
    void resized() override;

    // called by the editor timer
    void setStereoMode (int m);
    void setStepped (bool s);
    void setMeters (float vuL, float vuR, bool peakL, bool peakR, float bplus, float bplusNominal, float drive, bool active);
    void setSerial (int s) { if (s != serial) { serial = s; repaint(); } }

    std::function<void()> onRear, onScreenToggle, onCopyAtoB;
    juce::TextButton rearButton { "REAR" }, screenButton { "SCREEN" }, copyButton { juce::String::fromUTF8 ("A\xe2\x86\x92" "B") };

private:
    using SA = juce::AudioProcessorValueTreeState::SliderAttachment;
    void renderPlate (int w, int h, float scale);
    void drawScales (juce::Graphics&);
    void attachRowB (bool linked);
    void applySnapping();

    juce::AudioProcessorValueTreeState& state;

    struct Row
    {
        Knob hpf, weight;
        std::array<Knob, 4> freq, gain;
        std::unique_ptr<ChoiceButtons> focus;
        std::vector<std::unique_ptr<SA>> att;
    };
    std::array<Row, 2> rows;
    Knob drive, output;
    std::unique_ptr<SA> driveAtt, outputAtt;
    std::unique_ptr<ChoiceButtons> mode, stereo, iron, stepped;
    std::unique_ptr<ToggleKey> bypass;

    VuMeter vuL { "OUTPUT  L" }, vuR { "OUTPUT  R" };
    BPlusMeter bplusMeter;
    Lamp driveLamp { juce::Colour (0xffff8a2a) }, onLamp { col::amber };

    int stereoMode = 0, serial = 1;
    bool steppedMode = true, linkedNow = false;
    juce::Image plate;
    float plateScale = 0.0f;
};

} // namespace vt4ui
