#pragma once

#include <juce_audio_processors/juce_audio_processors.h>
#include "Widgets.h"

#include <memory>

namespace vt4ui
{

// The "rear panel": supply and unit settings.
class RearPanel : public juce::Component
{
public:
    explicit RearPanel (juce::AudioProcessorValueTreeState&);
    void paint (juce::Graphics&) override;
    void resized() override;
    void setWarmth (float w) { if (std::abs (w - warmth) > 0.005f) { warmth = w; repaint(); } }

private:
    juce::AudioProcessorValueTreeState& state;
    std::unique_ptr<ChoiceButtons> mains, heaters, rectifier, tubes, source;
    std::unique_ptr<ToggleKey> warmup, noise;
    Knob tolerance;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> tolAtt;
    juce::TextButton serialDown { "-" }, serialUp { "+" }, serialNew { "NEW UNIT" };
    juce::Label serialLabel;
    std::unique_ptr<juce::ParameterAttachment> serialAtt;
    float warmth = 1.0f;
    juce::Image bgImage;
    float bgScale = 0.0f;
};

} // namespace vt4ui
