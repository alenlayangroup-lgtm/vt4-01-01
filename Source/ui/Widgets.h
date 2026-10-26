#pragma once

#include <juce_audio_processors/juce_audio_processors.h>
#include "LookAndFeel.h"

#include <functional>
#include <memory>
#include <vector>

namespace vt4ui
{

// Rotary knob with an optional snapping rule (STEPPED detents).
class Knob : public juce::Slider
{
public:
    Knob()
    {
        setSliderStyle (juce::Slider::RotaryVerticalDrag);
        setTextBoxStyle (juce::Slider::NoTextBox, false, 0, 0);
        setRotaryParameters (juce::MathConstants<float>::pi * (1.0f + 1.0f / 6.0f),
                             juce::MathConstants<float>::pi * (3.0f - 1.0f / 6.0f), true);
        setMouseDragSensitivity (240);
        setPopupDisplayEnabled (true, true, nullptr, 1200);
        setScrollWheelEnabled (true);
    }
    std::function<double (double)> snapper;
    double snapValue (double v, DragMode) override { return snapper ? snapper (v) : v; }
};

// A row of illuminated buttons bound to a choice parameter (radio group).
class ChoiceButtons : public juce::Component
{
public:
    ChoiceButtons (juce::RangedAudioParameter& p, juce::StringArray labels, float fontSize = 10.5f)
        : param (p),
          attachment (p, [this] (float v) { setIndex ((int) std::lround (v)); }, nullptr)
    {
        for (int i = 0; i < labels.size(); ++i)
        {
            auto* b = buttons.add (new juce::TextButton (labels[i]));
            b->getProperties().set ("fontSize", fontSize);
            b->setClickingTogglesState (false);
            b->onClick = [this, i] { attachment.setValueAsCompleteGesture ((float) i); };
            addAndMakeVisible (b);
        }
        attachment.sendInitialUpdate();
    }

    void setIndex (int idx)
    {
        index = idx;
        for (int i = 0; i < buttons.size(); ++i) buttons[i]->setToggleState (i == idx, juce::dontSendNotification);
        if (onChange) onChange (idx);
    }
    int getIndex() const noexcept { return index; }

    void resized() override
    {
        auto r = getLocalBounds();
        const int n = buttons.size();
        if (vertical)
        {
            const int h = (r.getHeight() - gap * (n - 1)) / juce::jmax (1, n);
            for (auto* b : buttons) { b->setBounds (r.removeFromTop (h)); r.removeFromTop (gap); }
        }
        else
        {
            const int w = (r.getWidth() - gap * (n - 1)) / juce::jmax (1, n);
            for (auto* b : buttons) { b->setBounds (r.removeFromLeft (w)); r.removeFromLeft (gap); }
        }
    }

    bool vertical = false;
    int gap = 4;
    std::function<void (int)> onChange;

private:
    juce::RangedAudioParameter& param;
    juce::ParameterAttachment attachment;
    juce::OwnedArray<juce::TextButton> buttons;
    int index = 0;
};

// Toggle bound to a bool parameter.
class ToggleKey : public juce::TextButton
{
public:
    ToggleKey (juce::RangedAudioParameter& p, const juce::String& caption, float fontSize = 10.5f)
        : juce::TextButton (caption),
          attachment (p, [this] (float v) { setToggleState (v > 0.5f, juce::dontSendNotification); if (onState) onState (v > 0.5f); }, nullptr)
    {
        getProperties().set ("fontSize", fontSize);
        onClick = [this] { attachment.setValueAsCompleteGesture (getToggleState() ? 0.0f : 1.0f); };
        attachment.sendInitialUpdate();
    }
    std::function<void (bool)> onState;

private:
    juce::ParameterAttachment attachment;
};

} // namespace vt4ui
