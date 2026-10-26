#pragma once

#include "LookAndFeel.h"

namespace vt4ui
{

// Back-lit VU meter. The needle deflection is proportional to the rectified
// voltage (as in a real VU), value 1.0 = 0 VU.
class VuMeter : public juce::Component
{
public:
    explicit VuMeter (juce::String label) : name (std::move (label)) {}
    void setValue (float v) { if (std::abs (v - value) > 1.0e-4f) { value = v; repaint(); } }
    void setPeak (bool p) { if (p != peak) { peak = p; repaint(); } }
    void setLabel (const juce::String& l) { if (l != name) { name = l; face = {}; repaint(); } }
    void paint (juce::Graphics&) override;

private:
    void renderFace (int w, int h, float scale);
    juce::String name;
    float value = 0.0f;
    bool peak = false;
    juce::Image face;
    float faceScale = 0.0f;
};

// Small round meter for the main B+ rail: deviation from its nominal value.
class BPlusMeter : public juce::Component
{
public:
    void setValue (float volts, float nominal)
    {
        if (std::abs (volts - v) > 0.004f || nominal != nom) { v = volts; nom = nominal; repaint(); }
    }
    void paint (juce::Graphics&) override;

private:
    void renderFace (int w, int h, float scale);
    float v = 300.0f, nom = 300.0f;
    juce::Image face;
    float faceScale = 0.0f;
};

class Lamp : public juce::Component
{
public:
    explicit Lamp (juce::Colour c) : colour (c) { setInterceptsMouseClicks (false, false); }
    void setLevel (float l) { if (std::abs (l - level) > 0.01f) { level = l; repaint(); } }
    void paint (juce::Graphics& g) override
    {
        const auto r = getLocalBounds().toFloat();
        const float d = juce::jmin (r.getWidth(), r.getHeight()) * 0.42f;
        drawJewel (g, juce::Rectangle<float> (d, d).withCentre (r.getCentre()), colour, level);
    }

private:
    juce::Colour colour;
    float level = 0.0f;
};

} // namespace vt4ui
