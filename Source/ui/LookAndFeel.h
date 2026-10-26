#pragma once

#include <juce_gui_basics/juce_gui_basics.h>

namespace vt4ui
{

// ---- palette ------------------------------------------------------------------
namespace col
{
    const juce::Colour plateTop    { 0xff2b2e33 };
    const juce::Colour plateBottom { 0xff1c1e21 };
    const juce::Colour brass       { 0xffd2b47a };
    const juce::Colour brassDim    { 0xff9c8358 };
    const juce::Colour brassFaint  { 0x55c8a96b };
    const juce::Colour engraveShadow { 0xd0050506 };
    const juce::Colour amber       { 0xffffb648 };
    const juce::Colour amberDim    { 0xff7a5520 };
    const juce::Colour green       { 0xff9be7a4 };
    const juce::Colour glass       { 0xff08090a };
    const juce::Colour cream       { 0xfff4e6c3 };
    const juce::Colour ink         { 0xff171310 };
    const juce::Colour red         { 0xffd2412f };
}

// ---- fonts ---------------------------------------------------------------------
struct Fonts
{
    static Fonts& get();
    juce::Typeface::Ptr medium, semibold, displayLight, displayMedium;

    static juce::Font label (float h, float tracking = 0.12f);     // engraved small caps
    static juce::Font strong (float h, float tracking = 0.08f);
    static juce::Font display (float h, float tracking = 0.0f);    // light display face
    static juce::Font displayBold (float h, float tracking = 0.0f);
};

// Text that looks engraved into the faceplate and filled with brass paint.
void drawEngraved (juce::Graphics&, const juce::String&, juce::Rectangle<float>, const juce::Font&,
                   juce::Justification, juce::Colour fill = col::brass, float depth = 1.0f);

// A machined, knurled aluminium knob (no scale: that is engraved on the plate).
void drawKnob (juce::Graphics&, juce::Rectangle<float> bounds, float angle, bool large, bool enabled, float hover);

// Brushed, anodised metal generated per physical pixel (fills the image).
void renderBrushedMetal (juce::Image&, float scale, juce::Colour top, juce::Colour mid, juce::Colour bottom, uint32_t seed);

// A hairline frame engraved in a plate, interrupted where its title sits.
void drawFrame (juce::Graphics&, juce::Rectangle<float> frame, juce::Rectangle<float> gapArea, float corner, juce::Colour line);

// A jewel indicator lamp; level 0 … 1.
void drawJewel (juce::Graphics&, juce::Rectangle<float>, juce::Colour, float level);

class VT4LookAndFeel : public juce::LookAndFeel_V4
{
public:
    VT4LookAndFeel();

    void drawRotarySlider (juce::Graphics&, int x, int y, int w, int h, float pos,
                           float start, float end, juce::Slider&) override;
    void drawButtonBackground (juce::Graphics&, juce::Button&, const juce::Colour&, bool over, bool down) override;
    void drawButtonText (juce::Graphics&, juce::TextButton&, bool over, bool down) override;
    juce::Font getTextButtonFont (juce::TextButton&, int h) override;
    void drawBubble (juce::Graphics&, juce::BubbleComponent&, const juce::Point<float>&, const juce::Rectangle<float>&) override;
    juce::Font getSliderPopupFont (juce::Slider&) override;
    int getSliderPopupPlacement (juce::Slider&) override;
    juce::Label* createSliderTextBox (juce::Slider&) override;
    void drawTooltip (juce::Graphics&, const juce::String&, int w, int h) override;
    void drawCornerResizer (juce::Graphics&, int w, int h, bool isMouseOver, bool isMouseDragging) override;
};

} // namespace vt4ui
