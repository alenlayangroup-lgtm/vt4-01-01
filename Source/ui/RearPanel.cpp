#include "RearPanel.h"
#include "../Parameters.h"

namespace vt4ui
{

namespace
{
    const juce::Colour kStamp { 0xffc9ccd1 };
    const juce::Colour kStampDim { 0xff8d9097 };

    juce::Rectangle<int> rowRect (int x, int y, int w) { return { x, y, w, 26 }; }
}

RearPanel::RearPanel (juce::AudioProcessorValueTreeState& s) : state (s)
{
    using namespace vt4p;
    mains = std::make_unique<ChoiceButtons> (*state.getParameter (ids::mains), juce::StringArray { "50 Hz", "60 Hz" }, 10.0f);
    heaters = std::make_unique<ChoiceButtons> (*state.getParameter (ids::heaters), juce::StringArray { "DC", "AC" }, 10.0f);
    rectifier = std::make_unique<ChoiceButtons> (*state.getParameter (ids::rectifier), juce::StringArray { "AUTO", "GZ34", "SILICON" }, 10.0f);
    tubes = std::make_unique<ChoiceButtons> (*state.getParameter (ids::tubes), juce::StringArray { "MATCHED", "RANDOM" }, 10.0f);
    source = std::make_unique<ChoiceButtons> (*state.getParameter (ids::source),
        juce::StringArray { juce::String::fromUTF8 ("50 \xce\xa9"), juce::String::fromUTF8 ("150 \xce\xa9"), juce::String::fromUTF8 ("600 \xce\xa9") }, 10.0f);
    warmup = std::make_unique<ToggleKey> (*state.getParameter (ids::warmup), "WARM-UP", 10.0f);
    noise = std::make_unique<ToggleKey> (*state.getParameter (ids::noise), "NOISE & HUM", 10.0f);
    for (auto* c : std::initializer_list<juce::Component*> { mains.get(), heaters.get(), rectifier.get(), tubes.get(), source.get(), warmup.get(), noise.get() })
    {
        if (auto* cb = dynamic_cast<ChoiceButtons*> (c)) cb->gap = 6;
        addAndMakeVisible (c);
    }
    addAndMakeVisible (tolerance);
    tolAtt = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (state, ids::tolerance, tolerance);
    tolerance.setDoubleClickReturnValue (true, 100.0);

    serialLabel.setJustificationType (juce::Justification::centred);
    serialLabel.setFont (Fonts::displayBold (22.0f, 0.12f));
    serialLabel.setColour (juce::Label::textColourId, juce::Colour (0xff1b1c1e));
    addAndMakeVisible (serialLabel);
    auto* sp = state.getParameter (ids::serial);
    serialAtt = std::make_unique<juce::ParameterAttachment> (*sp, [this] (float v)
    {
        serialLabel.setText (juce::String ((int) std::lround (v)).paddedLeft ('0', 4), juce::dontSendNotification);
    }, nullptr);
    serialAtt->sendInitialUpdate();
    auto bump = [this, sp] (int d)
    {
        const int cur = (int) std::lround (sp->convertFrom0to1 (sp->getValue()));
        serialAtt->setValueAsCompleteGesture ((float) juce::jlimit (1, 999, cur + d));
    };
    serialDown.onClick = [bump] { bump (-1); };
    serialUp.onClick = [bump] { bump (+1); };
    serialNew.onClick = [this] { serialAtt->setValueAsCompleteGesture ((float) juce::Random::getSystemRandom().nextInt ({ 1, 1000 })); };
    for (auto* b : { &serialDown, &serialUp, &serialNew })
    {
        b->getProperties().set ("fontSize", 11.0f);
        addAndMakeVisible (b);
    }
}

void RearPanel::resized()
{
    mains->setBounds (rowRect (170, 70, 170));
    rectifier->setBounds (rowRect (170, 108, 260));
    heaters->setBounds (rowRect (170, 146, 170));
    warmup->setBounds (rowRect (170, 184, 120));

    tubes->setBounds (rowRect (640, 70, 220));
    tolerance.setBounds (630, 108, 92, 92);
    noise->setBounds (rowRect (640, 228, 150));

    source->setBounds (rowRect (1004, 70, 270));
    serialLabel.setBounds (1064, 144, 150, 34);
    serialDown.setBounds (1012, 148, 32, 26);
    serialUp.setBounds (1234, 148, 32, 26);
    serialNew.setBounds (1094, 198, 90, 24);
}

void RearPanel::paint (juce::Graphics& g)
{
    const auto b = getLocalBounds().toFloat();
    // the same brushed, anodised stock as the front, a shade darker
    {
        const float scale = g.getInternalContext().getPhysicalPixelScaleFactor();
        if (bgImage.isNull() || std::abs (scale - bgScale) > 0.01f || bgImage.getWidth() != (int) std::ceil (b.getWidth() * scale))
        {
            bgImage = juce::Image (juce::Image::ARGB, juce::jmax (1, (int) std::ceil (b.getWidth() * scale)), juce::jmax (1, (int) std::ceil (b.getHeight() * scale)), true);
            renderBrushedMetal (bgImage, scale, juce::Colour (0xff292c30), juce::Colour (0xff232528), juce::Colour (0xff1a1c1f), 0x6a09e667u);
            bgScale = scale;
        }
        g.drawImageTransformed (bgImage, juce::AffineTransform::scale (1.0f / bgScale));
    }
    g.setColour (juce::Colours::black.withAlpha (0.55f));
    g.drawLine (0.0f, 0.5f, b.getWidth(), 0.5f, 1.0f);
    g.setColour (juce::Colours::white.withAlpha (0.06f));
    g.drawLine (0.0f, b.getHeight() - 0.5f, b.getWidth(), b.getHeight() - 0.5f, 1.0f);

    auto stamp = [&] (const juce::String& t, juce::Rectangle<float> r, float size, juce::Justification j = juce::Justification::centredLeft, juce::Colour c = kStamp)
    {
        drawEngraved (g, t, r, Fonts::label (size, 0.22f), j, c, 0.7f);
    };
    auto group = [&] (juce::Rectangle<float> r, const juce::String& title)
    {
        const auto f = Fonts::strong (10.0f, 0.3f);
        const float tw = juce::GlyphArrangement::getStringWidth (f, title) + 16.0f;
        const auto tr = juce::Rectangle<float> (r.getX() + 14.0f, r.getY() - 7.0f, tw, 14.0f);
        drawFrame (g, r, tr, 7.0f, kStamp.withAlpha (0.26f));
        drawEngraved (g, title, tr, f, juce::Justification::centred, kStamp, 0.7f);
    };

    stamp ("REAR PANEL", { 30.0f, 10.0f, 300.0f, 20.0f }, 11.0f, juce::Justification::centredLeft, kStampDim);

    group ({ 30.0f, 46.0f, 560.0f, 236.0f }, "POWER SUPPLY");
    stamp ("MAINS", { 50.0f, 70.0f, 110.0f, 26.0f }, 10.0f);
    stamp ("RECTIFIER", { 50.0f, 108.0f, 110.0f, 26.0f }, 10.0f);
    stamp ("HEATERS", { 50.0f, 146.0f, 110.0f, 26.0f }, 10.0f);
    stamp ("START-UP", { 50.0f, 184.0f, 110.0f, 26.0f }, 10.0f);
    stamp (juce::String::fromUTF8 ("AUTO:  CLEAN \xe2\x86\x92 SILICON"), { 440.0f, 106.0f, 150.0f, 14.0f }, 7.4f, juce::Justification::centredLeft, kStampDim);
    stamp (juce::String::fromUTF8 ("WARM, BOLD \xe2\x86\x92 GZ34"), { 470.0f, 119.0f, 120.0f, 14.0f }, 7.4f, juce::Justification::centredLeft, kStampDim);
    if (warmth < 0.999f)
        stamp ("WARMING UP  " + juce::String ((int) std::lround (warmth * 100.0f)) + " %", { 300.0f, 184.0f, 200.0f, 26.0f }, 10.0f, juce::Justification::centredLeft, col::amber);
    // IEC inlet and fuse, for the eye
    {
        const juce::Rectangle<float> iec (60.0f, 222.0f, 62.0f, 44.0f);
        g.setColour (juce::Colour (0xff0d0e0f));
        g.fillRoundedRectangle (iec, 5.0f);
        g.setColour (juce::Colour (0xff3a3c40));
        g.drawRoundedRectangle (iec, 5.0f, 1.2f);
        g.setColour (juce::Colour (0xff6b6e73));
        for (float x : { 76.0f, 90.0f, 104.0f }) g.fillRect (x - 2.0f, 236.0f, 4.0f, 14.0f);
        stamp (juce::String::fromUTF8 ("230 V ~ 50/60 Hz  \xc2\xb7  65 VA"), { 134.0f, 228.0f, 250.0f, 14.0f }, 8.0f, juce::Justification::centredLeft, kStampDim);
        stamp ("FUSE T 500 mA", { 134.0f, 246.0f, 250.0f, 14.0f }, 8.0f, juce::Justification::centredLeft, kStampDim);
        const auto fuse = juce::Rectangle<float> (300.0f, 228.0f, 30.0f, 30.0f);
        juce::ColourGradient fg (juce::Colour (0xff4a4c50), fuse.getX(), fuse.getY(), juce::Colour (0xff141516), fuse.getX(), fuse.getBottom(), false);
        g.setGradientFill (fg);
        g.fillEllipse (fuse);
        g.setColour (juce::Colours::black);
        g.drawLine (fuse.getCentreX() - 8.0f, fuse.getCentreY(), fuse.getCentreX() + 8.0f, fuse.getCentreY(), 2.4f);
    }

    group ({ 610.0f, 46.0f, 360.0f, 236.0f }, "TUBES & PARTS");
    stamp ("TOLERANCE", { 734.0f, 128.0f, 200.0f, 18.0f }, 10.0f);
    stamp ("SPREAD OF EVERY PART:", { 734.0f, 148.0f, 220.0f, 14.0f }, 7.6f, juce::Justification::centredLeft, kStampDim);
    stamp (juce::String::fromUTF8 ("0 % IDEAL  \xc2\xb7  100 % TYPICAL  \xc2\xb7  200 % LOOSE"), { 734.0f, 162.0f, 230.0f, 14.0f }, 7.0f, juce::Justification::centredLeft, kStampDim);
    {
        const juce::Point<float> c (676.0f, 154.0f);
        const float r = 39.0f;
        for (int k = 0; k <= 8; ++k)
        {
            const float a = (-150.0f + 300.0f * k / 8.0f) * juce::MathConstants<float>::pi / 180.0f;
            const juce::Point<float> d (std::sin (a), -std::cos (a));
            g.setColour (kStamp.withAlpha (0.8f));
            g.drawLine ({ c + d * (r + 2.0f), c + d * (r + (k % 4 == 0 ? 8.0f : 5.0f)) }, 1.0f);
        }
    }

    group ({ 990.0f, 46.0f, 300.0f, 236.0f }, "SOURCE  &  UNIT");
    stamp ("SOURCE IMPEDANCE (INPUT TRANSFORMER)", { 1004.0f, 100.0f, 280.0f, 14.0f }, 7.5f, juce::Justification::centredLeft, kStampDim);
    // ID plate
    {
        const juce::Rectangle<float> idp (1060.0f, 134.0f, 158.0f, 54.0f);
        juce::ColourGradient pg (juce::Colour (0xffd9d5cb), idp.getX(), idp.getY(), juce::Colour (0xffa8a397), idp.getRight(), idp.getBottom(), false);
        g.setGradientFill (pg);
        g.fillRoundedRectangle (idp, 3.0f);
        g.setColour (juce::Colours::black.withAlpha (0.4f));
        g.drawRoundedRectangle (idp, 3.0f, 1.0f);
        g.setColour (juce::Colour (0xff2b2a27));
        g.setFont (Fonts::label (7.0f, 0.25f));
        g.drawText (juce::String::fromUTF8 ("HALDEN VT-4  \xc2\xb7  SERIAL N\xc2\xba"), idp.withHeight (14.0f).translated (0.0f, 2.0f), juce::Justification::centred, false);
        for (auto p : { idp.getTopLeft().translated (6.0f, 6.0f), idp.getTopRight().translated (-6.0f, 6.0f) })
        {
            g.setColour (juce::Colour (0xff6e6a62));
            g.fillEllipse (p.x - 2.0f, p.y - 2.0f, 4.0f, 4.0f);
        }
    }
    stamp ("EACH SERIAL IS A DIFFERENT UNIT:", { 1004.0f, 232.0f, 280.0f, 14.0f }, 7.2f, juce::Justification::centredLeft, kStampDim);
    stamp ("ITS OWN PARTS, TUBES AND L / R MATCH.", { 1004.0f, 246.0f, 280.0f, 14.0f }, 7.2f, juce::Justification::centredLeft, kStampDim);
    stamp ("NOISE & HUM: TUBE AND RESISTOR NOISE, RIPPLE, AC HEATER HUM.", { 640.0f, 260.0f, 330.0f, 14.0f }, 7.0f, juce::Justification::centredLeft, kStampDim);

    stamp (juce::String::fromUTF8 ("CAUTION \xc2\xb7 HIGH VOLTAGE INSIDE \xc2\xb7 NO USER SERVICEABLE PARTS"), { 30.0f, b.getHeight() - 30.0f, 600.0f, 16.0f }, 8.0f, juce::Justification::centredLeft, kStampDim);
    stamp ("DESIGNED BY HALDEN AUDIO", { b.getWidth() - 330.0f, b.getHeight() - 30.0f, 300.0f, 16.0f }, 8.0f, juce::Justification::centredRight, kStampDim);
}

} // namespace vt4ui
