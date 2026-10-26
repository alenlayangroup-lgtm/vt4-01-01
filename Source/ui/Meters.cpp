#include "Meters.h"

namespace vt4ui
{

namespace
{
    constexpr float kVuMax = 1.4125375f;     // +3 VU
    constexpr float kSweep = 0.84f;          // ± radians of the needle swing

    float vuAngle (float v) { return -kSweep + 2.0f * kSweep * juce::jlimit (0.0f, 1.08f, v / kVuMax); }

    void meterBezel (juce::Graphics& g, juce::Rectangle<float> r, float cr)
    {
        g.setColour (juce::Colours::black.withAlpha (0.65f));
        g.fillRoundedRectangle (r.expanded (2.0f), cr + 2.0f);
        juce::ColourGradient bz (juce::Colour (0xff4b4d52), r.getX(), r.getY(), juce::Colour (0xff121315), r.getX(), r.getBottom(), false);
        g.setGradientFill (bz);
        g.fillRoundedRectangle (r, cr);
        g.setColour (juce::Colours::white.withAlpha (0.10f));
        g.drawRoundedRectangle (r.reduced (0.5f), cr, 1.0f);
    }
}

void VuMeter::renderFace (int w, int h, float scale)
{
    face = juce::Image (juce::Image::ARGB, juce::jmax (1, (int) std::ceil (w * scale)), juce::jmax (1, (int) std::ceil (h * scale)), true);
    faceScale = scale;
    juce::Graphics g (face);
    g.addTransform (juce::AffineTransform::scale (scale));
    const auto all = juce::Rectangle<float> ((float) w, (float) h);
    meterBezel (g, all.reduced (2.0f), 6.0f);
    const auto win = all.reduced (9.0f);

    // polished brass trim around the window
    {
        juce::ColourGradient trim (juce::Colour (0xfff0d9a4), win.getX(), win.getY() - 2.0f, juce::Colour (0xff5a4527), win.getX(), win.getBottom() + 2.0f, false);
        trim.addColour (0.45, juce::Colour (0xffb89659));
        g.setGradientFill (trim);
        g.fillRoundedRectangle (win.expanded (2.2f), 4.6f);
        g.setColour (juce::Colours::black.withAlpha (0.8f));
        g.fillRoundedRectangle (win.expanded (0.6f), 3.6f);
    }

    // back-lit card: two lamps behind the scale, warm paper
    juce::ColourGradient card (juce::Colour (0xfffcf1d2), win.getCentreX(), win.getBottom() - win.getHeight() * 0.25f,
                               juce::Colour (0xffd8bf86), win.getX(), win.getY(), true);
    g.setGradientFill (card);
    g.fillRoundedRectangle (win, 3.0f);
    for (float fx : { 0.28f, 0.72f })
    {
        juce::ColourGradient lamp (juce::Colour (0x30fff4d8), win.getX() + win.getWidth() * fx, win.getBottom(),
                                   juce::Colour (0x00fff4d8), win.getX() + win.getWidth() * fx, win.getBottom() - win.getHeight() * 0.9f, true);
        g.setGradientFill (lamp);
        g.fillRoundedRectangle (win, 3.0f);
    }
    // paper grain
    {
        juce::Random rnd (77);
        for (int i = 0; i < (int) (win.getWidth() * win.getHeight() * 0.12f); ++i)
        {
            g.setColour (rnd.nextBool() ? juce::Colour (0x0a000000) : juce::Colour (0x0cffffff));
            g.fillRect (win.getX() + rnd.nextFloat() * win.getWidth(), win.getY() + rnd.nextFloat() * win.getHeight(), 0.6f, 0.6f);
        }
    }

    // scale
    const juce::Point<float> pivot (win.getCentreX(), win.getBottom() + win.getHeight() * 0.38f);
    const float r = win.getHeight() * 1.06f;
    auto pt = [&] (float ang, float rad) { return pivot + juce::Point<float> (std::sin (ang), -std::cos (ang)) * rad; };

    // red zone (0 … +3 VU)
    {
        juce::Path red;
        const float a0 = vuAngle (1.0f), a1 = vuAngle (kVuMax);
        red.addCentredArc (pivot.x, pivot.y, r + 3.5f, r + 3.5f, 0.0f, a0, a1, true);
        g.setColour (col::red.withAlpha (0.9f));
        g.strokePath (red, juce::PathStrokeType (5.0f));
    }
    juce::Path arc;
    arc.addCentredArc (pivot.x, pivot.y, r, r, 0.0f, vuAngle (0.1f), vuAngle (kVuMax), true);
    g.setColour (col::ink);
    g.strokePath (arc, juce::PathStrokeType (1.1f));

    struct Mark { float db; const char* txt; bool major; };
    const Mark marks[] = { { -20, "20", true }, { -10, "10", true }, { -7, "7", true }, { -5, "5", true }, { -3, "3", true },
                           { -2, "2", false }, { -1, "1", false }, { 0, "0", true }, { 1, "1", false }, { 2, "2", false }, { 3, "3", true } };
    g.setFont (Fonts::strong (win.getHeight() * 0.13f, 0.0f));
    for (const auto& m : marks)
    {
        const float a = vuAngle (std::pow (10.0f, m.db / 20.0f));
        g.setColour (m.db > 0.0f ? col::red : col::ink);
        g.drawLine ({ pt (a, r), pt (a, r + (m.major ? 9.0f : 6.0f)) }, m.major ? 1.4f : 1.0f);
        const auto tp = pt (a, r + 17.0f);
        g.drawText (m.txt, juce::Rectangle<float> (24.0f, 12.0f).withCentre (tp), juce::Justification::centred, false);
    }
    // minor ticks
    for (float db = -20.0f; db <= 3.0f; db += 1.0f)
    {
        const float a = vuAngle (std::pow (10.0f, db / 20.0f));
        g.setColour ((db > 0.0f ? col::red : col::ink).withAlpha (0.7f));
        g.drawLine ({ pt (a, r), pt (a, r + 4.0f) }, 0.8f);
    }
    // percent scale (inner)
    {
        juce::Path pa;
        pa.addCentredArc (pivot.x, pivot.y, r - 9.0f, r - 9.0f, 0.0f, vuAngle (0.0f), vuAngle (1.0f), true);
        g.setColour (col::ink.withAlpha (0.55f));
        g.strokePath (pa, juce::PathStrokeType (0.8f));
        for (int k = 0; k <= 10; ++k)
        {
            const float a = vuAngle (0.1f * (float) k);
            g.drawLine ({ pt (a, r - 9.0f), pt (a, r - (k % 5 == 0 ? 14.0f : 12.0f)) }, 0.7f);
        }
    }
    g.setColour (col::ink);
    g.setFont (Fonts::displayBold (win.getHeight() * 0.20f, 0.06f));
    g.drawText ("VU", win.withTrimmedTop (win.getHeight() * 0.50f).withHeight (win.getHeight() * 0.24f), juce::Justification::centred, false);
    g.setFont (Fonts::label (win.getHeight() * 0.085f, 0.25f));
    g.setColour (col::ink.withAlpha (0.65f));
    g.drawText (name, win.withTrimmedTop (win.getHeight() * 0.74f).withHeight (win.getHeight() * 0.12f), juce::Justification::centred, false);
    g.setColour (col::red.withAlpha (0.85f));
    g.setFont (Fonts::label (win.getHeight() * 0.075f, 0.2f));
    g.drawText ("-", win.reduced (6.0f).withHeight (win.getHeight() * 0.18f), juce::Justification::topLeft, false);
    g.drawText ("+", win.reduced (6.0f).withHeight (win.getHeight() * 0.18f), juce::Justification::topRight, false);

    // warm inner vignette
    juce::ColourGradient vig (juce::Colours::transparentBlack, win.getCentreX(), win.getCentreY(),
                              juce::Colours::black.withAlpha (0.30f), win.getX(), win.getY(), true);
    g.setGradientFill (vig);
    g.fillRoundedRectangle (win, 3.0f);
}

void VuMeter::paint (juce::Graphics& g)
{
    const float scale = g.getInternalContext().getPhysicalPixelScaleFactor();
    if (face.isNull() || std::abs (scale - faceScale) > 0.01f || face.getWidth() != (int) std::ceil (getWidth() * scale))
        renderFace (getWidth(), getHeight(), scale);
    g.drawImageTransformed (face, juce::AffineTransform::scale (1.0f / faceScale));

    const auto win = getLocalBounds().toFloat().reduced (9.0f);
    g.saveState();
    g.reduceClipRegion (win.toNearestInt());
    const juce::Point<float> pivot (win.getCentreX(), win.getBottom() + win.getHeight() * 0.38f);
    const float r = win.getHeight() * 1.06f;
    const float a = vuAngle (value);
    const juce::Point<float> dir (std::sin (a), -std::cos (a));
    const auto tip = pivot + dir * (r + 6.0f);
    const auto base = pivot + dir * (win.getHeight() * 0.42f);
    // shadow of the needle on the card (the needle runs a few mm above it)
    g.setColour (juce::Colours::black.withAlpha (0.10f));
    g.drawLine ({ base.translated (3.5f, 4.0f), tip.translated (3.5f, 4.0f) }, 2.4f);
    g.setColour (juce::Colours::black.withAlpha (0.16f));
    g.drawLine ({ base.translated (2.2f, 2.8f), tip.translated (2.2f, 2.8f) }, 1.2f);
    g.setColour (col::ink);
    g.drawLine ({ base, tip }, 1.3f);
    // tapered tip
    g.drawLine ({ tip - dir * 14.0f, tip }, 0.9f);
    g.restoreState();

    // glass
    juce::ColourGradient gl (juce::Colours::white.withAlpha (0.20f), win.getX(), win.getY(),
                             juce::Colours::white.withAlpha (0.0f), win.getX(), win.getY() + win.getHeight() * 0.55f, false);
    g.setGradientFill (gl);
    juce::Path refl;
    refl.startNewSubPath (win.getX(), win.getY());
    refl.lineTo (win.getRight(), win.getY());
    refl.lineTo (win.getRight(), win.getY() + win.getHeight() * 0.18f);
    refl.quadraticTo (win.getCentreX(), win.getY() + win.getHeight() * 0.36f, win.getX(), win.getY() + win.getHeight() * 0.30f);
    refl.closeSubPath();
    g.fillPath (refl);

    // peak lamp
    if (peak)
    {
        const auto lp = juce::Rectangle<float> (8.0f, 8.0f).withCentre ({ win.getRight() - 10.0f, win.getBottom() - 10.0f });
        drawJewel (g, lp, col::red, 1.0f);
    }
}

// ---------------------------------------------------------------------------
namespace { constexpr float kBpRange = 4.0f, kBpSweep = 0.95f; }   // ±4 V full scale, one tick per volt

void BPlusMeter::renderFace (int w, int h, float scale)
{
    face = juce::Image (juce::Image::ARGB, juce::jmax (1, (int) std::ceil (w * scale)), juce::jmax (1, (int) std::ceil (h * scale)), true);
    faceScale = scale;
    juce::Graphics g (face);
    g.addTransform (juce::AffineTransform::scale (scale));
    const auto all = juce::Rectangle<float> ((float) w, (float) h);
    const float d = juce::jmin ((float) w, (float) h) - 4.0f;
    const auto ring = juce::Rectangle<float> (d, d).withCentre (all.getCentre());
    juce::ColourGradient bz (juce::Colour (0xff55575c), ring.getX(), ring.getY(), juce::Colour (0xff111214), ring.getX(), ring.getBottom(), false);
    g.setColour (juce::Colours::black.withAlpha (0.6f));
    g.fillEllipse (ring.expanded (2.0f));
    g.setGradientFill (bz);
    g.fillEllipse (ring);
    const auto dial = ring.reduced (d * 0.07f);
    {
        juce::ColourGradient trim (juce::Colour (0xfff0d9a4), dial.getX(), dial.getY(), juce::Colour (0xff5a4527), dial.getX(), dial.getBottom(), false);
        trim.addColour (0.45, juce::Colour (0xffb89659));
        g.setGradientFill (trim);
        g.fillEllipse (dial.expanded (2.0f));
        g.setColour (juce::Colours::black.withAlpha (0.8f));
        g.fillEllipse (dial.expanded (0.5f));
    }
    juce::ColourGradient card (juce::Colour (0xfffbefcf), dial.getCentreX(), dial.getCentreY() + dial.getHeight() * 0.2f,
                               juce::Colour (0xffd2b97f), dial.getX(), dial.getY(), true);
    g.setGradientFill (card);
    g.fillEllipse (dial);
    const float D = dial.getHeight();
    const auto c = dial.getCentre().translated (0.0f, D * 0.05f);
    const float r = D * 0.30f;
    auto pt = [&] (float ang, float rad) { return c + juce::Point<float> (std::sin (ang), -std::cos (ang)) * rad; };
    juce::Path arc;
    arc.addCentredArc (c.x, c.y, r, r, 0.0f, -kBpSweep, kBpSweep, true);
    g.setColour (col::ink);
    g.strokePath (arc, juce::PathStrokeType (0.9f));
    // the band of a healthy supply (±1.5 V) in green
    {
        juce::Path ok;
        ok.addCentredArc (c.x, c.y, r - 2.2f, r - 2.2f, 0.0f, -kBpSweep * 1.5f / kBpRange, kBpSweep * 1.5f / kBpRange, true);
        g.setColour (juce::Colour (0xff3f7d4a).withAlpha (0.75f));
        g.strokePath (ok, juce::PathStrokeType (2.6f));
    }
    g.setColour (col::ink);
    g.setFont (Fonts::strong (D * 0.105f, 0.0f));
    for (int k = -4; k <= 4; ++k)
    {
        const float a = kBpSweep * (float) k / 4.0f;
        const bool major = k % 2 == 0;
        g.drawLine ({ pt (a, r), pt (a, r + (major ? 5.5f : 3.2f)) }, major ? 1.1f : 0.7f);
        if (major)
            g.drawText (k == 0 ? juce::String ("0") : ((k > 0 ? "+" : juce::String::fromUTF8 ("\xe2\x88\x92")) + juce::String (std::abs (k))),
                        juce::Rectangle<float> (18.0f, 10.0f).withCentre (pt (a, r + 11.5f)), juce::Justification::centred, false);
    }
    g.setFont (Fonts::displayBold (D * 0.135f, 0.04f));
    g.drawText ("B+", juce::Rectangle<float> (D * 0.5f, D * 0.14f).withCentre (c.translated (0.0f, D * 0.16f)), juce::Justification::centred, false);
}

void BPlusMeter::paint (juce::Graphics& g)
{
    const float scale = g.getInternalContext().getPhysicalPixelScaleFactor();
    if (face.isNull() || std::abs (scale - faceScale) > 0.01f || face.getWidth() != (int) std::ceil (getWidth() * scale))
        renderFace (getWidth(), getHeight(), scale);
    g.drawImageTransformed (face, juce::AffineTransform::scale (1.0f / faceScale));

    const auto all = getLocalBounds().toFloat();
    const float d = juce::jmin (all.getWidth(), all.getHeight()) - 4.0f;
    const auto dial = juce::Rectangle<float> (d, d).withCentre (all.getCentre()).reduced (d * 0.07f);
    const float D = dial.getHeight();
    const auto c = dial.getCentre().translated (0.0f, D * 0.05f);
    const float r = D * 0.30f;
    const float a = kBpSweep * juce::jlimit (-1.1f, 1.1f, (v - nom) / kBpRange);
    const juce::Point<float> dir (std::sin (a), -std::cos (a));
    g.setColour (juce::Colours::black.withAlpha (0.16f));
    g.drawLine ({ c.translated (1.6f, 2.2f), (c + dir * (r + 4.0f)).translated (1.6f, 2.2f) }, 1.3f);
    g.setColour (col::red.darker (0.25f));
    g.drawLine ({ c, c + dir * (r + 4.0f) }, 1.15f);
    // pivot cap
    {
        const auto pc = juce::Rectangle<float> (7.0f, 7.0f).withCentre (c);
        juce::ColourGradient cap (juce::Colour (0xff5b5b5e), pc.getX(), pc.getY(), juce::Colour (0xff0b0b0c), pc.getRight(), pc.getBottom(), false);
        g.setGradientFill (cap);
        g.fillEllipse (pc);
    }
    g.setColour (col::ink.withAlpha (0.8f));
    g.setFont (Fonts::strong (D * 0.10f, 0.02f));
    g.drawText (juce::String (v, 1) + " V", juce::Rectangle<float> (D * 0.6f, D * 0.12f).withCentre (c.translated (0.0f, D * 0.29f)), juce::Justification::centred, false);
    // glass
    juce::ColourGradient gl (juce::Colours::white.withAlpha (0.22f), dial.getX(), dial.getY(), juce::Colours::white.withAlpha (0.0f), dial.getX(), dial.getCentreY(), false);
    g.setGradientFill (gl);
    g.fillEllipse (dial.reduced (dial.getWidth() * 0.06f).withHeight (dial.getHeight() * 0.45f));
}

} // namespace vt4ui
