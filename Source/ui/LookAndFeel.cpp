#include "LookAndFeel.h"
#include "VT4BinaryData.h"

#include <map>

namespace vt4ui
{

Fonts& Fonts::get()
{
    static Fonts f = []
    {
        Fonts x;
        x.medium        = juce::Typeface::createSystemTypefaceFor (VT4BinaryData::InterMedium_otf,       (size_t) VT4BinaryData::InterMedium_otfSize);
        x.semibold      = juce::Typeface::createSystemTypefaceFor (VT4BinaryData::InterSemiBold_otf,     (size_t) VT4BinaryData::InterSemiBold_otfSize);
        x.displayLight  = juce::Typeface::createSystemTypefaceFor (VT4BinaryData::InterDisplayLight_otf, (size_t) VT4BinaryData::InterDisplayLight_otfSize);
        x.displayMedium = juce::Typeface::createSystemTypefaceFor (VT4BinaryData::InterDisplayMedium_otf,(size_t) VT4BinaryData::InterDisplayMedium_otfSize);
        return x;
    }();
    return f;
}

static juce::Font makeFont (juce::Typeface::Ptr t, float h, float tracking)
{
    return juce::Font (juce::FontOptions (t).withHeight (h).withKerningFactor (tracking));
}

juce::Font Fonts::label (float h, float tracking)       { return makeFont (get().medium, h, tracking); }
juce::Font Fonts::strong (float h, float tracking)      { return makeFont (get().semibold, h, tracking); }
juce::Font Fonts::display (float h, float tracking)     { return makeFont (get().displayLight, h, tracking); }
juce::Font Fonts::displayBold (float h, float tracking) { return makeFont (get().displayMedium, h, tracking); }

void drawEngraved (juce::Graphics& g, const juce::String& text, juce::Rectangle<float> r, const juce::Font& f,
                   juce::Justification j, juce::Colour fill, float depth)
{
    g.setFont (f);
    // cut edge (dark, upper left: the light comes from above) and catch-light on the lower lip
    g.setColour (col::engraveShadow.withMultipliedAlpha (0.85f));
    g.drawText (text, r.translated (-0.25f * depth, -0.7f * depth), j, false);
    g.setColour (juce::Colours::white.withAlpha (0.085f));
    g.drawText (text, r.translated (0.0f, 0.75f * depth), j, false);
    // paint fill: a little brighter at the top edge, as lit inlay
    const float fh = f.getHeight();
    const float cy = r.getCentreY();
    juce::ColourGradient paint (fill.brighter (0.22f), 0.0f, cy - fh * 0.45f, fill.darker (0.18f), 0.0f, cy + fh * 0.45f, false);
    g.setGradientFill (paint);
    g.drawText (text, r, j, false);
}

// ---------------------------------------------------------------------------
// Brushed metal: every row of the plate gets its own grain level plus a long
// correlated streak along x (an AR(1) process, correlation length ≈ 90 design
// px), under a broad vertical sheen — the way a horizontally brushed,
// anodised plate reflects a light from above.
void renderBrushedMetal (juce::Image& img, float scale, juce::Colour top, juce::Colour mid, juce::Colour bottom, uint32_t seed)
{
    const int pw = img.getWidth(), ph = img.getHeight();
    juce::Image::BitmapData bd (img, juce::Image::BitmapData::writeOnly);
    if (seed == 0) seed = 0x2545f491u;
    auto uni = [&seed]
    {
        seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
        return (float) (seed & 0xffffffu) * (1.0f / 16777215.0f);
    };
    auto tri = [&uni] { return (uni() + uni() - 1.0f) * 1.2247f; };     // unit variance
    const float rho = std::exp (-1.0f / (90.0f * scale));
    const float drive = std::sqrt (1.0f - rho * rho);
    for (int y = 0; y < ph; ++y)
    {
        const float fy = (float) y / (float) juce::jmax (1, ph - 1);
        const juce::Colour base = fy < 0.30f ? top.interpolatedWith (mid, fy / 0.30f) : mid.interpolatedWith (bottom, (fy - 0.30f) / 0.70f);
        const float br = base.getFloatRed(), bgc = base.getFloatGreen(), bb = base.getFloatBlue();
        const float rowN = tri();
        const float rowAmp = 0.35f + 0.65f * uni();
        float sN = tri();
        for (int x = 0; x < pw; ++x)
        {
            sN = rho * sN + drive * tri();
            const float fx = (float) x / (float) juce::jmax (1, pw - 1);
            const float sheen = 0.075f * std::exp (-((fx - 0.30f) * (fx - 0.30f)) / 0.045f) + 0.03f * std::exp (-((fx - 0.78f) * (fx - 0.78f)) / 0.02f);
            const float vig = 1.0f - 0.10f * std::pow (std::abs (2.0f * fx - 1.0f), 3.0f) - 0.06f * std::pow (std::abs (2.0f * fy - 1.0f), 4.0f);
            const float k = (1.0f + 0.050f * rowN + 0.040f * rowAmp * sN + sheen) * vig;
            bd.setPixelColour (x, y, juce::Colour::fromFloatRGBA (juce::jlimit (0.0f, 1.0f, br * k),
                                                                  juce::jlimit (0.0f, 1.0f, bgc * k),
                                                                  juce::jlimit (0.0f, 1.0f, bb * k), 1.0f));
        }
    }
}

void drawFrame (juce::Graphics& g, juce::Rectangle<float> fr, juce::Rectangle<float> gapArea, float corner, juce::Colour line)
{
    g.saveState();
    if (! gapArea.isEmpty()) g.excludeClipRegion (gapArea.toNearestInt());
    g.setColour (juce::Colours::black.withAlpha (0.50f));
    g.drawRoundedRectangle (fr.translated (0.0f, -0.7f), corner, 1.0f);
    g.setColour (juce::Colours::white.withAlpha (0.045f));
    g.drawRoundedRectangle (fr.translated (0.0f, 0.8f), corner, 1.0f);
    g.setColour (line);
    g.drawRoundedRectangle (fr, corner, 0.9f);
    g.restoreState();
}

// ---------------------------------------------------------------------------
// Knob body: rendered per pixel with a small physical lighting model and
// cached per pixel size (the body is rotation-invariant: fixed lights, a
// symmetric knurl), so only the pointer is drawn per frame.
//
//   cap      turned aluminium: concentric tool marks; anisotropic (Ward)
//            reflection of two lights gives the characteristic "bow tie"
//   chamfer  diamond-cut polished bevel (sharp Blinn–Phong highlight)
//   skirt    black anodised cone with a V-knurl, both ridge faces lit
//            separately, 4×4 supersampled
namespace
{
    struct V3 { float x, y, z; };
    inline V3 nrm (V3 v) { const float l = std::sqrt (v.x * v.x + v.y * v.y + v.z * v.z); return { v.x / l, v.y / l, v.z / l }; }
    inline float dot (V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline float hash01 (int i)
    {
        uint32_t x = (uint32_t) i * 747796405u + 2891336453u;
        x = ((x >> ((x >> 28u) + 4u)) ^ x) * 277803737u;
        x = (x >> 22u) ^ x;
        return (float) (x & 0xffffffu) / 16777215.0f;
    }
    struct Rgb { float r = 0, g = 0, b = 0; };
    inline Rgb operator* (Rgb a, float k) { return { a.r * k, a.g * k, a.b * k }; }
    inline Rgb operator+ (Rgb a, Rgb b) { return { a.r + b.r, a.g + b.g, a.b + b.b }; }
}

static juce::Image renderKnobBody (int px, bool large)
{
    juce::Image img (juce::Image::ARGB, px, px, true);
    const float s = (float) px;
    const float cx = s * 0.5f, cy = s * 0.5f;
    const float R = s * 0.5f / 1.18f;                  // skirt radius (room left for the shadow)
    const float rc = R * 0.76f;                        // cap radius
    const float chamfer = juce::jmax (1.1f, rc * 0.085f);
    const float gap = juce::jmax (0.9f, R * 0.028f);
    const float rimW = juce::jmax (0.8f, R * 0.03f);
    const int nKnurl = large ? 100 : 76;

    {
        juce::Graphics g (img);
        juce::Path disc;
        disc.addEllipse (cx - R, cy - R, 2 * R, 2 * R);
        juce::DropShadow (juce::Colours::black.withAlpha (0.50f), juce::jmax (2, (int) std::lround (R * 0.32f)), { 0, juce::jmax (1, (int) std::lround (R * 0.17f)) }).drawForPath (g, disc);
        juce::DropShadow (juce::Colours::black.withAlpha (0.70f), juce::jmax (1, (int) std::lround (R * 0.07f)), { 0, juce::jmax (1, (int) std::lround (R * 0.035f)) }).drawForPath (g, disc);
    }

    const V3 L[2] = { nrm ({ -0.30f, -0.85f, 0.75f }), nrm ({ 0.80f, 0.10f, 0.55f }) };   // key (top-left), weak fill (right)
    const float Lw[2] = { 1.0f, 0.28f };
    V3 H[2];
    for (int k = 0; k < 2; ++k) H[k] = nrm ({ L[k].x, L[k].y, L[k].z + 1.0f });
    const Rgb alu { 0.655f, 0.650f, 0.640f }, specCol { 1.0f, 0.975f, 0.93f };
    const Rgb ano { 0.050f, 0.051f, 0.055f };

    auto shade = [&] (float dx, float dy) -> Rgb
    {
        const float r = std::sqrt (dx * dx + dy * dy);
        const float inv = r > 1.0e-4f ? 1.0f / r : 0.0f;
        const float ux = dx * inv, uy = dy * inv;          // radial
        const float tx = -uy, ty = ux;                      // tangential
        if (r <= rc - chamfer)
        {
            // tool marks: random ring brightness, ~1.6 rings per pixel of the cap
            const float rr = r / rc * 140.0f;
            const int i0 = (int) rr;
            const float fr = rr - (float) i0;
            const float ring = (1.0f - fr) * hash01 (i0) + fr * hash01 (i0 + 1) - 0.5f;
            const float slow = 0.5f * std::sin (r / rc * 23.0f + 1.3f) + 0.5f * std::sin (r / rc * 61.0f + 0.4f);
            float spec = 0.0f;
            for (int k = 0; k < 2; ++k)
            {
                const float ht = H[k].x * tx + H[k].y * ty;      // across the grooves: narrow lobe
                const float hr = H[k].x * ux + H[k].y * uy;
                const float aT = 0.085f, aR = 0.55f;
                spec += Lw[k] * std::exp (-(ht * ht) / (aT * aT) - (hr * hr) / (aR * aR)) / (H[k].z * H[k].z);
            }
            const float diff = 0.62f + 0.20f * dot ({ 0, 0, 1 }, L[0]);
            // a slight dish towards the centre
            const float dish = 1.0f - 0.05f * (1.0f - r / rc);
            const float tex = 1.0f + 0.07f * ring + 0.03f * slow;
            return alu * (diff * tex * dish) + specCol * (0.50f * spec * (0.85f + 0.5f * ring));
        }
        if (r <= rc)
        {
            const float tilt = 0.80f;
            const V3 N = nrm ({ ux * std::sin (tilt), uy * std::sin (tilt), std::cos (tilt) });
            float d = 0.0f, sp = 0.0f;
            for (int k = 0; k < 2; ++k)
            {
                d += Lw[k] * juce::jmax (0.0f, dot (N, L[k]));
                sp += Lw[k] * std::pow (juce::jmax (0.0f, dot (N, H[k])), 90.0f);
            }
            return Rgb { 0.62f, 0.615f, 0.60f } * (0.18f + 0.55f * d) + specCol * (1.6f * sp);
        }
        if (r <= rc + gap)
            return { 0.012f, 0.012f, 0.014f };
        // skirt
        const float t = (r - rc - gap) / (R - rc - gap);    // 0 inner … 1 outer
        const float phi = std::atan2 (dy, dx);
        const float kk = phi * (float) nKnurl / juce::MathConstants<float>::twoPi;
        const float fk = kk - std::floor (kk);
        const float faceSign = fk < 0.5f ? 1.0f : -1.0f;
        const float crest = 1.0f - std::abs (fk - 0.5f) * 2.0f;   // 1 at the crest … 0 in the valley
        const float beta = 0.50f + 0.25f * t;                       // the cone steepens towards the rim
        const float gamma = 0.62f;
        V3 N = nrm ({ ux * std::sin (beta) + faceSign * tx * std::sin (gamma),
                      uy * std::sin (beta) + faceSign * ty * std::sin (gamma),
                      std::cos (beta) });
        float d = 0.0f, sp = 0.0f;
        for (int k = 0; k < 2; ++k)
        {
            d += Lw[k] * juce::jmax (0.0f, dot (N, L[k]));
            sp += Lw[k] * std::pow (juce::jmax (0.0f, dot (N, H[k])), 24.0f);
        }
        const float valley = 0.55f + 0.45f * std::sqrt (juce::jlimit (0.0f, 1.0f, crest * 1.6f));
        Rgb c = ano * (0.6f + 2.2f * d) * valley + specCol * (0.16f * sp * valley);
        // turned outer rim: a thin bright edge, strongest towards the light
        if (r > R - rimW)
        {
            const float w = (r - (R - rimW)) / rimW;
            const float lit = juce::jmax (0.0f, -(ux * 0.30f + uy * 0.85f));
            c = c + Rgb { 0.33f, 0.33f, 0.33f } * (lit * (1.0f - w) * 0.9f);
        }
        return c;
    };

    juce::Image::BitmapData bd (img, juce::Image::BitmapData::readWrite);
    for (int y = 0; y < px; ++y)
        for (int x = 0; x < px; ++x)
        {
            const float dx0 = (float) x + 0.5f - cx, dy0 = (float) y + 0.5f - cy;
            const float r0 = std::sqrt (dx0 * dx0 + dy0 * dy0);
            if (r0 > R + 1.0f) continue;
            const int ss = r0 > rc - chamfer - 1.5f ? 4 : 2;
            Rgb acc;
            int cov = 0;
            for (int sy = 0; sy < ss; ++sy)
                for (int sx = 0; sx < ss; ++sx)
                {
                    const float dx = (float) x + ((float) sx + 0.5f) / (float) ss - cx;
                    const float dy = (float) y + ((float) sy + 0.5f) / (float) ss - cy;
                    if (dx * dx + dy * dy > R * R) continue;
                    acc = acc + shade (dx, dy);
                    ++cov;
                }
            if (cov == 0) continue;
            const float a = (float) cov / (float) (ss * ss);
            const auto src = juce::Colour::fromFloatRGBA (juce::jlimit (0.0f, 1.0f, acc.r / (float) cov),
                                                          juce::jlimit (0.0f, 1.0f, acc.g / (float) cov),
                                                          juce::jlimit (0.0f, 1.0f, acc.b / (float) cov), a);
            bd.setPixelColour (x, y, bd.getPixelColour (x, y).overlaidWith (src));
        }
    return img;
}

void drawKnob (juce::Graphics& g, juce::Rectangle<float> b, float angle, bool large, bool enabled, float hover)
{
    static std::map<int, juce::Image> cache[2];
    const float scale = g.getInternalContext().getPhysicalPixelScaleFactor();
    const int px = juce::jmax (8, (int) std::ceil (juce::jmin (b.getWidth(), b.getHeight()) * scale));
    auto& m = cache[large ? 1 : 0];
    auto it = m.find (px);
    if (it == m.end())
    {
        if (m.size() > 24) m.clear();
        it = m.emplace (px, renderKnobBody (px, large)).first;
    }
    const float side = (float) px / scale;
    const auto area = juce::Rectangle<float> (side, side).withCentre (b.getCentre());
    g.drawImage (it->second, area, juce::RectanglePlacement::stretchToFit);

    const juce::Point<float> c = area.getCentre();
    const float R = side * 0.5f / 1.18f, rc = R * 0.76f;
    const juce::Point<float> d (std::sin (angle), -std::cos (angle));
    // engraved pointer line, paint-filled
    const auto p0 = c + d * (rc * 0.26f);
    const auto p1 = c + d * (rc * 0.88f);
    const float w = juce::jmax (1.3f, rc * 0.095f);
    g.setColour (juce::Colours::black.withAlpha (0.55f));
    g.drawLine ({ p0.translated (0.0f, -0.5f), p1.translated (0.0f, -0.5f) }, w + 1.2f);
    g.setColour (juce::Colours::white.withAlpha (0.35f));
    g.drawLine ({ p0.translated (0.0f, 0.6f), p1.translated (0.0f, 0.6f) }, w);
    g.setColour (enabled ? juce::Colour (0xff17171a) : juce::Colour (0xff3a3a3c));
    g.drawLine ({ p0, p1 }, w);
    // index mark on the skirt: brass inlay
    const auto q0 = c + d * (R * 0.84f);
    const auto q1 = c + d * (R * 0.985f);
    g.setColour (juce::Colours::black.withAlpha (0.5f));
    g.drawLine ({ q0, q1 }, juce::jmax (1.8f, R * 0.075f));
    g.setColour (col::brass.withAlpha (enabled ? 1.0f : 0.45f));
    g.drawLine ({ q0, q1 }, juce::jmax (1.1f, R * 0.045f));
    if (hover > 0.0f)
    {
        g.setColour (col::amber.withAlpha (0.12f * hover));
        g.drawEllipse (area.reduced (side * 0.07f), 1.2f);
    }
}

void drawJewel (juce::Graphics& g, juce::Rectangle<float> r, juce::Colour c, float level)
{
    const auto cen = r.getCentre();
    const float rad = juce::jmin (r.getWidth(), r.getHeight()) * 0.5f;
    level = juce::jlimit (0.0f, 1.0f, level);
    // bezel
    juce::ColourGradient bz (juce::Colour (0xff6e6a62), cen.x, cen.y - rad, juce::Colour (0xff151515), cen.x, cen.y + rad, false);
    g.setGradientFill (bz);
    g.fillEllipse (r);
    const float rg = rad * 0.72f;
    // glow halo
    if (level > 0.01f)
    {
        juce::ColourGradient halo (c.withAlpha (0.45f * level), cen.x, cen.y, c.withAlpha (0.0f), cen.x, cen.y + rad * 2.6f, true);
        g.setGradientFill (halo);
        g.fillEllipse (cen.x - rad * 2.6f, cen.y - rad * 2.6f, rad * 5.2f, rad * 5.2f);
    }
    // glass dome
    const juce::Colour lit = c.interpolatedWith (juce::Colours::white, 0.35f * level);
    const juce::Colour off = c.darker (2.4f).withSaturation (0.5f);
    juce::ColourGradient dome (lit.interpolatedWith (off, 1.0f - level), cen.x - rg * 0.3f, cen.y - rg * 0.3f,
                               off.darker (0.6f), cen.x + rg, cen.y + rg, true);
    g.setGradientFill (dome);
    g.fillEllipse (cen.x - rg, cen.y - rg, 2 * rg, 2 * rg);
    // facets
    for (int i = 0; i < 6; ++i)
    {
        const float a = juce::MathConstants<float>::twoPi * (float) i / 6.0f + 0.3f;
        g.setColour (juce::Colours::white.withAlpha (0.05f + 0.05f * level));
        g.drawLine (cen.x, cen.y, cen.x + std::cos (a) * rg * 0.95f, cen.y + std::sin (a) * rg * 0.95f, 0.6f);
    }
    g.setColour (juce::Colours::white.withAlpha (0.55f));
    g.fillEllipse (cen.x - rg * 0.55f, cen.y - rg * 0.6f, rg * 0.45f, rg * 0.32f);
}

// ---------------------------------------------------------------------------
VT4LookAndFeel::VT4LookAndFeel()
{
    setColour (juce::Slider::textBoxTextColourId, col::amber);
    setColour (juce::Slider::textBoxBackgroundColourId, juce::Colour (0xff0b0c0d));
    setColour (juce::Slider::textBoxOutlineColourId, juce::Colours::transparentBlack);
    setColour (juce::TooltipWindow::backgroundColourId, juce::Colour (0xf0101113));
    setColour (juce::TooltipWindow::textColourId, col::brass);
    setColour (juce::BubbleComponent::backgroundColourId, juce::Colour (0xf20d0e10));
    setColour (juce::BubbleComponent::outlineColourId, col::amberDim);
    setColour (juce::TextButton::buttonColourId, juce::Colour (0xff1a1b1e));
}

void VT4LookAndFeel::drawRotarySlider (juce::Graphics& g, int x, int y, int w, int h, float pos,
                                       float start, float end, juce::Slider& s)
{
    const bool large = s.getProperties()["large"];
    const float hover = s.isMouseOverOrDragging() ? 1.0f : 0.0f;
    drawKnob (g, juce::Rectangle<float> ((float) x, (float) y, (float) w, (float) h), start + pos * (end - start), large, s.isEnabled(), hover);
}

void VT4LookAndFeel::drawButtonBackground (juce::Graphics& g, juce::Button& b, const juce::Colour&, bool over, bool down)
{
    auto r = b.getLocalBounds().toFloat().reduced (1.0f);
    const bool on = b.getToggleState();
    const float cr = juce::jmin (3.0f, r.getHeight() * 0.18f);
    // recess in the plate
    g.setColour (juce::Colours::black.withAlpha (0.55f));
    g.fillRoundedRectangle (r.expanded (1.0f), cr + 1.0f);
    // key cap
    juce::ColourGradient cap (juce::Colour (down ? 0xff141518 : 0xff2a2c30), r.getX(), r.getY(),
                              juce::Colour (0xff111214), r.getX(), r.getBottom(), false);
    g.setGradientFill (cap);
    g.fillRoundedRectangle (r, cr);
    if (on)
    {
        // back-lit window
        juce::ColourGradient glow (col::amber.withAlpha (0.20f), r.getCentreX(), r.getCentreY(),
                                   col::amber.withAlpha (0.02f), r.getX(), r.getY(), true);
        g.setGradientFill (glow);
        g.fillRoundedRectangle (r.reduced (1.5f), cr);
        g.setColour (col::amber.withAlpha (0.85f));
        g.fillRoundedRectangle (r.getX() + r.getWidth() * 0.3f, r.getY() + 2.0f, r.getWidth() * 0.4f, 1.6f, 0.8f);
    }
    g.setColour (juce::Colours::white.withAlpha (over ? 0.16f : 0.09f));
    g.drawLine (r.getX() + cr, r.getY() + 0.5f, r.getRight() - cr, r.getY() + 0.5f, 1.0f);
    g.setColour (juce::Colours::black.withAlpha (0.7f));
    g.drawRoundedRectangle (r, cr, 0.8f);
}

juce::Font VT4LookAndFeel::getTextButtonFont (juce::TextButton& b, int h)
{
    const float sz = b.getProperties().contains ("fontSize") ? (float) b.getProperties()["fontSize"] : juce::jmin (11.0f, (float) h * 0.48f);
    return Fonts::label (sz, 0.1f);
}

void VT4LookAndFeel::drawButtonText (juce::Graphics& g, juce::TextButton& b, bool, bool)
{
    const bool on = b.getToggleState();
    auto r = b.getLocalBounds().toFloat().reduced (2.0f);
    const auto f = getTextButtonFont (b, b.getHeight());
    g.setFont (f);
    if (on)
    {
        g.setColour (col::amber.withAlpha (0.35f));
        g.drawText (b.getButtonText(), r.translated (0.0f, 0.6f), juce::Justification::centred, false);
        g.setColour (juce::Colour (0xffffd79a));
        g.drawText (b.getButtonText(), r, juce::Justification::centred, false);
    }
    else
    {
        g.setColour (juce::Colours::black.withAlpha (0.8f));
        g.drawText (b.getButtonText(), r.translated (0.5f, 0.7f), juce::Justification::centred, false);
        g.setColour (b.isEnabled() ? col::brassDim : col::brassDim.withAlpha (0.4f));
        g.drawText (b.getButtonText(), r, juce::Justification::centred, false);
    }
}

void VT4LookAndFeel::drawBubble (juce::Graphics& g, juce::BubbleComponent&, const juce::Point<float>&, const juce::Rectangle<float>& body)
{
    g.setColour (juce::Colour (0xf20b0c0d));
    g.fillRoundedRectangle (body, 3.0f);
    g.setColour (col::amberDim);
    g.drawRoundedRectangle (body.reduced (0.5f), 3.0f, 1.0f);
}

juce::Font VT4LookAndFeel::getSliderPopupFont (juce::Slider&) { return Fonts::strong (12.0f, 0.04f); }
int VT4LookAndFeel::getSliderPopupPlacement (juce::Slider&) { return juce::BubbleComponent::above; }

juce::Label* VT4LookAndFeel::createSliderTextBox (juce::Slider& s)
{
    auto* l = LookAndFeel_V4::createSliderTextBox (s);
    l->setFont (Fonts::strong (11.0f));
    l->setColour (juce::Label::textColourId, col::amber);
    return l;
}

void VT4LookAndFeel::drawCornerResizer (juce::Graphics& g, int w, int h, bool over, bool dragging)
{
    // three engraved dots in the corner of the bezel
    const float a = dragging ? 0.95f : (over ? 0.75f : 0.45f);
    const float d = juce::jmin ((float) w, (float) h);
    const float step = d * 0.26f, r = juce::jmax (1.0f, d * 0.065f);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j <= i; ++j)
        {
            const float x = (float) w - step * 0.7f - step * (float) (i - j);
            const float y = (float) h - step * 0.7f - step * (float) j;
            g.setColour (juce::Colours::black.withAlpha (0.6f));
            g.fillEllipse (x - r, y - r + 0.6f, 2 * r, 2 * r);
            g.setColour (col::brass.withAlpha (a));
            g.fillEllipse (x - r, y - r, 2 * r, 2 * r);
        }
}

void VT4LookAndFeel::drawTooltip (juce::Graphics& g, const juce::String& text, int w, int h)
{
    g.fillAll (juce::Colour (0xf0101113));
    g.setColour (col::amberDim);
    g.drawRect (0, 0, w, h, 1);
    g.setColour (col::brass);
    g.setFont (Fonts::label (12.0f, 0.02f));
    g.drawFittedText (text, 6, 2, w - 12, h - 4, juce::Justification::centredLeft, 3);
}

} // namespace vt4ui
