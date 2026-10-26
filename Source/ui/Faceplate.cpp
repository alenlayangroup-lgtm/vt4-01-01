#include "Faceplate.h"
#include "../Parameters.h"

namespace vt4ui
{
using namespace layout;

namespace
{
    constexpr float kA0 = -150.0f * juce::MathConstants<float>::pi / 180.0f;
    constexpr float kA1 =  150.0f * juce::MathConstants<float>::pi / 180.0f;

    juce::Point<float> polar (juce::Point<float> c, float ang, float r)
    {
        return c + juce::Point<float> (std::sin (ang), -std::cos (ang)) * r;
    }

    float angleFor (double v) { return kA0 + (float) v * (kA1 - kA0); }

    void scaleTick (juce::Graphics& g, juce::Point<float> c, float r, double v, float len, float w, juce::Colour colr)
    {
        const float a = angleFor (v);
        g.setColour (col::engraveShadow);
        g.drawLine ({ polar (c, a, r + 2.0f).translated (0.5f, 0.7f), polar (c, a, r + 2.0f + len).translated (0.5f, 0.7f) }, w);
        g.setColour (colr);
        g.drawLine ({ polar (c, a, r + 2.0f), polar (c, a, r + 2.0f + len) }, w);
    }

    void scaleLabel (juce::Graphics& g, juce::Point<float> c, float r, double v, const juce::String& t, float size = 9.5f, juce::Colour colr = col::brass)
    {
        const float a = angleFor (v);
        const auto p = polar (c, a, r);
        drawEngraved (g, t, juce::Rectangle<float> (34.0f, 12.0f).withCentre (p), Fonts::label (size, 0.02f), juce::Justification::centred, colr, 0.8f);
    }

    juce::String stepLabel (int band, int k)
    {
        const double f = vt4::eq::kSteps[band][k];
        if (band == 0) return juce::String ((int) f);
        if (band == 1) return f >= 1000.0 ? juce::String (f / 1000.0, 1) + "k" : juce::String ((int) f);
        return juce::String (f / 1000.0, f / 1000.0 < 10.0 ? 1 : 0);
    }

    void screw (juce::Graphics& g, juce::Point<float> c, float r, float rot)
    {
        g.setColour (juce::Colours::black.withAlpha (0.6f));
        g.fillEllipse (c.x - r - 1.2f, c.y - r - 0.6f, 2 * r + 2.4f, 2 * r + 2.4f);
        juce::ColourGradient hd (juce::Colour (0xffb9b5ad), c.x - r * 0.6f, c.y - r * 0.8f, juce::Colour (0xff3d3c3a), c.x + r * 0.7f, c.y + r, false);
        g.setGradientFill (hd);
        g.fillEllipse (c.x - r, c.y - r, 2 * r, 2 * r);
        g.setColour (juce::Colours::black.withAlpha (0.7f));
        for (int k = 0; k < 2; ++k)
        {
            const float a = rot + juce::MathConstants<float>::halfPi * (float) k;
            const juce::Point<float> d (std::cos (a), std::sin (a));
            g.drawLine ({ c - d * r * 0.62f, c + d * r * 0.62f }, r * 0.26f);
        }
        g.setColour (juce::Colours::white.withAlpha (0.25f));
        g.drawEllipse (c.x - r + 0.4f, c.y - r + 0.4f, 2 * r - 0.8f, 2 * r - 0.8f, 0.6f);
    }

    void engravedLine (juce::Graphics& g, float x0, float y0, float x1, float y1, juce::Colour c = col::brassFaint)
    {
        g.setColour (juce::Colours::black.withAlpha (0.55f));
        g.drawLine (x0 + 0.5f, y0 + 0.7f, x1 + 0.5f, y1 + 0.7f, 1.0f);
        g.setColour (c);
        g.drawLine (x0, y0, x1, y1, 1.0f);
    }
}

// ---------------------------------------------------------------------------
Faceplate::Faceplate (juce::AudioProcessorValueTreeState& s) : state (s)
{
    using namespace vt4p;
    for (int r = 0; r < 2; ++r)
    {
        auto& row = rows[(size_t) r];
        addAndMakeVisible (row.hpf);
        addAndMakeVisible (row.weight);
        for (int b = 0; b < 4; ++b)
        {
            addAndMakeVisible (row.freq[(size_t) b]);
            row.gain[(size_t) b].getProperties().set ("large", true);
            addAndMakeVisible (row.gain[(size_t) b]);
        }
        row.focus = std::make_unique<ChoiceButtons> (*state.getParameter (ids::focus (r)), juce::StringArray { "WIDE", "NARROW" }, 9.0f);
        row.focus->gap = 3;
        addAndMakeVisible (*row.focus);
    }
    attachRowB (false);
    {
        auto& row = rows[0];
        row.att.push_back (std::make_unique<SA> (state, ids::hpf (0), row.hpf));
        row.att.push_back (std::make_unique<SA> (state, ids::weight (0), row.weight));
        for (int b = 0; b < 4; ++b)
        {
            row.att.push_back (std::make_unique<SA> (state, ids::freq (0, b), row.freq[(size_t) b]));
            row.att.push_back (std::make_unique<SA> (state, ids::gain (0, b), row.gain[(size_t) b]));
        }
    }
    addAndMakeVisible (drive);
    addAndMakeVisible (output);
    drive.getProperties().set ("large", false);
    driveAtt = std::make_unique<SA> (state, ids::drive, drive);
    outputAtt = std::make_unique<SA> (state, ids::output, output);

    mode = std::make_unique<ChoiceButtons> (*state.getParameter (ids::mode), juce::StringArray { "CLEAN", "WARM", "BOLD" }, 11.0f);
    mode->gap = 9;
    stereo = std::make_unique<ChoiceButtons> (*state.getParameter (ids::stereo), juce::StringArray { "LINK", "DUAL", "M/S" }, 10.0f);
    stereo->gap = 6;
    iron = std::make_unique<ChoiceButtons> (*state.getParameter (ids::iron), juce::StringArray { "OFF", "NICKEL", "STEEL" }, 10.0f);
    iron->gap = 6;
    // bool parameter: index 0 = false (SMOOTH), 1 = true (STEPPED)
    stepped = std::make_unique<ChoiceButtons> (*state.getParameter (ids::stepped), juce::StringArray { "SMOOTH", "STEPPED" }, 10.0f);
    stepped->gap = 6;
    bypass = std::make_unique<ToggleKey> (*state.getParameter (ids::bypass), "BYPASS", 10.0f);
    for (auto* c : std::initializer_list<juce::Component*> { mode.get(), stereo.get(), iron.get(), stepped.get(), bypass.get() })
        addAndMakeVisible (c);

    for (auto* b : { &rearButton, &screenButton, &copyButton })
    {
        b->getProperties().set ("fontSize", 10.0f);
        addAndMakeVisible (b);
    }
    screenButton.setClickingTogglesState (false);
    rearButton.onClick = [this] { if (onRear) onRear(); };
    screenButton.onClick = [this] { if (onScreenToggle) onScreenToggle(); };
    copyButton.onClick = [this] { if (onCopyAtoB) onCopyAtoB(); };
    copyButton.setTooltip (juce::String::fromUTF8 ("Copy channel A settings to channel B"));

    for (auto* c : std::initializer_list<juce::Component*> { &vuL, &vuR, &bplusMeter, &driveLamp, &onLamp })
        addAndMakeVisible (c);

    // double-click returns to the default
    auto setDefault = [this] (Knob& k, const juce::String& id)
    {
        if (auto* p = state.getParameter (id))
            k.setDoubleClickReturnValue (true, p->convertFrom0to1 (p->getDefaultValue()));
    };
    for (int r = 0; r < 2; ++r)
    {
        auto& row = rows[(size_t) r];
        setDefault (row.hpf, ids::hpf (r));
        setDefault (row.weight, ids::weight (r));
        for (int b = 0; b < 4; ++b) { setDefault (row.freq[(size_t) b], ids::freq (r, b)); setDefault (row.gain[(size_t) b], ids::gain (r, b)); }
    }
    setDefault (drive, ids::drive);
    setDefault (output, ids::output);
    applySnapping();
}

Faceplate::~Faceplate() = default;

void Faceplate::attachRowB (bool linked)
{
    using namespace vt4p;
    auto& row = rows[1];
    row.att.clear();
    const int src = linked ? 0 : 1;
    row.att.push_back (std::make_unique<SA> (state, ids::hpf (src), row.hpf));
    row.att.push_back (std::make_unique<SA> (state, ids::weight (src), row.weight));
    for (int b = 0; b < 4; ++b)
    {
        row.att.push_back (std::make_unique<SA> (state, ids::freq (src, b), row.freq[(size_t) b]));
        row.att.push_back (std::make_unique<SA> (state, ids::gain (src, b), row.gain[(size_t) b]));
    }
    linkedNow = linked;
    const float alpha = linked ? 0.42f : 1.0f;
    for (auto* c : std::initializer_list<juce::Component*> { &row.hpf, &row.weight })
    {
        c->setAlpha (alpha); c->setEnabled (! linked);
    }
    for (int b = 0; b < 4; ++b)
    {
        row.freq[(size_t) b].setAlpha (alpha); row.freq[(size_t) b].setEnabled (! linked);
        row.gain[(size_t) b].setAlpha (alpha); row.gain[(size_t) b].setEnabled (! linked);
    }
    row.focus->setAlpha (alpha);
    row.focus->setEnabled (! linked);
}

void Faceplate::applySnapping()
{
    for (auto& row : rows)
    {
        if (steppedMode)
        {
            row.hpf.snapper = [] (double v) { return std::round (v * 6.0) / 6.0; };
            row.weight.snapper = [] (double v) { return std::round (v); };
            for (int b = 0; b < 4; ++b)
            {
                const int n = vt4::eq::kNumSteps[b];
                row.freq[(size_t) b].snapper = [n] (double v) { return std::round (v * (n - 1)) / (n - 1); };
                row.gain[(size_t) b].snapper = [] (double v) { return std::round (v * 2.0) / 2.0; };
            }
        }
        else
        {
            row.hpf.snapper = nullptr; row.weight.snapper = nullptr;
            for (int b = 0; b < 4; ++b) { row.freq[(size_t) b].snapper = nullptr; row.gain[(size_t) b].snapper = nullptr; }
        }
    }
}

void Faceplate::setStereoMode (int m)
{
    if (m == stereoMode && linkedNow == (m == vt4::kLink)) return;
    stereoMode = m;
    if ((m == vt4::kLink) != linkedNow) attachRowB (m == vt4::kLink);
    vuL.setLabel (m == vt4::kMidSide ? "OUTPUT  L" : "OUTPUT  L");
    repaint();
}

void Faceplate::setStepped (bool s)
{
    if (s == steppedMode) return;
    steppedMode = s;
    applySnapping();
}

void Faceplate::setMeters (float l, float r, bool pl, bool pr, float bp, float bpNom, float drv, bool active)
{
    vuL.setValue (l); vuR.setValue (r);
    vuL.setPeak (pl); vuR.setPeak (pr);
    bplusMeter.setValue (bp, bpNom);
    driveLamp.setLevel (drv);
    onLamp.setLevel (active ? 0.95f : 0.06f);
}

void Faceplate::resized()
{
    auto place = [] (juce::Component& c, KnobPos k, int cy)
    {
        const int box = (int) std::lround (k.d * 1.18);       // the knob body leaves room for its shadow
        c.setBounds (k.x - box / 2, cy - box / 2, box, box);
    };
    for (int r = 0; r < 2; ++r)
    {
        auto& row = rows[(size_t) r];
        const int cy = rowY[r] + knobY;
        place (row.hpf, hpf, cy);
        place (row.weight, weight, cy);
        for (int b = 0; b < 4; ++b) { place (row.freq[(size_t) b], freq[b], cy); place (row.gain[(size_t) b], gain[b], cy); }
        row.focus->setBounds (secX1[3] - 108, rowY[r] + 9, 100, 20);
    }
    place (drive, layout::drive, controlKnobY);
    place (output, layout::output, controlKnobY);
    mode->setBounds (modeButtons);
    driveLamp.setBounds (layout::driveLamp);
    vuL.setBounds (layout::vuL);
    vuR.setBounds (layout::vuR);
    bplusMeter.setBounds (layout::bplusMeter);
    stereo->setBounds (stereoButtons);
    copyButton.setBounds (layout::copyButton);
    iron->setBounds (ironButtons);
    stepped->setBounds (steppedButton);
    bypass->setBounds (bypassButton);
    onLamp.setBounds (bypassLamp);
    rearButton.setBounds (layout::rearButton);
    screenButton.setBounds (layout::screenButton);
}

// ---------------------------------------------------------------------------
void Faceplate::renderPlate (int w, int h, float scale)
{
    plate = juce::Image (juce::Image::ARGB, juce::jmax (1, (int) std::ceil (w * scale)), juce::jmax (1, (int) std::ceil (h * scale)), true);
    plateScale = scale;
    renderBrushedMetal (plate, scale, juce::Colour (0xff2e3136), juce::Colour (0xff26292d), juce::Colour (0xff1b1d20), 0x2545f491u);
    juce::Graphics g (plate);
    g.addTransform (juce::AffineTransform::scale (scale));
    const float W = (float) w, H = (float) h;

    // plate edges: a turned bevel, light on top, dark below
    g.setColour (juce::Colours::white.withAlpha (0.13f));
    g.drawLine (0, 0.6f, W, 0.6f, 1.2f);
    g.setColour (juce::Colours::white.withAlpha (0.05f));
    g.drawLine (0.6f, 0, 0.6f, H, 1.0f);
    g.setColour (juce::Colours::black.withAlpha (0.65f));
    g.drawLine (0, H - 0.6f, W, H - 0.6f, 1.2f);
    g.drawLine (W - 0.6f, 0, W - 0.6f, H, 1.0f);

    // ---- header -------------------------------------------------------------
    drawEngraved (g, "HALDEN", { 40.0f, 24.0f, 320.0f, 44.0f }, Fonts::display (40.0f, 0.42f), juce::Justification::centredLeft, col::brass, 1.2f);
    engravedLine (g, 42.0f, 74.0f, 336.0f, 74.0f, col::brassDim.withAlpha (0.7f));
    drawEngraved (g, "VT-4", { 40.0f, 80.0f, 160.0f, 36.0f }, Fonts::displayBold (30.0f, 0.06f), juce::Justification::centredLeft, col::brass, 1.1f);
    drawEngraved (g, "STEREO TUBE EQUALISER", { 150.0f, 86.0f, 220.0f, 24.0f }, Fonts::label (10.5f, 0.24f), juce::Justification::centredLeft, col::brassDim);
    drawEngraved (g, juce::String::fromUTF8 ("PASSIVE LC  \xc2\xb7  CLASS A  \xc2\xb7  TRANSFORMER COUPLED"), { 42.0f, 120.0f, 330.0f, 16.0f },
                  Fonts::label (8.6f, 0.22f), juce::Justification::centredLeft, col::brassDim);

    // control captions
    {
        const float capY = (float) controlCaptionY;
        drawEngraved (g, "INPUT DRIVE", { (float) layout::drive.x - 50.0f, capY, 100.0f, 12.0f }, Fonts::label (9.0f, 0.2f), juce::Justification::centred);
        drawEngraved (g, "OUTPUT", { (float) layout::output.x - 50.0f, capY, 100.0f, 12.0f }, Fonts::label (9.0f, 0.2f), juce::Justification::centred);
        const auto lampR = layout::driveLamp.toFloat();
        drawEngraved (g, "DRIVE", { lampR.getCentreX() - 30.0f, lampR.getBottom() + 1.0f, 60.0f, 12.0f }, Fonts::label (8.4f, 0.2f), juce::Justification::centred, col::brassDim);
    }
    {
        const juce::Point<float> dc ((float) layout::drive.x, (float) controlKnobY), oc ((float) layout::output.x, (float) controlKnobY);
        const float rd = layout::drive.d * 0.5f;
        for (int db = -6; db <= 12; ++db)
            scaleTick (g, dc, rd, (db + 6) / 18.0, db % 6 == 0 ? 6.0f : 3.5f, db % 6 == 0 ? 1.1f : 0.8f, col::brass.withAlpha (0.9f));
        for (int db : { -6, 0, 6, 12 })
            scaleLabel (g, dc, rd + 15.0f, (db + 6) / 18.0, (db > 0 ? "+" : (db < 0 ? juce::String::fromUTF8 ("\xe2\x88\x92") : juce::String())) + juce::String (std::abs (db)), 9.0f);
        for (int db = -6; db <= 6; ++db)
            scaleTick (g, oc, rd, (db + 6) / 12.0, db % 3 == 0 ? 6.0f : 3.5f, db % 3 == 0 ? 1.1f : 0.8f, col::brass.withAlpha (0.9f));
        for (int db : { -6, -3, 0, 3, 6 })
            scaleLabel (g, oc, rd + 15.0f, (db + 6) / 12.0, (db > 0 ? "+" : (db < 0 ? juce::String::fromUTF8 ("\xe2\x88\x92") : juce::String())) + juce::String (std::abs (db)), 9.0f);
    }
    // header separator
    engravedLine (g, 16.0f, (float) headerH, W - 16.0f, (float) headerH, juce::Colours::white.withAlpha (0.06f));

    // ---- channel rows -------------------------------------------------------
    static const char* titles[5] = { "HIGH-PASS", "LOW", "BODY", "PRESENCE", "AIR" };
    for (int r = 0; r < 2; ++r)
    {
        const float y0 = (float) rowY[r];
        // channel tag: a milled pocket
        {
            auto tag = juce::Rectangle<float> (20.0f, y0 + 12.0f, 44.0f, (float) rowH - 26.0f);
            juce::ColourGradient pocket (juce::Colours::black.withAlpha (0.50f), 0.0f, tag.getY(), juce::Colours::black.withAlpha (0.28f), 0.0f, tag.getBottom(), false);
            g.setGradientFill (pocket);
            g.fillRoundedRectangle (tag, 5.0f);
            g.setColour (juce::Colours::black.withAlpha (0.55f));
            g.drawRoundedRectangle (tag.translated (0.0f, -0.6f), 5.0f, 1.0f);
            g.setColour (juce::Colours::white.withAlpha (0.07f));
            g.drawRoundedRectangle (tag.translated (0.0f, 0.8f), 5.0f, 0.8f);
            drawEngraved (g, r == 0 ? "A" : "B", tag.withHeight (52.0f).translated (0.0f, 8.0f), Fonts::displayBold (30.0f), juce::Justification::centred);
        }
        for (int s = 0; s < 5; ++s)
        {
            const float x0 = (float) secX0[s], x1 = (float) secX1[s];
            const auto fr = juce::Rectangle<float> (x0, y0 + 18.0f, x1 - x0, (float) rowH - 28.0f);
            const auto tf = Fonts::label (10.0f, 0.26f);
            const float tw = juce::GlyphArrangement::getStringWidth (tf, titles[s]) + 16.0f;
            const auto tr = juce::Rectangle<float> (x0 + 12.0f, y0 + 11.0f, tw, 14.0f);
            auto gapArea = tr;
            if (s == 3)   // PRESENCE: the WIDE / NARROW keys also sit on the frame
                gapArea = gapArea.getUnion (juce::Rectangle<float> (x1 - 112.0f, y0 + 6.0f, 108.0f, 26.0f));
            drawFrame (g, fr, gapArea, 7.0f, col::brass.withAlpha (0.30f));
            drawEngraved (g, titles[s], tr, tf, juce::Justification::centred);
        }

        const float cy = y0 + (float) knobY;
        auto caption = [&] (int x, int d, const juce::String& t)
        {
            drawEngraved (g, t, juce::Rectangle<float> (90.0f, 12.0f).withCentre ({ (float) x, cy + d * 0.5f + 34.0f }), Fonts::label (8.6f, 0.22f), juce::Justification::centred, col::brassDim);
        };
        // HPF positions
        {
            const juce::Point<float> c ((float) hpf.x, cy);
            const float rr = hpf.d * 0.5f;
            static const char* hl[7] = { "OFF", "18", "25", "35", "50", "70", "100" };
            for (int k = 0; k < 7; ++k)
            {
                scaleTick (g, c, rr, k / 6.0, 4.5f, 1.0f, col::brass.withAlpha (0.9f));
                scaleLabel (g, c, rr + 16.0f, k / 6.0, hl[k], 9.0f);
            }
            caption (hpf.x, hpf.d, "Hz  12 dB/oct");
        }
        for (int b = 0; b < 4; ++b)
        {
            // frequency switch positions
            const juce::Point<float> c ((float) freq[b].x, cy);
            const float rr = freq[b].d * 0.5f;
            const int n = vt4::eq::kNumSteps[b];
            for (int k = 0; k < n; ++k)
            {
                scaleTick (g, c, rr, k / (double) (n - 1), 4.5f, 1.0f, col::brass.withAlpha (0.9f));
                scaleLabel (g, c, rr + 16.0f, k / (double) (n - 1), stepLabel (b, k), n > 6 ? 8.6f : 9.2f);
            }
            caption (freq[b].x, freq[b].d, b == 0 ? "FREQ  Hz" : (b == 1 ? "FREQ  Hz" : "FREQ  kHz"));
            // gain scale
            const juce::Point<float> cg ((float) gain[b].x, cy);
            const float rg = gain[b].d * 0.5f;
            const int mx = (int) vt4::eq::kMaxGain[b];
            const int half = mx / 2;
            for (int dB = -mx; dB <= mx; ++dB)
            {
                const double v = (dB + mx) / (2.0 * mx);
                const bool major = dB % half == 0;
                scaleTick (g, cg, rg, v, major ? 6.5f : 3.5f, major ? 1.2f : 0.8f, dB == 0 ? col::brass : col::brass.withAlpha (0.85f));
            }
            for (int dB : { -mx, -half, 0, half, mx })
            {
                const double v = (dB + mx) / (2.0 * mx);
                scaleLabel (g, cg, rg + 17.0f, v, dB == 0 ? juce::String ("0") : ((dB > 0 ? "+" : juce::String::fromUTF8 ("\xe2\x88\x92")) + juce::String (std::abs (dB))), 9.6f);
            }
            caption (gain[b].x, gain[b].d - 6, "GAIN  dB");
        }
        // weight
        {
            const juce::Point<float> c ((float) weight.x, cy);
            const float rr = weight.d * 0.5f;
            for (int k = 0; k <= 10; ++k)
                scaleTick (g, c, rr, k / 10.0, k % 5 == 0 ? 5.0f : 3.0f, k % 5 == 0 ? 1.0f : 0.7f, col::brass.withAlpha (0.9f));
            for (int k : { 0, 5, 10 })
                scaleLabel (g, c, rr + 14.0f, k / 10.0, juce::String (k), 9.0f);
            caption (weight.x, weight.d + 8, "WEIGHT");
        }
    }

    // ---- bottom strip ---------------------------------------------------------
    engravedLine (g, 16.0f, (float) stripY - 2.0f, W - 16.0f, (float) stripY - 2.0f, juce::Colours::white.withAlpha (0.06f));
    drawEngraved (g, "STEREO", layout::stereoCaption, Fonts::label (9.0f, 0.24f), juce::Justification::centredLeft);
    drawEngraved (g, "IRON", layout::ironCaption, Fonts::label (9.0f, 0.24f), juce::Justification::centredLeft);
    drawEngraved (g, "KNOBS", layout::knobsCaption, Fonts::label (9.0f, 0.24f), juce::Justification::centredLeft);

    // screws
    for (auto p : { juce::Point<float> (13.0f, 13.0f), juce::Point<float> (W - 13.0f, 13.0f) })
        screw (g, p, 5.0f, 0.4f + p.x * 0.01f);
}

void Faceplate::paint (juce::Graphics& g)
{
    const float scale = g.getInternalContext().getPhysicalPixelScaleFactor();
    if (plate.isNull() || std::abs (scale - plateScale) > 0.01f)
        renderPlate (getWidth(), getHeight(), scale);
    g.drawImageTransformed (plate, juce::AffineTransform::scale (1.0f / plateScale));

    // dynamic engraving: channel assignment and the unit's serial number
    const char* subA = stereoMode == vt4::kMidSide ? "M" : (stereoMode == vt4::kDual ? "L" : "L+R");
    const char* subB = stereoMode == vt4::kMidSide ? "S" : (stereoMode == vt4::kDual ? "R" : "LINK");
    for (int r = 0; r < 2; ++r)
    {
        const auto tag = juce::Rectangle<float> (20.0f, (float) rowY[r] + 12.0f, 44.0f, (float) rowH - 26.0f);
        drawEngraved (g, r == 0 ? subA : subB, tag.withTrimmedTop (64.0f).withHeight (16.0f), Fonts::label (r == 1 && stereoMode == vt4::kLink ? 8.0f : 11.0f, 0.12f),
                      juce::Justification::centred, r == 1 && stereoMode == vt4::kLink ? col::amber.withAlpha (0.8f) : col::brass);
    }
    drawEngraved (g, juce::String::fromUTF8 ("SERIAL N\xc2\xba ") + juce::String (serial).paddedLeft ('0', 4), { 42.0f, 138.0f, 200.0f, 14.0f },
                  Fonts::label (8.4f, 0.24f), juce::Justification::centredLeft, col::brassDim);
}

} // namespace vt4ui
