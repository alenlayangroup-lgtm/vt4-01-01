#include "Screen.h"
#include "../PluginProcessor.h"

namespace vt4ui
{

namespace
{
    constexpr double kFMin = 10.0, kFMax = 40000.0;
    constexpr double kDbRange = 16.0;        // curve: ±16 dB
    constexpr double kAnaFloor = -96.0;      // analyser: 0 … −96 dBFS
    const juce::Colour kSetCol[2] = { col::amber, col::green };
    const char* kBandLetter[5] = { "H", "L", "B", "P", "A" };
    const char* kBandName[5] = { "HIGH-PASS", "LOW", "BODY", "PRESENCE", "AIR" };

    juce::String minus() { return juce::String::fromUTF8 ("\xe2\x88\x92"); }
}

Screen::Screen (VT4AudioProcessor& p, vt4::DisplayModel& m) : proc (p), model (m)
{
    setOpaque (false);
    std::vector<double> grid ((size_t) vt4::DisplayModel::kN);
    for (int i = 0; i < vt4::DisplayModel::kN; ++i) grid[(size_t) i] = model.freq (i);
    anaRate = proc.getHostRate();
    anaIn.prepare (grid, anaRate);
    anaOut.prepare (grid, anaRate);
    showIn.assign (grid.size(), -200.0);
    showOut.assign (grid.size(), -200.0);
    analyserMode = (int) proc.apvts.state.getProperty ("analyser", 1);
}

// ---- geometry ------------------------------------------------------------------
juce::Rectangle<float> Screen::glassArea() const { return getLocalBounds().toFloat().reduced (24.0f, 16.0f).withTrimmedBottom (2.0f); }
juce::Rectangle<float> Screen::plotArea() const
{
    return glassArea().withTrimmedLeft (38.0f).withTrimmedRight (44.0f).withTrimmedTop (30.0f).withTrimmedBottom (24.0f);
}
float Screen::xFor (double f) const
{
    const auto p = plotArea();
    return p.getX() + p.getWidth() * (float) (std::log (f / kFMin) / std::log (kFMax / kFMin));
}
double Screen::fFor (float x) const
{
    const auto p = plotArea();
    return kFMin * std::exp ((double) (x - p.getX()) / p.getWidth() * std::log (kFMax / kFMin));
}
float Screen::yFor (double db) const
{
    const auto p = plotArea();
    return p.getCentreY() - (float) (db / kDbRange) * p.getHeight() * 0.5f;
}
double Screen::dbFor (float y) const
{
    const auto p = plotArea();
    return (double) (p.getCentreY() - y) / (p.getHeight() * 0.5) * kDbRange;
}
float Screen::yAna (double db) const
{
    const auto p = plotArea();
    return p.getY() + p.getHeight() * (float) juce::jlimit (0.0, 1.0, db / kAnaFloor);
}

int Screen::activeSet() const
{
    const int st = model.params().stereo;
    return st == vt4::kLink ? 0 : editSet;
}

void Screen::resized() { glassCache = {}; }

// ---- nodes ----------------------------------------------------------------------
std::vector<Screen::Node> Screen::nodes (int set) const
{
    std::vector<Node> v;
    const auto& prm = model.params();
    const auto& ch = prm.ch[(size_t) (prm.stereo == vt4::kLink ? 0 : set)];
    {
        const bool on = ch.hpfHz > 0.0;
        v.push_back ({ 0, { on ? xFor (ch.hpfHz) : xFor (12.0), yFor (on ? -3.0 : 0.0) }, on });
    }
    for (int b = 0; b < vt4::kNumBands; ++b)
    {
        const auto& bp = ch.band[(size_t) b];
        v.push_back ({ b + 1, { xFor (bp.freq), yFor (bp.gainDb) }, std::fabs (bp.gainDb) > 1.0e-6 });
    }
    return v;
}

juce::Rectangle<float> Screen::buttonRect (Hit h) const
{
    const auto g = glassArea();
    const float y = g.getY() + 7.0f;
    switch (h)
    {
        case Hit::AnaOff:   return { g.getX() + 92.0f, y, 34.0f, 16.0f };
        case Hit::AnaOut:   return { g.getX() + 130.0f, y, 34.0f, 16.0f };
        case Hit::AnaInOut: return { g.getX() + 168.0f, y, 52.0f, 16.0f };
        case Hit::EditA:    return { g.getRight() - 64.0f, y, 26.0f, 16.0f };
        case Hit::EditB:    return { g.getRight() - 34.0f, y, 26.0f, 16.0f };
        default: break;
    }
    if (selBand >= 0 && (h == Hit::Solo || h == Hit::Delta))
    {
        for (const auto& n : nodes (activeSet()))
            if (n.band == selBand)
            {
                const auto p = plotArea();
                const float tx = juce::jlimit (p.getX() + 2.0f, p.getRight() - 172.0f, n.p.x - 86.0f);
                const float ty = n.p.y > p.getY() + 50.0f ? n.p.y - 40.0f : n.p.y + 16.0f;
                return h == Hit::Solo ? juce::Rectangle<float> (tx + 130.0f, ty + 3.0f, 18.0f, 18.0f)
                                      : juce::Rectangle<float> (tx + 151.0f, ty + 3.0f, 18.0f, 18.0f);
            }
    }
    return {};
}

Screen::Hit Screen::hitTestAt (juce::Point<float> pt, int& band) const
{
    band = -1;
    for (Hit h : { Hit::AnaOff, Hit::AnaOut, Hit::AnaInOut, Hit::Solo, Hit::Delta })
        if (buttonRect (h).contains (pt)) { band = selBand; return h; }
    if (model.params().stereo != vt4::kLink)
        for (Hit h : { Hit::EditA, Hit::EditB })
            if (buttonRect (h).contains (pt)) return h;
    float best = 13.0f;
    for (const auto& n : nodes (activeSet()))
    {
        const float d = n.p.getDistanceFrom (pt);
        if (d < best) { best = d; band = n.band; }
    }
    return band >= 0 ? Hit::Node : Hit::None;
}

// ---- parameters ----------------------------------------------------------------------
void Screen::setParam (const juce::String& id, float norm)
{
    if (auto* p = proc.apvts.getParameter (id))
        p->setValueNotifyingHost (juce::jlimit (0.0f, 1.0f, norm));
}

void Screen::beginDrag (int band)
{
    endDrag();
    const int s = activeSet();
    if (band == 0) gestureIds = { vt4p::ids::hpf (s) };
    else gestureIds = { vt4p::ids::freq (s, band - 1), vt4p::ids::gain (s, band - 1) };
    for (auto& id : gestureIds)
        if (auto* p = proc.apvts.getParameter (id)) p->beginChangeGesture();
}

void Screen::endDrag()
{
    for (auto& id : gestureIds)
        if (auto* p = proc.apvts.getParameter (id)) p->endChangeGesture();
    gestureIds.clear();
}

void Screen::dragTo (juce::Point<float> pt, bool fine)
{
    if (dragBand < 0) return;
    const int s = activeSet();
    const bool stepped = proc.pp.stepped->load() > 0.5f;
    const float k = fine ? 0.15f : 1.0f;
    const float dx = (pt.x - dragStart.x) * k, dy = (pt.y - dragStart.y) * k;
    const double f = fFor (xFor (dragF0) + dx);
    if (dragBand == 0)
    {
        double pos = f < 15.0 ? 0.0 : vt4p::posFromHpf (juce::jlimit (18.0, 100.0, f));
        if (stepped) pos = std::round (pos * 6.0) / 6.0;
        setParam (vt4p::ids::hpf (s), (float) pos);
        return;
    }
    const int b = dragBand - 1;
    double pos = vt4p::posFromFreq (b, f);
    if (stepped) { const int n = vt4::eq::kNumSteps[b]; pos = std::round (pos * (n - 1)) / (n - 1); }
    setParam (vt4p::ids::freq (s, b), (float) pos);
    const double mx = vt4::eq::kMaxGain[b];
    double db = juce::jlimit (-mx, mx, dbFor (yFor (dragG0) + dy));
    if (stepped) db = std::round (db * 2.0) / 2.0;
    if (auto* p = proc.apvts.getParameter (vt4p::ids::gain (s, b)))
        p->setValueNotifyingHost (p->convertTo0to1 ((float) db));
}

// ---- mouse ---------------------------------------------------------------------------
void Screen::mouseMove (const juce::MouseEvent& e)
{
    int band = -1;
    const Hit h = hitTestAt (e.position, band);
    if (h != hoverHit || band != hoverBand) { hoverHit = h; hoverBand = h == Hit::Node ? band : -1; repaint(); }
    setMouseCursor (h == Hit::Node ? juce::MouseCursor::DraggingHandCursor
                                   : (h == Hit::None ? juce::MouseCursor::NormalCursor : juce::MouseCursor::PointingHandCursor));
}

void Screen::mouseExit (const juce::MouseEvent&) { hoverHit = Hit::None; hoverBand = -1; repaint(); }

void Screen::mouseDown (const juce::MouseEvent& e)
{
    int band = -1;
    const Hit h = hitTestAt (e.position, band);
    switch (h)
    {
        case Hit::AnaOff: case Hit::AnaOut: case Hit::AnaInOut:
            analyserMode = h == Hit::AnaOff ? 0 : (h == Hit::AnaOut ? 1 : 2);
            proc.apvts.state.setProperty ("analyser", analyserMode, nullptr);
            repaint(); return;
        case Hit::EditA: editSet = 0; selBand = -1; repaint(); return;
        case Hit::EditB: editSet = 1; selBand = -1; repaint(); return;
        case Hit::Solo: case Hit::Delta:
        {
            const int want = h == Hit::Solo ? 1 : 2;
            if (latchedMode == want && latchedBand == selBand && latchedSet == activeSet()) latchedMode = 0;
            else { latchedMode = want; latchedBand = selBand; latchedSet = activeSet(); }
            repaint(); return;
        }
        case Hit::Node:
        {
            selBand = band;
            dragBand = band;
            dragStart = e.position;
            const auto& prm = model.params();
            const auto& ch = prm.ch[(size_t) (prm.stereo == vt4::kLink ? 0 : activeSet())];
            if (band == 0) { dragF0 = ch.hpfHz > 0.0 ? ch.hpfHz : 12.0; dragG0 = 0.0; }
            else { dragF0 = ch.band[(size_t) (band - 1)].freq; dragG0 = ch.band[(size_t) (band - 1)].gainDb; }
            beginDrag (band);
            if (e.mods.isAltDown()) tempMode = (e.mods.isCommandDown() || e.mods.isCtrlDown()) ? 2 : 1;
            repaint();
            return;
        }
        default:
            selBand = -1;
            repaint();
            return;
    }
}

void Screen::mouseDrag (const juce::MouseEvent& e)
{
    if (dragBand < 0) return;
    dragTo (e.position, e.mods.isShiftDown());
}

void Screen::mouseUp (const juce::MouseEvent&)
{
    if (dragBand >= 0) endDrag();
    dragBand = -1;
    tempMode = 0;
    repaint();
}

void Screen::mouseDoubleClick (const juce::MouseEvent& e)
{
    int band = -1;
    if (hitTestAt (e.position, band) != Hit::Node) return;
    const int s = activeSet();
    if (band == 0) { setParam (vt4p::ids::hpf (s), 0.0f); return; }
    if (auto* p = proc.apvts.getParameter (vt4p::ids::gain (s, band - 1)))
        p->setValueNotifyingHost (p->convertTo0to1 (0.0f));
}

void Screen::mouseWheelMove (const juce::MouseEvent& e, const juce::MouseWheelDetails& w)
{
    int band = -1;
    if (hitTestAt (e.position, band) != Hit::Node) return;
    const int s = activeSet();
    const float dir = (w.deltaY != 0.0f ? w.deltaY : w.deltaX) > 0.0f ? 1.0f : -1.0f;
    if (band == 1)
    {
        if (auto* p = proc.apvts.getParameter (vt4p::ids::weight (s)))
        {
            const bool stepped = proc.pp.stepped->load() > 0.5f;
            const float cur = p->convertFrom0to1 (p->getValue());
            p->setValueNotifyingHost (p->convertTo0to1 (juce::jlimit (0.0f, 10.0f, (stepped ? std::round (cur) : cur) + dir * (stepped ? 1.0f : 0.5f))));
        }
    }
    else if (band == 3)
        setParam (vt4p::ids::focus (s), dir > 0.0f ? 1.0f : 0.0f);
}

void Screen::audition (int& mode, int& set, int& band) const
{
    if (tempMode != 0 && dragBand >= 0) { mode = tempMode; set = activeSet(); band = dragBand; return; }
    mode = latchedMode; set = latchedSet; band = juce::jmax (0, latchedBand);
    if (latchedBand < 0) mode = 0;
}

// ---- analyser ---------------------------------------------------------------------------
void Screen::tick()
{
    if (proc.getHostRate() != anaRate)
    {
        std::vector<double> grid ((size_t) vt4::DisplayModel::kN);
        for (int i = 0; i < vt4::DisplayModel::kN; ++i) grid[(size_t) i] = model.freq (i);
        anaRate = proc.getHostRate();
        anaIn.prepare (grid, anaRate);
        anaOut.prepare (grid, anaRate);
    }
    if (proc.anaIn.pull (scratch) > 0) anaIn.push (scratch.data(), (int) scratch.size());
    if (proc.anaOut.pull (scratch) > 0) anaOut.push (scratch.data(), (int) scratch.size());
    if (analyserMode == 0) return;
    if (++frameDiv < 2) return;          // ≈ 25 frames per second
    frameDiv = 0;
    // Ballistics in the power domain (fast attack, ≈ 180 ms release): noise
    // settles to a calm trace while a steady sine keeps its calibrated level.
    auto ballistics = [] (std::vector<double>& show, const std::vector<double>& fresh)
    {
        for (size_t i = 0; i < show.size(); ++i)
        {
            if (show[i] < -190.0) { show[i] = fresh[i]; continue; }
            const double a = fresh[i] > show[i] ? 0.55 : 0.20;
            const double pw = a * std::pow (10.0, 0.1 * fresh[i]) + (1.0 - a) * std::pow (10.0, 0.1 * show[i]);
            show[i] = 10.0 * std::log10 (pw + 1.0e-30);
        }
    };
    anaOut.compute();
    ballistics (showOut, anaOut.values());
    if (analyserMode == 2)
    {
        anaIn.compute();
        ballistics (showIn, anaIn.values());
    }
    repaint (plotArea().toNearestInt().expanded (2));
}

// ---- painting -----------------------------------------------------------------------------
void Screen::drawBezel (juce::Graphics& g)
{
    const auto outer = getLocalBounds().toFloat().reduced (12.0f, 6.0f);
    // recess in the faceplate
    g.setColour (juce::Colours::black.withAlpha (0.55f));
    g.fillRoundedRectangle (outer.expanded (2.0f), 12.0f);
    juce::ColourGradient bz (juce::Colour (0xff3a3c41), outer.getX(), outer.getY(), juce::Colour (0xff121315), outer.getX(), outer.getBottom(), false);
    g.setGradientFill (bz);
    g.fillRoundedRectangle (outer, 10.0f);
    g.setColour (juce::Colours::white.withAlpha (0.08f));
    g.drawRoundedRectangle (outer.reduced (1.0f), 10.0f, 1.0f);
    // glass
    const auto gl = glassArea();
    g.setColour (juce::Colours::black);
    g.fillRoundedRectangle (gl.expanded (2.0f), 6.0f);
    juce::ColourGradient glass (juce::Colour (0xff121110), gl.getCentreX(), gl.getCentreY(), juce::Colour (0xff050506), gl.getX(), gl.getY(), true);
    g.setGradientFill (glass);
    g.fillRoundedRectangle (gl, 5.0f);
    // scanlines (instrument tube)
    g.setColour (juce::Colours::black.withAlpha (0.18f));
    for (float y = gl.getY(); y < gl.getBottom(); y += 2.0f) g.drawHorizontalLine ((int) y, gl.getX(), gl.getRight());
    // screws on the bezel
    for (auto p : { outer.getTopLeft().translated (12.0f, 12.0f), outer.getTopRight().translated (-12.0f, 12.0f),
                    outer.getBottomLeft().translated (12.0f, -12.0f), outer.getBottomRight().translated (-12.0f, -12.0f) })
    {
        juce::ColourGradient hd (juce::Colour (0xffa9a59d), p.x - 2.0f, p.y - 3.0f, juce::Colour (0xff3a3937), p.x + 3.0f, p.y + 3.0f, false);
        g.setGradientFill (hd);
        g.fillEllipse (p.x - 3.5f, p.y - 3.5f, 7.0f, 7.0f);
        g.setColour (juce::Colours::black.withAlpha (0.7f));
        g.drawLine (p.x - 2.2f, p.y - 1.0f, p.x + 2.2f, p.y + 1.0f, 1.0f);
    }
}

void Screen::drawGrid (juce::Graphics& g)
{
    const auto p = plotArea();
    const auto amberGrid = col::amber.withAlpha (0.07f);
    const auto amberMajor = col::amber.withAlpha (0.16f);
    g.setFont (Fonts::label (9.0f, 0.04f));
    for (double dec : { 10.0, 100.0, 1000.0, 10000.0 })
        for (int k = 1; k <= 9; ++k)
        {
            const double f = dec * k;
            if (f < kFMin || f > kFMax) continue;
            const float x = xFor (f);
            g.setColour (k == 1 ? amberMajor : amberGrid);
            g.drawVerticalLine ((int) std::round (x), p.getY(), p.getBottom());
        }
    for (double f : { 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10000.0, 20000.0, 40000.0 })
    {
        const float x = xFor (f);
        const juce::String t = f >= 1000.0 ? juce::String ((int) (f / 1000.0)) + "k" : juce::String ((int) f);
        g.setColour (col::amber.withAlpha (0.55f));
        g.drawText (t, juce::Rectangle<float> (40.0f, 14.0f).withCentre ({ x, p.getBottom() + 12.0f }), juce::Justification::centred, false);
    }
    for (int db = -15; db <= 15; db += 3)
    {
        const float y = yFor (db);
        g.setColour (db == 0 ? col::amber.withAlpha (0.28f) : amberGrid);
        g.drawHorizontalLine ((int) std::round (y), p.getX(), p.getRight());
        if (db % 6 == 0)
        {
            g.setColour (col::amber.withAlpha (db == 0 ? 0.8f : 0.5f));
            g.drawText ((db > 0 ? "+" : (db < 0 ? minus() : juce::String())) + juce::String (std::abs (db)),
                        juce::Rectangle<float> (p.getRight() + 4.0f, y - 7.0f, 34.0f, 14.0f), juce::Justification::centredLeft, false);
        }
    }
    g.setColour (col::amber.withAlpha (0.35f));
    g.drawText ("dB", juce::Rectangle<float> (p.getRight() + 4.0f, p.getY() - 2.0f, 34.0f, 12.0f), juce::Justification::centredLeft, false);
    // analyser scale (left)
    if (analyserMode != 0)
    {
        g.setColour (juce::Colour (0xff8fb3c9).withAlpha (0.45f));
        for (int db : { 0, -24, -48, -72 })
            g.drawText (db == 0 ? juce::String ("0") : minus() + juce::String (-db), juce::Rectangle<float> (p.getX() - 36.0f, yAna (db) - 6.0f, 30.0f, 12.0f), juce::Justification::centredRight, false);
        g.drawText ("dBFS", juce::Rectangle<float> (p.getX() - 38.0f, p.getBottom() - 12.0f, 34.0f, 12.0f), juce::Justification::centredRight, false);
    }
    // above the host Nyquist frequency nothing reaches the output
    const double ny = 0.5 * model.hostRate();
    if (ny < kFMax)
    {
        const float x = xFor (ny);
        g.saveState();
        g.reduceClipRegion (juce::Rectangle<float> (x, p.getY(), p.getRight() - x, p.getHeight()).toNearestInt());
        g.setColour (juce::Colours::black.withAlpha (0.35f));
        g.fillRect (juce::Rectangle<float> (x, p.getY(), p.getRight() - x, p.getHeight()));
        g.setColour (col::amber.withAlpha (0.05f));
        for (float k = -p.getHeight(); k < p.getRight() - x; k += 7.0f)
            g.drawLine (x + k, p.getBottom(), x + k + p.getHeight(), p.getY(), 1.0f);
        g.restoreState();
        g.setColour (col::amber.withAlpha (0.4f));
        g.setFont (Fonts::label (8.5f, 0.1f));
        g.drawText ("fs/2", juce::Rectangle<float> (x + 4.0f, p.getY() + 2.0f, 40.0f, 12.0f), juce::Justification::centredLeft, false);
    }
}

void Screen::drawAnalyser (juce::Graphics& g)
{
    if (analyserMode == 0) return;
    const auto p = plotArea();
    const int N = vt4::DisplayModel::kN;
    const double ny = 0.5 * model.hostRate();
    auto build = [&] (const std::vector<double>& v, bool closed)
    {
        juce::Path path;
        bool started = false;
        float lastX = p.getX();
        for (int i = 0; i < N; ++i)
        {
            const double f = model.freq (i);
            if (f > ny * 0.995) break;
            const float x = xFor (f), y = yAna (v[(size_t) i]);
            if (! started) { if (closed) path.startNewSubPath (x, p.getBottom()); else path.startNewSubPath (x, y); started = true; }
            path.lineTo (x, y);
            lastX = x;
        }
        if (closed && started) { path.lineTo (lastX, p.getBottom()); path.closeSubPath(); }
        return path;
    };
    g.saveState();
    g.reduceClipRegion (p.toNearestInt());
    if (analyserMode == 2)
    {
        g.setColour (juce::Colour (0xff8fb3c9).withAlpha (0.38f));
        g.strokePath (build (showIn, false), juce::PathStrokeType (1.0f));
    }
    const auto fill = build (showOut, true);
    juce::ColourGradient gr (col::amber.withAlpha (0.16f), 0.0f, p.getY(), col::amber.withAlpha (0.03f), 0.0f, p.getBottom(), false);
    g.setGradientFill (gr);
    g.fillPath (fill);
    g.setColour (col::amber.withAlpha (0.30f));
    g.strokePath (build (showOut, false), juce::PathStrokeType (0.9f));
    g.restoreState();
}

void Screen::drawAuditionWindow (juce::Graphics& g)
{
    int mode = 0, set = 0, band = 0;
    audition (mode, set, band);
    if (mode == 0) return;
    const auto p = plotArea();
    const auto c = kSetCol[model.params().stereo == vt4::kLink ? 0 : set];
    juce::String label = (mode == 1 ? "SOLO  " : juce::String::fromUTF8 ("\xce\x94  ")) + kBandName[band];
    if (mode == 1)
    {
        const int type = proc.soloType.load();
        float x0 = p.getX(), x1 = p.getRight();
        if (type == vt4::SoloWindow::kBandPass || type == vt4::SoloWindow::kHighPass) x0 = xFor (proc.soloF1.load());
        if (type == vt4::SoloWindow::kBandPass || type == vt4::SoloWindow::kLowPass)  x1 = xFor (proc.soloF2.load());
        x0 = juce::jlimit (p.getX(), p.getRight(), x0); x1 = juce::jlimit (p.getX(), p.getRight(), x1);
        g.setColour (c.withAlpha (0.07f));
        g.fillRect (juce::Rectangle<float> (x0, p.getY(), x1 - x0, p.getHeight()));
        const float dash[] = { 3.0f, 3.0f };
        g.setColour (c.withAlpha (0.55f));
        for (float x : { x0, x1 })
            if (x > p.getX() + 1.0f && x < p.getRight() - 1.0f)
                g.drawDashedLine ({ x, p.getY(), x, p.getBottom() }, dash, 2, 1.0f);
        g.setFont (Fonts::strong (9.5f, 0.16f));
        g.drawText (label, juce::Rectangle<float> (x0 + 4.0f, p.getY() + 3.0f, juce::jmax (90.0f, x1 - x0 - 8.0f), 12.0f), juce::Justification::centredLeft, false);
    }
    else
    {
        g.setColour (c.withAlpha (0.85f));
        g.setFont (Fonts::strong (9.5f, 0.16f));
        g.drawText (label + juce::String::fromUTF8 ("  \xc2\xb7  ONLY WHAT THE BAND CHANGES"), juce::Rectangle<float> (p.getX() + 6.0f, p.getY() + 3.0f, 400.0f, 12.0f), juce::Justification::centredLeft, false);
    }
}

void Screen::drawCurves (juce::Graphics& g)
{
    if (! model.ready()) return;
    const auto p = plotArea();
    const int N = vt4::DisplayModel::kN;
    const int st = model.params().stereo;
    const int act = activeSet();
    g.saveState();
    // nothing exists above the host Nyquist frequency: the curves end there
    const double ny = 0.5 * model.hostRate();
    auto clip = p.toNearestInt().expanded (0, 2);
    if (ny < kFMax) clip.setRight ((int) std::ceil (xFor (ny)));
    g.reduceClipRegion (clip);

    int nEnd = N;                       // grid points below the host Nyquist frequency
    while (nEnd > 2 && model.freq (nEnd - 1) >= ny) --nEnd;

    // band contributions of the edited channel
    for (int b = 0; b < 5; ++b)
    {
        if (! model.bandActive (act, b)) continue;
        juce::Path fill, edge;
        fill.startNewSubPath (xFor (model.freq (0)), yFor (0.0));
        for (int i = 0; i < nEnd; ++i)
        {
            const float x = xFor (model.freq (i)), y = yFor (juce::jlimit (-kDbRange, kDbRange, model.bandDb (act, b, i)));
            fill.lineTo (x, y);
            if (i == 0) edge.startNewSubPath (x, y); else edge.lineTo (x, y);
        }
        fill.lineTo (xFor (model.freq (nEnd - 1)), yFor (0.0));
        fill.closeSubPath();
        const bool hot = b == hoverBand || b == selBand || b == dragBand;
        const auto c = kSetCol[st == vt4::kLink ? 0 : act];
        juce::ColourGradient gr (c.withAlpha (hot ? 0.20f : 0.075f), 0.0f, yFor (kDbRange * 0.6), c.withAlpha (hot ? 0.20f : 0.075f), 0.0f, yFor (-kDbRange * 0.6), false);
        gr.addColour (0.5, c.withAlpha (hot ? 0.05f : 0.015f));
        g.setGradientFill (gr);
        g.fillPath (fill);
        if (hot)
        {
            g.setColour (c.withAlpha (0.55f));
            g.strokePath (edge, juce::PathStrokeType (1.0f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
        }
    }

    auto curve = [&] (int set)
    {
        juce::Path path;
        for (int i = 0; i < nEnd; ++i)
        {
            const float x = xFor (model.freq (i)), y = yFor (juce::jlimit (-kDbRange - 2.0, kDbRange + 2.0, model.totalDb (set, i)));
            if (i == 0) path.startNewSubPath (x, y); else path.lineTo (x, y);
        }
        return path;
    };
    auto stroke = [&] (const juce::Path& path, juce::Colour c, float strength)
    {
        g.setColour (c.withAlpha (0.06f * strength)); g.strokePath (path, juce::PathStrokeType (9.0f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
        g.setColour (c.withAlpha (0.16f * strength)); g.strokePath (path, juce::PathStrokeType (4.0f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
        g.setColour (c.withAlpha (0.95f * strength)); g.strokePath (path, juce::PathStrokeType (1.7f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    };
    if (st == vt4::kLink)
        stroke (curve (0), kSetCol[0], 1.0f);
    else
    {
        const int other = act ^ 1;
        stroke (curve (other), kSetCol[other], 0.55f);
        stroke (curve (act), kSetCol[act], 1.0f);
    }
    g.restoreState();
}

juce::String Screen::nodeText (int set, int band) const
{
    const auto& prm = model.params();
    const auto& ch = prm.ch[(size_t) (prm.stereo == vt4::kLink ? 0 : set)];
    if (band == 0) return juce::String ("HIGH-PASS  ") + (ch.hpfHz > 0.0 ? vt4p::hzText (ch.hpfHz) : juce::String ("OFF"));
    const auto& bp = ch.band[(size_t) (band - 1)];
    juce::String t = juce::String (kBandName[band]) + "  " + vt4p::hzText (bp.freq) + "  " + vt4p::dbText (bp.gainDb);
    if (band == 1 && bp.weight > 0.0) t << "  W " << juce::String (bp.weight, bp.weight == std::round (bp.weight) ? 0 : 1);
    if (band == 3) t << (bp.narrow ? "  NARROW" : "  WIDE");
    return t;
}

void Screen::drawNodes (juce::Graphics& g)
{
    if (! model.ready()) return;
    const auto p = plotArea();
    const int st = model.params().stereo;
    const int act = activeSet();
    auto drawSet = [&] (int set, float alpha)
    {
        const auto c = kSetCol[st == vt4::kLink ? 0 : set];
        for (const auto& n : nodes (set))
        {
            const bool hot = alpha > 0.9f && (n.band == hoverBand || n.band == dragBand);
            const bool sel = alpha > 0.9f && n.band == selBand;
            const float r = hot || sel ? 9.0f : 7.5f;
            auto pt = n.p;
            pt.y = juce::jlimit (p.getY() + 4.0f, p.getBottom() - 4.0f, pt.y);
            if (sel || hot)
            {
                g.setColour (c.withAlpha (0.18f * alpha));
                g.fillEllipse (juce::Rectangle<float> (r * 3.2f, r * 3.2f).withCentre (pt));
            }
            g.setColour (sel ? c.withAlpha (alpha) : juce::Colour (0xff0b0b0c).withAlpha (0.92f * alpha));
            g.fillEllipse (juce::Rectangle<float> (2 * r, 2 * r).withCentre (pt));
            g.setColour (c.withAlpha ((n.on ? 1.0f : 0.45f) * alpha));
            g.drawEllipse (juce::Rectangle<float> (2 * r, 2 * r).withCentre (pt), 1.5f);
            g.setColour (sel ? juce::Colour (0xff0b0b0c) : c.withAlpha ((n.on ? 1.0f : 0.6f) * alpha));
            g.setFont (Fonts::strong (r * 1.15f, 0.0f));
            g.drawText (kBandLetter[n.band], juce::Rectangle<float> (2 * r, 2 * r).withCentre (pt.translated (0.0f, 0.5f)), juce::Justification::centred, false);
        }
    };
    if (st != vt4::kLink) drawSet (act ^ 1, 0.35f);
    drawSet (act, 1.0f);

    // tag of the hovered or selected node
    const int show = dragBand >= 0 ? dragBand : (hoverBand >= 0 ? hoverBand : selBand);
    if (show < 0) return;
    for (const auto& n : nodes (act))
    {
        if (n.band != show) continue;
        const float tx = juce::jlimit (p.getX() + 2.0f, p.getRight() - 172.0f, n.p.x - 86.0f);
        const float ty = n.p.y > p.getY() + 50.0f ? n.p.y - 40.0f : n.p.y + 16.0f;
        const auto tag = juce::Rectangle<float> (tx, ty, show == selBand ? 172.0f : 128.0f, 24.0f);
        g.setColour (juce::Colour (0xe8080809));
        g.fillRoundedRectangle (tag, 3.0f);
        const auto c = kSetCol[st == vt4::kLink ? 0 : act];
        g.setColour (c.withAlpha (0.55f));
        g.drawRoundedRectangle (tag.reduced (0.5f), 3.0f, 1.0f);
        g.setColour (c);
        g.setFont (Fonts::strong (9.5f, 0.06f));
        g.drawFittedText (nodeText (act, show), tag.withWidth (128.0f).reduced (6.0f, 2.0f).toNearestInt(), juce::Justification::centredLeft, 2, 0.8f);
        if (show == selBand)
        {
            for (Hit h : { Hit::Solo, Hit::Delta })
            {
                const auto r = buttonRect (h);
                const bool on = latchedMode == (h == Hit::Solo ? 1 : 2) && latchedBand == selBand && latchedSet == act;
                g.setColour (on ? c : c.withAlpha (hoverHit == h ? 0.35f : 0.18f));
                g.fillRoundedRectangle (r, 2.0f);
                g.setColour (on ? juce::Colour (0xff0b0b0c) : c);
                if (h == Hit::Solo)
                {
                    // headphones glyph
                    juce::Path ph;
                    const auto cc = r.getCentre();
                    ph.addCentredArc (cc.x, cc.y + 1.0f, 5.0f, 5.0f, 0.0f, -1.9f, 1.9f, true);
                    g.strokePath (ph, juce::PathStrokeType (1.3f));
                    g.fillRoundedRectangle (cc.x - 6.2f, cc.y + 0.5f, 2.6f, 4.5f, 1.0f);
                    g.fillRoundedRectangle (cc.x + 3.6f, cc.y + 0.5f, 2.6f, 4.5f, 1.0f);
                }
                else
                {
                    g.setFont (Fonts::strong (12.0f));
                    g.drawText (juce::String::fromUTF8 ("\xce\x94"), r, juce::Justification::centred, false);
                }
            }
        }
    }
}

void Screen::drawTopBar (juce::Graphics& g)
{
    const auto gl = glassArea();
    g.setFont (Fonts::label (9.0f, 0.2f));
    g.setColour (col::amber.withAlpha (0.55f));
    g.drawText ("ANALYSER", juce::Rectangle<float> (gl.getX() + 14.0f, gl.getY() + 7.0f, 76.0f, 16.0f), juce::Justification::centredLeft, false);
    auto key = [&] (Hit h, const juce::String& t, bool on)
    {
        const auto r = buttonRect (h);
        g.setColour (on ? col::amber.withAlpha (0.85f) : col::amber.withAlpha (hoverHit == h ? 0.22f : 0.10f));
        g.fillRoundedRectangle (r, 2.0f);
        g.setColour (on ? juce::Colour (0xff0b0b0c) : col::amber.withAlpha (0.8f));
        g.setFont (Fonts::strong (8.6f, 0.12f));
        g.drawText (t, r, juce::Justification::centred, false);
    };
    key (Hit::AnaOff, "OFF", analyserMode == 0);
    key (Hit::AnaOut, "OUT", analyserMode == 1);
    key (Hit::AnaInOut, "IN+OUT", analyserMode == 2);

    // legend
    const int st = model.params().stereo;
    const float lx = gl.getX() + 240.0f;
    g.setFont (Fonts::label (9.0f, 0.16f));
    if (st == vt4::kLink)
    {
        g.setColour (col::amber); g.fillEllipse (lx, gl.getY() + 12.0f, 6.0f, 6.0f);
        g.drawText ("A  L+R", juce::Rectangle<float> (lx + 10.0f, gl.getY() + 7.0f, 80.0f, 16.0f), juce::Justification::centredLeft, false);
    }
    else
    {
        const char* a = st == vt4::kMidSide ? "A  MID" : "A  LEFT";
        const char* b = st == vt4::kMidSide ? "B  SIDE" : "B  RIGHT";
        g.setColour (col::amber); g.fillEllipse (lx, gl.getY() + 12.0f, 6.0f, 6.0f);
        g.drawText (a, juce::Rectangle<float> (lx + 10.0f, gl.getY() + 7.0f, 70.0f, 16.0f), juce::Justification::centredLeft, false);
        g.setColour (col::green); g.fillEllipse (lx + 84.0f, gl.getY() + 12.0f, 6.0f, 6.0f);
        g.drawText (b, juce::Rectangle<float> (lx + 94.0f, gl.getY() + 7.0f, 70.0f, 16.0f), juce::Justification::centredLeft, false);
        g.setColour (col::amber.withAlpha (0.55f));
        g.setFont (Fonts::label (9.0f, 0.2f));
        g.drawText ("EDIT", juce::Rectangle<float> (gl.getRight() - 106.0f, gl.getY() + 7.0f, 40.0f, 16.0f), juce::Justification::centredRight, false);
        auto ek = [&] (Hit h, const char* t, bool on, juce::Colour c)
        {
            const auto r = buttonRect (h);
            g.setColour (on ? c.withAlpha (0.9f) : c.withAlpha (hoverHit == h ? 0.24f : 0.10f));
            g.fillRoundedRectangle (r, 2.0f);
            g.setColour (on ? juce::Colour (0xff0b0b0c) : c);
            g.setFont (Fonts::strong (9.0f, 0.1f));
            g.drawText (t, r, juce::Justification::centred, false);
        };
        ek (Hit::EditA, "A", editSet == 0, col::amber);
        ek (Hit::EditB, "B", editSet == 1, col::green);
    }
    // hint
    g.setFont (Fonts::label (8.4f, 0.08f));
    g.setColour (col::amber.withAlpha (0.32f));
    const auto hintArea = juce::Rectangle<float> (gl.getX() + 430.0f, gl.getY() + 7.0f, gl.getWidth() - 560.0f, 16.0f);
    g.drawText (juce::String::fromUTF8 ("drag \xc2\xb7 shift fine \xc2\xb7 alt-drag solo \xc2\xb7 wheel: weight / focus \xc2\xb7 double-click: 0 dB"),
                hintArea, juce::Justification::centred, true);
}

void Screen::paint (juce::Graphics& g)
{
    drawBezel (g);
    drawGrid (g);
    drawAnalyser (g);
    drawAuditionWindow (g);
    drawCurves (g);
    drawNodes (g);
    drawTopBar (g);
    if (! model.ready())
    {
        g.setColour (col::amber.withAlpha (0.6f));
        g.setFont (Fonts::label (11.0f, 0.3f));
        g.drawText ("WARMING UP", plotArea(), juce::Justification::centred, false);
    }
    // glass reflection
    const auto gl = glassArea();
    juce::ColourGradient refl (juce::Colours::white.withAlpha (0.045f), gl.getX(), gl.getY(), juce::Colours::white.withAlpha (0.0f), gl.getX() + gl.getWidth() * 0.35f, gl.getY() + gl.getHeight() * 0.8f, false);
    g.setGradientFill (refl);
    g.fillRoundedRectangle (gl, 5.0f);
}

} // namespace vt4ui
