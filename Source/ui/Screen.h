#pragma once

#include <juce_audio_processors/juce_audio_processors.h>
#include "LookAndFeel.h"
#include "../dsp/DisplayModel.h"
#include "../dsp/Spectrum.h"

class VT4AudioProcessor;

namespace vt4ui
{

// The instrument screen: response curves computed from the circuit, the
// spectrum analyser, draggable band nodes, SOLO / DELTA audition.
class Screen : public juce::Component
{
public:
    Screen (VT4AudioProcessor&, vt4::DisplayModel&);

    void paint (juce::Graphics&) override;
    void resized() override;
    void mouseMove (const juce::MouseEvent&) override;
    void mouseExit (const juce::MouseEvent&) override;
    void mouseDown (const juce::MouseEvent&) override;
    void mouseDrag (const juce::MouseEvent&) override;
    void mouseUp (const juce::MouseEvent&) override;
    void mouseDoubleClick (const juce::MouseEvent&) override;
    void mouseWheelMove (const juce::MouseEvent&, const juce::MouseWheelDetails&) override;

    // editor timer: analyser frames, meters of the audition state
    void tick();
    // current audition request (mode 0 none, 1 solo, 2 delta)
    void audition (int& mode, int& set, int& band) const;

    int analyserMode = 1;     // 0 off, 1 output, 2 input + output
    int editSet = 0;

private:
    struct Node { int band; juce::Point<float> p; bool on; };
    enum class Hit { None, Node, Solo, Delta, AnaOff, AnaOut, AnaInOut, EditA, EditB };

    juce::Rectangle<float> glassArea() const;
    juce::Rectangle<float> plotArea() const;
    float xFor (double f) const;
    double fFor (float x) const;
    float yFor (double db) const;
    double dbFor (float y) const;
    float yAna (double db) const;

    int activeSet() const;
    std::vector<Node> nodes (int set) const;
    Hit hitTestAt (juce::Point<float>, int& band) const;
    juce::Rectangle<float> buttonRect (Hit) const;

    void setParam (const juce::String& id, float normalised);
    void beginDrag (int band);
    void endDrag();
    void dragTo (juce::Point<float>, bool fine);
    juce::String nodeText (int set, int band) const;

    void drawBezel (juce::Graphics&);
    void drawGrid (juce::Graphics&);
    void drawAnalyser (juce::Graphics&);
    void drawCurves (juce::Graphics&);
    void drawNodes (juce::Graphics&);
    void drawTopBar (juce::Graphics&);
    void drawAuditionWindow (juce::Graphics&);

    VT4AudioProcessor& proc;
    vt4::DisplayModel& model;

    vt4::SpectrumAnalyser anaIn, anaOut;
    std::vector<double> showIn, showOut;     // smoothed, dB
    std::vector<float> scratch;
    int frameDiv = 0;
    double anaRate = 0.0;

    int hoverBand = -1, dragBand = -1, selBand = -1;
    Hit hoverHit = Hit::None;
    juce::Point<float> dragStart;
    double dragF0 = 0.0, dragG0 = 0.0, dragPos0 = 0.0;
    std::vector<juce::String> gestureIds;

    int latchedMode = 0, latchedBand = -1, latchedSet = 0;
    int tempMode = 0;           // alt-drag
    juce::Image glassCache;
    float glassScale = 0.0f;
    juce::Rectangle<int> glassCacheSize;
};

} // namespace vt4ui
