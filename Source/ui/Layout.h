#pragma once

#include <juce_gui_basics/juce_gui_basics.h>

namespace vt4ui::layout
{
    // design space (the editor scales it)
    constexpr int W = 1320;
    constexpr int faceH = 590;
    constexpr int screenH = 300;
    constexpr int headerH = 166;
    constexpr int rowY[2] = { 170, 356 };
    constexpr int rowH = 186;
    constexpr int stripY = 542;
    constexpr int knobY = 96;                 // knob centre, relative to the row

    struct KnobPos { int x; int d; };
    constexpr KnobPos hpf    { 125, 56 };
    constexpr KnobPos freq[4] { { 245, 56 }, { 597, 56 }, { 857, 56 }, { 1115, 56 } };
    constexpr KnobPos gain[4] { { 369, 76 }, { 721, 76 }, { 981, 76 }, { 1239, 76 } };
    constexpr KnobPos weight { 483, 46 };

    // section frames (x range) within a row: HPF, LOW, BODY, PRESENCE, AIR
    constexpr int secX0[5] = { 74, 186, 538, 798, 1056 };
    constexpr int secX1[5] = { 178, 532, 792, 1050, 1308 };

    // header
    inline const juce::Rectangle<int> vuL  { 380, 20, 220, 124 };
    inline const juce::Rectangle<int> vuR  { 612, 20, 220, 124 };
    inline const juce::Rectangle<int> bplusMeter { 850, 30, 104, 104 };
    inline const juce::Rectangle<int> modeButtons { 990, 18, 300, 26 };
    constexpr KnobPos drive  { 1040, 58 };
    constexpr KnobPos output { 1236, 58 };
    constexpr int controlKnobY = 101;
    constexpr int controlCaptionY = 147;      // top of the INPUT DRIVE / OUTPUT captions
    inline const juce::Rectangle<int> driveLamp { 1120, 78, 36, 36 };

    // bottom strip
    constexpr int stripRowY = 552, stripRowH = 26;
    inline const juce::Rectangle<int> stereoButtons { 96, stripRowY, 208, stripRowH };
    inline const juce::Rectangle<int> copyButton    { 312, stripRowY, 56, stripRowH };
    inline const juce::Rectangle<int> ironButtons   { 446, stripRowY, 240, stripRowH };
    inline const juce::Rectangle<int> steppedButton { 774, stripRowY, 168, stripRowH };
    inline const juce::Rectangle<int> bypassButton  { 1004, stripRowY, 92, stripRowH };
    inline const juce::Rectangle<int> bypassLamp    { 1098, stripRowY - 5, 36, 36 };
    inline const juce::Rectangle<int> rearButton    { 1150, stripRowY, 70, stripRowH };
    inline const juce::Rectangle<int> screenButton  { 1228, stripRowY, 74, stripRowH };
    // engraved captions of the strip
    inline const juce::Rectangle<float> stereoCaption { 30.0f, (float) stripRowY, 62.0f, (float) stripRowH };
    inline const juce::Rectangle<float> ironCaption   { 398.0f, (float) stripRowY, 46.0f, (float) stripRowH };
    inline const juce::Rectangle<float> knobsCaption  { 706.0f, (float) stripRowY, 64.0f, (float) stripRowH };
}
