#pragma once

#include <juce_audio_processors/juce_audio_processors.h>
#include "PluginProcessor.h"
#include "ui/Faceplate.h"
#include "ui/LookAndFeel.h"
#include "ui/RearPanel.h"
#include "ui/Screen.h"

#include <condition_variable>
#include <mutex>
#include <thread>

// Builds display models off the message thread (a new unit or sample rate
// means a complete device, ≈ 0.1 s of computation). Only the latest request
// is built; take() hands over the finished model.
class DisplayModelBuilder
{
public:
    struct Key
    {
        double osRate = 0.0, hostRate = 0.0;
        int osOrder = 0;
        uint32_t serial = 0;
        double tolerance = -1.0;
        bool matched = false;
        bool operator== (const Key& o) const noexcept
        {
            return osRate == o.osRate && hostRate == o.hostRate && osOrder == o.osOrder
                && serial == o.serial && tolerance == o.tolerance && matched == o.matched;
        }
        bool operator!= (const Key& o) const noexcept { return ! (*this == o); }
    };

    DisplayModelBuilder() : worker ([this] { run(); }) {}
    ~DisplayModelBuilder()
    {
        { std::lock_guard<std::mutex> l (m); quit = true; }
        cv.notify_all();
        worker.join();
    }

    void request (const Key& k, const vt4::DeviceParams& p)
    {
        { std::lock_guard<std::mutex> l (m); wantKey = k; wantParams = p; pending = true; }
        cv.notify_all();
    }

    std::unique_ptr<vt4::DisplayModel> take (Key& k)
    {
        std::lock_guard<std::mutex> l (m);
        k = doneKey;
        return std::move (done);
    }

private:
    void run()
    {
        for (;;)
        {
            Key k; vt4::DeviceParams p;
            {
                std::unique_lock<std::mutex> l (m);
                cv.wait (l, [this] { return quit || pending; });
                if (quit) return;
                k = wantKey; p = wantParams; pending = false;
            }
            auto model = std::make_unique<vt4::DisplayModel>();
            model->prepare (k.osRate, k.hostRate, p);
            model->setOversamplingResponse (VT4AudioProcessor::oversamplingResponseFor (k.hostRate, k.osOrder, *model));
            std::lock_guard<std::mutex> l (m);
            if (quit) return;
            done = std::move (model);
            doneKey = k;
        }
    }

    std::mutex m;
    std::condition_variable cv;
    Key wantKey, doneKey;
    vt4::DeviceParams wantParams;
    bool pending = false, quit = false;
    std::unique_ptr<vt4::DisplayModel> done;
    std::thread worker;
};

class VT4AudioProcessorEditor : public juce::AudioProcessorEditor, private juce::Timer
{
public:
    explicit VT4AudioProcessorEditor (VT4AudioProcessor&);
    ~VT4AudioProcessorEditor() override;

    void paint (juce::Graphics&) override;
    void resized() override;

    // the screen's model is built and matches the processor's unit (tests)
    bool screenModelCurrent() const { return model.ready() && model.sameUnit (proc.currentParams()); }

private:
    void timerCallback() override;
    void setScreenOpen (bool open);
    void setRearShown (bool shown);
    void copyAtoB();
    void updateAudition (const vt4::DeviceParams&);

    VT4AudioProcessor& proc;
    vt4ui::VT4LookAndFeel lnf;
    juce::TooltipWindow tooltips { this, 700 };

    vt4::DisplayModel model;
    juce::Component content;                  // design space, scaled to the window
    vt4ui::Faceplate faceplate;
    vt4ui::Screen screen;
    vt4ui::RearPanel rear;

    bool screenOpen = true, rearShown = false;
    DisplayModelBuilder modelBuilder;
    DisplayModelBuilder::Key modelKey, requestedKey;
    int lastAudMode = -1, lastAudSet = -1, lastAudBand = -1;
    vt4::DeviceParams lastSoloParams;
    float vuL = 0.0f, vuR = 0.0f;
    int peakHoldL = 0, peakHoldR = 0;
    float stressLamp = 0.0f;
    std::unique_ptr<juce::ComponentBoundsConstrainer> constrainer;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (VT4AudioProcessorEditor)
};
