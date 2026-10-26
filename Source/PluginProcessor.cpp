#include "PluginProcessor.h"
#include "PluginEditor.h"

VT4AudioProcessor::VT4AudioProcessor()
    : AudioProcessor (BusesProperties()
                          .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                          .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
      apvts (*this, nullptr, "VT4", vt4p::createLayout (flags))
{
    pp.attach (apvts);
    bypassParam = dynamic_cast<juce::AudioParameterBool*> (apvts.getParameter (vt4p::ids::bypass));
    jassert (bypassParam != nullptr);
    lastParams = pp.read();
}

bool VT4AudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    const auto& out = layouts.getMainOutputChannelSet();
    if (out != juce::AudioChannelSet::mono() && out != juce::AudioChannelSet::stereo())
        return false;
    return layouts.getMainInputChannelSet() == out;
}

void VT4AudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    prepared = false;
    hostRate = sampleRate;
    maxBlock = juce::jmax (1, samplesPerBlock);

    // the circuit runs at >= 176.4 kHz: 4x for 44.1 / 48 k, 2x for 88.2 / 96 k, 1x above
    osOrder = sampleRate < 70000.0 ? 2 : (sampleRate < 140000.0 ? 1 : 0);
    osRate = sampleRate * (double) (1 << osOrder);
    oversampler = std::make_unique<juce::dsp::Oversampling<float>> (
        (size_t) 2, (size_t) osOrder, juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR, true, true);
    oversampler->initProcessing ((size_t) maxBlock);

    lastParams = pp.read();
    flags->stepped.store (pp.stepped->load() > 0.5f);
    // Never a long wait on the host's thread: the engine restarts from its
    // pristine copy when it can, otherwise it builds in the background and
    // the plugin passes the (latency-aligned) input until it is ready.
    // Offline rendering builds here, so a bounce is processed from sample 0.
    engine.prepare (osRate, lastParams, isNonRealtime());
    engineLive = engine.ready();
    sleeping = false; bypassIdle = 0;
    solo.prepare (sampleRate);

    const int latency = (int) std::lround (oversampler->getLatencyInSamples());
    setLatencySamples (latency);
    dryDelay.setMaximumDelayInSamples (juce::jmax (8, latency + 8));
    dryDelay.prepare ({ sampleRate, (juce::uint32) maxBlock, 2u });
    dryDelay.setDelay ((float) latency);
    dryBuffer.setSize (2, maxBlock);
    work.setSize (2, maxBlock);

    wetMix.reset (sampleRate, 0.03);
    wetMix.setCurrentAndTargetValue (bypassParam->get() || ! engineLive ? 0.0f : 1.0f);
    outGain.reset (sampleRate, 0.03);
    outGain.setCurrentAndTargetValue (juce::Decibels::decibelsToGain (pp.output->load()));
    vuState[0] = vuState[1] = {};
    if (engineLive) bplusNominal.store ((float) engine.device().bplus());
    prepared = true;
    preparedCount.fetch_add (1);
}

std::vector<double> VT4AudioProcessor::oversamplingResponse (const vt4::DisplayModel& grid) const
{
    return oversamplingResponseFor (hostRate, osOrder, grid);
}

// Magnitude of the up + down oversampling filters at the display grid, from
// their impulse response. Safe on any thread (its own filter instance).
std::vector<double> VT4AudioProcessor::oversamplingResponseFor (double hostRate, int osOrder, const vt4::DisplayModel& grid)
{
    std::vector<double> mag ((size_t) vt4::DisplayModel::kN, 1.0);
    if (osOrder == 0) return mag;
    const int N = 16384;
    juce::dsp::Oversampling<float> os (1, (size_t) osOrder, juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR, true, true);
    os.initProcessing ((size_t) N);
    juce::AudioBuffer<float> b (1, N);
    b.clear();
    b.setSample (0, 0, 1.0f);
    juce::dsp::AudioBlock<float> blk (b);
    auto up = os.processSamplesUp (blk);
    juce::ignoreUnused (up);
    os.processSamplesDown (blk);
    const float* h = b.getReadPointer (0);
    for (int i = 0; i < vt4::DisplayModel::kN; ++i)
    {
        const double f = grid.freq (i);
        if (f >= 0.5 * hostRate) break;
        // DFT at f by a rotating phasor (re-normalised in blocks)
        const double w = 2.0 * juce::MathConstants<double>::pi * f / hostRate;
        const std::complex<double> step (std::cos (w), -std::sin (w));
        std::complex<double> acc (0.0, 0.0);
        for (int n0 = 0; n0 < N; n0 += 256)
        {
            std::complex<double> z = std::polar (1.0, -w * n0);
            const int n1 = std::min (N, n0 + 256);
            for (int n = n0; n < n1; ++n) { acc += (double) h[n] * z; z *= step; }
        }
        mag[(size_t) i] = std::abs (acc);
    }
    return mag;
}

void VT4AudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    const int numIn = getTotalNumInputChannels(), numOut = getTotalNumOutputChannels();
    for (int ch = numIn; ch < numOut; ++ch)
        buffer.clear (ch, 0, buffer.getNumSamples());
    if (! prepared || numIn <= 0 || buffer.getNumChannels() <= 0)
        return;

    const vt4::DeviceParams p = pp.read();
    flags->stepped.store (pp.stepped->load() > 0.5f);
    if (! vt4p::sameParams (p, lastParams))
    {
        engine.setParams (p);
        lastParams = p;
    }

    engine.poll();
    if (! engine.ready() && isNonRealtime())
        engine.waitUntilReady (10000);
    if (engine.ready() && ! engineLive)
    {
        // the background build has arrived: fade the circuit in
        engineLive = true;
        wetMix.setCurrentAndTargetValue (0.0f);
        bplusNominal.store ((float) engine.device().bplus());
    }

    // audition requests from the editor
    const int am = auditionMode.load (std::memory_order_relaxed);
    const int aset = juce::jlimit (0, 1, auditionSet.load (std::memory_order_relaxed));
    const int aband = juce::jlimit (0, 4, auditionBand.load (std::memory_order_relaxed));
    vt4::SoloWindow w;
    w.type = soloType.load (std::memory_order_relaxed);
    w.f1 = soloF1.load (std::memory_order_relaxed);
    w.f2 = soloF2.load (std::memory_order_relaxed);
    engine.setDelta (am == (int) vt4::Audition::kDelta, aset, aband);
    solo.setRequest (am == (int) vt4::Audition::kSolo, aset, aband, p.stereo, w);

    const bool bypassed = bypassParam->get();
    wetMix.setTargetValue (bypassed || ! engineLive ? 0.0f : 1.0f);

    // Bypassed for a while: the circuit rests (no CPU). When it is switched
    // back in it restarts at its operating point and fades in.
    if (engineLive)
    {
        if (bypassed && ! wetMix.isSmoothing() && wetMix.getCurrentValue() == 0.0f)
        {
            if (! sleeping && (bypassIdle += buffer.getNumSamples()) > (int) (0.2 * hostRate))
                sleeping = true;
        }
        else
            bypassIdle = 0;
        if (sleeping && ! bypassed)
        {
            engine.restartAtRest();
            oversampler->reset();
            sleeping = false;
        }
    }
    outGain.setTargetValue (juce::Decibels::decibelsToGain (pp.output->load()));

    const int total = buffer.getNumSamples();
    for (int pos = 0; pos < total; pos += maxBlock)
        processChunk (buffer, pos, juce::jmin (maxBlock, total - pos), juce::jmin (numIn, buffer.getNumChannels()));

    if (engineLive)
    {
        bplus.store ((float) engine.device().bplus());
        warmth.store ((float) engine.device().warmth());
        const float s = (float) engine.device().takeStress();
        float prev = stress.load();
        while (s > prev && ! stress.compare_exchange_weak (prev, s)) {}
    }
}

void VT4AudioProcessor::processChunk (juce::AudioBuffer<float>& buffer, int start, int num, int numIn)
{
    // input (mono hosts: the same signal on both device channels)
    for (int ch = 0; ch < 2; ++ch)
        work.copyFrom (ch, 0, buffer, juce::jmin (ch, numIn - 1), start, num);

    // dry path for bypass, aligned with the oversampling latency; analyser input
    {
        float* a = work.getWritePointer (0);
        float* b = work.getWritePointer (1);
        float* da = dryBuffer.getWritePointer (0);
        float* db = dryBuffer.getWritePointer (1);
        float mono[1024];
        for (int i = 0; i < num; ++i)
        {
            dryDelay.pushSample (0, a[i]); da[i] = dryDelay.popSample (0);
            dryDelay.pushSample (1, b[i]); db[i] = dryDelay.popSample (1);
        }
        for (int i = 0; i < num; i += 1024)
        {
            const int n = juce::jmin (1024, num - i);
            for (int k = 0; k < n; ++k) mono[k] = 0.5f * (a[i + k] + b[i + k]);
            anaIn.push (mono, n);
        }
    }

    if (! sleeping)
    {
        juce::dsp::AudioBlock<float> block (work);
        auto sub = block.getSubBlock (0, (size_t) num);
        auto up = oversampler->processSamplesUp (sub);
        const int upN = (int) up.getNumSamples();
        float* uL = up.getChannelPointer (0);
        float* uR = up.getChannelPointer (1);
        if (engineLive)
            for (int i = 0; i < upN; ++i)
            {
                double l, r;
                engine.process ((double) uL[i], (double) uR[i], l, r);
                uL[i] = (float) l; uR[i] = (float) r;
            }
        oversampler->processSamplesDown (sub);
    }

    solo.process (work.getWritePointer (0), work.getWritePointer (1), num);

    // output trim, bypass, meters
    const int numOut = juce::jmin (getTotalNumOutputChannels(), buffer.getNumChannels());
    float* wa = work.getWritePointer (0);
    float* wb = work.getWritePointer (1);
    const float* da = dryBuffer.getReadPointer (0);
    const float* db = dryBuffer.getReadPointer (1);
    float pk[2] { 0.0f, 0.0f };
    const double dt = 1.0 / hostRate;
    const double vuScale = 1.0 / (0.6366197723675814 * std::pow (10.0, -18.0 / 20.0));   // 0 VU = −18 dBFS sine
    float mono[1024];
    int mono_n = 0;
    for (int i = 0; i < num; ++i)
    {
        const float g = outGain.getNextValue();
        const float m = wetMix.getNextValue();
        // fully bypassed: the delayed input itself (bit-exact, whatever the circuit does)
        const float l = m == 0.0f ? da[i] : da[i] + m * (g * wa[i] - da[i]);
        const float r = m == 0.0f ? db[i] : db[i] + m * (g * wb[i] - db[i]);
        wa[i] = l; wb[i] = r;
        pk[0] = juce::jmax (pk[0], std::abs (l));
        pk[1] = juce::jmax (pk[1], std::abs (r));
        const double x[2] { std::abs ((double) l) * vuScale, std::abs ((double) r) * vuScale };
        for (int c = 0; c < 2; ++c)
        {
            auto& s = vuState[c];
            const double acc = vuW * vuW * (x[c] - s.y) - 2.0 * vuZ * vuW * s.v;
            s.v += acc * dt;
            s.y += s.v * dt;
        }
        mono[mono_n++] = 0.5f * (l + r);
        if (mono_n == 1024) { anaOut.push (mono, mono_n); mono_n = 0; }
    }
    if (mono_n > 0) anaOut.push (mono, mono_n);

    if (numOut == 1)
        buffer.copyFrom (0, start, work, 0, 0, num);
    else
        for (int ch = 0; ch < numOut; ++ch)
            buffer.copyFrom (ch, start, work, juce::jmin (ch, 1), 0, num);

    for (int c = 0; c < 2; ++c)
    {
        vu[(size_t) c].store ((float) vuState[c].y);
        float prev = peak[(size_t) c].load();
        while (pk[c] > prev && ! peak[(size_t) c].compare_exchange_weak (prev, pk[c])) {}
    }
}

void VT4AudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    if (auto xml = apvts.copyState().createXml())
        copyXmlToBinary (*xml, destData);
}

void VT4AudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    if (auto xml = getXmlFromBinary (data, sizeInBytes))
        if (xml->hasTagName (apvts.state.getType()))
            apvts.replaceState (juce::ValueTree::fromXml (*xml));
}

juce::AudioProcessorEditor* VT4AudioProcessor::createEditor()
{
    return new VT4AudioProcessorEditor (*this);
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new VT4AudioProcessor();
}
