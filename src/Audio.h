// ebaner - a Vulkan viewer for terrainmapper rail/terrain exports.
// Copyright (C) 2026 Jan-Espen Oversand <sigsegv@radiotube.org>
//
// This file is part of ebaner. ebaner is free software: you can redistribute it
// and/or modify it under the terms of version 3 of the GNU General Public License
// as published by the Free Software Foundation.
//
// ebaner is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public License for more details. You
// should have received a copy of the license along with ebaner; if not, see
// <https://www.gnu.org/licenses/>.

#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

class Vehicle;
class Consist;

// Everything the wheel-on-rail voices run on: how fast, how heavily each axle presses,
// and how hard the contact is being worked along and across the rail.
struct RollingSample {
    float speed = 0.0f;      // m/s
    float axleLoadN = 0.0f;  // N carried by one axle
    float work = 0.0f;       // [0,1] traction + running resistance, vs adhesion
    float brake = 0.0f;      // [0,1] friction-brake force, vs adhesion
    float flange = 0.0f;     // [0,1] lateral acceleration the cant does not take out
    float gain = 0.0f;       // [0,1] camera distance attenuation
    bool railborne = true;   // false when derailed: no rail to roar on
    // Wheels that have crossed a turnout, counted from the start of the run: the synth
    // knocks once for each new one. A count and not a rate, because these are events
    // at places and not a rhythm - continuous welded rail has nothing to beat against.
    unsigned impacts = 0;
};

// Synthesised air-brake sound. Procedurally generates a hiss whose loudness tracks
// the brake-cylinder airflow (charging on apply / venting on release, fading as the
// pressure equalises) plus a valve click on each change of the brake command. Uses
// PortAudio when built with HAVE_AUDIO; otherwise it is a silent no-op. The DSP
// (`render`) is independent of PortAudio so it can be rendered offline (`dumpTest`).
class Audio {
public:
    Audio() = default;
    ~Audio();
    Audio(const Audio&) = delete;
    Audio& operator=(const Audio&) = delete;

    void init();                              // open the device (silent on failure / headless)
    // The engines a train can be heard as separate voices. Two per Class 93 set, so a
    // three-set train fills it: walking the length of the train, the near engines
    // swell and the far ones recede rather than the whole train being one sound.
    //
    // A slot no engine is driving is exactly inert, not merely quiet: render() bails on
    // it before it touches the shared noise generator, and its two envelopes are zero
    // plus zero for ever - so widening this cannot shift a sample of anything already
    // sounding. It costs six flops and a predicted branch per sample, which is why the
    // ceiling is set by the longest train there is and not trimmed to the common case.
    static constexpr int kMaxEngines = 6;
    // One engine's voice: how fast it is turning and how loudly it is heard from here.
    //
    // Handed in as a flat list rather than read off a train, because the slots are not
    // a train's - once a train can be parted there are two trains' engines competing
    // for six voices, and which of them get one is the caller's business to decide.
    struct EngineVoice {
        float rpm = 0.0f;  // rev/min; 0 is a silent slot
        float gain = 0.0f; // [0,1] camera distance attenuation
    };
    // Main thread, per sim frame. `sounded` is the train the single rolling, brake and
    // compressor voices are taken from - there is one of each and one set of filters
    // behind them, so exactly one train can have them, and the caller picks which.
    // `brakeGain` attenuates the brake sound by camera distance to the bogies;
    // `engines` is the engine voices, in slot order (extra entries ignored, missing
    // ones silent); `rollGain` the wheel/rail noise, which comes from the same wheels
    // but carries much further. All gains in [0,1].
    void update(const Consist& sounded, float dt, float brakeGain,
                const EngineVoice* engines, int engineCount, float rollGain);
    // Main thread: how loudly the nearest ringing level crossing is heard, already
    // attenuated by camera distance, in [0,1]. Zero silences it.
    //
    // Only a level, never a strike: the strike clock lives on the audio thread, so the
    // bell keeps an even rhythm no matter what the frame rate does. Driving one strike
    // per frame would make the bell speed up and stutter with the graphics.
    void setCrossingBell(float gain) { bellGain_.store(gain, std::memory_order_relaxed); }
    // Main thread: the wheel/rail state. `update` fills this from the Vehicle; it is
    // public so the synth can be driven straight from known numbers and measured,
    // which is the only way to check that speed, weight and load do what they claim.
    void setRolling(const RollingSample& r);
    void toggleMuted() { muted_.store(!muted_.load()); }
    bool muted() const { return muted_.load(); }
    void shutdown();

    // Realtime synth: fill `n` mono samples. Public so the PortAudio callback (and
    // the offline dump) can drive it. Only touched by the audio thread at runtime.
    void render(float* out, int n);

    // Render a scripted brake / engine sequence to a mono 16-bit WAV (offline
    // verification; needs no audio device).
    static void dumpTest(const std::string& wavPath);
    static void dumpEngineTest(const std::string& wavPath);
    // A crossing activating: the bell rings and then falls silent under the still
    // flashing lights, which is the whole point of the sequence and cannot be heard in
    // any single strike.
    static void dumpCrossingTest(const std::string& wavPath);
    // Rolling noise with one control moved at a time, so each is audible on its own:
    // standing, running up to line speed, the same speed at three axle loads, through a
    // curve, and braking to a stand.
    static void dumpRollingTest(const std::string& wavPath);

private:
    // Shared main -> audio thread (lock-free).
    std::atomic<float> amp_{0.0f};        // target hiss amplitude [0,1]
    std::atomic<float> brightness_{0.0f}; // 0 = apply (warm), 1 = release (bright vent)
    std::atomic<float> envGain_{1.0f};    // brake distance attenuation [0,1]
    std::atomic<float> engRpm_[kMaxEngines]{};      // per-engine speed (rev/min)
    std::atomic<float> engGain_[kMaxEngines]{};     // per-engine distance attenuation [0,1]
    std::atomic<bool> compActive_{false}; // a compressor is pumping
    std::atomic<unsigned> valveEvents_{0};
    std::atomic<float> bellGain_{0.0f};   // nearest ringing crossing [0,1]
    std::atomic<bool> muted_{false};

    // --- Wheel on rail -----------------------------------------------------------
    // Everything the rolling voices need, as plain numbers rather than events. The
    // rhythm of the joints is clocked on the audio thread for the same reason the
    // bell's strikes are (see setCrossingBell): tied to the frame rate it would
    // stutter and change tempo with the graphics.
    std::atomic<float> rollSpeed_{0.0f};    // m/s
    std::atomic<float> rollAxleLoad_{0.0f}; // N per axle (weight / axle count)
    std::atomic<float> rollWork_{0.0f};     // traction + running resistance, [0,1]
    std::atomic<float> brakeWork_{0.0f};    // friction-brake force / adhesion cap, [0,1]
    std::atomic<float> flangeLoad_{0.0f};   // unbalanced lateral accel / g, [0,1]
    std::atomic<unsigned> impacts_{0};      // wheels over a frog, counted since the start
    std::atomic<float> rollGain_{0.0f};     // camera distance attenuation [0,1]
    std::atomic<bool> railborne_{true};     // false when derailed: no rail to roar on

    // Main-thread only.
    int lastCmd_ = 0;
    bool firstUpdate_ = true;
    // Keeping the frog-knock count continuous when the rolling voice changes trains.
    //
    // `impacts` is a running total, and the synth knocks once for each new one, so it
    // is only ever read as a difference. Two trains have unrelated totals: swapping
    // which one feeds it would hand the audio thread a step, and a step is either a
    // burst of knocks or - going the other way, on unsigned - about four billion of
    // them. So the raw count is carried across by an offset and what the synth sees
    // never jumps, whoever it is coming from.
    const void* soundedTrain_ = nullptr;
    unsigned rawImpacts_ = 0;    // last raw count read off the sounded train
    unsigned impactOffset_ = 0;  // added to it; wraps with the count, which is intended

    // Frame counter (the device probe uses it) + a hidden test tone
    // (EBANER_AUDIO_TEST) that checks the output path.
    std::atomic<unsigned long> cbFrames_{0};
    bool testTone_ = false;
    float testPhase_ = 0.0f;

    // Audio-thread only synth state.
    float ampEnv_ = 0.0f, brightEnv_ = 0.0f, envEnv_ = 1.0f;
    float svfLow_ = 0.0f, svfBand_ = 0.0f;   // hiss band-pass
    float clkLow_ = 0.0f, clkBand_ = 0.0f;   // click band-pass
    float clickEnv_ = 0.0f, clickPhase_ = 0.0f;
    // Per-engine diesel voice state.
    float engPhase_[kMaxEngines] = {};       // firing phase
    float engRpmEnv_[kMaxEngines] = {};      // smoothed rpm
    float engGainEnv_[kMaxEngines] = {};     // smoothed distance gain
    float engLp_[kMaxEngines] = {};          // insulation low-pass
    float engKnock_[kMaxEngines] = {};       // per-firing knock envelope
    float engKnLp_[kMaxEngines] = {};        // knock noise low-pass
    float engHunt_[kMaxEngines] = {};        // slow random load/rpm hunting
    float exhaustBuf_[1024] = {};            // exhaust comb (smears knocks into a hum)
    int exhaustIdx_ = 0;
    float exhaustLp_ = 0.0f;
    float compPhase_ = 0.0f, compLp_ = 0.0f, compEnv_ = 0.0f; // compressor pump voice
    // Crossing bell: a struck gong, so one running envelope and phase per partial. The
    // partials are inharmonic, which is what makes it a bell rather than a horn.
    static constexpr int kBellPartials = 7;
    float bellEnv_[kBellPartials] = {};
    float bellPhase_[kBellPartials] = {};
    float bellGainEnv_ = 0.0f;  // smoothed distance level
    float bellTimer_ = 0.0f;    // seconds until the next strike
    float bellClap_ = 0.0f;     // clapper transient envelope
    float bellClapLp_ = 0.0f;
    std::uint32_t bellStrike_ = 0; // strike counter, for the small per-strike variation
    // Rolling-noise synth state (audio thread only).
    float rollSpeedEnv_ = 0.0f, rollLoadEnv_ = 0.0f, rollWorkEnv_ = 0.0f;
    float brakeWorkEnv_ = 0.0f, flangeEnv_ = 0.0f, rollGainEnv_ = 0.0f;
    float rollLow_ = 0.0f, rollBand_ = 0.0f;     // main roughness band-pass
    float rumbLow_ = 0.0f, rumbBand_ = 0.0f;     // the low band weight brings up
    float gritLp_ = 0.0f, gritAm_ = 0.0f;        // working-hard edge + its modulation
    float brkLow_ = 0.0f, brkBand_ = 0.0f;       // the quiet low brake rumble
    unsigned lastImpacts_ = 0;                   // impacts already sounded
    int impactQueue_ = 0;                        // knocks still to sound
    float impactWait_ = 0.0f;                    // s until the next of them
    float jointEnv_ = 0.0f;                      // the knock's envelope...
    float jointLp_ = 0.0f;                       // ...its noise...
    float jointThud_ = 0.0f;                     // ...and the phase of its low thud
    // The knock draws from its own noise generator rather than the shared one. It only
    // draws while a knock is sounding, so sharing would shift every other voice's noise
    // depending on whether a wheel had just crossed a switch - which makes the roar
    // unrepeatable and two renders impossible to compare.
    std::uint32_t jointRng_ = 0x9e3779b9u;
    float sqLow_ = 0.0f, sqBand_ = 0.0f;         // squeal resonance
    float sqPhase_ = 0.0f, sqSlip_ = 0.0f;       // its tone and the stick-slip cycle
    unsigned lastEvents_ = 0;
    std::uint32_t rng_ = 0x1234567u;
    float sampleRate_ = 44100.0f;

    // PulseAudio backend (preferred): a writer thread pushes rendered blocks.
    bool startPulse();
    void pulseLoop();
    void* pa_ = nullptr; // pa_simple* (opaque here)
    std::thread pulseThread_;
    std::atomic<bool> running_{false};

    // PortAudio backend (fallback).
    bool startPortaudio();
    void* stream_ = nullptr; // PaStream* (opaque here)
    bool paReady_ = false;   // Pa_Initialize succeeded
};
