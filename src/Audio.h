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

class Vehicle;

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
    // Main thread, per sim frame. `brakeGain` attenuates the brake sound by camera
    // distance to the bogies; `engGain0/1` do the same per engine end. All in [0,1].
    void update(const Vehicle& v, float dt, float brakeGain, float engGain0,
                float engGain1);
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

private:
    // Shared main -> audio thread (lock-free).
    std::atomic<float> amp_{0.0f};        // target hiss amplitude [0,1]
    std::atomic<float> brightness_{0.0f}; // 0 = apply (warm), 1 = release (bright vent)
    std::atomic<float> envGain_{1.0f};    // brake distance attenuation [0,1]
    std::atomic<float> engRpm_[2]{};      // per-engine speed (rev/min)
    std::atomic<float> engGain_[2]{};     // per-engine distance attenuation [0,1]
    std::atomic<bool> compActive_{false}; // a compressor is pumping
    std::atomic<unsigned> valveEvents_{0};
    std::atomic<bool> muted_{false};

    // Main-thread only.
    int lastCmd_ = 0;
    bool firstUpdate_ = true;

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
    float engPhase_[2] = {0.0f, 0.0f};       // firing phase
    float engRpmEnv_[2] = {0.0f, 0.0f};      // smoothed rpm
    float engGainEnv_[2] = {0.0f, 0.0f};     // smoothed distance gain
    float engLp_[2] = {0.0f, 0.0f};          // insulation low-pass
    float engKnock_[2] = {0.0f, 0.0f};       // per-firing knock envelope
    float engKnLp_[2] = {0.0f, 0.0f};        // knock noise low-pass
    float engHunt_[2] = {0.0f, 0.0f};        // slow random load/rpm hunting
    float exhaustBuf_[1024] = {};            // exhaust comb (smears knocks into a hum)
    int exhaustIdx_ = 0;
    float exhaustLp_ = 0.0f;
    float compPhase_ = 0.0f, compLp_ = 0.0f, compEnv_ = 0.0f; // compressor pump voice
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
