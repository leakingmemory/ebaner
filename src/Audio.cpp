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

#include "Audio.h"

#include "Vehicle.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#if HAVE_PORTAUDIO
#include <portaudio.h>
#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <unistd.h>
#endif
#endif

#if HAVE_PULSE
#include <pulse/error.h>
#include <pulse/simple.h>
#endif

namespace {
constexpr float kPi = 3.14159265358979f;
} // namespace

#if HAVE_PORTAUDIO
namespace {
// Silence ALSA's noisy device-probing messages on stderr for a scope (POSIX).
struct StderrSilencer {
    int saved_ = -1;
    StderrSilencer() {
#if defined(__unix__) || defined(__APPLE__)
        std::fflush(stderr);
        saved_ = dup(2);
        const int nul = open("/dev/null", O_WRONLY);
        if (nul >= 0) { dup2(nul, 2); close(nul); }
#endif
    }
    ~StderrSilencer() {
#if defined(__unix__) || defined(__APPLE__)
        if (saved_ >= 0) { std::fflush(stderr); dup2(saved_, 2); close(saved_); }
#endif
    }
};

// Ordered candidate output devices to probe. Explicit EBANER_AUDIO_DEVICE (index or
// name) wins; otherwise try the system default(s) first, then the server PCMs.
std::vector<PaDeviceIndex> candidateDevices() {
    const int n = Pa_GetDeviceCount();
    auto outCh = [&](int i) {
        const PaDeviceInfo* d = Pa_GetDeviceInfo(i);
        return d ? d->maxOutputChannels : 0;
    };
    std::vector<PaDeviceIndex> c;
    auto add = [&](PaDeviceIndex i) {
        if (i >= 0 && i < n && outCh(i) > 0 &&
            std::find(c.begin(), c.end(), i) == c.end())
            c.push_back(i);
    };
    auto addName = [&](const char* nm) {
        for (int i = 0; i < n; ++i) {
            const PaDeviceInfo* d = Pa_GetDeviceInfo(i);
            if (d && outCh(i) > 0 && std::strstr(d->name, nm)) add(i);
        }
    };
    if (const char* e = std::getenv("EBANER_AUDIO_DEVICE")) {
        char* end = nullptr;
        const long idx = std::strtol(e, &end, 10);
        if (end && *end == '\0') add(static_cast<PaDeviceIndex>(idx));
        else addName(e);
        return c;
    }
    addName("default");
    addName("sysdefault");
    add(Pa_GetDefaultOutputDevice());
    addName("pulse");
    addName("pipewire");
    for (int i = 0; i < n; ++i) add(i); // any remaining output device
    return c;
}
} // namespace
#endif

void Audio::render(float* out, int n) {
    const bool muted = muted_.load(std::memory_order_relaxed);
    if (testTone_) { // output path check: a steady 220 Hz tone (ignores the brakes)
        for (int i = 0; i < n; ++i) {
            out[i] = muted ? 0.0f : 0.25f * std::sin(2.0f * kPi * testPhase_);
            testPhase_ += 220.0f / sampleRate_;
            if (testPhase_ > 1.0f) testPhase_ -= 1.0f;
        }
        cbFrames_.fetch_add(static_cast<unsigned long>(n), std::memory_order_relaxed);
        return;
    }
    const float targetAmp = amp_.load(std::memory_order_relaxed);
    const float targetBright = brightness_.load(std::memory_order_relaxed);
    const float targetEnv = envGain_.load(std::memory_order_relaxed);
    const float engRpmT[2] = {engRpm_[0].load(std::memory_order_relaxed),
                              engRpm_[1].load(std::memory_order_relaxed)};
    const float engGainT[2] = {engGain_[0].load(std::memory_order_relaxed),
                               engGain_[1].load(std::memory_order_relaxed)};
    const float compTarget = compActive_.load(std::memory_order_relaxed) ? 1.0f : 0.0f;
    const unsigned ev = valveEvents_.load(std::memory_order_relaxed);
    if (ev != lastEvents_) { // a valve just operated -> click
        lastEvents_ = ev;
        clickEnv_ = 1.0f;
        clickPhase_ = 0.0f;
    }
    const float fs = sampleRate_;
    for (int i = 0; i < n; ++i) {
        // Smooth the control signals (fast attack, slower release) to avoid clicks.
        ampEnv_ += (targetAmp - ampEnv_) * (targetAmp > ampEnv_ ? 0.004f : 0.0015f);
        brightEnv_ += (targetBright - brightEnv_) * 0.002f;
        envEnv_ += (targetEnv - envEnv_) * 0.001f; // smooth distance fade

        // White noise in [-1,1).
        rng_ = rng_ * 1664525u + 1013904223u;
        const float noise = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;

        // Air hiss: a resonant band-pass whose centre rises for the brighter
        // atmospheric vent (release) vs the warmer cylinder charge (apply).
        const float fc = 1200.0f + brightEnv_ * 2200.0f;
        const float f = 2.0f * std::sin(kPi * fc / fs);
        svfLow_ += f * svfBand_;
        const float high = noise - svfLow_ - 1.0f * svfBand_;
        svfBand_ += f * high;
        const float hiss = svfBand_ * ampEnv_;

        // Valve click: a short band-passed noise edge plus a low "clunk" sine.
        float click = 0.0f;
        if (clickEnv_ > 1e-4f) {
            rng_ = rng_ * 1664525u + 1013904223u;
            const float cn = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
            const float cff = 2.0f * std::sin(kPi * 2000.0f / fs);
            clkLow_ += cff * clkBand_;
            const float chigh = cn - clkLow_ - 1.2f * clkBand_;
            clkBand_ += cff * chigh;
            const float clunk = std::sin(2.0f * kPi * clickPhase_);
            clickPhase_ += 150.0f / fs;
            click = (clkBand_ * 0.7f + clunk * 0.5f) * clickEnv_;
            clickEnv_ *= 0.9990f; // ~23 ms decay
        }

        // Diesel engines: a muffled idle drone per end. Firing thrum (harmonics of
        // the ~35 Hz firing rate) + a soft per-firing knock + a noise hum, heavily
        // low-passed for the insulated/modern character; the two ends are detuned so
        // they beat. Continuous while running; scaled by per-engine distance.
        float engine = 0.0f;
        for (int k = 0; k < 2; ++k) {
            engRpmEnv_[k] += (engRpmT[k] - engRpmEnv_[k]) * 0.002f;
            engGainEnv_[k] += (engGainT[k] - engGainEnv_[k]) * 0.001f;
            const float rpm = engRpmEnv_[k];
            if (rpm <= 20.0f) continue;
            // Slow random load/rpm hunting (mean-reverting walk), most audible at
            // idle: wanders the firing pitch and level a little.
            rng_ = rng_ * 1664525u + 1013904223u;
            const float wn = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
            engHunt_[k] += -engHunt_[k] * 0.00005f + wn * 0.0016f;
            const float firingHz = rpm / 20.0f * (k == 0 ? 1.0f : 1.007f) *
                                   (1.0f + engHunt_[k] * 0.05f); // 3/rev, detuned + hunt
            engPhase_[k] += firingHz / fs;
            if (engPhase_[k] >= 1.0f) { engPhase_[k] -= 1.0f; engKnock_[k] = 1.0f; }
            const float ph = engPhase_[k];
            const float thrum = std::sin(2.0f * kPi * ph) + 0.5f * std::sin(4.0f * kPi * ph) +
                                0.3f * std::sin(6.0f * kPi * ph);
            rng_ = rng_ * 1664525u + 1013904223u;
            engKnLp_[k] += ((static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f) - engKnLp_[k]) * 0.5f;
            const float knock = engKnLp_[k] * engKnock_[k];
            engKnock_[k] *= 0.9985f; // ~15 ms decay (about half a firing period)
            rng_ = rng_ * 1664525u + 1013904223u;
            const float hum = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
            float voice = (thrum * 0.55f + knock * 0.28f + hum * 0.10f) *
                          std::clamp((rpm - 100.0f) / 200.0f, 0.0f, 1.0f) * // crank-in
                          (1.0f + engHunt_[k] * 0.18f);                     // load fluctuation
            engLp_[k] += (voice - engLp_[k]) * 0.11f; // insulation LP (~800 Hz)
            engine += engLp_[k] * engGainEnv_[k];
        }
        // Exhaust muffler: a short low-passed feedback comb that smears the firing
        // pulses into a resonant hum and adds body.
        {
            const int D = 441; // ~10 ms delay
            const float delayed = exhaustBuf_[exhaustIdx_];
            exhaustLp_ += (delayed - exhaustLp_) * 0.25f; // dark feedback
            exhaustBuf_[exhaustIdx_] = engine + exhaustLp_ * 0.5f;
            exhaustIdx_ = (exhaustIdx_ + 1) % D;
            engine = engine * 0.7f + exhaustLp_ * 0.7f;
        }

        // Compressor: a higher, muffled piston-pump hum while charging the reservoir,
        // faded in/out and heard near either engine end.
        compEnv_ += (compTarget - compEnv_) * 0.00008f; // ~0.3 s fade in/out
        float comp = 0.0f;
        if (compEnv_ > 1e-4f) {
            compPhase_ += 90.0f / fs; // ~90 Hz pump (above the ~35 Hz engine hum)
            if (compPhase_ >= 1.0f) compPhase_ -= 1.0f;
            const float cph = compPhase_;
            const float tone = std::sin(2.0f * kPi * cph) + 0.5f * std::sin(4.0f * kPi * cph) +
                               0.3f * std::sin(6.0f * kPi * cph);
            rng_ = rng_ * 1664525u + 1013904223u;
            const float cn = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
            const float air = cn * (0.5f + 0.5f * std::sin(2.0f * kPi * cph)); // pump-modulated
            compLp_ += (tone * 0.5f + air * 0.15f - compLp_) * 0.14f; // ~1 kHz LP (muffled)
            comp = compLp_ * compEnv_ * std::max(engGainEnv_[0], engGainEnv_[1]);
        }

        float s = muted ? 0.0f
                        : (hiss * 1.2f + click * 0.9f) * envEnv_ + engine * 0.30f +
                              comp * 0.22f;
        s = std::clamp(s, -1.0f, 1.0f);
        out[i] = s;
    }
    cbFrames_.fetch_add(static_cast<unsigned long>(n), std::memory_order_relaxed);
}

void Audio::update(const Vehicle& v, float /*dt*/, float brakeGain, float engGain0,
                   float engGain1) {
    const float rate = v.bcRate();
    // The release (venting to atmosphere) is the prominent sound; the filling
    // (charging the cylinders) is quieter, as in reality.
    const float amp = (rate < 0.0f)
                          ? std::min(std::fabs(rate) / 1.5f, 1.0f)          // release
                          : std::min(std::fabs(rate) / 5.0f, 1.0f) * 0.5f;  // apply
    amp_.store(amp, std::memory_order_relaxed);
    brightness_.store(rate < 0.0f ? 1.0f : 0.0f, std::memory_order_relaxed);
    envGain_.store(std::clamp(brakeGain, 0.0f, 1.0f), std::memory_order_relaxed);
    engRpm_[0].store(v.engineRpm(0), std::memory_order_relaxed);
    engRpm_[1].store(v.engineRpm(1), std::memory_order_relaxed);
    engGain_[0].store(std::clamp(engGain0, 0.0f, 1.0f), std::memory_order_relaxed);
    engGain_[1].store(std::clamp(engGain1, 0.0f, 1.0f), std::memory_order_relaxed);
    compActive_.store(v.compressorRunning(), std::memory_order_relaxed);
    // Valve operates whenever the effective brake command changes (handle or the
    // low-reservoir safety).
    const int cmd = v.safetyBrakeActive() ? Vehicle::kEmergencyNotch : v.brakeNotch();
    if (!firstUpdate_ && cmd != lastCmd_)
        valveEvents_.fetch_add(1, std::memory_order_relaxed);
    lastCmd_ = cmd;
    firstUpdate_ = false;
}

#if HAVE_PORTAUDIO
static int paCallback(const void*, void* out, unsigned long frames,
                      const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags,
                      void* user) {
    static_cast<Audio*>(user)->render(static_cast<float*>(out),
                                      static_cast<int>(frames));
    return paContinue;
}
#endif

void Audio::init() {
    if (std::getenv("EBANER_SCREENSHOT")) return; // headless: stay silent
    testTone_ = std::getenv("EBANER_AUDIO_TEST") != nullptr;
#if HAVE_PULSE
    if (startPulse()) return; // preferred: talks to the sound server directly
#endif
#if HAVE_PORTAUDIO
    if (startPortaudio()) return; // fallback: ALSA
#endif
    std::fprintf(stderr, "audio: no working backend; silent\n");
}

#if HAVE_PULSE
bool Audio::startPulse() {
    pa_sample_spec ss;
    ss.format = PA_SAMPLE_FLOAT32NE;
    ss.rate = static_cast<std::uint32_t>(sampleRate_);
    ss.channels = 1;
    const auto bps = static_cast<std::uint32_t>(ss.rate * sizeof(float));
    pa_buffer_attr attr;
    attr.maxlength = static_cast<std::uint32_t>(-1);
    attr.tlength = static_cast<std::uint32_t>(bps * 0.08f); // ~80 ms target buffer
    attr.prebuf = static_cast<std::uint32_t>(-1);
    attr.minreq = static_cast<std::uint32_t>(-1);
    attr.fragsize = static_cast<std::uint32_t>(-1);
    int err = 0;
    pa_simple* s = pa_simple_new(nullptr, "ebaner", PA_STREAM_PLAYBACK, nullptr,
                                 "brake air", &ss, nullptr, &attr, &err);
    if (!s) {
        std::fprintf(stderr, "audio: PulseAudio unavailable (%s)\n", pa_strerror(err));
        return false;
    }
    pa_ = s;
    running_.store(true);
    pulseThread_ = std::thread(&Audio::pulseLoop, this);
    std::fprintf(stderr, "audio: playing via PulseAudio @ %.0f Hz%s\n", sampleRate_,
                 testTone_ ? " [TEST TONE]" : "");
    return true;
}

void Audio::pulseLoop() {
    constexpr int N = 512;
    float buf[N];
    while (running_.load(std::memory_order_relaxed)) {
        render(buf, N); // blocking write paces the loop
        int err = 0;
        if (pa_simple_write(static_cast<pa_simple*>(pa_), buf,
                            static_cast<std::size_t>(N) * sizeof(float), &err) < 0) {
            std::fprintf(stderr, "audio: PulseAudio write failed (%s)\n", pa_strerror(err));
            break;
        }
    }
}
#endif

#if HAVE_PORTAUDIO
bool Audio::startPortaudio() {
    PaError ini;
    {
        StderrSilencer hush; // hide ALSA's probing spam during enumeration
        ini = Pa_Initialize();
    }
    if (ini != paNoError) {
        std::fprintf(stderr, "audio: Pa_Initialize failed; silent\n");
        return false;
    }
    paReady_ = true;

    // List output devices so a choice is visible and overridable.
    for (int i = 0; i < Pa_GetDeviceCount(); ++i) {
        const PaDeviceInfo* d = Pa_GetDeviceInfo(i);
        if (!d || d->maxOutputChannels <= 0) continue;
        const PaHostApiInfo* h = Pa_GetHostApiInfo(d->hostApi);
        std::fprintf(stderr, "audio: [%d] %s (%s)\n", i, d->name, h ? h->name : "?");
    }

    // Probe candidates in order and keep the first whose callback actually pulls
    // samples — device names that "open" but never run the callback (or block) are
    // common on ALSA, so opening alone isn't enough.
    auto tryDevice = [&](PaDeviceIndex dev) -> bool {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(dev);
        if (!info) return false;
        PaStream* s = nullptr;
        double latency = 0.0;
        {
            StderrSilencer hush; // hush ALSA open-time chatter
            PaStreamParameters out{};
            out.device = dev;
            out.channelCount = 1;
            out.sampleFormat = paFloat32;
            out.suggestedLatency =
                info->defaultHighOutputLatency > 0 ? info->defaultHighOutputLatency : 0.08;
            latency = out.suggestedLatency;
            if (Pa_OpenStream(&s, nullptr, &out, sampleRate_, paFramesPerBufferUnspecified,
                              paClipOff, &paCallback, this) != paNoError)
                return false;
            if (Pa_StartStream(s) != paNoError) { Pa_CloseStream(s); return false; }
            cbFrames_.store(0, std::memory_order_relaxed);
            Pa_Sleep(250); // let the callback run
            if (cbFrames_.load(std::memory_order_relaxed) == 0) {
                Pa_StopStream(s);
                Pa_CloseStream(s);
                return false;
            }
            stream_ = s;
        }
        std::fprintf(stderr,
                     "audio: playing on [%d] %s @ %.0f Hz (%.0f ms)%s — set "
                     "EBANER_AUDIO_DEVICE=<index|name> to change\n",
                     dev, info->name, sampleRate_, latency * 1000.0,
                     testTone_ ? " [TEST TONE]" : "");
        return true;
    };

    for (const PaDeviceIndex dev : candidateDevices())
        if (tryDevice(dev)) return true;

    std::fprintf(stderr, "audio: no working output device (callback never ran); "
                         "silent. Try EBANER_AUDIO_DEVICE=<index>.\n");
    Pa_Terminate();
    paReady_ = false;
    return false;
}
#endif

void Audio::shutdown() {
#if HAVE_PULSE
    if (running_.exchange(false) && pulseThread_.joinable()) pulseThread_.join();
    if (pa_) {
        pa_simple_free(static_cast<pa_simple*>(pa_));
        pa_ = nullptr;
    }
#endif
#if HAVE_PORTAUDIO
    if (stream_) {
        Pa_StopStream(static_cast<PaStream*>(stream_));
        Pa_CloseStream(static_cast<PaStream*>(stream_));
        stream_ = nullptr;
    }
    if (paReady_) {
        Pa_Terminate();
        paReady_ = false;
    }
#endif
}

Audio::~Audio() { shutdown(); }

namespace {
void writeWav(const std::string& path, const std::vector<std::int16_t>& pcm, int fs) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return;
    const std::uint32_t dataBytes = static_cast<std::uint32_t>(pcm.size() * 2);
    const std::uint32_t byteRate = static_cast<std::uint32_t>(fs) * 2;
    auto u32 = [&](std::uint32_t v) { std::fwrite(&v, 4, 1, f); };
    auto u16 = [&](std::uint16_t v) { std::fwrite(&v, 2, 1, f); };
    std::fwrite("RIFF", 1, 4, f); u32(36 + dataBytes); std::fwrite("WAVE", 1, 4, f);
    std::fwrite("fmt ", 1, 4, f); u32(16); u16(1); u16(1);
    u32(static_cast<std::uint32_t>(fs)); u32(byteRate); u16(2); u16(16);
    std::fwrite("data", 1, 4, f); u32(dataBytes);
    std::fwrite(pcm.data(), 2, pcm.size(), f);
    std::fclose(f);
}
} // namespace

void Audio::dumpTest(const std::string& wavPath) {
    const float fs = 44100.0f;
    Audio a;
    a.sampleRate_ = fs;
    struct Seg { float dur, rate; };
    // released -> apply B3 -> hold(equalised) -> release -> hold -> emergency -> hold
    const Seg segs[] = {{0.6f, 0.0f}, {1.5f, 1.5f}, {1.0f, 0.0f}, {2.0f, -1.2f},
                        {1.0f, 0.0f}, {0.6f, 6.0f}, {1.2f, 0.0f}};
    std::vector<std::int16_t> pcm;
    for (const Seg& s : segs) {
        if (s.rate != 0.0f) a.valveEvents_.fetch_add(1); // valve opens as flow starts
        a.amp_.store(s.rate < 0.0f ? std::min(std::fabs(s.rate) / 1.5f, 1.0f)
                                   : std::min(std::fabs(s.rate) / 5.0f, 1.0f) * 0.5f);
        a.brightness_.store(s.rate < 0.0f ? 1.0f : 0.0f);
        const int total = static_cast<int>(s.dur * fs);
        float buf[256];
        for (int done = 0; done < total; done += 256) {
            const int n = std::min(256, total - done);
            a.render(buf, n);
            for (int i = 0; i < n; ++i)
                pcm.push_back(static_cast<std::int16_t>(
                    std::clamp(buf[i], -1.0f, 1.0f) * 32767.0f));
        }
    }
    writeWav(wavPath, pcm, static_cast<int>(fs));
    std::fprintf(stderr, "audio: wrote %s (%zu samples)\n", wavPath.c_str(), pcm.size());
}

void Audio::dumpEngineTest(const std::string& wavPath) {
    const float fs = 44100.0f;
    Audio a;
    a.sampleRate_ = fs;
    struct Seg { float dur, rpm; bool comp; };
    // off -> crank -> idle -> idle+compressor -> idle -> stop -> off
    const Seg segs[] = {{0.6f, 0.0f, false},   {4.0f, 700.0f, false}, {2.0f, 700.0f, false},
                        {3.0f, 700.0f, true},  {2.0f, 700.0f, false}, {3.0f, 0.0f, false},
                        {0.6f, 0.0f, false}};
    std::vector<std::int16_t> pcm;
    for (const Seg& s : segs) {
        a.engRpm_[0].store(s.rpm);
        a.engRpm_[1].store(s.rpm);
        a.engGain_[0].store(1.0f);
        a.engGain_[1].store(1.0f);
        a.compActive_.store(s.comp);
        const int total = static_cast<int>(s.dur * fs);
        float buf[256];
        for (int done = 0; done < total; done += 256) {
            const int n = std::min(256, total - done);
            a.render(buf, n);
            for (int i = 0; i < n; ++i)
                pcm.push_back(static_cast<std::int16_t>(
                    std::clamp(buf[i], -1.0f, 1.0f) * 32767.0f));
        }
    }
    writeWav(wavPath, pcm, static_cast<int>(fs));
    std::fprintf(stderr, "audio: wrote %s (%zu samples)\n", wavPath.c_str(), pcm.size());
}
