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

#include "Consist.h"

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
// The grid blower. The blade-passing rate of a big axial fan at full song, and how fast it
// gets there: a couple of seconds spinning up, and slower coasting down with nothing
// driving it any more. Per audio sample, so these are small numbers.
constexpr float kGridBladeHz = 118.0f;
constexpr float kGridSpinUp = 0.000020f;
constexpr float kGridCoast = 0.000009f;
constexpr float kGridIdleSpeed = 0.45f; // what it turns at with the brake barely on

// --- The crossing bell's voice ---------------------------------------------------
// Struck once per flash of the lights (SignalMesh.h's kCrossingFastS): bell and lamps
// run off the same relay at a real crossing, and hearing them agree is most of what
// makes the thing read as one machine.
constexpr float kBellStrikeS = 0.5f;
// Nominal pitch. A crossing gong is small and deliberately piercing, so the strong
// partial an ear latches onto lands near 1.3 kHz - right where it cuts through an engine.
constexpr float kBellHz = 660.0f;
// Bell partials: hum, prime, minor third, fifth, nominal, and two above it. Inharmonic
// on purpose - equal-tempered ratios here would sound like an organ, not a bell.
constexpr float kBellRatio[7] = {0.5f, 1.0f, 1.19f, 1.5f, 2.0f, 2.66f, 4.14f};
// The high partials start loudest and die first, which is the shape of a clang.
constexpr float kBellAmp[7] = {0.10f, 0.26f, 0.20f, 0.22f, 0.34f, 0.20f, 0.11f};
constexpr float kBellDecayS[7] = {1.40f, 0.85f, 0.55f, 0.40f, 0.30f, 0.16f, 0.10f};

// --- Wheel on rail ----------------------------------------------------------------
constexpr float kGravity = 9.81f;
// Wheel/rail adhesion, the ceiling everything through the contact patch works against.
// Vehicle.cpp splits this into a braking and a powering value; one number is enough to
// normalise a loudness by, and it is not part of the physics here.
constexpr float kRailAdhesionMu = 0.25f;
// Tighter than about a 300 m radius before a curve can squeal at all.
constexpr float kSquealCurvature = 1.0f / 300.0f;
// Rolling noise runs with the 30*log10(v) law measured on real track, which is an
// amplitude proportional to v^1.5. Referenced to a brisk line speed so that is ~1.
constexpr float kRollRefSpeed = 33.0f; // m/s (~120 km/h)
// Below a walking pace there is nothing to hear; faded rather than switched, or
// stopping would click.
constexpr float kRollFloorSpeed = 0.6f; // m/s
// Axle load the timbre is normalised against - about what a loaded Class 93 axle
// carries, and the heaviest of the vehicles on offer.
constexpr float kRefAxleLoad = 11500.0f * kGravity; // N
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
    float engRpmT[kMaxEngines] = {}, engGainT[kMaxEngines] = {};
    for (int k = 0; k < kMaxEngines; ++k) {
        engRpmT[k] = engRpm_[k].load(std::memory_order_relaxed);
        engGainT[k] = engGain_[k].load(std::memory_order_relaxed);
    }
    const float compTarget = compActive_.load(std::memory_order_relaxed) ? 1.0f : 0.0f;
    const float gridTarget = gridLoad_.load(std::memory_order_relaxed);
    const float bellTarget = bellGain_.load(std::memory_order_relaxed);
    const bool railborne = railborne_.load(std::memory_order_relaxed);
    const float rollSpeedT = railborne ? rollSpeed_.load(std::memory_order_relaxed) : 0.0f;
    const float rollLoadT = rollAxleLoad_.load(std::memory_order_relaxed);
    const float rollWorkT = rollWork_.load(std::memory_order_relaxed);
    const float brakeWorkT = brakeWork_.load(std::memory_order_relaxed);
    const float flangeT = railborne ? flangeLoad_.load(std::memory_order_relaxed) : 0.0f;
    const float rollGainT = rollGain_.load(std::memory_order_relaxed);
    const unsigned impacts = impacts_.load(std::memory_order_relaxed);
    if (impacts != lastImpacts_) { // wheels have crossed a frog since the last block
        impactQueue_ += static_cast<int>(impacts - lastImpacts_);
        lastImpacts_ = impacts;
    }
    const unsigned ev = valveEvents_.load(std::memory_order_relaxed);
    if (ev != lastEvents_) { // a valve just operated -> click
        lastEvents_ = ev;
        clickEnv_ = 1.0f;
        clickPhase_ = 0.0f;
    }
    const float fs = sampleRate_;
    // The band-pass width the roar's level is referenced to (see `bw` below).
    const float rollBandRef = 2.0f * std::sin(kPi * 800.0f / fs);
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

        // Diesel engines, one voice each. Firing thrum (harmonics of the firing rate) +
        // a soft per-firing knock + a noise hum, low-passed for the engine room it sits
        // in; each is detuned from the last so they beat. Continuous while running;
        // scaled by per-engine distance.
        //
        // How fast it beats, how loud it is, how much weight it carries below the firing
        // rate and how much of the bark gets out are the engine's own, not this file's.
        // A 6-cylinder four-stroke Cummins behind a modern railcar's sound-deadening
        // beats 3 to the rev, quietly and darkly; a 16-cylinder two-stroke EMD of 170
        // litres in an uninsulated 1981 engine room beats 16, twice as loud, and is full
        // of half- and quarter-order content underneath - which is what is heard as size.
        float engine = 0.0f;
        for (int k = 0; k < kMaxEngines; ++k) {
            engRpmEnv_[k] += (engRpmT[k] - engRpmEnv_[k]) * 0.002f;
            engGainEnv_[k] += (engGainT[k] - engGainEnv_[k]) * 0.001f;
            const float rpm = engRpmEnv_[k];
            if (rpm <= 20.0f) continue;
            // Slow random load/rpm hunting (mean-reverting walk), most audible at
            // idle: wanders the firing pitch and level a little.
            rng_ = rng_ * 1664525u + 1013904223u;
            const float wn = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
            engHunt_[k] += -engHunt_[k] * 0.00005f + wn * 0.0016f;
            // Each engine is detuned a little from the last, so several of them beat
            // against one another rather than doubling into one louder engine.
            const float fire = engFire_[k].load(std::memory_order_relaxed);
            const float vol = engVol_[k].load(std::memory_order_relaxed);
            const float rum = engRum_[k].load(std::memory_order_relaxed);
            const float bri = engBri_[k].load(std::memory_order_relaxed);
            const float firingHz = rpm / 60.0f * fire *
                                   (1.0f + 0.007f * static_cast<float>(k)) *
                                   (1.0f + engHunt_[k] * 0.05f); // detuned + hunt
            // How much of the heavy-engine character this voice gets. Taken from the
            // rumble weight rather than from a knob of its own: an engine big enough to
            // radiate below its firing rate is the same engine whose exhaust beats are
            // separate events and whose gear train can be heard. A railcar's is 0 and
            // keeps the smooth voice it had.
            const float heavy = std::min(1.0f, rum * 0.5f);
            engPhase_[k] += firingHz / fs;
            if (engPhase_[k] >= 1.0f) {
                engPhase_[k] -= 1.0f;
                engKnock_[k] = 1.0f;
                // No two firings alike. Sixteen cylinders are never quite in step - fuel,
                // wear and temperature all differ - and it is that unevenness, not the
                // pitch, that separates an engine from an oscillator. A tenth of a dB
                // would not be heard; a third of the amplitude is.
                //
                // Drawn only for an engine that uses it, so that a voice with no rumble
                // weight takes exactly the samples out of the generator it always did
                // and the railcar's engine is unchanged to the last bit.
                if (heavy > 0.0f) {
                    rng_ = rng_ * 1664525u + 1013904223u;
                    const float r = static_cast<float>(rng_ >> 8) / 8388608.0f; // [-1,1]
                    engFireAmp_[k] = 1.0f + heavy * r * 0.35f;
                }
            }
            const float ph = engPhase_[k];
            float thrum = std::sin(2.0f * kPi * ph) + 0.5f * std::sin(4.0f * kPi * ph) +
                          0.3f * std::sin(6.0f * kPi * ph);
            // The exhaust blast. A sine stack is a smooth thing and a cylinder emptying
            // is not: the port opens, the charge goes out in a fraction of the cycle, and
            // the rest of the cycle is comparatively quiet. So a raised-cosine pulse over
            // the first third of the firing period, with its mean taken out so it adds
            // harmonics rather than a DC step. The harmonics are the point - they are
            // what is heard as volume in the cylinder and length in the pipe, and they
            // are what the smooth voice was missing.
            if (heavy > 0.0f) {
                constexpr float kDuty = 0.34f; // how much of the period the blast occupies
                const float blast =
                    ph < kDuty ? (0.5f - 0.5f * std::cos(2.0f * kPi * ph / kDuty)) - 0.5f
                               : -0.5f;
                thrum += heavy * blast * 0.80f * engFireAmp_[k];
            }
            // The weight underneath, and on a V engine it is not a fudge: a V16 is two
            // banks of eight with a manifold each, so each bank fires eight times a
            // revolution and radiates at half the whole engine's rate. The quarter order
            // below that is the pairing across the vee. It is those, an octave and two
            // under the firing rate, that make an engine sound large - not the firing
            // rate, which goes the other way from intuition. A 16-cylinder two-stroke at
            // 315 rpm fires at 84 Hz where a 6-cylinder four-stroke at 700 fires at 35:
            // the big slow engine has the HIGHER firing rate of the two. Weight has to
            // come from going below it, not from slowing it down.
            if (rum > 0.0f) {
                engSubPhase_[k] += firingHz * 0.25f / fs;
                if (engSubPhase_[k] >= 1.0f) engSubPhase_[k] -= 1.0f;
                const float sp = engSubPhase_[k];
                thrum += rum * (0.85f * std::sin(2.0f * kPi * sp) +   // quarter order
                                0.55f * std::sin(4.0f * kPi * sp));   // half order
            }
            rng_ = rng_ * 1664525u + 1013904223u;
            engKnLp_[k] += ((static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f) - engKnLp_[k]) * 0.5f;
            const float knock = engKnLp_[k] * engKnock_[k] * engFireAmp_[k];
            engKnock_[k] *= 0.9985f; // ~15 ms decay (about half a firing period)
            rng_ = rng_ * 1664525u + 1013904223u;
            const float hum = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
            float voice = (thrum * 0.55f + knock * 0.28f + hum * 0.10f) *
                          std::clamp((rpm - 100.0f) / 200.0f, 0.0f, 1.0f) * // crank-in
                          (1.0f + engHunt_[k] * 0.18f);                     // load fluctuation
            engLp_[k] += (voice - engLp_[k]) * bri; // insulation LP
            // A pulse carries more level than the sine stack it replaced, and this was
            // meant to change what the engine sounds like and not how loud it is - it is
            // already the loudest thing on the locomotive, and the headroom belongs to
            // the wheels and the brakes as much as to it. So the heavy voice is trimmed
            // back to about the level it had, and the difference is all spectrum.
            voice = engLp_[k] * vol / (1.0f + 0.22f * heavy);
            // Moving parts, and they are deliberately outside the insulation low-pass.
            // Exhaust noise comes out of a pipe and arrives dark; gear train, injectors
            // and valve gear are radiated by the block and the covers and keep their
            // treble, which is why a big diesel at idle is heard as a clatter over a hum
            // and not as a hum alone. Clocked at half the firing rate so it is its own
            // slower rattle rather than more weight on the firing beat.
            if (heavy > 0.0f) {
                engClPhase_[k] += firingHz * 0.5f / fs;
                if (engClPhase_[k] >= 1.0f) {
                    engClPhase_[k] -= 1.0f;
                    rng_ = rng_ * 1664525u + 1013904223u;
                    const float r = static_cast<float>(rng_ >> 8) / 8388608.0f;
                    engClEnv_[k] = 0.65f + 0.35f * r; // uneven, like the firings
                }
                rng_ = rng_ * 1664525u + 1013904223u;
                const float n = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
                // Two low-passes differenced into a band-pass, around a kilohertz: the
                // rattle wants presence, not hiss and not more bottom end.
                engClA_[k] += (n - engClA_[k]) * 0.30f;
                engClB_[k] += (engClA_[k] - engClB_[k]) * 0.09f;
                const float rattle = (engClA_[k] - engClB_[k]) * engClEnv_[k];
                engClEnv_[k] *= 0.9970f; // ~7 ms, a tick and not a tone
                voice += rattle * vol * heavy * 0.34f *
                         std::clamp((rpm - 100.0f) / 200.0f, 0.0f, 1.0f);
            }
            engine += voice * engGainEnv_[k];
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
            {
            float near = 0.0f;
            for (int k = 0; k < kMaxEngines; ++k) near = std::max(near, engGainEnv_[k]);
            comp = compLp_ * compEnv_ * near; // heard from whichever end is nearest
        }
        }

        // Grid blower: the fans that force air through the rheostatic brake's resistors.
        //
        // Fed from the braking current itself, so the fan speeds up the harder the brake is
        // working - which is the whole character of the thing. Wind the controller back and
        // the roar builds; notch off and it coasts down. It is not a switch, and the fan
        // does not follow the load instantly either: it has inertia, and it runs down more
        // slowly than it runs up because nothing is driving it any more.
        //
        // (That the blower is current-fed is an assumption. It is reasonable for a BBC
        // machine of this period and it is the more interesting behaviour, but it is not
        // something I could source for this locomotive in particular.)
        //
        // Three parts: rushing air through the ducts, which dominates and which rises in
        // pitch as well as level; the blade-passing tone and its harmonics; and enough of
        // the driving motor to keep it from sounding like plain noise. Fan noise goes up
        // very steeply with speed - near the fifth power - so this is quiet at E1 and
        // unmistakable at E5 rather than fading up evenly across the range.
        {
            // The fan does not crawl at the first notch. A blower motor fed from the
            // braking current still has a substantial no-load speed, so the useful range
            // is something like half to full rather than nothing to full - which is also
            // what makes the first notch audible instead of a fiftieth of the sound.
            const float want = gridTarget > 0.0f ? kGridIdleSpeed +
                                                       (1.0f - kGridIdleSpeed) * gridTarget
                                                 : 0.0f;
            const float rate = want > gridFan_ ? kGridSpinUp : kGridCoast;
            gridFan_ += (want - gridFan_) * rate;
        }
        float grid = 0.0f;
        if (gridFan_ > 1e-3f) {
            const float sp = gridFan_;
            gridPhase_ += kGridBladeHz * (0.35f + 0.65f * sp) / fs;
            if (gridPhase_ >= 1.0f) gridPhase_ -= 1.0f;
            const float gp = gridPhase_;
            // More of the tone, and more harmonics in it. A grid blower heard from the
            // cab is not a rushing sound with a note somewhere in it: it is a hum, and
            // loud enough to talk over. The harmonics are what make it a hum and not a
            // sine - they are the blades passing a structure, not air moving.
            const float blade = std::sin(2.0f * kPi * gp) + 0.55f * std::sin(4.0f * kPi * gp) +
                                0.32f * std::sin(6.0f * kPi * gp) +
                                0.17f * std::sin(8.0f * kPi * gp);
            // And the shaft turning under all of it. A fan this size is never balanced to
            // nothing, so the rush is louder once per revolution - a slow throb, about
            // ten a second at full song, which is heard as size rather than as a pitch.
            gridShaft_ += kGridBladeHz / 12.0f * (0.35f + 0.65f * sp) / fs;
            if (gridShaft_ >= 1.0f) gridShaft_ -= 1.0f;
            const float throb = 1.0f + 0.15f * std::sin(2.0f * kPi * gridShaft_);
            rng_ = rng_ * 1664525u + 1013904223u;
            const float wn = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
            // A two-pole band-pass made of one low-pass chasing another: the centre rides
            // up with the fan, which is what makes it a fan and not a volume control.
            const float k = std::clamp(0.05f + 0.22f * sp, 0.02f, 0.5f);
            gridBp1_ += (wn - gridBp1_) * k;
            gridBp2_ += (gridBp1_ - gridBp2_) * k * 0.45f;
            const float air = gridBp1_ - gridBp2_;
            // Steep with fan speed, as fan noise is, but not as steep as the fifth-power
            // law on its own would make it: taken literally that puts a fifth of the brake
            // at a fiftieth of the sound, and the first notch becomes inaudible. The ear
            // wants to hear the brake come on.
            const float loud = sp * std::sqrt(sp);
            float near = 0.0f;
            for (int k2 = 0; k2 < kMaxEngines; ++k2) near = std::max(near, engGainEnv_[k2]);
            // Soft-limited rather than scaled down. Band-passed noise has a high crest
            // factor - all peak and little body - so the level that sounds right is the
            // level that clips, and taking the tops off is what lets a rushing sound be
            // loud without eating the headroom the wheels and the brakes need.
            // Driven harder into the same soft limit, and the ceiling brought down a
            // little to pay for it: what comes out is louder on average and far more of
            // it is hum, without the peaks growing to match. A hard clamp on the master
            // sum is the only thing below this, and the wheels and the brakes are sounding
            // over the top of the blower whenever the blower is working at all.
            grid = std::tanh((air * 2.15f * throb + blade * 0.78f) * loud) * 0.58f * near;
        }

        // --- Crossing warning bell ------------------------------------------------
        // A struck gong rather than a tone. The partials are inharmonic - the ratios a
        // cast bell gives, hum through nominal and above - and each decays at its own
        // rate, the high ones fastest. That is what turns a chord into a clang: the
        // strike is bright and metallic and rings down into the low partials.
        bellGainEnv_ += (bellTarget - bellGainEnv_) * (bellTarget > bellGainEnv_ ? 0.01f
                                                                                 : 0.0006f);
        float bell = 0.0f;
        if (bellGainEnv_ > 1e-4f) {
            bellTimer_ -= 1.0f / fs;
            if (bellTimer_ <= 0.0f && bellTarget > 1e-4f) {
                bellTimer_ += kBellStrikeS;
                ++bellStrike_;
                // A hair of variation per strike, so a long ring does not turn into an
                // obvious loop of one identical sample.
                const float vary = 0.92f + 0.08f * static_cast<float>(bellStrike_ & 3u) / 3.0f;
                for (int p = 0; p < kBellPartials; ++p)
                    bellEnv_[p] += kBellAmp[p] * vary; // add, so close strikes overlap
                bellClap_ = 1.0f;
            }
            for (int p = 0; p < kBellPartials; ++p) {
                if (bellEnv_[p] < 1e-5f) continue;
                bellEnv_[p] *= std::exp(-1.0f / (kBellDecayS[p] * fs));
                bellPhase_[p] += kBellHz * kBellRatio[p] / fs;
                if (bellPhase_[p] > 1.0f) bellPhase_[p] -= 1.0f;
                bell += bellEnv_[p] * std::sin(2.0f * kPi * bellPhase_[p]);
            }
            // The clapper itself: a few milliseconds of bright noise on the strike, which
            // is most of what makes it read as struck metal rather than a synth tone.
            if (bellClap_ > 1e-4f) {
                bellClap_ *= std::exp(-1.0f / (0.004f * fs));
                rng_ = rng_ * 1664525u + 1013904223u;
                const float bn = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
                bellClapLp_ += (bn - bellClapLp_) * 0.7f; // gentle high-pass-ish shaping
                bell += (bn - bellClapLp_) * bellClap_ * 0.5f;
            }
            bell *= bellGainEnv_;
        }

        // --- Wheel on rail --------------------------------------------------------
        // The roar is roughness on both surfaces exciting both into vibration, so it
        // starts as noise; what makes it a train rather than a hiss is the shape put
        // on it by how fast, how heavy and how hard-worked the contact is.
        rollSpeedEnv_ += (rollSpeedT - rollSpeedEnv_) * 0.0012f;
        rollLoadEnv_ += (rollLoadT - rollLoadEnv_) * 0.0008f;
        rollWorkEnv_ += (rollWorkT - rollWorkEnv_) * 0.0008f;
        brakeWorkEnv_ += (brakeWorkT - brakeWorkEnv_) * 0.0010f;
        flangeEnv_ += (flangeT - flangeEnv_) * 0.0008f;
        rollGainEnv_ += (rollGainT - rollGainEnv_) * 0.0010f;

        const float vv = rollSpeedEnv_;
        // The measured law: 30*log10(v) in level, so v^1.5 in amplitude. Nothing else
        // in this file has anything like the range - a factor of eight between 36 and
        // 144 km/h - and that range is the feature.
        const float vN = vv / kRollRefSpeed;
        const float speedLevel = vN * std::sqrt(std::max(vN, 0.0f));
        // Fade in off the floor rather than switch, or coming to a stand would click.
        const float moving = std::clamp((vv - kRollFloorSpeed) / kRollFloorSpeed,
                                        0.0f, 1.0f);
        // How heavily each axle presses. A big contact patch cannot be excited by the
        // short-wavelength roughness a small one can, and heavier wheels radiate lower:
        // weight darkens the sound as much as it loudens it, which is also how the ear
        // tells a loaded vehicle from an empty one.
        const float loadN = std::clamp(rollLoadEnv_ / kRefAxleLoad, 0.0f, 1.2f);
        const float rollLvl = speedLevel * moving * rollGainEnv_ *
                              (0.55f + 0.45f * loadN) * (1.0f + 0.25f * rollWorkEnv_);

        float roll = 0.0f;
        if (rollLvl > 1e-5f) {
            rng_ = rng_ * 1664525u + 1013904223u;
            const float rn = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
            // The dominant roughness wavelength passes at a rate that goes with speed,
            // so the band climbs as the train accelerates; weight pulls it back down.
            const float rfc = (350.0f + 26.0f * vv) * (1.0f - 0.35f * loadN);
            const float rf = 2.0f * std::sin(kPi * std::min(rfc, 0.45f * fs) / fs);
            rollLow_ += rf * rollBand_;
            rollBand_ += rf * (rn - rollLow_ - 0.9f * rollBand_);
            // A state-variable band-pass gets wider as its centre rises, so the same
            // noise through it comes out louder - which would lay a second speed law
            // on top of the measured one and make the level climb far too steeply.
            // Normalise it away: speed and weight move the band, the law sets the level.
            const float bw = std::sqrt(rollBandRef / std::max(rf, 1e-6f));
            // The rumble underneath, which is almost all of what "heavy" sounds like.
            const float lfc = 90.0f + 40.0f * loadN;
            const float lf = 2.0f * std::sin(kPi * lfc / fs);
            rumbLow_ += lf * rumbBand_;
            rumbBand_ += lf * (rn - rumbLow_ - 0.5f * rumbBand_);
            // Working hard puts a gritty edge on top, wandering slowly so it is a
            // texture and not a tone.
            rng_ = rng_ * 1664525u + 1013904223u;
            const float gn = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
            gritLp_ += (gn - gritLp_) * 0.6f;
            gritAm_ += (-gritAm_ * 0.0004f) + gn * 0.004f;
            const float grit = (gn - gritLp_) * rollWorkEnv_ * (0.7f + 0.3f * gritAm_);
            roll = (rollBand_ * bw + rumbLow_ * (0.35f + 0.65f * loadN) + grit * 0.5f) *
                   rollLvl;
        }

        // Wheels over the points. Welded rail has nothing to knock against between
        // them, so these are not a rhythm to be clocked but events at places: the sim
        // says an axle has crossed a frog and one knock is sounded for it. A bogie's
        // pair, a carriage's four, the spacing between them - all of that is the train
        // going over the switch, and none of it has to be described here.
        float joints = 0.0f;
        {
            // Several arriving in one block are spread rather than collapsed into one
            // knock; at line speed a bogie's two axles are 60 ms apart, so this only
            // ever bites after a dropped frame.
            if (impactWait_ > 0.0f) impactWait_ -= 1.0f / fs;
            if (impactQueue_ > 0 && impactWait_ <= 0.0f) {
                --impactQueue_;
                impactWait_ = 0.012f;
                jointEnv_ = 1.0f;
                jointThud_ = 0.0f;
            }
            if (jointEnv_ > 1e-4f) {
                // Impact energy climbs with both the speed it is struck at and the
                // weight behind it, which is why a loaded train is heard over the
                // points from further off than a light one at the same speed.
                jointRng_ = jointRng_ * 1664525u + 1013904223u;
                const float jn = static_cast<float>(jointRng_ >> 8) / 8388608.0f - 1.0f;
                jointLp_ += (jn - jointLp_) * 0.35f;
                const float thud = std::sin(2.0f * kPi * jointThud_);
                jointThud_ += 70.0f / fs;
                joints = (jointLp_ * 0.8f + thud * 0.6f) * jointEnv_ *
                         std::clamp(vv / kRollRefSpeed, 0.0f, 1.3f) *
                         (0.35f + 0.65f * loadN) * rollGainEnv_;
                jointEnv_ *= std::exp(-1.0f / (0.020f * fs)); // ~20 ms knock
            }
        }

        // Two trains meeting. One voice for the whole range, because it is one event:
        // severity moves the pitch of the thud down, the noise darker and the decay
        // longer all at once, which is the difference between two coupler heads
        // clacking together and a train being wrecked.
        float bang = 0.0f;
        {
            const unsigned nb = bangs_.load(std::memory_order_relaxed);
            if (nb != lastBangs_) {
                lastBangs_ = nb;
                bangSev_ = std::clamp(bangSeverity_.load(std::memory_order_relaxed),
                                      0.0f, 1.0f);
                bangHeard_ = std::clamp(bangGain_.load(std::memory_order_relaxed),
                                        0.0f, 1.0f);
                bangEnv_ = 1.0f;
                bangThud_ = 0.0f;
            }
            if (bangEnv_ > 1e-4f) {
                bangRng_ = bangRng_ * 1664525u + 1013904223u;
                const float bn = static_cast<float>(bangRng_ >> 8) / 8388608.0f - 1.0f;
                // Darker the harder it was: a light clack is metal on metal, a crash is
                // mostly the low end of everything at once.
                bangLp_ += (bn - bangLp_) * (0.50f - 0.35f * bangSev_);
                const float thud = std::sin(2.0f * kPi * bangThud_);
                bangThud_ += (90.0f - 50.0f * bangSev_) / fs;
                if (bangThud_ > 1.0f) bangThud_ -= 1.0f;
                bang = (bangLp_ * (0.45f + 0.55f * bangSev_) + thud * 0.9f) * bangEnv_ *
                       (0.25f + 0.75f * bangSev_) * bangHeard_;
                // 50 ms for a touch, half a second for a wreck.
                bangEnv_ *= std::exp(-1.0f / ((0.05f + 0.45f * bangSev_ * bangSev_) * fs));
            }
        }

        // Curve squeal: stick-slip as the flange is dragged across the railhead. A
        // near-tone, because it is one wheel mode ringing, and gated hard - the sim
        // hands over a zero unless the curve is genuinely tight and unbalanced, and a
        // squeal on straight track would be worse than none at all.
        float squeal = 0.0f;
        if (flangeEnv_ > 1e-3f && vv > 1.5f) {
            rng_ = rng_ * 1664525u + 1013904223u;
            const float sn = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
            const float sfc = 2100.0f;
            const float sf = 2.0f * std::sin(kPi * sfc / fs);
            sqLow_ += sf * sqBand_;
            sqBand_ += sf * (sn - sqLow_ - 0.02f * sqBand_); // high Q: a ringing band
            sqPhase_ += sfc / fs;
            if (sqPhase_ > 1.0f) sqPhase_ -= 1.0f;
            sqSlip_ += 23.0f / fs;                            // the slip cycle itself
            if (sqSlip_ > 1.0f) sqSlip_ -= 1.0f;
            const float am = 0.55f + 0.45f * std::sin(2.0f * kPi * sqSlip_);
            squeal = (sqBand_ * 0.5f + std::sin(2.0f * kPi * sqPhase_) * 0.5f) * am *
                     flangeEnv_ * std::clamp((vv - 1.5f) / 8.0f, 0.0f, 1.0f) *
                     rollGainEnv_;
        }

        // The friction brake. Discs, not shoes on the tread: on a Class 93 there is
        // very little to hear and what there is sits low, so this is a soft rumble
        // that follows the brake force and nothing above it. It is not the air hiss
        // above, which is the valve and the pipe rather than the friction surfaces.
        float brakeRub = 0.0f;
        if (brakeWorkEnv_ > 1e-3f && vv > 0.3f) {
            rng_ = rng_ * 1664525u + 1013904223u;
            const float bn = static_cast<float>(rng_ >> 8) / 8388608.0f - 1.0f;
            const float bfc = 60.0f + 90.0f * std::clamp(vv / kRollRefSpeed, 0.0f, 1.0f);
            const float bf = 2.0f * std::sin(kPi * bfc / fs);
            brkLow_ += bf * brkBand_;
            brkBand_ += bf * (bn - brkLow_ - 0.35f * brkBand_);
            brakeRub = brkLow_ * brakeWorkEnv_ *
                       std::clamp((vv - 0.3f) / 4.0f, 0.0f, 1.0f) * rollGainEnv_;
        }

        // The four wheel/rail voices are one group and are mixed as one: their balance
        // against each other is what makes the sound, so a level change scales all four
        // together rather than picking at one of them. This is a quarter down on where
        // they started, which sat a little loud against the engines.
        const float rollMix = 0.75f;
        float s = muted ? 0.0f
                        : (hiss * 1.2f + click * 0.9f) * envEnv_ + engine * 0.30f +
                              comp * 0.22f + grid * 0.45f + bell * 0.34f + bang * 0.55f +
                              (roll * 0.45f + joints * 0.30f + squeal * 0.16f +
                               brakeRub * 0.16f) * rollMix;
        s = std::clamp(s, -1.0f, 1.0f);
        out[i] = s;
    }
    cbFrames_.fetch_add(static_cast<unsigned long>(n), std::memory_order_relaxed);
}

void Audio::setRolling(const RollingSample& r) {
    const bool on = r.railborne;
    rollSpeed_.store(on ? std::max(r.speed, 0.0f) : 0.0f, std::memory_order_relaxed);
    rollAxleLoad_.store(std::max(r.axleLoadN, 0.0f), std::memory_order_relaxed);
    rollWork_.store(std::clamp(r.work, 0.0f, 1.0f), std::memory_order_relaxed);
    brakeWork_.store(std::clamp(r.brake, 0.0f, 1.0f), std::memory_order_relaxed);
    // The squeal is gated on the way in, not in the synth: off the rails, or on track
    // that is not being worked across, there is nothing for a flange to grind on.
    flangeLoad_.store(on ? std::clamp(r.flange, 0.0f, 1.0f) : 0.0f,
                      std::memory_order_relaxed);
    railborne_.store(on, std::memory_order_relaxed);
    rollGain_.store(std::clamp(r.gain, 0.0f, 1.0f), std::memory_order_relaxed);
    impacts_.store(r.impacts, std::memory_order_relaxed);
}

void Audio::impact(float severity, float gain) {
    // Severity and gain first, then the count: the audio thread latches them when it
    // sees the count move, so they must already be there when it does.
    bangSeverity_.store(std::clamp(severity, 0.0f, 1.0f), std::memory_order_relaxed);
    bangGain_.store(std::clamp(gain, 0.0f, 1.0f), std::memory_order_relaxed);
    bangs_.fetch_add(1, std::memory_order_release);
}

void Audio::update(const Consist& sounded, float /*dt*/, float brakeGain,
                   const EngineVoice* engines, int engineCount, float rollGain) {
    const Consist& v = sounded; // everything below asks the train, not a set
    // Two different sounds of air, and the brake makes both at once.
    //
    // The train line is the one you hear: it runs the length of the train, and a
    // reduction or a dump is a long loud rush from everywhere at once. The cylinders are
    // local - a bogie filling is a short sigh under your feet - so they are quieter, and
    // they matter mostly for the moment after the pipe has gone quiet again.
    //
    // Venting to atmosphere is prominent and bright; charging is subdued and dull. That
    // holds for both, which is why the pipe recharging after a release is the soft fill
    // you hear a long way into the platform.
    const float bpR = v.bpRate();
    const float bcR = v.bcRate();
    const float pipeAmp = (bpR < 0.0f)
                              ? std::min(std::fabs(bpR) / 3.0f, 1.0f)          // vent
                              : std::min(std::fabs(bpR) / 1.6f, 1.0f) * 0.40f; // charge
    const float cylAmp = (bcR < 0.0f)
                             ? std::min(std::fabs(bcR) / 1.6f, 1.0f) * 0.55f   // exhaust
                             : std::min(std::fabs(bcR) / 6.0f, 1.0f) * 0.30f;  // fill
    const float amp = std::max(pipeAmp, cylAmp);
    amp_.store(amp, std::memory_order_relaxed);
    // Bright if whichever of the two is louder is venting rather than filling.
    const float loud = (pipeAmp >= cylAmp) ? bpR : bcR;
    brightness_.store(loud < 0.0f ? 1.0f : 0.0f, std::memory_order_relaxed);
    envGain_.store(std::clamp(brakeGain, 0.0f, 1.0f), std::memory_order_relaxed);
    // Every voice handed in gets a slot, up to what the synth holds. A slot nothing is
    // driving is silent rather than absent, so a train losing engines - a set uncoupled
    // away - leaves its slots quiet instead of shifting the others along, and no filter
    // state is smeared from one engine onto another.
    const int n = engines ? std::min(engineCount, kMaxEngines) : 0;
    for (int k = 0; k < kMaxEngines; ++k) {
        engRpm_[k].store(k < n ? engines[k].rpm : 0.0f, std::memory_order_relaxed);
        engGain_[k].store(k < n ? std::clamp(engines[k].gain, 0.0f, 1.0f) : 0.0f,
                          std::memory_order_relaxed);
        const Audio::EngineVoice d;
        const Audio::EngineVoice& e = k < n ? engines[k] : d;
        engFire_[k].store(std::max(0.5f, e.firingsPerRev), std::memory_order_relaxed);
        engVol_[k].store(std::max(0.0f, e.volume), std::memory_order_relaxed);
        engRum_[k].store(std::clamp(e.rumble, 0.0f, 2.0f), std::memory_order_relaxed);
        engBri_[k].store(std::clamp(e.bright, 0.01f, 1.0f), std::memory_order_relaxed);
    }
    compActive_.store(v.compressorRunning(), std::memory_order_relaxed);
    gridLoad_.store(std::clamp(v.dynamicBrakeFrac(), 0.0f, 1.0f), std::memory_order_relaxed);

    // --- Wheel on rail ------------------------------------------------------------
    // Four things the contact is doing, kept apart rather than summed into one number,
    // because each drives a different sound: how fast, how heavily loaded, how hard it
    // is being worked along the rail, and how hard it is being pushed across it.
    RollingSample r;
    r.speed = v.speed();
    r.railborne = v.state() == VehicleState::OnRail;
    r.gain = rollGain;
    // See soundedTrain_: the count the synth is shown is continuous across a change of
    // train even though the two trains' own counts have nothing to do with each other.
    const unsigned raw = v.railImpacts();
    if (&v != soundedTrain_) {
        impactOffset_ += rawImpacts_ - raw; // unsigned, and the wrap is the point
        soundedTrain_ = &v;
    }
    rawImpacts_ = raw;
    r.impacts = impactOffset_ + raw;
    const float weight = v.mass() * kGravity;
    // Spread over every axle of every set: a longer train is heavier but presses no
    // harder per wheel, which is why this is a load and not a weight.
    r.axleLoadN = weight / static_cast<float>(std::max(v.axleCount(), 1));

    // Work along the rail: what the wheels are pulling or pushing through the contact
    // patch, against what the contact patch can hold. The running resistance belongs
    // here too - it is the same surfaces, just always present.
    const float adhesion = std::max(kRailAdhesionMu * weight, 1.0f);
    r.work = (std::fabs(v.tractiveEffort()) + v.rollingResistance(r.speed)) / adhesion;
    r.brake = v.brakeForce() / adhesion;

    // Work across the rail: the lateral acceleration the cant does *not* take out is
    // what the flange has to. On a well canted curve at line speed that is near zero
    // and there is nothing to hear, which is right - a curve only squeals when it is
    // not being taken the way it was built to be.
    const TrackPose pose = v.pose();
    if (std::fabs(pose.curvature) > kSquealCurvature)
        r.flange = (r.speed * r.speed * std::fabs(pose.curvature) -
                    kGravity * std::sin(std::fabs(pose.cant))) / kGravity;
    setRolling(r);

    // Valve operates whenever the effective brake command changes (handle or the
    // low-reservoir safety).
    const int cmd = v.effectiveNotch();
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
    // Which engine to render. EBANER_AUDIO_ENGINE names a fragment of a vehicle's name;
    // without it, the railcar, so the existing reference dump is unchanged. The point of
    // being able to pick is that "louder and heavier" is a claim about two sounds and
    // cannot be checked by listening to one of them.
    const VehicleSpec* sp = &kVehicleSpecs[0];
    for (const VehicleSpec& v : kVehicleSpecs)
        if (std::string(v.name).find("Class 93 (T") != std::string::npos) sp = &v;
    if (const char* want = std::getenv("EBANER_AUDIO_ENGINE"))
        for (const VehicleSpec& v : kVehicleSpecs)
            if (std::string(v.name).find(want) != std::string::npos) sp = &v;
    const float fire = static_cast<float>(sp->cylinders) * (sp->twoStroke ? 1.0f : 0.5f);
    std::fprintf(stderr, "audio: %s - idle %.0f rpm, %.0f firings/rev = %.0f Hz\n",
                 sp->name, sp->idleRpm, fire, sp->idleRpm / 60.0f * fire);
    struct Seg { float dur, rpm; bool comp; };
    // off -> crank -> idle -> idle+compressor -> idle -> full song -> idle -> stop -> off.
    // Full speed is in here because loudness has to be checked at both ends: an engine
    // tuned to sit right at idle can be into the clamp at governed speed, and the mix has
    // to leave room for the wheels and the brakes on top of whichever is worse.
    const float idle = sp->idleRpm, full = sp->governedRpm;
    const Seg segs[] = {{0.6f, 0.0f, false}, {4.0f, idle, false}, {2.0f, idle, false},
                        {3.0f, idle, true},  {2.0f, idle, false}, {4.0f, full, false},
                        {2.0f, idle, false}, {3.0f, 0.0f, false}, {0.6f, 0.0f, false}};
    std::vector<std::int16_t> pcm;
    // As many engines as the machine has, not always two: a locomotive with one prime
    // mover sounded through two slots is twice as loud as it should be, and the whole
    // point of this dump is to compare loudness between machines.
    const int nEng = std::max(1, sp->engines);
    for (const Seg& s : segs) {
        for (int k = 0; k < nEng; ++k) {
            a.engRpm_[k].store(s.rpm);
            a.engGain_[k].store(1.0f);
            a.engFire_[k].store(fire);
            a.engVol_[k].store(sp->engineVolume);
            a.engRum_[k].store(sp->engineRumble);
            a.engBri_[k].store(sp->engineBright);
        }
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

void Audio::dumpGridTest(const std::string& wavPath) {
    const float fs = 44100.0f;
    Audio a;
    a.sampleRate_ = fs;
    const VehicleSpec* sp = specNamed("Di 4 (Hen");
    if (sp == nullptr) return;
    const float fire = static_cast<float>(sp->cylinders) * (sp->twoStroke ? 1.0f : 0.5f);
    // What a driver does: running at idle, then the controller back through the notches,
    // held at full, then off - and the fans left to coast down, which is the half of it
    // that a switch would get wrong.
    struct Seg { float dur, load; };
    const Seg segs[] = {{3.0f, 0.0f},  {4.0f, 0.2f}, {4.0f, 0.6f}, {8.0f, 1.0f},
                        {10.0f, 0.0f}, {2.0f, 0.0f}};
    std::vector<std::int16_t> pcm;
    for (const Seg& s : segs) {
        a.engRpm_[0].store(sp->idleRpm);
        a.engGain_[0].store(1.0f);
        a.engFire_[0].store(fire);
        a.engVol_[0].store(sp->engineVolume);
        a.engRum_[0].store(sp->engineRumble);
        a.engBri_[0].store(sp->engineBright);
        a.gridLoad_.store(s.load);
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

void Audio::dumpRollingTest(const std::string& wavPath) {
    const float fs = 44100.0f;
    Audio a;
    a.sampleRate_ = fs;
    // A Class 93's running gear: three bogies, six axles, 2.5 m within a bogie and
    // 30 m end to end. Where the wheels are is what a switch beats out, so passing one
    // is scripted here from the real spacing, exactly as the sim would deliver it.
    const float bogie = 0.5f * 30.0f, half = 0.5f * 2.5f;
    const float axles[6] = {bogie + half,  bogie - half,  half,
                            -half,         -bogie + half, -bogie - half};
    RollingSample r;
    r.gain = 1.0f;
    // One control moves at a time, so each is audible on its own rather than as part
    // of a mixture that could hide any of them. `points` is how many turnouts are run
    // over during the segment, spread evenly through it.
    struct Seg { float dur, speed, axleTonnes, work, brake, flange; int points;
                 const char* what; };
    const Seg segs[] = {
        {1.5f, 0.0f, 11.5f, 0.0f, 0.0f, 0.0f, 0, "standing"},
        {6.0f, 6.0f, 11.5f, 0.5f, 0.0f, 0.0f, 1, "moving off, over the points"},
        {6.0f, 16.0f, 11.5f, 0.4f, 0.0f, 0.0f, 1, "60 km/h, over the points"},
        {6.0f, 33.0f, 11.5f, 0.3f, 0.0f, 0.0f, 1, "120 km/h, over the points"},
        {6.0f, 20.0f, 11.5f, 0.2f, 0.0f, 0.0f, 3, "through a station throat"},
        {5.0f, 22.0f, 1.3f, 0.1f, 0.0f, 0.0f, 0, "80 km/h, a light wheelset"},
        {5.0f, 22.0f, 5.0f, 0.1f, 0.0f, 0.0f, 0, "80 km/h, half loaded"},
        {5.0f, 22.0f, 11.5f, 0.1f, 0.0f, 0.0f, 0, "80 km/h, fully loaded"},
        {4.0f, 12.0f, 11.5f, 0.2f, 0.0f, 0.0f, 0, "into a curve..."},
        {6.0f, 12.0f, 11.5f, 0.2f, 0.0f, 0.55f, 0, "...squealing round it"},
        {4.0f, 12.0f, 11.5f, 0.2f, 0.0f, 0.0f, 0, "...and out"},
        {5.0f, 20.0f, 11.5f, 0.0f, 0.8f, 0.0f, 0, "braking (discs: very little to hear)"},
        {5.0f, 6.0f, 11.5f, 0.0f, 0.9f, 0.0f, 0, "nearly stopped"},
        {2.5f, 0.0f, 11.5f, 0.0f, 0.0f, 0.0f, 0, "stood"},
    };
    std::vector<std::int16_t> pcm;
    for (const Seg& s : segs) {
        r.speed = s.speed;
        r.axleLoadN = s.axleTonnes * 1000.0f * kGravity;
        r.work = s.work;
        r.brake = s.brake;
        r.flange = s.flange;
        const int total = static_cast<int>(s.dur * fs);
        // When each axle reaches each turnout: the nose axle first, the rest as far
        // behind it as they are down the train, which at 33 m/s is a bogie's two 76 ms
        // apart and the next bogie a second later.
        std::vector<int> when;
        for (int p = 0; p < s.points && s.speed > 0.1f; ++p) {
            const float t0 = s.dur * (p + 0.5f) / static_cast<float>(s.points);
            for (const float o : axles)
                when.push_back(static_cast<int>((t0 + (axles[0] - o) / s.speed) * fs));
        }
        std::sort(when.begin(), when.end());
        std::size_t next = 0;
        float buf[256];
        for (int done = 0; done < total; done += 256) {
            const int n = std::min(256, total - done);
            while (next < when.size() && when[next] < done + n) { ++r.impacts; ++next; }
            a.setRolling(r);
            a.render(buf, n);
            for (int i = 0; i < n; ++i)
                pcm.push_back(static_cast<std::int16_t>(
                    std::clamp(buf[i], -1.0f, 1.0f) * 32767.0f));
        }
    }
    writeWav(wavPath, pcm, static_cast<int>(fs));
    std::fprintf(stderr, "audio: wrote %s (%zu samples)\n", wavPath.c_str(), pcm.size());
}

void Audio::dumpImpactTest(const std::string& wavPath) {
    const float fs = 44100.0f;
    Audio a;
    a.sampleRate_ = fs;
    // The closing speeds the bands are drawn at, and two either side of them, all heard
    // from alongside. The severity curve is the caller's, repeated here so this dump is
    // of what the sim will actually ask for and not of a tidier set of numbers.
    const float closing[] = {0.3f, 0.8f, 1.4f, 2.5f, 4.0f, 6.0f, 9.0f};
    std::vector<std::int16_t> pcm;
    auto run = [&](float seconds) {
        const int total = static_cast<int>(seconds * fs);
        float buf[256];
        for (int done = 0; done < total; done += 256) {
            const int n = std::min(256, total - done);
            a.render(buf, n);
            for (int i = 0; i < n; ++i)
                pcm.push_back(static_cast<std::int16_t>(
                    std::clamp(buf[i], -1.0f, 1.0f) * 32767.0f));
        }
    };
    run(0.3f);
    for (const float v : closing) {
        a.impact(std::sqrt(std::clamp(v / 8.0f, 0.0f, 1.0f)), 1.0f);
        std::fprintf(stderr, "  %.1f m/s (%.0f km/h)\n", v, v * 3.6f);
        run(1.7f); // long enough that a heavy one's tail is not cut off by the next
    }
    // And one at a distance, which is what most of them will be.
    a.impact(1.0f, 0.15f);
    std::fprintf(stderr, "  a wreck, a long way off\n");
    run(1.5f);
    writeWav(wavPath, pcm, static_cast<int>(fs));
    std::fprintf(stderr, "audio: wrote %s (%zu samples)\n", wavPath.c_str(), pcm.size());
}

void Audio::dumpCrossingTest(const std::string& wavPath) {
    const float fs = 44100.0f;
    Audio a;
    a.sampleRate_ = fs;
    // A train approaches, the crossing activates, the bell runs its kBellS and stops
    // while the lights keep flashing, then the train passes and the crossing opens. The
    // level also swings with distance, which is what a bell heard from a moving cab does.
    struct Seg { float dur, gain; const char* what; };
    const Seg segs[] = {
        {1.0f, 0.00f, "idle"},
        {8.0f, 0.45f, "activated, heard from a distance"},
        {12.0f, 1.00f, "closer"},
        {10.0f, 0.70f, "still ringing, drawing away"},
        {6.0f, 0.00f, "bell done - the lights flash on in silence"},
    };
    std::vector<std::int16_t> pcm;
    for (const Seg& s : segs) {
        a.bellGain_.store(s.gain);
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
