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

// What the rolling noise actually does.
//
// A synth is the one part of this program that cannot be checked by looking at it, and
// "it sounds about right" is not a check at all - every claim the feature makes is a
// claim about a number. That a heavier vehicle sounds heavier means its energy moved
// down the spectrum; that speed drives the level means the level follows a law that was
// measured on real track; that the brakes are quiet means they add nothing above a
// kilohertz. So the synth is rendered offline and measured.
//
// `Audio.cpp` builds into the viewer rather than the engine library, but without a
// backend defined it opens no device and is pure DSP, so this links it directly: what is
// measured here is the shipped code and not a copy of it.

#include "Audio.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

constexpr float kFs = 44100.0f;
int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  %-58s %s\n", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

// A Class 93's running gear: three bogies, 2.5 m within a bogie, 30 m end to end.
std::vector<float> classNinetyThreeAxles() {
    const float b = 15.0f, h = 1.25f;
    return {-b - h, -b + h, -h, h, b - h, b + h};
}

// Render `sec` seconds at fixed controls. The settle pass runs the same controls first
// and is thrown away: every control is smoothed into the synth over a few hundred
// milliseconds, so measuring from the first sample would measure the glide.
std::vector<float> renderSteady(const RollingSample& r, float sec, float settle = 2.5f) {
    Audio a;
    a.setRolling(r);
    std::vector<float> buf(512);
    for (int done = 0; done < static_cast<int>(settle * kFs); done += 512)
        a.render(buf.data(), 512);
    std::vector<float> out;
    out.reserve(static_cast<std::size_t>(sec * kFs));
    for (int done = 0; done < static_cast<int>(sec * kFs); done += 512) {
        a.render(buf.data(), 512);
        out.insert(out.end(), buf.begin(), buf.end());
    }
    return out;
}

float rms(const std::vector<float>& x) {
    double acc = 0.0;
    for (float v : x) acc += double(v) * v;
    return x.empty() ? 0.0f : static_cast<float>(std::sqrt(acc / x.size()));
}

float peak(const std::vector<float>& x) {
    float m = 0.0f;
    for (float v : x) m = std::max(m, std::fabs(v));
    return m;
}

// Energy in [lo,hi) Hz, by Goertzel over a spread of bins. Absolute scale is arbitrary
// and only ever compared against another band of the same length.
float band(const std::vector<float>& x, float lo, float hi) {
    const int N = std::min<int>(static_cast<int>(x.size()), 1 << 15);
    if (N < 64) return 0.0f;
    const int k0 = static_cast<int>(lo * N / kFs), k1 = static_cast<int>(hi * N / kFs);
    const int step = std::max(1, (k1 - k0) / 40);
    double total = 0.0;
    for (int k = k0; k < k1; k += step) {
        const double w = 2.0 * 3.14159265358979 * k / N;
        const double c = std::cos(w);
        double s1 = 0.0, s2 = 0.0;
        for (int i = 0; i < N; ++i) {
            const double s0 = x[i] + 2.0 * c * s1 - s2;
            s2 = s1;
            s1 = s0;
        }
        total += s1 * s1 + s2 * s2 - 2.0 * c * s1 * s2;
    }
    return static_cast<float>(std::sqrt(total / N));
}

// Where the energy sits: the amplitude-weighted mean frequency over a coarse sweep. A
// direct reading of "brighter or darker", which is the whole claim weight makes.
float centroid(const std::vector<float>& x) {
    double num = 0.0, den = 0.0;
    for (float f = 60.0f; f < 6000.0f; f *= 1.15f) {
        const float e = band(x, f, f * 1.15f);
        num += double(e) * f;
        den += e;
    }
    return den > 0.0 ? static_cast<float>(num / den) : 0.0f;
}

// Impacts per second: transients, found as the fast envelope pulling away from the slow
// one, so the roar underneath does not count as an impact however loud it is.
float impactRate(const std::vector<float>& x) {
    std::vector<float> tr(x.size());
    float fast = 0.0f, slow = 0.0f;
    for (std::size_t i = 0; i < x.size(); ++i) {
        const float a = std::fabs(x[i]);
        fast += (a - fast) * 0.02f;
        slow += (a - slow) * 0.0006f;
        tr[i] = fast - slow;
    }
    // Threshold against how big an impact is, not against a percentile of everything:
    // the impacts in a window are all much the same size and all far above the roar, so
    // a fraction of the largest separates them cleanly. A percentile rides up with the
    // roar instead, and at line speed lets the noise itself count as impacts.
    float biggest = 0.0f;
    for (float v : tr) biggest = std::max(biggest, v);
    const float thr = 0.45f * biggest;
    int peaks = 0, hold = 0;
    for (std::size_t i = 1; i + 1 < tr.size(); ++i) {
        if (hold > 0) { --hold; continue; }
        if (tr[i] > thr && tr[i] >= tr[i - 1] && tr[i] > tr[i + 1]) {
            ++peaks;
            hold = static_cast<int>(0.030f * kFs); // one impact per 30 ms at most
        }
    }
    return static_cast<float>(peaks) / (static_cast<float>(x.size()) / kFs);
}

RollingSample running(float speed, float axleTonnes) {
    RollingSample r;
    r.speed = speed;
    r.axleLoadN = axleTonnes * 1000.0f * 9.81f;
    r.gain = 1.0f;
    r.axleOffsets = classNinetyThreeAxles();
    return r;
}

} // namespace

int main() {
    std::puts("Rolling noise:");

    // 1. A train that is not moving makes no rolling noise at all. Not "almost none":
    //    the level law is gated to a hard zero, because a synth that idles just above
    //    silence is audible in a quiet cab and there is nothing there to hear.
    {
        const std::vector<float> x = renderSteady(running(0.0f, 11.5f), 1.0f);
        check(peak(x) == 0.0f, "a standing train is silent, exactly");
        RollingSample crawl = running(0.3f, 11.5f); // under the walking-pace floor
        check(peak(renderSteady(crawl, 1.0f)) == 0.0f, "and so is one below walking pace");
    }

    // 2. Speed sets the level by the law measured on real track: 30*log10(v), which is
    //    an amplitude proportional to v^1.5. Checked as a ratio over a 4x speed range,
    //    where the law predicts 8x.
    {
        std::puts("\n  Speed:");
        // The roar on its own: no axles, so no joint impacts riding on top of it. The
        // law is a claim about the roar, and mixed with the impacts - which climb with
        // speed too, by a different exponent - it cannot be read cleanly.
        auto roarOnly = [](float v) {
            RollingSample r = running(v, 11.5f);
            r.axleOffsets.clear();
            return rms(renderSteady(r, 4.0f));
        };
        const float a = roarOnly(10.0f), b = roarOnly(20.0f), c = roarOnly(40.0f);
        std::printf("    10 m/s %.5f   20 m/s %.5f   40 m/s %.5f\n", a, b, c);
        check(a < b && b < c, "louder at every step up in speed");
        const float ratio = c / std::max(a, 1e-9f);
        std::printf("    40:10 level ratio %.2f (v^1.5 predicts 8.00)\n", double(ratio));
        check(ratio > 7.2f && ratio < 8.8f, "and the 4x speed range spans 8x, as it should");
        // With the joints back in, the climb is *shallower*. Impact noise grows with
        // speed more slowly than rolling noise does, so the joints are most of what is
        // heard at low speed and the roar has swallowed them by line speed - which is
        // how a real train goes from a clatter to a roar rather than getting louder.
        const float withJoints = rms(renderSteady(running(40.0f, 11.5f), 4.0f)) /
                                 std::max(rms(renderSteady(running(10.0f, 11.5f), 4.0f)),
                                          1e-9f);
        std::printf("    with the joints in, 40:10 is %.2f\n", double(withJoints));
        check(withJoints < ratio, "and the joints, growing slower, flatten it a little");
        // The band also climbs with speed: the roughness that matters passes faster.
        const float ca = centroid(renderSteady(running(10.0f, 11.5f), 2.0f));
        const float cc = centroid(renderSteady(running(40.0f, 11.5f), 2.0f));
        std::printf("    centroid %.0f Hz at 10 m/s -> %.0f Hz at 40 m/s\n", ca, cc);
        check(cc > ca, "and the sound brightens with speed");
    }

    // 3. Weight. The claim is that a heavier axle sounds heavier - which means not just
    //    louder but *lower*, because a bigger contact patch cannot be excited by the
    //    short-wavelength roughness a small one can. If weight only changed the volume
    //    it would be indistinguishable from moving the camera closer.
    {
        std::puts("\n  Weight (at one speed):");
        const std::vector<float> light = renderSteady(running(22.0f, 1.3f), 3.0f);
        const std::vector<float> heavy = renderSteady(running(22.0f, 11.5f), 3.0f);
        const float cl = centroid(light), ch = centroid(heavy);
        std::printf("    1.3 t/axle: rms %.5f centroid %.0f Hz\n", rms(light), cl);
        std::printf("   11.5 t/axle: rms %.5f centroid %.0f Hz\n", rms(heavy), ch);
        check(ch < cl * 0.9f, "a heavier axle load is audibly darker");
        check(rms(heavy) > rms(light), "and not quieter");
        const float lowL = band(light, 30.0f, 250.0f), lowH = band(heavy, 30.0f, 250.0f);
        std::printf("    below 250 Hz: %.3f -> %.3f\n", lowL, lowH);
        check(lowH > lowL * 1.5f, "with the weight of it in the low end");
    }

    // 4. Rail joints. The rate is not a tuning choice - it is the speed divided by the
    //    rail length, times the axles, and if it comes out wrong the train is running
    //    on rails of the wrong length.
    {
        std::puts("\n  Rail joints:");
        bool allOk = true;
        for (const float v : {8.0f, 12.0f, 16.0f}) {
            const float want = v / 25.0f * 6.0f; // 6 axles over 25 m rails
            const float got = impactRate(renderSteady(running(v, 11.5f), 8.0f));
            std::printf("    %4.0f m/s: %5.2f impacts/s (expected %5.2f)\n", v, got, want);
            allOk = allOk && std::fabs(got - want) < 0.6f;
        }
        check(allOk, "impacts land at speed / 25 m x axles, at every speed");
        // Two axles instead of six beat a third as often over the same ground.
        RollingSample pair = running(12.0f, 11.5f);
        pair.axleOffsets = {-0.9f, 0.9f};
        const float got = impactRate(renderSteady(pair, 8.0f));
        std::printf("    a lone bogie at 12 m/s: %.2f/s (expected %.2f)\n", got,
                    12.0f / 25.0f * 2.0f);
        check(std::fabs(got - 12.0f / 25.0f * 2.0f) < 0.4f,
              "and the pattern is the vehicle's own axles, not a fixed beat");
    }

    // 5. Curve squeal is gated. A squeal that leaked onto straight track would be far
    //    worse than none, so what is checked is the silence as much as the sound.
    {
        std::puts("\n  Curve squeal:");
        const std::vector<float> straight = renderSteady(running(12.0f, 11.5f), 3.0f);
        RollingSample curve = running(12.0f, 11.5f);
        curve.flange = 0.55f;
        const std::vector<float> squealing = renderSteady(curve, 3.0f);
        const float qs = band(straight, 1900.0f, 2300.0f);
        const float qc = band(squealing, 1900.0f, 2300.0f);
        std::printf("    1.9-2.3 kHz: straight %.3f -> curve %.3f (x%.1f)\n", qs, qc,
                    double(qc / std::max(qs, 1e-6f)));
        check(qc > qs * 8.0f, "the curve puts a howl in the wheel-mode band");
        // ...and only there: the rest of the sound must not lift with it.
        const float ls = band(straight, 250.0f, 1000.0f);
        const float lc = band(squealing, 250.0f, 1000.0f);
        check(lc < ls * 1.6f, "and does not simply turn everything up");
        // Standing still in a curve does not squeal: it is sliding that howls.
        RollingSample stopped = curve;
        stopped.speed = 0.0f;
        check(peak(renderSteady(stopped, 1.0f)) == 0.0f,
              "a train stopped in a curve is silent");
    }

    // 6. The brakes. The Class 93 brakes on discs and is very quiet - a little low
    //    frequency noise and nothing else - so this is stated as a measurement: energy
    //    below 250 Hz, none above a kilohertz, and well under the roar.
    {
        std::puts("\n  Friction brake:");
        const std::vector<float> off = renderSteady(running(8.0f, 11.5f), 3.0f);
        RollingSample braked = running(8.0f, 11.5f);
        braked.brake = 0.9f;
        const std::vector<float> on = renderSteady(braked, 3.0f);
        const float lowOff = band(off, 30.0f, 250.0f), lowOn = band(on, 30.0f, 250.0f);
        const float hiOff = band(off, 1000.0f, 4000.0f), hiOn = band(on, 1000.0f, 4000.0f);
        std::printf("    below 250 Hz: %.3f -> %.3f\n", lowOff, lowOn);
        std::printf("    above  1 kHz: %.3f -> %.3f\n", hiOff, hiOn);
        check(lowOn > lowOff * 1.3f, "braking is heard, in the low end");
        check(hiOn < hiOff * 1.15f, "and nowhere else - no squeal, no rub, no hiss");
        // Quiet: full brake at a crawl stays under the plain roar at line speed.
        const float loud = rms(renderSteady(running(33.0f, 11.5f), 2.0f));
        std::printf("    rms: braking hard at 8 m/s %.5f vs rolling at 33 m/s %.5f\n",
                    rms(on), loud);
        check(rms(on) < loud * 0.5f, "and quiet, as a disc brake is");
    }

    // 7. Off the rails there is no rail noise. The vehicle is sliding on ballast, and
    //    whatever that sounds like it is not this.
    {
        std::puts("\n  Derailed:");
        RollingSample off = running(20.0f, 11.5f);
        off.flange = 0.5f;
        off.railborne = false;
        check(peak(renderSteady(off, 1.0f)) == 0.0f,
              "a derailed vehicle makes no wheel-on-rail sound");
    }

    // 8. Distance. The gain is the caller's, but it has to reach zero, or a train on the
    //    far side of the fjord would be heard as clearly as one alongside.
    {
        std::puts("\n  Distance:");
        RollingSample near = running(25.0f, 11.5f), far = near;
        far.gain = 0.0f;
        check(peak(renderSteady(far, 1.0f)) == 0.0f, "out of earshot is silent");
        RollingSample half = near;
        half.gain = 0.5f;
        const float n = rms(renderSteady(near, 2.0f)), h = rms(renderSteady(half, 2.0f));
        std::printf("    gain 1.0 %.5f, gain 0.5 %.5f\n", n, h);
        check(h < n * 0.75f, "and half the gain is audibly further off");
    }

    std::printf("\n%s\n", failures ? "FAILED" : "PASSED");
    return failures ? 1 : 0;
}
