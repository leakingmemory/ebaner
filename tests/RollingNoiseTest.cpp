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
#include "Consist.h"
#include "TrackPath.h"
#include "Vehicle.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace {

constexpr float kFs = 44100.0f;
int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  %-58s %s\n", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
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

// The same, but delivering `impactTimes` (seconds into the measured stretch) as wheels
// crossing a frog, the way the sim delivers them frame by frame.
std::vector<float> renderWithImpacts(RollingSample r, float sec,
                                     const std::vector<float>& impactTimes) {
    Audio a;
    a.setRolling(r);
    std::vector<float> buf(512);
    for (int done = 0; done < static_cast<int>(2.5f * kFs); done += 512)
        a.render(buf.data(), 512);
    std::vector<float> out;
    std::size_t next = 0;
    for (int done = 0; done < static_cast<int>(sec * kFs); done += 512) {
        while (next < impactTimes.size() &&
               impactTimes[next] * kFs < static_cast<float>(done + 512)) {
            ++r.impacts;
            ++next;
        }
        a.setRolling(r);
        a.render(buf.data(), 512);
        out.insert(out.end(), buf.begin(), buf.end());
    }
    return out;
}

// A train of `units` sets standing on a straight line, engines running. No dataset:
// TrackPath takes points directly, and a kilometre of straight is all the engines need.
struct Bench {
    std::vector<TrackPath> paths;
    std::optional<Consist> train;

    explicit Bench(int units) {
        std::vector<glm::vec3> pts;
        for (int i = 0; i <= 40; ++i) pts.push_back({float(i) * 25.0f, 0.0f, 0.0f});
        paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 100));
        VehicleSpec sp{};
        for (const VehicleSpec& v : kVehicleSpecs)
            if (v.body == BodyClass93 && v.units == 1) sp = v;
        sp.units = units;
        train.emplace(&paths, &paths[0], sp, 500.0f);
        train->toggleEngines();
    }
};

// Render the engines of an `units`-set train with the given per-engine distance gains,
// after letting them crank up to idle and the smoothing settle.
std::vector<float> renderEngines(int units, const std::vector<float>& gains,
                                 float sec = 1.5f) {
    Bench b(units);
    Audio a;
    std::vector<float> buf(512);
    std::vector<float> g(Audio::kMaxEngines, 0.0f);
    for (std::size_t i = 0; i < gains.size() && i < g.size(); ++i) g[i] = gains[i];
    // Ten seconds of sim to crank the diesels to idle, and a settle pass through the
    // synth's own smoothing, both thrown away.
    for (int i = 0; i < 600; ++i) b.train->update(1.0f / 60.0f);
    for (int pass = 0; pass < 2; ++pass) {
        a.update(*b.train, 1.0f / 60.0f, 0.0f, g.data(), static_cast<int>(g.size()), 0.0f);
        for (int done = 0; done < static_cast<int>(2.5f * kFs); done += 512)
            a.render(buf.data(), 512);
    }
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

RollingSample running(float speed, float axleTonnes) {
    RollingSample r;
    r.speed = speed;
    r.axleLoadN = axleTonnes * 1000.0f * 9.81f;
    r.gain = 1.0f;
    return r;
}

// When each of a Class 93's six axles reaches one turnout, in seconds from the first:
// three bogies, 2.5 m within a bogie, 30 m end to end.
std::vector<float> passOverPoints(float speed, float from) {
    const float b = 15.0f, h = 1.25f;
    const float nose = b + h;
    std::vector<float> t;
    for (const float o : {b + h, b - h, h, -h, -b + h, -b - h})
        t.push_back(from + (nose - o) / speed);
    return t;
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
        // On plain line, which is where the law is a claim about the roar and nothing
        // else is sounding.
        auto roar = [](float v) { return rms(renderSteady(running(v, 11.5f), 4.0f)); };
        const float a = roar(10.0f), b = roar(20.0f), c = roar(40.0f);
        std::printf("    10 m/s %.5f   20 m/s %.5f   40 m/s %.5f\n", a, b, c);
        check(a < b && b < c, "louder at every step up in speed");
        const float ratio = c / std::max(a, 1e-9f);
        std::printf("    40:10 level ratio %.2f (v^1.5 predicts 8.00)\n", double(ratio));
        check(ratio > 7.2f && ratio < 8.8f, "and the 4x speed range spans 8x, as it should");
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

    // 4. Wheels over the points. Continuous welded rail has nothing to knock against
    //    between the switches, so a knock is an event at a place and not a beat: the
    //    synth sounds one for each wheel the sim says has crossed a frog, and nothing
    //    at all in between.
    //
    //    Counted by energy rather than by finding peaks in the waveform. The knocks sit
    //    inside a roar that is louder than they are at line speed, and a peak-finder
    //    good enough to separate them is a measuring instrument that has to be tuned
    //    until it agrees - at which point it is no longer evidence. Energy is additive
    //    for uncorrelated sounds, so twice the knocks is twice the added energy, and
    //    nothing needs tuning.
    {
        std::puts("\n  Wheels over the points:");
        auto energy = [](const std::vector<float>& x) {
            double e = 0.0;
            for (float v : x) e += double(v) * v;
            return e;
        };
        const float v = 20.0f;
        std::vector<float> one = passOverPoints(v, 1.0f), three;
        for (int p = 0; p < 3; ++p)
            for (const float x : passOverPoints(v, 1.0f + 2.0f * p)) three.push_back(x);

        const std::vector<float> plainRun = renderSteady(running(v, 11.5f), 8.0f);
        const std::vector<float> oneRun = renderWithImpacts(running(v, 11.5f), 8.0f, one);
        const std::vector<float> threeRun = renderWithImpacts(running(v, 11.5f), 8.0f, three);
        const double e0 = energy(plainRun), e1 = energy(oneRun), e3 = energy(threeRun);
        std::printf("    energy: plain %.4f, one switch %.4f, three %.4f\n", e0, e1, e3);
        check(e1 > e0 * 1.02, "passing a switch adds sound");
        const double ratio = (e3 - e0) / std::max(e1 - e0, 1e-12);
        std::printf("    three switches add %.2fx what one does (18 knocks vs 6)\n", ratio);
        check(ratio > 2.4 && ratio < 3.6, "and three add three times as much as one");

        // Impulsive, not just louder: knocks push the peak far above the roar's own
        // level, which is what a knock is and a rise in volume is not.
        const float crestPlain = peak(plainRun) / std::max(rms(plainRun), 1e-9f);
        const float crestPoints = peak(threeRun) / std::max(rms(threeRun), 1e-9f);
        std::printf("    crest factor: plain %.2f, over points %.2f\n", crestPlain,
                    crestPoints);
        check(crestPoints > crestPlain * 1.3f, "and they are knocks, not a swell");

        // Weight is behind the knock: the same pass, loaded, hits harder.
        const float lightPk = peak(renderWithImpacts(running(v, 1.3f), 4.0f, one));
        const float heavyPk = peak(renderWithImpacts(running(v, 11.5f), 4.0f, one));
        std::printf("    peak over a switch: 1.3 t %.3f, 11.5 t %.3f\n", lightPk, heavyPk);
        check(heavyPk > lightPk * 1.3f, "a loaded axle hits the frog harder");
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

    // --- Every engine of a long train is its own voice -----------------------------
    //
    // Two diesels to a Class 93 set, so a three-set train has six, and the synth holds
    // six voices. What has to be true is not just that the far ones make *a* noise: a
    // set at the back has to sound like the same machine as the set at the front, and
    // the voices have to stay separate rather than piling into one loud engine.
    {
        std::puts("\n  Six engines, three sets:");
        const std::vector<float> rear{0, 0, 0, 0, 1, 1};
        const std::vector<float> none(6, 0.0f);
        const float r = rms(renderEngines(3, rear));
        const float q = rms(renderEngines(3, none));
        std::printf("  rear set alone rms %.5f  (all gains zero %.5f)\n", r, q);
        // The one that fails outright before the ceiling was widened: with four slots
        // the third set's engines are zeroed and this is exact silence.
        check(r > 10.0f * std::max(q, 1e-7f), "the third set is heard at all");
        const float fire = band(renderEngines(3, rear), 25.0f, 50.0f);
        const float quiet = band(renderEngines(3, none), 25.0f, 50.0f);
        check(fire > 10.0f * std::max(quiet, 1e-12f),
              "and it is a diesel firing, not a hiss");

        // Every engine is the same machine. Measured one at a time rather than a set at
        // a time: the two engines of a set are detuned against each other and beat at
        // about a quarter of a hertz, so a window shorter than several seconds catches
        // whatever part of the beat it lands on and a pair's loudness swings by half.
        // One voice does not beat with anything, so this compares what it claims to.
        float lo = 1e30f, hi = 0.0f;
        for (int k = 0; k < 6; ++k) {
            std::vector<float> one(6, 0.0f);
            one[k] = 1.0f;
            const float v = rms(renderEngines(3, one));
            std::printf("  engine %d alone rms %.5f\n", k, v);
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
        // They are not identical and should not be: each is detuned a little further
        // than the last so that several together beat rather than doubling into one.
        check(hi < 1.15f * lo, "every engine of the train sounds like the others");

        // Six at once must not run the mix into the clipper.
        const std::vector<float> all(6, 1.0f);
        const std::vector<float> x = renderEngines(3, all);
        int hard = 0;
        for (float v : x) if (std::fabs(v) >= 0.999f) ++hard;
        std::printf("  all six at full gain: peak %.4f, %d clipped sample(s)\n",
                    peak(x), hard);
        check(hard == 0, "six engines at once do not clip");
    }

    // A slot with no engine behind it is inert, not merely quiet: the render loop bails
    // on it before it reaches the shared noise generator, so widening the ceiling cannot
    // move a sample of a train that does not fill it. Asserted as exact equality, in one
    // process, so it cannot drift with the compiler or the maths library.
    {
        std::puts("\n  An empty engine slot changes nothing:");
        const std::vector<float> two = renderEngines(2, {1, 1, 1, 1});
        const std::vector<float> padded = renderEngines(2, {1, 1, 1, 1, 0, 0});
        bool same = two.size() == padded.size();
        for (std::size_t i = 0; same && i < two.size(); ++i)
            same = two[i] == padded[i];
        check(same, "two sets render identically with the spare slots zeroed");
    }

    std::printf("\n%s\n", failures ? "FAILED" : "PASSED");
    return failures ? 1 : 0;
}
