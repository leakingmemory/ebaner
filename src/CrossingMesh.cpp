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

#include "CrossingMesh.h"

#include "LampGeometry.h"
#include "TrackPath.h"

#include <algorithm>
#include <cmath>

namespace {

const glm::vec3 kBody{0.10f, 0.10f, 0.11f}; // near-black cast housing
const glm::vec3 kRedOn{1.0f, 0.14f, 0.10f};
const glm::vec3 kRedOff{0.24f, 0.10f, 0.10f};
const glm::vec3 kWhiteOn{1.0f, 0.97f, 0.88f};
const glm::vec3 kWhiteOff{0.26f, 0.25f, 0.23f};
// The distant repeats the crossing's own indication in violet, so a driver can tell at a
// glance whether they are reading the crossing or the warning for it - the aspects being
// otherwise identical is the whole point, and identical aspects in the same colour a mile
// apart would be worse than no repeat.
//
// Weighted toward red and kept low in green. Green is what washes a violet out: with much
// of it in the mix the lens reads as a pale lavender, and at the distance one of these is
// read from - most of a braking distance - it blends into a bright sky and looks white.
const glm::vec3 kVioletOn{0.72f, 0.12f, 0.92f};
const glm::vec3 kVioletOff{0.20f, 0.07f, 0.22f};

constexpr float kMastH = 3.2f;      // shorter than a main signal: this is not one
constexpr float kRoadOffsetM = 6.0f; // how far the road heads stand off the track
constexpr float kRoadRightM = 3.0f;  // and to the right of the road traffic they face
constexpr float kLensR = 0.15f;
constexpr float kLensSp = 0.26f;    // half the vertical lamp spacing

// The boom of a half-barrier. It hangs on the same post as the road head, as most are
// built, and reaches from there in to about the road's centre line - one lane, the one
// the traffic on that side arrives on.
const glm::vec3 kBoomRed{0.85f, 0.10f, 0.09f};
const glm::vec3 kBoomWhite{0.92f, 0.92f, 0.88f};
constexpr float kBoomPivotH = 1.05f; // above the road, where the boom is hinged
constexpr float kBoomLenM = 3.6f;    // post is kRoadRightM from the centre; this clears it
constexpr float kBoomHalfW = 0.07f;  // half its width and thickness
constexpr int kBoomStripes = 6;      // alternating red and white along its length

// One head: a mast, a housing, and a red over a white. `fwd` is the direction the head
// looks, so a driver coming the other way sees it face-on.
void head(std::vector<TrackVertex>& v, std::vector<std::uint32_t>& idx,
          const glm::vec3& base, const glm::vec3& fwd, bool red, bool white, float period,
          float phase, const glm::vec3& onCol = kRedOn,
          const glm::vec3& offCol = kRedOff) {
    const glm::vec3 UP(0.0f, 0.0f, 1.0f);
    const glm::vec3 F = glm::normalize(glm::vec3(fwd.x, fwd.y, 0.0f));
    const glm::vec3 R(F.y, -F.x, 0.0f);

    lampgeom::box(v, idx, base + UP * (kMastH * 0.5f), R, F, UP, 0.06f, 0.06f,
                  kMastH * 0.5f, kBody);
    const float hw = 0.28f, hd = 0.14f, hh = 0.50f;
    const glm::vec3 C = base + UP * (kMastH + hh);
    lampgeom::box(v, idx, C, R, F, UP, hw, hd, hh, kBody);
    // Backing plate, so the head reads against a bright sky as a signal does.
    lampgeom::box(v, idx, C + F * (hd + 0.01f), R, F, UP, hw * 1.3f, 0.02f, hh * 1.1f,
                  kBody);

    const glm::vec3 face = C + F * (hd + 0.04f);
    if (red)
        lampgeom::lamp(v, idx, face + UP * kLensSp, R, UP, kLensR, onCol, period, phase);
    else
        lampgeom::disc(v, idx, face + UP * kLensSp, F, R, UP, kLensR, offCol);
    if (white)
        lampgeom::lamp(v, idx, face - UP * kLensSp, R, UP, kLensR, kWhiteOn, period, phase);
    else
        lampgeom::disc(v, idx, face - UP * kLensSp, F, R, UP, kLensR, kWhiteOff);
}

// One boom, hinged at `pivot` and swung by `pos`: 0 stands it upright, 1 lays it across
// the lane. `in` points from the post toward the road's centre line - the way the boom
// falls. `lit` flashes the lamp it carries on the crossing's own period and phase.
void boom(std::vector<TrackVertex>& v, std::vector<std::uint32_t>& idx,
          const glm::vec3& pivot, const glm::vec3& in, float pos, bool lit, float period,
          float phase) {
    const glm::vec3 UP(0.0f, 0.0f, 1.0f);
    const float a = pos * 1.5707963f; // upright -> horizontal
    const float ca = std::cos(a), sa = std::sin(a);
    // Along the boom, and the face that is its top. Upright, that top faces away from the
    // road; laid down it faces the sky, which is where the lamp has to end up. The two
    // stay perpendicular at every angle, which is the whole reason for writing them as a
    // pair rather than rotating a fixed normal and hoping.
    const glm::vec3 L = UP * ca + in * sa;
    const glm::vec3 N = UP * sa - in * ca;
    const glm::vec3 S = glm::cross(L, N); // across the boom

    // The hinge housing, so the boom does not sprout out of bare post.
    lampgeom::box(v, idx, pivot, S, in, UP, 0.12f, 0.12f, 0.16f, kBody);

    // Striped along its length. Each stripe is its own box: one long box could not be
    // painted, and the stripes are what make the boom read as a barrier at a distance.
    const float seg = kBoomLenM / kBoomStripes;
    for (int i = 0; i < kBoomStripes; ++i) {
        const glm::vec3 c = pivot + L * (seg * (static_cast<float>(i) + 0.5f));
        lampgeom::box(v, idx, c, S, L, N, kBoomHalfW, seg * 0.5f, kBoomHalfW,
                      (i % 2) ? kBoomWhite : kBoomRed);
    }

    // The lamp, halfway along and sitting on the boom's top face, so it swings with it.
    const glm::vec3 lc = pivot + L * (kBoomLenM * 0.5f) + N * (kBoomHalfW + 0.05f);
    if (lit)
        lampgeom::lamp(v, idx, lc, S, L, kLensR * 0.8f, kRedOn, period, phase);
    else
        lampgeom::disc(v, idx, lc, N, S, L, kLensR * 0.8f, kRedOff);
}

} // namespace

float crossingPhase(int id) {
    // The same shape of deterministic hash the tunnel wobble uses: stable across runs,
    // and spread enough that neighbouring crossings do not land together.
    std::uint32_t h = static_cast<std::uint32_t>(id) * 0x9E3779B9u;
    h ^= h >> 15;
    h *= 0x2545F491u;
    h ^= h >> 13;
    // Over the slow period, which is the longer of the two - a phase inside it is inside
    // the fast one as well, since the fast period divides it.
    return static_cast<float>(h & 0xFFFFu) / 65535.0f * kCrossingSlowS;
}

void CrossingMesh::build(const std::vector<LevelCrossing>& xs,
                         const std::vector<CrossingSite>& sites,
                         const std::vector<CrossingState>& states,
                         const std::vector<TrackPath>& paths, const glm::dvec3& origin,
                         const DistantFor& distantFor) {
    vertices_.clear();
    indices_.clear();
    (void)origin; // paths are already scene-relative

    for (std::size_t i = 0; i < xs.size(); ++i) {
        if (i >= sites.size() || !sites[i].resolved()) continue;
        const CrossingSite& site = sites[i];
        const CrossingState empty;
        const CrossingState& st = i < states.size() ? states[i] : empty;
        // One pulse for the whole crossing: a track sitting idle beside one that is arming
        // still flashes fast, because a driver reading either head is reading the same
        // crossing. Only the lamps are the track's own.
        const bool fast = st.shut();
        const float period = fast ? kCrossingFastS : kCrossingSlowS;
        const float phase = crossingPhase(xs[i].id);

        // Where the crossing meets each of its tracks, and the frame of the first one -
        // the road runs across all of them, so one frame serves for the road heads.
        int lead = -1;
        for (std::size_t t = 0; t < site.tracks.size() && lead < 0; ++t)
            if (site.tracks[t].path >= 0 &&
                site.tracks[t].path < static_cast<int>(paths.size()))
                lead = static_cast<int>(t);
        if (lead < 0) continue;
        const TrackPath& lp = paths[site.tracks[lead].path];
        const TrackPose at = lp.poseAt(site.tracks[lead].s);
        // Across the track, level: `right` is banked by the cant and would tilt the masts.
        const glm::vec3 tan = glm::normalize(glm::vec3(at.tangent.x, at.tangent.y, 0.0f));
        const glm::vec3 across(tan.y, -tan.x, 0.0f);

        // Each track its own pair of heads, showing that track's own sequence.
        float farNeg = 0.0f, farPos = 0.0f; // how far the tracks reach either way across
        for (std::size_t t = 0; t < site.tracks.size(); ++t) {
            const CrossingSite::On& on = site.tracks[t];
            if (on.path < 0 || on.path >= static_cast<int>(paths.size())) continue;
            const TrackPath& p = paths[on.path];
            const CrossingLights l = crossingLights(st.phase(t), fast);

            // How far this track sits across the road from the one the road heads are
            // measured off, so those heads can be put beyond every rail rather than
            // between two of them.
            const float off = glm::dot(p.poseAt(on.s).pos - at.pos, across);
            farNeg = std::min(farNeg, off);
            farPos = std::max(farPos, off);

            // The two heads the train reads. The one before the crossing on the -s side
            // serves trains running in +s, so it has to look back down the line at them -
            // a head that looked the way the train is going would only ever show its back.
            for (const float side : {-1.0f, 1.0f}) {
                const float s = on.s + side * static_cast<float>(kSignalOffsetM);
                const TrackPose q = p.poseAt(std::clamp(s, 0.0f, p.length()));
                const glm::vec3 qt =
                    glm::normalize(glm::vec3(q.tangent.x, q.tangent.y, 0.0f));
                const glm::vec3 qa(qt.y, -qt.x, 0.0f); // right of +s
                // Standing to the right of the direction of travel it governs, as a signal
                // does: that is -qa for a train running in -s.
                const glm::vec3 base = q.pos + qa * (side > 0.0f ? -3.2f : 3.2f);
                head(vertices_, indices_, base, qt * side, l.trainRed, l.trainWhite, period,
                     phase);
            }

            // The repeats: the same indication again, far enough back to stop from, in
            // violet so it cannot be mistaken for the crossing itself. Skipped rather than
            // clamped where the path is too short to hold one - a repeat standing somewhere
            // other than its braking distance is worse than none, because a driver reads
            // the distance off it as much as the aspect.
            //
            // What it repeats is not always its own track. Out on the single track beyond a
            // station the points decide which road a train is taken to, and the repeat
            // stands for that one; where they cannot say, it warns.
            for (int sideIdx = 0; sideIdx < 2; ++sideIdx) {
                const float side = sideIdx == 0 ? -1.0f : 1.0f;
                const float s = on.s + side * site.distantM;
                if (s < 0.0f || s > p.length()) continue;
                // Two repeats landing at the same spot are one mast: on the single track
                // both roads' repeats resolve to the same place, and a station does not
                // grow a second post there.
                const TrackPose q = p.poseAt(s);
                bool dup = false;
                for (std::size_t u = 0; u < t && !dup; ++u) {
                    const CrossingSite::On& other = site.tracks[u];
                    if (other.path < 0 || other.path >= static_cast<int>(paths.size()))
                        continue;
                    const float os = other.s + side * site.distantM;
                    if (os < 0.0f || os > paths[other.path].length()) continue;
                    dup = glm::distance(paths[other.path].poseAt(os).pos, q.pos) <= 3.0f;
                }
                if (dup) continue;

                const std::size_t slot = 2 * t + static_cast<std::size_t>(sideIdx);
                int repeats = static_cast<int>(t); // its own track, unless told otherwise
                if (i < distantFor.size() && slot < distantFor[i].size())
                    repeats = distantFor[i][slot];
                const CrossingLights rl =
                    repeats < 0 ? crossingLights(CrossingPhase::Closing, fast)
                                : crossingLights(st.phase(static_cast<std::size_t>(repeats)),
                                                 fast);
                const glm::vec3 qt =
                    glm::normalize(glm::vec3(q.tangent.x, q.tangent.y, 0.0f));
                const glm::vec3 qa(qt.y, -qt.x, 0.0f); // right of +s
                const glm::vec3 base = q.pos + qa * (side > 0.0f ? -3.2f : 3.2f);
                head(vertices_, indices_, base, qt * side, rl.trainRed, rl.trainWhite,
                     period, phase, kVioletOn, kVioletOff);
            }
        }

        // The two the road reads, which belong to the crossing rather than to a track: red
        // while any track is running its sequence. The head on the +across side looks back
        // along +across at the traffic coming that way, and stands to the right of it: for
        // traffic running in -across, right is (-across.y, across.x), which is the track's
        // own +s direction. So the offset follows `side` just as the standoff does.
        //
        // The standoff is measured from the outermost rail on that side, not from the first
        // track: on a crossing spanning two roads a post six metres from one of them would
        // stand between the rails of the other.
        const CrossingLights rd = crossingLights(st.shut() ? CrossingPhase::Closing
                                                           : CrossingPhase::Idle, fast);
        for (const float side : {-1.0f, 1.0f}) {
            const float reach = side > 0.0f ? farPos : farNeg;
            const glm::vec3 base = at.pos + across * (reach + side * kRoadOffsetM) +
                                   tan * (side * kRoadRightM);
            head(vertices_, indices_, base, across * side, rd.roadRed, rd.roadWhite, period,
                 phase);
            // The boom hangs on that same post. The post stands to the right of the
            // traffic it faces, so the lane to block is the one back toward the road's
            // centre - the way `-tan * side` points.
            if (xs[i].barriers)
                boom(vertices_, indices_, base + glm::vec3(0.0f, 0.0f, kBoomPivotH),
                     tan * -side, st.barrier, rd.roadRed, period, phase);
        }
    }
}
