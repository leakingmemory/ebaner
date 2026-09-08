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

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

class TerrainData;

// Pose sampled at some arc-length along a track: everything a mesh sweep or a
// vehicle simulation needs to sit on the rails at that point.
struct TrackPose {
    glm::vec3 pos;     // scene-relative metres (z up)
    glm::vec3 tangent; // unit, direction of travel
    glm::vec3 right;   // unit cross-track (banked by cant); points *left* of travel - see poseAt
    glm::vec3 up;      // unit cross-track up (banked by cant)
    float curvature;   // signed horizontal 1/R (m^-1); sign = curving left/right
    float cant;        // superelevation roll about the tangent (radians), signed like
                       // `curvature`: the track always banks into the curve, never adversely
};

// A smooth, arc-length-parameterised centreline for one track, interpolating the
// surveyed points with a centripetal Catmull-Rom spline (passes through every
// point, no overshoot). Query pose/curvature at any distance along it.
class TrackPath {
public:
    TrackPath(std::uint32_t trackId, std::uint8_t trackType,
              const std::vector<glm::vec3>& pts,        // scene-relative input
              const std::vector<std::uint16_t>& speed,  // per-point km/h (0=?)
              // Per-point medium (0x20 surface, 0x55 tunnel, 0x54 tube, 0x4C/0x42
              // bridge). Empty means "not known", which reads as surface everywhere.
              const std::vector<std::uint8_t>& medium = {},
              // Per-point exported track, and that point's fraction along that track's
              // own polyline. A path chains many tracks head to tail, so these are what
              // say which one any given point came from and where along it. Empty
              // disables the track/fraction bridge below; everything else still works.
              const std::vector<std::uint32_t>& ptTrack = {},
              const std::vector<float>& ptFrac = {},
              // At the first surviving point of each chained track, the fraction of the
              // point that was dropped as a duplicate of the seam before it - which is
              // that track's own end. -1 everywhere else. Without it a track whose only
              // other point was the seam has nowhere to put fraction 0.
              const std::vector<float>& ptSeam = {});

    float length() const { return length_; }
    TrackPose poseAt(float s) const;       // s clamped to [0, length]
    float speedLimitAt(float s) const;     // km/h at s (0 = unknown)
    // Whether the line runs underground here. Lineside furniture needs to know: a bore
    // is a far tighter space than the open line, and a sign set out for daylight ends
    // up inside the rock.
    bool undergroundAt(float s) const;

    // The surveyed limits as imported, in order along the path: km/h with 0 meaning
    // unknown, paired with where each takes effect. Raw on purpose - speedLimitAt() hides
    // an unknown point behind its neighbour's value, which is exactly the gap anything
    // filling gaps by rule has to be able to see.
    struct SpeedPoint {
        float s = 0.0f;
        std::uint16_t kmh = 0;
    };
    std::vector<SpeedPoint> speedPoints() const;

    // Axis-aligned bounds over the control points (scene-relative). A cheap first
    // question for "is this path anywhere near here" - with the whole line resident,
    // anything that used to walk every path end to end has to ask that first.
    const glm::vec3& boundsMin() const { return bmin_; }
    const glm::vec3& boundsMax() const { return bmax_; }
    // True if `pt` (scene x,y) is within `margin` of those bounds.
    bool nearXY(const glm::vec2& pt, float margin) const {
        return pt.x >= bmin_.x - margin && pt.x <= bmax_.x + margin &&
               pt.y >= bmin_.y - margin && pt.y <= bmax_.y + margin;
    }

    // The seed segment's id, which chaining makes near-meaningless: a path runs through
    // many tracks and the seed is generally not even the first of them. Kept because it
    // names the path in logs; use the bridge below to ask what is actually at a point.
    std::uint32_t trackId() const { return trackId_; }
    std::uint8_t trackType() const { return trackType_; }

    // --- The track/fraction bridge ------------------------------------------------------
    // Everything authored in the overlay - borders, section intervals, signal anchors - is
    // written against an exported track and a fraction along it. A train runs in this
    // path's arc length. These convert, and they have to: the two are different metrics
    // over different extents, so `s = frac * length` is wrong twice over and drifts by
    // metres over a kilometre. Every other consumer in the tree bridges the gap by
    // proximity instead, with tolerances from 2.5 m to 25 m.
    //
    // One maximal run of points per track, in the order the path runs through them.
    // `frac` descends over a run the chaining took backwards, which is decided by whichever
    // segment happened to seed the chain - so never assume it ascends.
    struct TrackRun {
        std::uint32_t trackId = 0;
        float s0 = 0.0f, s1 = 0.0f;       // arc length at the run's first and last point
        float frac0 = 0.0f, frac1 = 1.0f; // that track's own fraction there
        bool descending = false;          // frac0 > frac1
    };
    const std::vector<TrackRun>& trackRuns() const { return runs_; }

    // Arc length along this path of `frac` on `trackId`. False if that track is not on
    // this path. `frac` outside the run's own extent is clamped to it - which is right at
    // a seam, where the shared point belongs to the neighbour and the run stops a
    // millimetre short of its own end.
    bool fracToS(std::uint32_t trackId, double frac, float& s) const;
    // Which track is at arc length `s`, and where along it. False if the bridge is absent.
    bool trackAt(float s, std::uint32_t& trackId, double& frac) const;

private:
    // Control points (with reflected phantom endpoints prepended/appended, so
    // span i interpolates ctrl_[i+1]..ctrl_[i+2]) and their centripetal knots.
    std::vector<glm::vec3> ctrl_;
    std::vector<float> knot_;
    glm::vec3 bmin_{0.0f}, bmax_{0.0f};

    // Arc-length table entry mapping cumulative length s -> global parameter
    // g = span + local-u (in [0, numSpans]).
    struct Sample {
        float s;    // cumulative arc length
        float g;    // span + local u
        float cant; // smoothed superelevation roll (radians)
    };
    std::vector<Sample> table_;
    float length_ = 0.0f;

    std::vector<std::uint16_t> speed_; // one per surveyed point (aligned to pts)
    std::vector<std::uint8_t> medium_; // ditto; empty when the caller did not say

    // The bridge. `runPts_` holds (fraction, arc length) for the points of each run in
    // order, `runs_` indexes into it. A run carries one point more than it contributed to
    // `ctrl_` when it follows another: the seam point is dropped from the geometry as a
    // duplicate of its neighbour, but it is that track's own end and a border authored at
    // fraction 0 or 1 lands exactly there, so the run keeps it.
    struct RunPoint {
        float frac = 0.0f;
        float s = 0.0f;
    };
    std::vector<RunPoint> runPts_;
    std::vector<TrackRun> runs_;
    std::vector<int> runFirst_, runCount_; // into runPts_, parallel to runs_
    // Arc length of every surveyed point, which is what places a fraction on the path.
    std::vector<float> pointArcLengths() const;
    void buildTrackRuns(const std::vector<std::uint32_t>& ptTrack,
                        const std::vector<float>& ptFrac,
                        const std::vector<float>& ptSeam);

    std::uint32_t trackId_ = 0;
    std::uint8_t trackType_ = 0;

    // Map arc-length s -> interpolated span index, local u in [0,1], and the
    // interpolated (smoothed) cant angle.
    void locate(float s, int& span, float& u, float& cant) const;
    // Evaluate position / first / second derivative in knot space at span,u.
    void eval(int span, float u, glm::vec3* p, glm::vec3* d1, glm::vec3* d2) const;
};

// Build one smooth path per unique track in the loaded tiles: dedup by trackId (a
// through-track appears in full in every tile it crosses), convert world points
// to scene-relative, and drop coincident points.
std::vector<TrackPath> buildTrackPaths(const TerrainData& data);
