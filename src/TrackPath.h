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
              const std::vector<glm::vec3>& pts,      // scene-relative input
              const std::vector<std::uint16_t>& speed); // per-point km/h (0=?)

    float length() const { return length_; }
    TrackPose poseAt(float s) const;       // s clamped to [0, length]
    float speedLimitAt(float s) const;     // km/h at s (0 = unknown)

    // The surveyed limits as imported, in order along the path: km/h with 0 meaning
    // unknown, paired with where each takes effect. Raw on purpose - speedLimitAt() hides
    // an unknown point behind its neighbour's value, which is exactly the gap anything
    // filling gaps by rule has to be able to see.
    struct SpeedPoint {
        float s = 0.0f;
        std::uint16_t kmh = 0;
    };
    std::vector<SpeedPoint> speedPoints() const;

    std::uint32_t trackId() const { return trackId_; }
    std::uint8_t trackType() const { return trackType_; }

private:
    // Control points (with reflected phantom endpoints prepended/appended, so
    // span i interpolates ctrl_[i+1]..ctrl_[i+2]) and their centripetal knots.
    std::vector<glm::vec3> ctrl_;
    std::vector<float> knot_;

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
