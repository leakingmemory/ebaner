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
#include <utility>
#include <vector>

class TerrainData;
class TrackPath;

// Position of a switch: which of the two routes the points are set for, or Broken
// (neutral) after a train has forced it while trailing through set against it.
enum class SwitchState { Straight, Diverging, Broken };

// How a switch is operated: Manual (a weighted hand-lever, thrown on the ground) or
// Motor (a point machine, thrown remotely - cannot be hand-thrown in the cab). The
// type is authored in the track editor and persisted as a drop-in overlay.
enum class SwitchType { Manual, Motor };

// A detected turnout: the geometry needed to draw its stand, plus the routing needed
// for the vehicle to divert. A turnout joins three track ends at `world`: the common
// (toe) side and the through (straight) side are the two directions of the *main*
// path passing through the point, and the diverging (siding) branch is `sidingPath`.
struct Turnout {
    // Geometry (world coords, EPSG:25833).
    glm::dvec3 world{0.0}; // point on the through/main track (z = rail head there)
    glm::dvec2 thru{0.0};  // unit, along the through (main) track
    glm::dvec2 div{0.0};   // unit, the diverging branch's leaving direction from `world`

    // Routing, resolved against the TrackPath vector passed to build() (-1 = none).
    int   mainPath   = -1;   // through track the point lies on
    float sMain      = 0.0f; // arc-length of `world` along mainPath
    int   sidingPath = -1;   // diverging branch (one of its ends is at `world`)
    float sSiding    = 0.0f; // 0 or length: which end of sidingPath sits at `world`
    int   facingS    = 1;    // +1 if moving in +s on mainPath is a *facing* approach
    std::uint32_t sidingTrack = 0; // branch track id (dedup keeps one switch per branch)
};

// The set of throwable switches in the loaded network, with mutable per-switch state.
// Detection reuses the endpoint-on-another-track test (and stitch/angle filter) that
// SwitchMesh used; resolution maps each turnout onto the built TrackPaths so the
// vehicle can traverse it.
class SwitchNetwork {
public:
    void build(const TerrainData& data, const std::vector<TrackPath>& paths);

    const std::vector<Turnout>& turnouts() const { return turnouts_; }
    std::size_t size() const { return turnouts_.size(); }
    glm::dvec3 sceneOrigin() const { return origin_; }

    SwitchState state(int i) const { return state_[i]; }
    void setState(int i, SwitchState s) { state_[i] = s; }
    // Throw the switch: Straight <-> Diverging; a Broken switch is repaired to Straight.
    void toggle(int i);

    // Manual (hand-thrown) vs Motor (point machine). Defaults to Manual on build();
    // applySwitchTypes() (SwitchTypes.h) sets the authored overrides after build().
    SwitchType type(int i) const { return type_[i]; }
    void setType(int i, SwitchType t) { type_[i] = t; }

    // Locking set: the track-circuit section ids that must be clear for this (motor)
    // switch to be thrown remotely. Empty on build(); resolved by applySwitchLocks()
    // (SwitchTypes.h) to the authored set, or the sections the switch sits within.
    const std::vector<int>& lock(int i) const { return lock_[i]; }
    void setLock(int i, std::vector<int> ids) { lock_[i] = std::move(ids); }

private:
    std::vector<Turnout> turnouts_;
    std::vector<SwitchState> state_;
    std::vector<SwitchType> type_;
    std::vector<std::vector<int>> lock_; // locking section ids per switch
    glm::dvec3 origin_{0.0};
};
