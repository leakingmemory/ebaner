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

#include <cmath>

// --- Blink -----------------------------------------------------------------------
// A blinking lamp carries its own period and phase (see SignalMesh.cpp's lampAt), and
// the push constant carries a clock rather than a lit/dark flag. Two lamps can then run
// at different rates, and two crossings at the same rate can run out of step.
//
// The clock wraps so float32 keeps its resolution. Every period in use divides the wrap
// exactly, so the wrap is invisible; one that did not would show a single clipped blink
// every few minutes.
constexpr float kBlinkWrapS = 300.0f;
constexpr float kSignalBlinkS = 1.0f;   // a signal's flashing danger, as it always was
constexpr float kCrossingSlowS = 1.5f;  // a crossing at rest
constexpr float kCrossingFastS = 0.5f;  // a crossing with a train coming
// An avalanche warning signal, at rest and in warning alike. The same beat as a
// flashing danger, so the lineside has one flash rate and this does not read as a
// different system standing beside the railway.
constexpr float kAvalancheBlinkS = 1.0f;

// The value for PushConstants::params.y.
inline float blinkClock(double nowSeconds) {
    return static_cast<float>(std::fmod(nowSeconds, static_cast<double>(kBlinkWrapS)));
}

#include "SignalPaths.h" // SignalPlacement
#include "TrackMesh.h"   // TrackVertex (shared solid-lit vertex)

#include <cstdint>
#include <vector>

// Norwegian mini ground signal ("dvergsignal"): a black box on a short (~1 m) pole beside
// the track with two warm-white lamps. A fixed reference lamp sits at the lower-left; a second lamp lights on a
// quarter-arc around it - horizontal (lower-right) = Stop, 45 deg = train ahead on an
// otherwise-clear line, vertical (above the reference) = line clear. Only the Stop aspect is
// drawn for now (reference + horizontal lit; the other arc positions are unlit spots).
//
// Solid-lit TrackVertex geometry (texLayer < 0), drawn with the track/building pipeline;
// static, so it is merged into the static struct buffer like the switch stands.
class SignalMesh {
public:
    void build(const std::vector<SignalPlacement>& signals, glm::dvec3 sceneOrigin);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
