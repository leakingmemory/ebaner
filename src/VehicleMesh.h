// ebaner - a Norwegian railway simulator.
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

#include "TrackMesh.h" // TrackVertex (shared solid-lit vertex)

#include <glm/glm.hpp>

#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

class Vehicle;
class Consist;

// Shared wheelset dimensions (metres). kAxleCentreAboveBed is the height of the
// axle centre above the rail-bed centreline (pose.pos) — rail top + wheel radius.
namespace wheelset {
constexpr float kRailTopZ = 0.76f;    // matches TrackMesh rail top (scene offset)
constexpr float kWheelRadius = 0.46f; // 0.92 m wheel
constexpr float kAxleCentreAboveBed = kRailTopZ + kWheelRadius;
} // namespace wheelset

// How much of a vehicle is built, finest last. Each level is dropped beyond its distance
// from the camera: a gauge needle is a few pixels at 40 m, a seat behind a window is lost
// in the window's reflection at 400 m, a wheel is lost under the body at 1.5 km, and at
// 10 km a whole carriage is a pixel or two.
enum class Detail : std::uint8_t {
    Body,        // shell, roof, bogie frames, couplers, lamps: always, out to kTrainDrawDistance
    Wheels,      // axles and wheels
    Interior,    // saloons, seats, tables, cab rooms and their desks
    Instruments, // what is on a desk: gauges, needles, screens, levers, the radio
};
constexpr float kInstrumentsDistance = 40.0f;  // metres from the camera to the vehicle
constexpr float kInteriorDistance = 400.0f;
constexpr float kWheelsDistance = 1500.0f;
constexpr float kTrainDrawDistance = 10000.0f;

// Builds the geometry for a rail vehicle: one wheelset (axle + two wheels) per
// axle, plus a bogie frame box when the vehicle has a wheelbase. Emits TrackVertex
// + indices in scene coords, drawn by the track pipeline.
class VehicleMesh {
public:
    // The whole train, set by set, into one buffer.
    void build(const Consist& consist);
    // Every train in the world, in the order they are held, into that same one buffer.
    // Any change to what is in there or what order it is in has to be followed by an
    // attachVehicle and not merely an updateVehicleVertices: the index buffer is built
    // at attach time and never touched again, so a rebuild under the old one would draw
    // as a heap of triangles with nothing anywhere to say why.
    void build(const std::deque<Consist>& trains);
    // The per-frame path. The trains have moved, so every vertex is regenerated - but the
    // index buffer and the opaque/glass split it is sorted into cannot have changed, since
    // changing either is what attachVehicle is for. So this does the geometry and nothing
    // else: no indices, no re-sorting, just the glazing stamped back on to the vertices
    // that carried it when the buffer was attached.
    //
    // On a 23-unit train that was 8.4 ms of a 20 ms frame, and most of it was work thrown
    // away: 132 000 index pushes and a 44 000-triangle colour sort, every frame, to arrive
    // at exactly the buffer the GPU was already holding.
    void refresh(const std::deque<Consist>& trains);
    // The same, but building geometry only for the units the camera could see. The rest
    // have their slice of the buffer filled with one repeated point, which leaves every
    // triangle degenerate: nothing is rasterised, the vertex count does not move, and the
    // index buffer on the GPU stays exactly as valid as it was.
    //
    // This is where the time is. On the 600 m freight the walks are 4% of the mesh and the
    // other 96% is emitting 83 388 vertices - so the only way to make it cheaper is to
    // emit fewer of them, and the ones behind the camera are free to lose.
    //
    // And for those it builds, only as much as can be made out from `eye` (Detail): a
    // part left out is filled with one point the same way, so it costs a fill and not
    // its geometry.
    void refresh(const std::deque<Consist>& trains, const glm::mat4& viewProj,
                 const glm::vec3& eye);
    // How many units were built last refresh, and how many there were: what the culling
    // actually saved, for the profile line to report. And how many were built to each
    // level of detail (indexed by Detail).
    int lastBuilt() const { return lastBuilt_; }
    int lastUnits() const { return lastUnits_; }
    int lastBuiltAt(Detail d) const { return lastAtDetail_[static_cast<int>(d)]; }

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    // For the renderer to swap with rather than copy: 3 MB a frame is worth not copying,
    // and what it leaves behind here is overwritten whole by the next refresh.
    std::vector<TrackVertex>& mutableVertices() { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }
    // Index at which the transparent (glass) triangles begin; indices are ordered
    // opaque-first, then glass, so the two can be drawn in separate passes.
    std::uint32_t glassFirstIndex() const { return glassFirstIndex_; }

private:
    // One set's geometry, appended to the buffer.
    void emitUnit(const Vehicle& vehicle);
    // One part of a unit that is left out beyond some distance. Building, it runs `body`
    // and records how many vertices it made and where the parts nested in it end. On a
    // refresh it runs `body` if the unit is near enough for `d`, and otherwise puts that
    // many copies of the dead point in its place, skipping the parts inside it too. Parts
    // are met in the same order every time - the same rule the index buffer stands on -
    // so the record is read back in step.
    template <class F>
    void section(Detail d, F&& body) {
        if (wantIndices_) {
            const std::size_t k = sections_.size();
            sections_.push_back({});
            const std::size_t at = vertices_.size();
            body();
            sections_[k] = {static_cast<std::uint32_t>(vertices_.size() - at),
                            static_cast<std::uint32_t>(sections_.size())};
            return;
        }
        if (sectionAt_ >= sections_.size()) { body(); return; } // never recorded
        const Section s = sections_[sectionAt_];
        if (d > detail_) {
            vertices_.insert(vertices_.end(), s.vertexCount, dead_);
            sectionAt_ = s.end;
            return;
        }
        ++sectionAt_;
        body();
    }
    struct Section {
        std::uint32_t vertexCount = 0;
        std::uint32_t end = 0; // the section after it and everything nested in it
    };
    std::vector<Section> sections_;
    std::vector<std::uint32_t> unitSection_; // each unit's first section, as unitSpan_
    std::size_t sectionAt_ = 0;
    Detail detail_ = Detail::Instruments; // the unit being refreshed
    TrackVertex dead_{};                  // and the point its left-out parts collapse to
    int lastAtDetail_[4] = {0, 0, 0, 0};
    // Move the glazing to the back of the index buffer, over whatever has been emitted,
    // and record which vertices it stamped so refresh() can stamp them again without
    // having to work out which they are a second time.
    void sortGlass();
    // Index pushes go through here so refresh() can skip them wholesale.
    void idx(std::uint32_t i) {
        if (wantIndices_) indices_.push_back(i);
    }
    bool wantIndices_ = true;
    // Where each unit's vertices sit in the buffer, recorded when it was attached. Fixed
    // for as long as the index buffer is, which is the same condition everything else here
    // leans on.
    std::vector<std::pair<std::uint32_t, std::uint32_t>> unitSpan_; // {first, count}
    int lastBuilt_ = 0, lastUnits_ = 0;
    std::vector<std::uint32_t> glassVerts_; // vertices carrying the translucent sentinel

    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
    std::uint32_t glassFirstIndex_ = 0;
};

// Driver's-eye camera. `count` is the number of driver positions on the train (2 per
// Class 93 set, one per cab; 0 for vehicles with no cab) - every cab can be sat in and
// looked out of, including the shut-down ones at a coupler. `eyePose` gives the eye
// point and forward direction (world space) for cab `position` at the train's current
// pose; false when unavailable.
namespace drivercam {
int count(const Consist& c);
bool eyePose(const Consist& c, int position, glm::vec3& eye, glm::vec3& forward);
} // namespace drivercam
