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

#include <cstdint>
#include <utility>
#include <vector>

class TerrainData;

// Extrudes OSM building footprints into lit prisms (walls + flat roof), coloured
// by building kind. Reuses TrackVertex and the track pipeline; deduped by
// geometry (buildings carry no id).
class BuildingMesh {
public:
    void build(const TerrainData& data);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }
    // The built mesh, handed over: for when it is only being passed on, so a world's worth
    // of geometry is not held twice.
    std::vector<TrackVertex> takeVertices() { return std::move(vertices_); }
    std::vector<std::uint32_t> takeIndices() { return std::move(indices_); }
    // The buildings grouped by the draw cell they stand in (drawCell), so the ones the
    // camera is not looking at, or is too far from to make out, need not be drawn. A town
    // is tens of thousands of prisms and a view holds a wedge of it.
    const std::vector<TrackDrawChunk>& chunks() const { return chunks_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
    std::vector<TrackDrawChunk> chunks_;
};
