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

#include "AvalancheSignals.h"
#include "TrackMesh.h" // TrackVertex

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

struct TrackPoly;

// The avalanche warning signals: a mast, a housing and three lenses in a column.
//
// Which lens is lit is the aspect's business, and the flashing is nobody's business here
// at all - a lit lens carries its own period and phase and the shader reads them against
// the clock in the push constant, so a blinking head costs no rebuild and this mesh is
// remade only when a signal is placed, moved or changes aspect.
class AvalancheMesh {
public:
    // `shown` is parallel to `signals`: what each head is displaying this moment. Short
    // or empty, the rest read as Clear, which is what the editor wants.
    void build(const std::vector<AvalancheSignal>& signals,
               const std::vector<AvalancheAspect>& shown,
               const std::vector<TrackPoly>& polys, const glm::dvec3& origin);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
