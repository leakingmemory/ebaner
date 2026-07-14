#pragma once

#include "TrackMesh.h" // TrackVertex (shared solid-lit vertex)

#include <cstdint>
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

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
