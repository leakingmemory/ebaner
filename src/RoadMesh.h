#pragma once

#include "TrackMesh.h" // TrackVertex (shared ribbon vertex)

#include <cstdint>
#include <vector>

class TerrainData;

// Builds flat, category-coloured ribbon geometry for all roads in the loaded
// tiles, deduped by geometry (roads carry no unique id). Reuses TrackVertex and
// the track pipeline; private roads are de-emphasised (thin, muted) and emitted
// first so the public network draws on top at junctions.
class RoadMesh {
public:
    void build(const TerrainData& data);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
