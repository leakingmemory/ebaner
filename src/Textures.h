#pragma once

#include <cstdint>
#include <vector>

// Procedurally generated land-cover surface textures, one array layer per class.
namespace landtex {

constexpr int SIZE = 256;   // per-layer texture dimension (tileable)
constexpr int LAYERS = 11;  // number of class layers

// Texture-array layer indices (kept in sync with the fragment shader).
enum Layer {
    OTHER = 0,        // unclassified / not mapped
    BUILTUP = 1,      // AR50 10
    AGRI = 2,         // AR50 20
    FOREST = 3,       // AR50 30
    OPEN = 4,         // AR50 50
    BOG = 5,          // AR50 60
    GLACIER = 6,      // AR50 70
    FRESH = 7,        // AR50 81 (freshwater)
    OCEAN = 8,        // AR50 82 (ocean)
    BALLAST = 9,      // plain crushed stone (near, under the 3-D sleepers)
    BALLAST_TIES = 10, // ballast + repeating sleeper stripe (distant LOD)
};

// Maps a raw AR50 artype code to a texture-array layer.
int layerForArtype(int artype);

// Generates all layers as one RGBA8 buffer (layer-major, LAYERS*SIZE*SIZE*4).
std::vector<std::uint8_t> generate();

} // namespace landtex
