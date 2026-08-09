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

#include "TerrainData.h"

#include "TerrainCarve.h"
#include "TrackOverlay.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace fs = std::filesystem;

namespace {

// Ground resolution (metres/pixel) per LOD level.
constexpr double kLodResolution[4] = {10.0, 20.0, 40.0, 80.0};

// Bodo station node (EPSG:25833) — seed for locating the track terminus.
constexpr double kBodoSeedX = 473776.625;
constexpr double kBodoSeedY = 7463431.0;
constexpr double kBodoSeedZ = 4.197073;

std::string tileDir(const std::string& root, int lod, int col, int row) {
    return root + "/tiles/" + std::to_string(lod) + "/" + std::to_string(col) +
           "_" + std::to_string(row);
}

} // namespace

double TerrainData::lodResolution(int lod) {
    return kLodResolution[std::clamp(lod, 0, 3)];
}

void TerrainData::applyTrackEdits(const std::vector<TrackEdit>& edits) {
    applyTrackOverlay(networkTracks_, edits);
}

// The loaded tiles as a plain list, for the passes that walk all of them.
std::vector<Tile*> TerrainData::tileList() {
    std::vector<Tile*> out;
    out.reserve(tiles_.size());
    for (const auto& [key, t] : tiles_) out.push_back(t.get());
    return out;
}

void TerrainData::recarve() {
    // Reset each tile to its pre-carve height, then carve again with the current
    // (edited) track geometry — so cuttings/embankments reflect edits without
    // stacking on the previous carve. Mirrors the load()-time carve guard.
    for (const auto& [key, t] : tiles_)
        if (!t->pristine.empty()) t->heights = t->pristine;
    if (std::getenv("EBANER_NOCARVE") == nullptr) {
        std::vector<Tile*> list = tileList();
        carveTrackCuttings(list, networkTracks_, sceneOrigin_);
    }
}

bool TerrainData::loadTile(int lod, int col, int row) {
    const double res = kLodResolution[lod];
    const double extent = PIXELS * res;
    const std::string dir = tileDir(root_, lod, col, row);
    if (!fs::exists(dir)) return false;

    auto t = std::make_unique<Tile>();
    t->lod = lod;
    t->col = col;
    t->row = row;
    t->resolution = res;
    t->extent = extent;
    t->originX = static_cast<double>(col) * extent;
    t->originY = static_cast<double>(row) * extent;

    if (!loadHeightmap(dir + "/terrain.hm32", t->heights)) return false;

    // Track the elevation range for the colour ramp.
    for (float h : t->heights) {
        if (h <= NODATA + 1.0f) continue;
        minElev_ = std::min(minElev_, h);
        maxElev_ = std::max(maxElev_, h);
    }

    // Optional per-tile land cover (present only for AR50 exports).
    if (loadLandCover(dir + "/landcover.u8", t->landcover)) hasLandCover_ = true;

    // Road + building geometry intersecting this tile. The railway is not read here: it
    // is loaded whole, once, by loadNetworkTracks.
    parseRoadsBin(dir + "/roads.bin", t->roads);
    parseBuildingsBin(dir + "/buildings.bin", t->buildings);
    parsePlatformsBin(dir + "/platforms.bin", t->platforms);

    // The ground as found, to re-carve from after an edit.
    t->pristine = t->heights;

    tiles_[tileKey(lod, col, row)] = std::move(t);
    return true;
}

TerrainData::WindowChange TerrainData::updateWindow(double centreX, double centreY) {
    WindowChange ch;
    if (root_.empty()) return ch;
    windowCentre_ = glm::dvec2(centreX, centreY);

    // Drop first, so a long run does not hold both windows at once. The evict radius is
    // one rebuild step wider than the load radius, and no more: a tile on the boundary
    // would otherwise be read and dropped and read again as the camera drifts over it,
    // while a generous margin keeps a second window's worth of terrain resident for
    // nothing (at 20 km loaded, even a 35% margin is nearly twice the ground).
    constexpr double kEvictMarginM = 2500.0;
    const double keep = halfWindow_ + kEvictMarginM;
    std::vector<std::uint64_t> gone;
    for (const auto& [key, t] : tiles_) {
        const double tx = t->originX + t->extent * 0.5;
        const double ty = t->originY + t->extent * 0.5;
        if (std::hypot(tx - centreX, ty - centreY) > keep + t->extent * 0.5)
            gone.push_back(key);
    }
    for (const std::uint64_t key : gone) {
        const Tile& t = *tiles_[key];
        ch.touched.push_back({t.originX, t.originY, t.originX + t.extent,
                              t.originY + t.extent});
        ch.removed.push_back(key);
        tiles_.erase(key);
    }

    // Read whatever is now in range. Same rule as load(): every LOD whose footprint
    // falls in the window, finest winning wherever it exists.
    std::vector<Tile*> fresh;
    for (int lod = 0; lod < 4; ++lod) {
        const double extent = PIXELS * kLodResolution[lod];
        const int colMin = static_cast<int>(std::floor((centreX - halfWindow_) / extent));
        const int colMax = static_cast<int>(std::floor((centreX + halfWindow_) / extent));
        const int rowMin = static_cast<int>(std::floor((centreY - halfWindow_) / extent));
        const int rowMax = static_cast<int>(std::floor((centreY + halfWindow_) / extent));
        for (int col = colMin; col <= colMax; ++col)
            for (int row = rowMin; row <= rowMax; ++row) {
                const std::uint64_t key = tileKey(lod, col, row);
                if (tiles_.count(key)) continue;
                if (!loadTile(lod, col, row)) continue;
                const Tile& t = *tiles_[key];
                fresh.push_back(tiles_[key].get());
                ch.added.push_back(key);
                ch.touched.push_back({t.originX, t.originY, t.originX + t.extent,
                                      t.originY + t.extent});
            }
    }

    if (!fresh.empty() && std::getenv("EBANER_NOCARVE") == nullptr) {
        // Carve only what just arrived, and only against the track near it. The carve
        // buckets its edges, but building that index over the whole country's rails for
        // a couple of tiles is most of the cost - so hand it the neighbourhood.
        double x0 = 1e30, y0 = 1e30, x1 = -1e30, y1 = -1e30;
        for (const Tile* t : fresh) {
            x0 = std::min(x0, t->originX);
            y0 = std::min(y0, t->originY);
            x1 = std::max(x1, t->originX + t->extent);
            y1 = std::max(y1, t->originY + t->extent);
        }
        constexpr double kMargin = 500.0; // a cutting is shaped by track just outside
        std::vector<TrackSegment> near;
        for (const TrackSegment& s : networkTracks_) {
            bool touches = false;
            for (const glm::dvec3& p : s.pts)
                if (p.x >= x0 - kMargin && p.x <= x1 + kMargin && p.y >= y0 - kMargin &&
                    p.y <= y1 + kMargin) {
                    touches = true;
                    break;
                }
            if (touches) near.push_back(s);
        }
        carveTrackCuttings(fresh, near, sceneOrigin_);
    }

    if (ch.any()) {
        std::size_t samples = 0;
        for (const auto& [key, t] : tiles_) samples += t->heights.size();
        std::printf("[TerrainData] window at (%.0f, %.0f): +%zu -%zu tiles, %zu resident "
                    "(%zu samples)\n",
                    centreX, centreY, ch.added.size(), ch.removed.size(), tiles_.size(),
                    samples);
        std::fflush(stdout);
    }
    return ch;
}

void TerrainData::load(const std::string& datasetRoot, double halfWindow) {
    load(datasetRoot, glm::dvec3(kBodoSeedX, kBodoSeedY, kBodoSeedZ), halfWindow);
}

void TerrainData::load(const std::string& datasetRoot, const glm::dvec3& startSeed,
                       double halfWindow) {
    if (!fs::exists(datasetRoot)) {
        throw std::runtime_error("dataset root not found: " + datasetRoot);
    }

    root_ = datasetRoot;
    // The network first: it is read in world coordinates and so does not need an origin,
    // and resolving the start point onto the rails needs all of it rather than whichever
    // tile the station happens to sit in.
    loadNetworkTracks(datasetRoot);
    resolveStartPoint(startSeed);
    halfWindow_ = halfWindow;
    sceneOrigin_ = startWorld_;
    startPos_ = glm::vec3(0.0f); // camera start is the scene origin itself

    const double cx = startWorld_.x;
    const double cy = startWorld_.y;
    windowCentre_ = glm::dvec2(cx, cy);

    // Because LOD tiles don't overlap in coverage, loading every existing tile
    // whose footprint falls inside the window yields a continuous surface.
    std::size_t totalSamples = 0;
    for (int lod = 0; lod < 4; ++lod) {
        const double res = kLodResolution[lod];
        const double extent = PIXELS * res;

        const int colMin = static_cast<int>(std::floor((cx - halfWindow) / extent));
        const int colMax = static_cast<int>(std::floor((cx + halfWindow) / extent));
        const int rowMin = static_cast<int>(std::floor((cy - halfWindow) / extent));
        const int rowMax = static_cast<int>(std::floor((cy + halfWindow) / extent));

        for (int col = colMin; col <= colMax; ++col) {
            for (int row = rowMin; row <= rowMax; ++row) {
                if (loadTile(lod, col, row))
                    totalSamples += tiles_[tileKey(lod, col, row)]->heights.size();
            }
        }
    }

    // Apply the manual track-edit overlay (drop-in link fixes) before anything
    // downstream reads the geometry (carve, path building).
    if (std::getenv("EBANER_NOOVERLAY") == nullptr)
        applyTrackEdits(loadTrackOverlay(datasetRoot));

    // Carve trenches into the terrain where surface track falls below it, so the
    // rails/ballast sit in a cutting instead of being buried. Re-derive the
    // elevation range afterwards from the carved heights. Guarded for comparison.
    if (std::getenv("EBANER_NOCARVE") == nullptr) {
        std::vector<Tile*> list = tileList();
        carveTrackCuttings(list, networkTracks_, sceneOrigin_);
        for (const Tile* t : list)
            for (float h : t->heights) {
                if (h <= NODATA + 1.0f) continue;
                minElev_ = std::min(minElev_, h);
                maxElev_ = std::max(maxElev_, h);
            }
    }
    if (maxElev_ <= minElev_) maxElev_ = minElev_ + 1.0f;

    std::printf("[TerrainData] start (world UTM33): x=%.2f y=%.2f z=%.2f\n",
                startWorld_.x, startWorld_.y, startWorld_.z);
    std::printf("[TerrainData] look dir (scene): x=%.3f y=%.3f\n",
                startDir_.x, startDir_.y);
    std::printf("[TerrainData] loaded %zu tiles (%zu samples) within %.0f m; "
                "elev [%.1f, %.1f]; land cover: %s\n",
                tiles_.size(), totalSamples, halfWindow, minElev_, maxElev_,
                hasLandCover_ ? "yes" : "no");

    if (tiles_.empty()) {
        throw std::runtime_error("no terrain tiles loaded around start point");
    }
}

bool TerrainData::sampleGround(double worldX, double worldY,
                               float& elevation) const {
    bool found = false;
    // Finest LOD first, and straight to the tile that covers the point. This used to
    // scan every loaded tile, which is fine for a window and quadratic for a world:
    // the carve, the bores and the platforms all ask it per sample.
    for (int lod = 0; lod < 4 && !found; ++lod) {
        const double extent = PIXELS * kLodResolution[lod];
        const Tile* t = tileAt(lod, static_cast<int>(std::floor(worldX / extent)),
                               static_cast<int>(std::floor(worldY / extent)));
        if (t == nullptr || t->heights.empty()) continue;
        // Row 0 is the north edge (max Y); columns run west->east.
        int col = static_cast<int>((worldX - t->originX) / t->resolution);
        int row = static_cast<int>((t->originY + t->extent - worldY) / t->resolution);
        col = std::clamp(col, 0, PIXELS - 1);
        row = std::clamp(row, 0, PIXELS - 1);
        const float h = t->heights[static_cast<std::size_t>(row) * PIXELS + col];
        if (h <= NODATA) continue; // no data here in this tile
        elevation = h;
        found = true;
    }
    return found;
}

void TerrainData::loadNetworkTracks(const std::string& datasetRoot) {
    networkTracks_.clear();
    std::unordered_set<std::uint32_t> seen;
    std::size_t files = 0;

    // Finest LOD first, so the copy kept per trackId is the most detailed one. A
    // segment crossing tiles is written in full into every tile it touches, so one
    // copy per id is the whole segment - the same dedup buildTrackPaths does, just
    // done once here instead of over and over downstream.
    for (int lod = 0; lod < 4; ++lod) {
        const std::string lodDir = datasetRoot + "/tiles/" + std::to_string(lod);
        std::error_code ec;
        if (!fs::is_directory(lodDir, ec)) continue;
        for (const fs::directory_entry& e : fs::directory_iterator(lodDir, ec)) {
            if (!e.is_directory()) continue;
            std::vector<TrackSegment> segs;
            if (!parseTracksBin(e.path().string() + "/tracks.bin", segs)) continue;
            ++files;
            for (TrackSegment& s : segs) {
                if (s.pts.size() < 2) continue;
                if (!seen.insert(s.trackId).second) continue;
                networkTracks_.push_back(std::move(s));
            }
        }
    }

    // Repair missing elevations before anything downstream sees them. A few hundred
    // imported vertices carry the NODATA sentinel in z instead of a height, and every
    // consumer reads z as a real number: the carve digs a crater kilometres deep, and
    // TrackPath measures arc length in 3-D, so one such vertex adds ~20 km to a path's
    // length and wrecks every position along it. Interpolating between the valid
    // neighbours either side restores a sane grade; a run at the end takes the nearest
    // valid height.
    std::size_t repaired = 0, hopeless = 0;
    for (TrackSegment& sg : networkTracks_) {
        const std::size_t n = sg.pts.size();
        auto bad = [&](std::size_t i) { return sg.pts[i].z <= NODATA + 1.0; };
        std::size_t firstGood = n;
        for (std::size_t i = 0; i < n; ++i)
            if (!bad(i)) { firstGood = i; break; }
        if (firstGood == n) { // nothing to interpolate from
            for (glm::dvec3& q : sg.pts) q.z = 0.0;
            hopeless += n;
            continue;
        }
        for (std::size_t i = 0; i < n; ++i) {
            if (!bad(i)) continue;
            ++repaired;
            std::size_t lo = n, hi = n;
            for (std::size_t j = i; j-- > 0;)
                if (!bad(j)) { lo = j; break; }
            for (std::size_t j = i + 1; j < n; ++j)
                if (!bad(j)) { hi = j; break; }
            if (lo < n && hi < n) {
                const double f = static_cast<double>(i - lo) / static_cast<double>(hi - lo);
                sg.pts[i].z = sg.pts[lo].z + (sg.pts[hi].z - sg.pts[lo].z) * f;
            } else {
                sg.pts[i].z = sg.pts[lo < n ? lo : hi].z;
            }
        }
    }
    if (repaired > 0 || hopeless > 0)
        std::printf("[TerrainData] repaired %zu track vertices with no elevation"
                    "%s\n", repaired + hopeless,
                    hopeless ? " (some whole segments had none)" : "");

    std::size_t pts = 0;
    for (const TrackSegment& s : networkTracks_) pts += s.pts.size();
    std::printf("[TerrainData] network: %zu tracks (%zu points) from %zu tiles\n",
                networkTracks_.size(), pts, files);
}

bool TerrainData::parseTracksBin(const std::string& path,
                                 std::vector<TrackSegment>& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    std::vector<char> buf((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());
    const char* p = buf.data();
    const char* end = p + buf.size();

    auto readU32 = [&](std::uint32_t& v) -> bool {
        if (p + 4 > end) return false;
        std::memcpy(&v, p, 4);
        p += 4;
        return true;
    };
    auto readF32 = [&](float& v) -> bool {
        if (p + 4 > end) return false;
        std::memcpy(&v, p, 4);
        p += 4;
        return true;
    };
    auto readU16 = [&](std::uint16_t& v) -> bool {
        if (p + 2 > end) return false;
        std::memcpy(&v, p, 2);
        p += 2;
        return true;
    };

    // The per-vertex speed block (uint16 each) was added to the format later.
    // Detect whether this file carries it by checking which layout walks exactly
    // to EOF, so both old (no speed) and new exports parse correctly.
    auto validate = [&](bool hasSpeed) -> bool {
        const char* q = buf.data();
        const char* e = q + buf.size();
        if (q + 4 > e) return false;
        std::uint32_t n = 0;
        std::memcpy(&n, q, 4);
        q += 4;
        for (std::uint32_t i = 0; i < n; ++i) {
            if (q + 12 > e) return false;
            std::uint32_t nv = 0;
            std::memcpy(&nv, q + 8, 4);
            q += 12;
            const std::size_t need =
                static_cast<std::size_t>(nv) * (hasSpeed ? 14u : 12u);
            if (static_cast<std::size_t>(e - q) < need) return false;
            q += need;
        }
        return q == e;
    };
    const bool hasSpeed = validate(true) ? true : !validate(false);

    std::uint32_t numSegments = 0;
    if (!readU32(numSegments)) return false;

    for (std::uint32_t s = 0; s < numSegments; ++s) {
        std::uint32_t trackId = 0, numVertices = 0;
        if (p + 12 > end) break;
        std::memcpy(&trackId, p, 4);
        const std::uint8_t trackType = static_cast<std::uint8_t>(p[4]);
        const std::uint8_t medium = static_cast<std::uint8_t>(p[5]);
        p += 8; // consume trackId, trackType, medium, electrified, reserved
        std::memcpy(&numVertices, p, 4);
        p += 4;

        // Reject an implausible vertex count (truncated/corrupt) before reserving.
        const std::size_t stride = hasSpeed ? 14u : 12u;
        if (static_cast<std::size_t>(numVertices) * stride >
            static_cast<std::size_t>(end - p))
            break;

        TrackSegment seg;
        seg.trackId = trackId;
        seg.trackType = trackType;
        seg.medium = medium;
        seg.pts.reserve(numVertices);
        bool ok = true;
        for (std::uint32_t v = 0; v < numVertices; ++v) {
            float x, y, z;
            if (!readF32(x) || !readF32(y) || !readF32(z)) { ok = false; break; }
            seg.pts.emplace_back(x, y, z);
        }
        if (!ok) break;
        if (hasSpeed) {
            // Per-vertex OSM speed (km/h, 0 = unknown), one uint16 each.
            seg.speed.reserve(numVertices);
            for (std::uint32_t v = 0; v < numVertices; ++v) {
                std::uint16_t s16 = 0;
                if (!readU16(s16)) { ok = false; break; }
                seg.speed.push_back(s16);
            }
            if (!ok) break;
        }
        if (seg.pts.size() >= 2) out.push_back(std::move(seg));
    }
    return !out.empty();
}

bool TerrainData::parseRoadsBin(const std::string& path,
                                std::vector<RoadSegment>& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    std::vector<char> buf((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());
    const char* p = buf.data();
    const char* end = p + buf.size();

    auto readU32 = [&](std::uint32_t& v) -> bool {
        if (p + 4 > end) return false;
        std::memcpy(&v, p, 4);
        p += 4;
        return true;
    };
    auto readF32 = [&](float& v) -> bool {
        if (p + 4 > end) return false;
        std::memcpy(&v, p, 4);
        p += 4;
        return true;
    };

    std::uint32_t numSegments = 0;
    if (!readU32(numSegments)) return false;

    for (std::uint32_t s = 0; s < numSegments; ++s) {
        if (p + 12 > end) break;
        const std::uint8_t kategori = static_cast<std::uint8_t>(p[0]);
        std::uint32_t nummer = 0, numVertices = 0;
        std::memcpy(&nummer, p + 4, 4);  // skip kategori + 3 reserved
        std::memcpy(&numVertices, p + 8, 4);
        p += 12;

        // Sanity: each vertex is 12 bytes (x,y,z). Reject an implausible count.
        if (static_cast<std::size_t>(numVertices) * 12u >
            static_cast<std::size_t>(end - p))
            break;

        RoadSegment seg;
        seg.kategori = kategori;
        seg.nummer = nummer;
        seg.pts.reserve(numVertices);
        bool ok = true;
        for (std::uint32_t v = 0; v < numVertices; ++v) {
            float x, y, z;
            if (!readF32(x) || !readF32(y) || !readF32(z)) { ok = false; break; }
            seg.pts.emplace_back(x, y, z);
        }
        if (!ok) break;
        if (seg.pts.size() >= 2) out.push_back(std::move(seg));
    }
    return !out.empty();
}

bool TerrainData::parseBuildingsBin(const std::string& path,
                                    std::vector<BuildingSegment>& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    std::vector<char> buf((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());
    const char* p = buf.data();
    const char* end = p + buf.size();

    auto readU32 = [&](std::uint32_t& v) -> bool {
        if (p + 4 > end) return false;
        std::memcpy(&v, p, 4);
        p += 4;
        return true;
    };
    auto readF32 = [&](float& v) -> bool {
        if (p + 4 > end) return false;
        std::memcpy(&v, p, 4);
        p += 4;
        return true;
    };

    std::uint32_t numBuildings = 0;
    if (!readU32(numBuildings)) return false;

    for (std::uint32_t s = 0; s < numBuildings; ++s) {
        if (p + 16 > end) break; // kind + roof + reserved + baseZ + height + count
        const std::uint8_t kind = static_cast<std::uint8_t>(p[0]);
        const std::uint8_t roofShape = static_cast<std::uint8_t>(p[1]);
        float baseZ = 0.0f, height = 0.0f;
        std::uint32_t numVertices = 0;
        std::memcpy(&baseZ, p + 4, 4);
        std::memcpy(&height, p + 8, 4);
        std::memcpy(&numVertices, p + 12, 4);
        p += 16;

        // Sanity: each footprint vertex is 8 bytes (x,y). Reject an implausible
        // count before reserving.
        if (static_cast<std::size_t>(numVertices) * 8u >
            static_cast<std::size_t>(end - p))
            break;

        BuildingSegment b;
        b.kind = kind;
        b.roofShape = roofShape;
        b.baseZ = baseZ;
        b.height = height;
        b.footprint.reserve(numVertices);
        bool ok = true;
        for (std::uint32_t v = 0; v < numVertices; ++v) {
            float x, y;
            if (!readF32(x) || !readF32(y)) { ok = false; break; }
            b.footprint.emplace_back(x, y);
        }
        if (!ok) break;
        if (b.footprint.size() >= 3) out.push_back(std::move(b));
    }
    return !out.empty();
}

bool TerrainData::parsePlatformsBin(const std::string& path,
                                    std::vector<PlatformSegment>& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    std::vector<char> buf((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());
    const char* p = buf.data();
    const char* end = p + buf.size();

    auto readU32 = [&](std::uint32_t& v) -> bool {
        if (p + 4 > end) return false;
        std::memcpy(&v, p, 4);
        p += 4;
        return true;
    };
    auto readF32 = [&](float& v) -> bool {
        if (p + 4 > end) return false;
        std::memcpy(&v, p, 4);
        p += 4;
        return true;
    };

    std::uint32_t numPlatforms = 0;
    if (!readU32(numPlatforms)) return false;

    for (std::uint32_t s = 0; s < numPlatforms; ++s) {
        if (p + 12 > end) break; // baseZ + height + count
        float baseZ = 0.0f, height = 0.0f;
        std::uint32_t numVertices = 0;
        std::memcpy(&baseZ, p + 0, 4);
        std::memcpy(&height, p + 4, 4);
        std::memcpy(&numVertices, p + 8, 4);
        p += 12;

        // Sanity: each footprint vertex is 8 bytes (x,y). Reject an implausible
        // count before reserving.
        if (static_cast<std::size_t>(numVertices) * 8u >
            static_cast<std::size_t>(end - p))
            break;

        PlatformSegment pl;
        pl.baseZ = baseZ;
        pl.height = height;
        pl.footprint.reserve(numVertices);
        bool ok = true;
        for (std::uint32_t v = 0; v < numVertices; ++v) {
            float x, y;
            if (!readF32(x) || !readF32(y)) { ok = false; break; }
            pl.footprint.emplace_back(x, y);
        }
        if (!ok) break;
        if (pl.footprint.size() >= 3) out.push_back(std::move(pl));
    }
    return !out.empty();
}

// Put the start on the railway near `seed`, and face it somewhere sensible.
//
// A terminus and a through station want different answers. At Bodo the line ends: the
// only place to stand is the buffer stop, looking in. At Fauske or Rognan the line runs
// straight past, there is no end to find, and either direction is a valid departure -
// so the nearest point on the running line, along it, is the answer there.
void TerrainData::resolveStartPoint(const glm::dvec3& seed) {
    startWorld_ = seed;
    startDir_ = glm::vec3(1.0f, 0.0f, 0.0f);

    // A main-line end this close to the station node is its buffer stop.
    constexpr double kTerminusM = 150.0;

    double bestEndD = std::numeric_limits<double>::max();
    glm::dvec3 endPt{0.0}, endInto{0.0};
    double bestOnD = std::numeric_limits<double>::max();
    glm::dvec3 onPt{0.0};
    glm::dvec2 onDir{1.0, 0.0};

    for (const TrackSegment& sg : networkTracks_) {
        if (sg.trackType != 0 || sg.pts.size() < 2) continue; // main line only
        const std::vector<glm::dvec3>& v = sg.pts;

        const std::pair<glm::dvec3, glm::dvec3> ends[2] = {{v.front(), v[1]},
                                                           {v.back(), v[v.size() - 2]}};
        for (const auto& [pt, into] : ends) {
            const double d = std::hypot(pt.x - seed.x, pt.y - seed.y);
            if (d < bestEndD) {
                bestEndD = d;
                endPt = pt;
                endInto = into;
            }
        }

        for (std::size_t k = 0; k + 1 < v.size(); ++k) {
            const glm::dvec2 a(v[k].x, v[k].y), b(v[k + 1].x, v[k + 1].y);
            const glm::dvec2 ab = b - a;
            const double l2 = glm::dot(ab, ab);
            if (l2 < 1e-9) continue;
            const double t =
                std::clamp(glm::dot(glm::dvec2(seed.x, seed.y) - a, ab) / l2, 0.0, 1.0);
            const glm::dvec2 q = a + ab * t;
            const double d = std::hypot(q.x - seed.x, q.y - seed.y);
            if (d < bestOnD) {
                bestOnD = d;
                onPt = glm::dvec3(q.x, q.y, v[k].z + (v[k + 1].z - v[k].z) * t);
                onDir = ab / std::sqrt(l2);
            }
        }
    }

    glm::dvec2 dir(1.0, 0.0);
    if (bestEndD <= kTerminusM) {
        startWorld_ = endPt;
        dir = glm::dvec2(endInto.x - endPt.x, endInto.y - endPt.y);
    } else if (bestOnD < std::numeric_limits<double>::max()) {
        startWorld_ = onPt;
        dir = onDir;
    } else {
        std::printf("[TerrainData] no main line near the start seed; using it as given\n");
        return;
    }

    const double len = glm::length(dir);
    if (len > 1e-6) {
        dir /= len;
        startDir_ = glm::vec3(static_cast<float>(dir.x), static_cast<float>(dir.y), 0.0f);
    }
}

bool TerrainData::loadHeightmap(const std::string& path,
                                std::vector<float>& out) const {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    out.resize(static_cast<std::size_t>(PIXELS) * PIXELS);
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(out.size() * sizeof(float)));
    if (f.gcount() != static_cast<std::streamsize>(out.size() * sizeof(float))) {
        out.clear();
        return false;
    }
    return true;
}

bool TerrainData::loadLandCover(const std::string& path,
                                std::vector<std::uint8_t>& out) const {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    out.resize(static_cast<std::size_t>(PIXELS) * PIXELS);
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(out.size()));
    if (f.gcount() != static_cast<std::streamsize>(out.size())) {
        out.clear();
        return false;
    }
    return true;
}
