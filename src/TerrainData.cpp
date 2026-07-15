#include "TerrainData.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

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

void TerrainData::load(const std::string& datasetRoot, double halfWindow) {
    if (!fs::exists(datasetRoot)) {
        throw std::runtime_error("dataset root not found: " + datasetRoot);
    }

    resolveStartPoint(datasetRoot);
    sceneOrigin_ = startWorld_;
    startPos_ = glm::vec3(0.0f); // camera start is the scene origin itself

    const double cx = startWorld_.x;
    const double cy = startWorld_.y;

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
                const std::string dir = tileDir(datasetRoot, lod, col, row);
                if (!fs::exists(dir)) continue;

                Tile t;
                t.lod = lod;
                t.col = col;
                t.row = row;
                t.resolution = res;
                t.extent = extent;
                t.originX = static_cast<double>(col) * extent;
                t.originY = static_cast<double>(row) * extent;

                if (!loadHeightmap(dir + "/terrain.hm32", t.heights)) continue;

                // Track the elevation range for the colour ramp.
                for (float h : t.heights) {
                    if (h <= NODATA + 1.0f) continue;
                    minElev_ = std::min(minElev_, h);
                    maxElev_ = std::max(maxElev_, h);
                }

                // Optional per-tile land cover (present only for AR50 exports).
                if (loadLandCover(dir + "/landcover.u8", t.landcover))
                    hasLandCover_ = true;

                // Railway track + road + building geometry intersecting this tile.
                parseTracksBin(dir + "/tracks.bin", t.tracks);
                parseRoadsBin(dir + "/roads.bin", t.roads);
                parseBuildingsBin(dir + "/buildings.bin", t.buildings);

                totalSamples += t.heights.size();
                tiles_.push_back(std::move(t));
            }
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
        p += 8; // skip trackId, trackType, medium, electrified, reserved
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

void TerrainData::resolveStartPoint(const std::string& datasetRoot) {
    // Default fallback: the station node itself.
    startWorld_ = glm::dvec3(kBodoSeedX, kBodoSeedY, kBodoSeedZ);
    startDir_ = glm::vec3(1.0f, 0.0f, 0.0f);

    // The Bodo LOD0 tile that contains the seed.
    const double extent0 = PIXELS * kLodResolution[0];
    const int col = static_cast<int>(std::floor(kBodoSeedX / extent0));
    const int row = static_cast<int>(std::floor(kBodoSeedY / extent0));
    const std::string path = tileDir(datasetRoot, 0, col, row) + "/tracks.bin";

    std::vector<TrackSegment> segs;
    if (!parseTracksBin(path, segs)) {
        std::printf("[TerrainData] tracks.bin missing/empty (%s); using station node\n",
                    path.c_str());
        return;
    }

    // Among main-line segments, the endpoint nearest the station seed is the
    // buffer-stop terminus ("track 1 end").
    double bestDist = std::numeric_limits<double>::max();
    glm::dvec3 bestEndpoint = startWorld_;
    glm::dvec3 bestInterior = startWorld_;
    bool found = false;
    for (const TrackSegment& seg : segs) {
        if (seg.trackType != 0) continue;
        const auto& v = seg.pts;
        struct End { glm::dvec3 pt, next; };
        const End ends[2] = {{v.front(), v[1]}, {v.back(), v[v.size() - 2]}};
        for (const End& e : ends) {
            const double dx = e.pt.x - kBodoSeedX;
            const double dy = e.pt.y - kBodoSeedY;
            const double d = std::sqrt(dx * dx + dy * dy);
            if (d < bestDist) {
                bestDist = d;
                bestEndpoint = e.pt;
                bestInterior = e.next;
                found = true;
            }
        }
    }

    if (found) {
        startWorld_ = bestEndpoint;
        glm::dvec2 dir(bestInterior.x - bestEndpoint.x,
                       bestInterior.y - bestEndpoint.y);
        const double len = glm::length(dir);
        if (len > 1e-6) {
            dir /= len;
            startDir_ = glm::vec3(static_cast<float>(dir.x),
                                  static_cast<float>(dir.y), 0.0f);
        }
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
