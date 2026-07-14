#include "TrackMesh.h"

#include "TerrainData.h"
#include "Textures.h" // landtex::BALLAST

#include <cstdio>
#include <unordered_set>
#include <vector>

namespace {

// --- Cross-section dimensions (metres) -------------------------------------
constexpr float kSleeperSpacing = 0.6f;  // tie pitch
constexpr int kSleepersPerChunk = 48;    // ~29 m runs for distance LOD
constexpr float kRailHalf = 0.7175f;     // half of 1.435 m standard gauge

// Ballast trapezoid.
constexpr float kBallastTopHalf = 2.0f;
constexpr float kBallastBotHalf = 2.8f;
constexpr float kBallastHeight = 0.65f; // tall enough that ties sit down in it
constexpr float kBallastSink = 0.1f;    // seat the bottom slightly into the DTM

// Sleeper box.
constexpr float kSleeperHalfLen = 1.3f;  // across the track (2.6 m)
constexpr float kSleeperHalfWid = 0.13f; // along the track (0.26 m)
constexpr float kSleeperHeight = 0.22f;
constexpr float kSleeperExposed = 0.06f; // how much of the tie stands proud

// Rail prism.
constexpr float kRailHalfWidth = 0.0375f; // 0.075 m
constexpr float kRailHeight = 0.15f;

// Vertical offsets from the rail-bed centreline z (which already carries the
// +0.6 m bed offset baked into the export). Sleepers are mostly submerged in the
// ballast; only ~kSleeperExposed of the tie shows above the bed, with the rails
// on top.
constexpr float kBallastBotZ = -kBallastSink;                 // -0.10
constexpr float kBallastTopZ = kBallastHeight - kBallastSink; //  0.55
constexpr float kSleeperTopZ = kBallastTopZ + kSleeperExposed;   // 0.61
constexpr float kSleeperBotZ = kSleeperTopZ - kSleeperHeight;    // 0.39
constexpr float kRailBotZ = kSleeperTopZ;                     //  0.61
constexpr float kRailTopZ = kRailBotZ + kRailHeight;          //  0.76

// Colours.
const glm::vec3 kBallastTint(1.0f, 1.0f, 1.0f); // multiplies the ballast texture
const glm::vec3 kBallastSide(0.42f, 0.40f, 0.36f);
const glm::vec3 kSleeperCol(0.58f, 0.58f, 0.56f); // concrete
const glm::vec3 kRailCol(0.40f, 0.25f, 0.18f);    // rusty steel

// Horizontal perpendicular at centreline vertex i (mitred: average of adjacent
// segment tangents).
glm::vec3 perpAt(const std::vector<glm::vec3>& c, int i) {
    const int n = static_cast<int>(c.size());
    glm::vec2 tang(0.0f);
    if (i > 0) tang += glm::vec2(c[i].x - c[i - 1].x, c[i].y - c[i - 1].y);
    if (i < n - 1) tang += glm::vec2(c[i + 1].x - c[i].x, c[i + 1].y - c[i].y);
    const float tl = glm::length(tang);
    const glm::vec2 p =
        (tl > 1e-6f) ? glm::vec2(-tang.y, tang.x) / tl : glm::vec2(1.0f, 0.0f);
    return glm::vec3(p.x, p.y, 0.0f);
}

} // namespace

void TrackMesh::build(const TerrainData& data) {
    const glm::dvec3 origin = data.sceneOrigin();
    std::unordered_set<std::uint32_t> seen;

    // Collect unique centrelines (scene-relative), deduped by trackId — a
    // through-track appears in full in every tile it crosses.
    std::vector<std::vector<glm::vec3>> lines;
    for (const Tile& t : data.tiles()) {
        for (const TrackSegment& seg : t.tracks) {
            if (!seen.insert(seg.trackId).second) continue;
            if (seg.pts.size() < 2) continue;
            std::vector<glm::vec3> c;
            c.reserve(seg.pts.size());
            for (const glm::dvec3& w : seg.pts)
                c.emplace_back(static_cast<float>(w.x - origin.x),
                               static_cast<float>(w.y - origin.y),
                               static_cast<float>(w.z - origin.z));
            lines.push_back(std::move(c));
        }
    }

    // Emit one quad (two triangles) with an outward-facing normal. The normal is
    // the geometric normal, flipped to point away from `inside` so lighting is
    // correct regardless of winding (the pipeline draws with culling off).
    auto emitQuad = [&](const glm::vec3& p0, const glm::vec3& p1,
                        const glm::vec3& p2, const glm::vec3& p3,
                        const glm::vec3& inside, const glm::vec3& color,
                        float texLayer, glm::vec2 uv0, glm::vec2 uv1,
                        glm::vec2 uv2, glm::vec2 uv3) {
        glm::vec3 n = glm::cross(p1 - p0, p3 - p0);
        const float nl = glm::length(n);
        n = (nl > 1e-12f) ? n / nl : glm::vec3(0.0f, 0.0f, 1.0f);
        const glm::vec3 cen = (p0 + p1 + p2 + p3) * 0.25f;
        if (glm::dot(n, cen - inside) < 0.0f) n = -n;
        const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
        vertices_.push_back({p0, n, color, uv0, texLayer});
        vertices_.push_back({p1, n, color, uv1, texLayer});
        vertices_.push_back({p2, n, color, uv2, texLayer});
        vertices_.push_back({p3, n, color, uv3, texLayer});
        indices_.push_back(base + 0);
        indices_.push_back(base + 1);
        indices_.push_back(base + 2);
        indices_.push_back(base + 0);
        indices_.push_back(base + 2);
        indices_.push_back(base + 3);
    };
    const glm::vec2 z2(0.0f);
    auto solidQuad = [&](const glm::vec3& p0, const glm::vec3& p1,
                         const glm::vec3& p2, const glm::vec3& p3,
                         const glm::vec3& inside, const glm::vec3& color) {
        emitQuad(p0, p1, p2, p3, inside, color, -1.0f, z2, z2, z2, z2);
    };
    // Point at centreline P, offset `across` along horizontal perp R and `z` up.
    auto pt = [](const glm::vec3& P, const glm::vec3& R, float across, float z) {
        return P + R * across + glm::vec3(0.0f, 0.0f, z);
    };

    // --- Pass 1: ballast bed + rails (always drawn) ------------------------
    for (const auto& c : lines) {
        const int n = static_cast<int>(c.size());
        std::vector<glm::vec3> perp(n);
        std::vector<float> cum(n, 0.0f);
        for (int i = 0; i < n; ++i) perp[i] = perpAt(c, i);
        for (int i = 1; i < n; ++i)
            cum[i] = cum[i - 1] + glm::distance(c[i], c[i - 1]);

        for (int i = 0; i + 1 < n; ++i) {
            const glm::vec3 A = c[i], B = c[i + 1];
            const glm::vec3 Ra = perp[i], Rb = perp[i + 1];
            const glm::vec3 mid = (A + B) * 0.5f;
            const float vA = cum[i] / kSleeperSpacing;
            const float vB = cum[i + 1] / kSleeperSpacing;

            // Ballast top (textured with the ballast/sleeper layer).
            emitQuad(pt(A, Ra, -kBallastTopHalf, kBallastTopZ),
                     pt(A, Ra, kBallastTopHalf, kBallastTopZ),
                     pt(B, Rb, kBallastTopHalf, kBallastTopZ),
                     pt(B, Rb, -kBallastTopHalf, kBallastTopZ), mid, kBallastTint,
                     static_cast<float>(landtex::BALLAST), glm::vec2(0.0f, vA),
                     glm::vec2(1.0f, vA), glm::vec2(1.0f, vB), glm::vec2(0.0f, vB));

            // Ballast side slopes.
            solidQuad(pt(A, Ra, -kBallastTopHalf, kBallastTopZ),
                      pt(A, Ra, -kBallastBotHalf, kBallastBotZ),
                      pt(B, Rb, -kBallastBotHalf, kBallastBotZ),
                      pt(B, Rb, -kBallastTopHalf, kBallastTopZ), mid, kBallastSide);
            solidQuad(pt(A, Ra, kBallastTopHalf, kBallastTopZ),
                      pt(A, Ra, kBallastBotHalf, kBallastBotZ),
                      pt(B, Rb, kBallastBotHalf, kBallastBotZ),
                      pt(B, Rb, kBallastTopHalf, kBallastTopZ), mid, kBallastSide);

            // Two rails: top + two vertical sides each.
            for (float sign : {-1.0f, 1.0f}) {
                const float rc = sign * kRailHalf;
                const glm::vec3 rmid =
                    (pt(A, Ra, rc, (kRailBotZ + kRailTopZ) * 0.5f) +
                     pt(B, Rb, rc, (kRailBotZ + kRailTopZ) * 0.5f)) *
                    0.5f;
                solidQuad(pt(A, Ra, rc - kRailHalfWidth, kRailTopZ),
                          pt(A, Ra, rc + kRailHalfWidth, kRailTopZ),
                          pt(B, Rb, rc + kRailHalfWidth, kRailTopZ),
                          pt(B, Rb, rc - kRailHalfWidth, kRailTopZ), rmid, kRailCol);
                solidQuad(pt(A, Ra, rc - kRailHalfWidth, kRailTopZ),
                          pt(A, Ra, rc - kRailHalfWidth, kRailBotZ),
                          pt(B, Rb, rc - kRailHalfWidth, kRailBotZ),
                          pt(B, Rb, rc - kRailHalfWidth, kRailTopZ), rmid, kRailCol);
                solidQuad(pt(A, Ra, rc + kRailHalfWidth, kRailTopZ),
                          pt(A, Ra, rc + kRailHalfWidth, kRailBotZ),
                          pt(B, Rb, rc + kRailHalfWidth, kRailBotZ),
                          pt(B, Rb, rc + kRailHalfWidth, kRailTopZ), rmid, kRailCol);
            }
        }
    }
    alwaysIndexCount_ = static_cast<std::uint32_t>(indices_.size());

    // --- Pass 2: sleeper boxes, grouped into distance-culled chunks --------
    auto emitSleeper = [&](const glm::vec3& P, const glm::vec3& R,
                           const glm::vec3& T) {
        const float L = kSleeperHalfLen, W = kSleeperHalfWid;
        const float zb = kSleeperBotZ, zt = kSleeperTopZ;
        const glm::vec3 inside = P + glm::vec3(0.0f, 0.0f, (zb + zt) * 0.5f);
        auto s = [&](float a, float b, float z) {
            return P + R * a + T * b + glm::vec3(0.0f, 0.0f, z);
        };
        solidQuad(s(-L, -W, zt), s(L, -W, zt), s(L, W, zt), s(-L, W, zt), inside,
                  kSleeperCol);
        solidQuad(s(L, -W, zt), s(L, -W, zb), s(L, W, zb), s(L, W, zt), inside,
                  kSleeperCol);
        solidQuad(s(-L, -W, zt), s(-L, -W, zb), s(-L, W, zb), s(-L, W, zt), inside,
                  kSleeperCol);
        solidQuad(s(-L, W, zt), s(-L, W, zb), s(L, W, zb), s(L, W, zt), inside,
                  kSleeperCol);
        solidQuad(s(-L, -W, zt), s(-L, -W, zb), s(L, -W, zb), s(L, -W, zt), inside,
                  kSleeperCol);
    };

    for (const auto& c : lines) {
        const int n = static_cast<int>(c.size());
        std::vector<float> cum(n, 0.0f);
        for (int i = 1; i < n; ++i)
            cum[i] = cum[i - 1] + glm::distance(c[i], c[i - 1]);
        const float total = cum[n - 1];

        std::uint32_t chunkFirst = static_cast<std::uint32_t>(indices_.size());
        glm::vec3 accum(0.0f);
        int accumN = 0;
        auto flush = [&]() {
            if (accumN == 0) return;
            TrackDrawChunk ch;
            ch.firstIndex = chunkFirst;
            ch.indexCount =
                static_cast<std::uint32_t>(indices_.size()) - chunkFirst;
            ch.centroid = accum / static_cast<float>(accumN);
            chunks_.push_back(ch);
            chunkFirst = static_cast<std::uint32_t>(indices_.size());
            accum = glm::vec3(0.0f);
            accumN = 0;
        };

        int j = 0;
        for (float dist = 0.0f; dist <= total + 1e-3f; dist += kSleeperSpacing) {
            while (j + 1 < n && cum[j + 1] < dist) ++j;
            if (j + 1 >= n) break;
            const float segLen = cum[j + 1] - cum[j];
            const float t = (segLen > 1e-6f) ? (dist - cum[j]) / segLen : 0.0f;
            const glm::vec3 P = glm::mix(c[j], c[j + 1], t);
            const glm::vec3 dir = c[j + 1] - c[j];
            const glm::vec2 h(dir.x, dir.y);
            const float hl = glm::length(h);
            const glm::vec3 T = (hl > 1e-6f) ? glm::vec3(h.x / hl, h.y / hl, 0.0f)
                                             : glm::vec3(1.0f, 0.0f, 0.0f);
            const glm::vec3 R(-T.y, T.x, 0.0f);
            emitSleeper(P, R, T);
            accum += P;
            ++accumN;
            if (accumN >= kSleepersPerChunk) flush();
        }
        flush(); // end of track
    }

    std::printf(
        "[TrackMesh] %zu tracks, %zu vertices, %zu triangles, %zu sleeper chunks\n",
        lines.size(), vertices_.size(), indices_.size() / 3, chunks_.size());
}
