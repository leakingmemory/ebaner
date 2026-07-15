#include "TrackMesh.h"

#include "TrackPath.h"

#include <cstdio>
#include <vector>

namespace {

// --- Cross-section dimensions (metres) -------------------------------------
constexpr float kSleeperSpacing = 0.6f;  // tie pitch
constexpr int kSleepersPerChunk = 48;    // ~29 m runs for distance LOD
constexpr float kRailSampleStep = 3.0f;  // ballast/rail sweep step along the curve
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

// Sample distances along a path: 0, step, 2*step, ... , length (endpoint always
// included so the sweep closes exactly).
std::vector<float> sampleDistances(float length, float step) {
    std::vector<float> ss;
    for (float s = 0.0f; s < length; s += step) ss.push_back(s);
    ss.push_back(length);
    return ss;
}

} // namespace

void TrackMesh::build(const TerrainData& data) {
    const std::vector<TrackPath> paths = buildTrackPaths(data);

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
    // Point at centreline P, offset `across` along cross-track right R and `z`
    // along cross-track up U (both banked by cant).
    auto pt = [](const glm::vec3& P, const glm::vec3& R, const glm::vec3& U,
                 float across, float z) {
        return P + R * across + U * z;
    };

    // --- Pass 1: ballast bed + rails (always drawn) ------------------------
    for (const TrackPath& path : paths) {
        const std::vector<float> ss = sampleDistances(path.length(), kRailSampleStep);
        for (std::size_t i = 0; i + 1 < ss.size(); ++i) {
            const TrackPose pa = path.poseAt(ss[i]);
            const TrackPose pb = path.poseAt(ss[i + 1]);
            const glm::vec3 A = pa.pos, B = pb.pos;
            const glm::vec3 Ra = pa.right, Rb = pb.right;
            const glm::vec3 Ua = pa.up, Ub = pb.up;
            const glm::vec3 mid = (A + B) * 0.5f;
            const float vA = ss[i] / kSleeperSpacing;
            const float vB = ss[i + 1] / kSleeperSpacing;

            // Ballast top (textured with the ballast/sleeper layer).
            emitQuad(pt(A, Ra, Ua,-kBallastTopHalf, kBallastTopZ),
                     pt(A, Ra, Ua,kBallastTopHalf, kBallastTopZ),
                     pt(B, Rb, Ub,kBallastTopHalf, kBallastTopZ),
                     pt(B, Rb, Ub,-kBallastTopHalf, kBallastTopZ), mid, kBallastTint,
                     0.0f, glm::vec2(0.0f, vA), glm::vec2(1.0f, vA),
                     glm::vec2(1.0f, vB), glm::vec2(0.0f, vB));

            // Ballast side slopes.
            solidQuad(pt(A, Ra, Ua,-kBallastTopHalf, kBallastTopZ),
                      pt(A, Ra, Ua,-kBallastBotHalf, kBallastBotZ),
                      pt(B, Rb, Ub,-kBallastBotHalf, kBallastBotZ),
                      pt(B, Rb, Ub,-kBallastTopHalf, kBallastTopZ), mid, kBallastSide);
            solidQuad(pt(A, Ra, Ua,kBallastTopHalf, kBallastTopZ),
                      pt(A, Ra, Ua,kBallastBotHalf, kBallastBotZ),
                      pt(B, Rb, Ub,kBallastBotHalf, kBallastBotZ),
                      pt(B, Rb, Ub,kBallastTopHalf, kBallastTopZ), mid, kBallastSide);

            // Two rails: top + two vertical sides each.
            for (float sign : {-1.0f, 1.0f}) {
                const float rc = sign * kRailHalf;
                const glm::vec3 rmid =
                    (pt(A, Ra, Ua,rc, (kRailBotZ + kRailTopZ) * 0.5f) +
                     pt(B, Rb, Ub,rc, (kRailBotZ + kRailTopZ) * 0.5f)) *
                    0.5f;
                solidQuad(pt(A, Ra, Ua,rc - kRailHalfWidth, kRailTopZ),
                          pt(A, Ra, Ua,rc + kRailHalfWidth, kRailTopZ),
                          pt(B, Rb, Ub,rc + kRailHalfWidth, kRailTopZ),
                          pt(B, Rb, Ub,rc - kRailHalfWidth, kRailTopZ), rmid, kRailCol);
                solidQuad(pt(A, Ra, Ua,rc - kRailHalfWidth, kRailTopZ),
                          pt(A, Ra, Ua,rc - kRailHalfWidth, kRailBotZ),
                          pt(B, Rb, Ub,rc - kRailHalfWidth, kRailBotZ),
                          pt(B, Rb, Ub,rc - kRailHalfWidth, kRailTopZ), rmid, kRailCol);
                solidQuad(pt(A, Ra, Ua,rc + kRailHalfWidth, kRailTopZ),
                          pt(A, Ra, Ua,rc + kRailHalfWidth, kRailBotZ),
                          pt(B, Rb, Ub,rc + kRailHalfWidth, kRailBotZ),
                          pt(B, Rb, Ub,rc + kRailHalfWidth, kRailTopZ), rmid, kRailCol);
            }
        }
    }
    alwaysIndexCount_ = static_cast<std::uint32_t>(indices_.size());

    // --- Pass 2: sleeper boxes, grouped into distance-culled chunks --------
    auto emitSleeper = [&](const glm::vec3& P, const glm::vec3& R,
                           const glm::vec3& U, const glm::vec3& T) {
        const float L = kSleeperHalfLen, W = kSleeperHalfWid;
        const float zb = kSleeperBotZ, zt = kSleeperTopZ;
        const glm::vec3 inside = P + U * ((zb + zt) * 0.5f);
        auto s = [&](float a, float b, float z) {
            return P + R * a + T * b + U * z;
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

    for (const TrackPath& path : paths) {
        const float total = path.length();
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

        for (float dist = 0.0f; dist <= total + 1e-3f; dist += kSleeperSpacing) {
            const TrackPose p = path.poseAt(dist);
            emitSleeper(p.pos, p.right, p.up, p.tangent);
            accum += p.pos;
            ++accumN;
            if (accumN >= kSleepersPerChunk) flush();
        }
        flush(); // end of track
    }

    std::printf(
        "[TrackMesh] %zu tracks, %zu vertices, %zu triangles, %zu sleeper chunks\n",
        paths.size(), vertices_.size(), indices_.size() / 3, chunks_.size());
}
