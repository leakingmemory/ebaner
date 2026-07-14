#include "TrackPath.h"

#include "TerrainData.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace {
constexpr int kSubStepsPerSpan = 20; // arc-length table resolution per span
} // namespace

TrackPath::TrackPath(std::uint32_t trackId, std::uint8_t trackType,
                     const std::vector<glm::vec3>& pts)
    : trackId_(trackId), trackType_(trackType) {
    const int n = static_cast<int>(pts.size());
    // Control points with reflected phantom endpoints, so span i interpolates
    // ctrl_[i+1]..ctrl_[i+2] using neighbours ctrl_[i] and ctrl_[i+3].
    ctrl_.reserve(n + 2);
    ctrl_.push_back(2.0f * pts.front() - pts[1]); // phantom before start
    for (const glm::vec3& p : pts) ctrl_.push_back(p);
    ctrl_.push_back(2.0f * pts.back() - pts[n - 2]); // phantom after end

    // Centripetal knots (alpha = 0.5).
    knot_.resize(ctrl_.size());
    knot_[0] = 0.0f;
    for (std::size_t i = 1; i < ctrl_.size(); ++i) {
        const float d = glm::distance(ctrl_[i], ctrl_[i - 1]);
        knot_[i] = knot_[i - 1] + std::sqrt(std::max(d, 1e-4f));
    }

    // Arc-length table: g = span + local-u in [0, numSpans]; s = cumulative length.
    const int numSpans = static_cast<int>(ctrl_.size()) - 3;
    table_.push_back({0.0f, 0.0f});
    glm::vec3 prev;
    eval(0, 0.0f, &prev, nullptr, nullptr);
    float s = 0.0f;
    for (int span = 0; span < numSpans; ++span) {
        for (int k = 1; k <= kSubStepsPerSpan; ++k) {
            const float u = static_cast<float>(k) / kSubStepsPerSpan;
            glm::vec3 p;
            eval(span, u, &p, nullptr, nullptr);
            s += glm::distance(p, prev);
            prev = p;
            table_.push_back({s, static_cast<float>(span) + u});
        }
    }
    length_ = table_.empty() ? 0.0f : table_.back().s;
}

void TrackPath::eval(int span, float u, glm::vec3* p, glm::vec3* d1,
                     glm::vec3* d2) const {
    const glm::vec3 P0 = ctrl_[span], P1 = ctrl_[span + 1];
    const glm::vec3 P2 = ctrl_[span + 2], P3 = ctrl_[span + 3];
    const float t0 = knot_[span], t1 = knot_[span + 1];
    const float t2 = knot_[span + 2], t3 = knot_[span + 3];

    // Non-uniform Catmull-Rom tangents (in global knot space) at P1 and P2.
    const glm::vec3 m1 =
        (P1 - P0) / (t1 - t0) - (P2 - P0) / (t2 - t0) + (P2 - P1) / (t2 - t1);
    const glm::vec3 m2 =
        (P2 - P1) / (t2 - t1) - (P3 - P1) / (t3 - t1) + (P3 - P2) / (t3 - t2);
    const float dt = t2 - t1;
    const glm::vec3 M1 = m1 * dt, M2 = m2 * dt; // scaled to local u in [0,1]

    const float u2 = u * u, u3 = u2 * u;
    if (p) {
        const float h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u;
        const float h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
        *p = h00 * P1 + h10 * M1 + h01 * P2 + h11 * M2;
    }
    if (d1) {
        const float g00 = 6 * u2 - 6 * u, g10 = 3 * u2 - 4 * u + 1;
        const float g01 = -6 * u2 + 6 * u, g11 = 3 * u2 - 2 * u;
        *d1 = g00 * P1 + g10 * M1 + g01 * P2 + g11 * M2;
    }
    if (d2) {
        const float k00 = 12 * u - 6, k10 = 6 * u - 4;
        const float k01 = -12 * u + 6, k11 = 6 * u - 2;
        *d2 = k00 * P1 + k10 * M1 + k01 * P2 + k11 * M2;
    }
}

TrackPose TrackPath::poseAt(float s) const {
    TrackPose pose;
    const int numSpans = static_cast<int>(ctrl_.size()) - 3;
    if (numSpans < 1 || table_.size() < 2 || length_ <= 0.0f) {
        pose.pos = ctrl_.size() > 1 ? ctrl_[1] : glm::vec3(0.0f);
        pose.tangent = glm::vec3(1.0f, 0.0f, 0.0f);
        pose.right = glm::vec3(0.0f, 1.0f, 0.0f);
        pose.curvature = 0.0f;
        return pose;
    }

    s = std::clamp(s, 0.0f, length_);
    // Binary search the arc-length table for the bracket around s.
    int lo = 0, hi = static_cast<int>(table_.size()) - 1;
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        if (table_[mid].s <= s)
            lo = mid;
        else
            hi = mid;
    }
    const Sample& a = table_[lo];
    const Sample& b = table_[hi];
    const float frac = (b.s > a.s) ? (s - a.s) / (b.s - a.s) : 0.0f;
    const float g = a.g + frac * (b.g - a.g);
    const int span = std::clamp(static_cast<int>(std::floor(g)), 0, numSpans - 1);
    const float u = std::clamp(g - static_cast<float>(span), 0.0f, 1.0f);

    glm::vec3 P, D1, D2;
    eval(span, u, &P, &D1, &D2);
    pose.pos = P;

    const float tl = glm::length(D1);
    pose.tangent = (tl > 1e-8f) ? D1 / tl : glm::vec3(1.0f, 0.0f, 0.0f);

    const glm::vec2 th(pose.tangent.x, pose.tangent.y);
    const float thl = glm::length(th);
    const glm::vec2 r =
        (thl > 1e-8f) ? glm::vec2(-th.y, th.x) / thl : glm::vec2(0.0f, 1.0f);
    pose.right = glm::vec3(r.x, r.y, 0.0f);

    // Horizontal signed curvature (x'y'' - y'x'') / |xy'|^3.
    const float xp = D1.x, yp = D1.y, xpp = D2.x, ypp = D2.y;
    const float denom = std::pow(xp * xp + yp * yp, 1.5f);
    pose.curvature = (denom > 1e-8f) ? (xp * ypp - yp * xpp) / denom : 0.0f;
    return pose;
}

std::vector<TrackPath> buildTrackPaths(const TerrainData& data) {
    const glm::dvec3 origin = data.sceneOrigin();
    std::unordered_set<std::uint32_t> seen;
    std::vector<TrackPath> paths;
    for (const Tile& t : data.tiles()) {
        for (const TrackSegment& seg : t.tracks) {
            if (!seen.insert(seg.trackId).second) continue; // one per through-track
            if (seg.pts.size() < 2) continue;
            std::vector<glm::vec3> pts;
            pts.reserve(seg.pts.size());
            for (const glm::dvec3& w : seg.pts) {
                const glm::vec3 p(static_cast<float>(w.x - origin.x),
                                  static_cast<float>(w.y - origin.y),
                                  static_cast<float>(w.z - origin.z));
                if (pts.empty() || glm::distance(pts.back(), p) > 1e-3f)
                    pts.push_back(p); // drop coincident points
            }
            if (pts.size() < 2) continue;
            paths.emplace_back(seg.trackId, seg.trackType, pts);
        }
    }
    return paths;
}
