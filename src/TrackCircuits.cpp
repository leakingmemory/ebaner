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

#include "TrackCircuits.h"

#include "TrackOverlay.h" // kRailIdBase (editor-added slip connectors)

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace fs = std::filesystem;

namespace {
constexpr double kJoinTol = 3.0; // m; track ends this close share a node (see buildTrackPaths)

std::string circuitsFile(const std::string& root) {
    return root + "/overlay/track-circuits.txt";
}

const std::vector<glm::dvec3>* findPoly(const std::vector<TrackPoly>& polys,
                                        std::uint32_t id) {
    for (const TrackPoly& p : polys)
        if (p.id == id) return &p.pts;
    return nullptr;
}
} // namespace

double polyLength(const std::vector<glm::dvec3>& pts) {
    double L = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i)
        L += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    return L;
}

glm::dvec3 fracToWorld(const std::vector<TrackPoly>& polys, std::uint32_t trackId,
                       double frac) {
    const std::vector<glm::dvec3>* pts = findPoly(polys, trackId);
    if (!pts || pts->size() < 2) return glm::dvec3(0.0);
    const double L = polyLength(*pts);
    const double target = std::clamp(frac, 0.0, 1.0) * L;
    double acc = 0.0;
    for (std::size_t i = 1; i < pts->size(); ++i) {
        const double seg = std::hypot((*pts)[i].x - (*pts)[i - 1].x,
                                      (*pts)[i].y - (*pts)[i - 1].y);
        if (acc + seg >= target || i + 1 == pts->size()) {
            const double t = seg > 1e-9 ? (target - acc) / seg : 0.0;
            return glm::mix((*pts)[i - 1], (*pts)[i], std::clamp(t, 0.0, 1.0));
        }
        acc += seg;
    }
    return pts->back();
}

// --------------------------------------------------------------------------- IO
TrackCircuits loadTrackCircuits(const std::string& datasetRoot) {
    TrackCircuits tc;
    std::ifstream f(circuitsFile(datasetRoot));
    if (!f) return tc;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string kind;
        is >> kind;
        if (kind == "border") {
            Border b;
            std::string idhex;
            is >> idhex >> b.frac; // trailing x/y (cached) ignored
            if (idhex.empty()) continue;
            b.trackId = static_cast<std::uint32_t>(std::strtoul(idhex.c_str(), nullptr, 16));
            tc.borders.push_back(b);
        } else if (kind == "section") {
            Section s;
            is >> s.id >> s.name;
            std::string tok;
            while (is >> tok) {
                // trackIdHex:from:to
                const auto c1 = tok.find(':');
                const auto c2 = tok.rfind(':');
                if (c1 == std::string::npos || c2 == c1) continue;
                SectionInterval iv;
                iv.trackId = static_cast<std::uint32_t>(
                    std::strtoul(tok.substr(0, c1).c_str(), nullptr, 16));
                iv.from = std::atof(tok.substr(c1 + 1, c2 - c1 - 1).c_str());
                iv.to = std::atof(tok.substr(c2 + 1).c_str());
                s.parts.push_back(iv);
            }
            if (!s.parts.empty()) tc.sections.push_back(s);
        }
    }
    return tc;
}

bool writeTrackCircuits(const std::string& datasetRoot, const TrackCircuits& tc) {
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(circuitsFile(datasetRoot), std::ios::trunc);
    if (!f) return false;
    f << "# ebaner track-circuit sections (train-present sensing).\n"
         "# border  <trackIdHex> <frac> <x> <y>\n"
         "# section <id> <name> <trackIdHex>:<from>:<to> ...\n";
    for (const Border& b : tc.borders)
        f << "border " << std::hex << b.trackId << std::dec << ' ' << b.frac << " 0 0\n";
    for (const Section& s : tc.sections) {
        f << "section " << s.id << ' ' << (s.name.empty() ? "-" : s.name);
        for (const SectionInterval& iv : s.parts)
            f << ' ' << std::hex << iv.trackId << std::dec << ':' << iv.from << ':' << iv.to;
        f << '\n';
    }
    return static_cast<bool>(f);
}

// ------------------------------------------------------------------ flood-fill
namespace {
// Nearest point on a polyline to p: fills the arc-length fraction and distance.
void projFrac(const std::vector<glm::dvec3>& pts, glm::dvec2 p, double& frac, double& dist) {
    dist = 1e30; frac = 0.0;
    const double total = polyLength(pts);
    double acc = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const glm::dvec2 a(pts[i - 1].x, pts[i - 1].y), b(pts[i].x, pts[i].y);
        const glm::dvec2 ab = b - a;
        const double L2 = glm::dot(ab, ab), seg = std::sqrt(L2);
        const double t = L2 > 1e-9 ? std::clamp(glm::dot(p - a, ab) / L2, 0.0, 1.0) : 0.0;
        const double d = glm::length(p - (a + ab * t));
        if (d < dist) { dist = d; frac = total > 1e-9 ? (acc + seg * t) / total : 0.0; }
        acc += seg;
    }
}

// A connection on a track: at `here` (frac) it joins `other` at that track's `there`
// frac. Junctions are where a track *endpoint* meets another track (its endpoint, or
// its interior = a turnout); a plain crossing (interior x interior) is NOT a junction
// (a train can't transfer, so the circuits don't connect).
struct Conn { double here; std::uint32_t other; double there; };

// Unit direction leading away from one end of a track (into its own body).
glm::dvec2 endDir(const TrackPoly& t, bool back) {
    const glm::dvec3& tip = back ? t.pts.back() : t.pts.front();
    const glm::dvec3& nb = back ? t.pts[t.pts.size() - 2] : t.pts[1];
    const glm::dvec2 d(nb.x - tip.x, nb.y - tip.y);
    const double L = glm::length(d);
    return L > 1e-9 ? d / L : glm::dvec2(1.0, 0.0);
}

std::unordered_map<std::uint32_t, std::vector<Conn>>
buildConns(const std::vector<TrackPoly>& polys) {
    std::unordered_map<std::uint32_t, std::vector<Conn>> conns;
    for (const TrackPoly& A : polys) {
        if (A.pts.size() < 2) continue;
        for (int endBit = 0; endBit < 2; ++endBit) {
            const glm::dvec3 e = endBit ? A.pts.back() : A.pts.front();
            const double aFrac = endBit ? 1.0 : 0.0;
            // Does another track carry straight on from this end? Then A does not stop
            // here: A and that track are one road, and anything else running through the
            // point is *crossed* (a diamond), not joined. A train can only change roads
            // there via a slip connector, so no junction is recorded onto the crossed
            // track - otherwise routes could be built straight across the diamond.
            const glm::dvec2 dirA = endDir(A, endBit != 0);
            bool continues = false;
            for (const TrackPoly& C : polys) {
                if (C.id == A.id || C.pts.size() < 2) continue;
                for (int cb = 0; cb < 2 && !continues; ++cb) {
                    const glm::dvec3& ce = cb ? C.pts.back() : C.pts.front();
                    if (std::hypot(ce.x - e.x, ce.y - e.y) > kJoinTol) continue;
                    if (glm::dot(dirA, endDir(C, cb != 0)) < -0.95) continues = true;
                }
                if (continues) break;
            }
            for (const TrackPoly& B : polys) {
                if (B.id == A.id || B.pts.size() < 2) continue;
                double bFrac, d;
                projFrac(B.pts, glm::dvec2(e.x, e.y), bFrac, d);
                if (d > kJoinTol) continue;
                if (continues) {
                    // Only the road A continues into may be joined here; a track merely
                    // passing through is crossed.
                    const double bTol = sameFracTol(polys, B.id);
                    const bool bEndsHere = bFrac <= bTol || bFrac >= 1.0 - bTol;
                    if (!bEndsHere) continue;
                }
                conns[A.id].push_back({aFrac, B.id, bFrac});
                conns[B.id].push_back({bFrac, A.id, aFrac});
            }
        }
    }
    return conns;
}
} // namespace

double sameFracTol(const std::vector<TrackPoly>& polys, std::uint32_t trackId) {
    // ~0.25 m expressed as a fraction: well below the spacing between distinct borders,
    // and comfortably above the rounding of the overlay file's 6 significant digits.
    constexpr double kNearM = 0.25;
    const std::vector<glm::dvec3>* pts = findPoly(polys, trackId);
    const double L = pts ? polyLength(*pts) : 0.0;
    return L > 1.0 ? kNearM / L : 1e-6;
}

bool canMoveBorder(const TrackCircuits& tc, const std::vector<TrackPoly>& polys,
                   int borderIdx, std::uint32_t newTrack, double newFrac, std::string& why) {
    if (borderIdx < 0 || borderIdx >= static_cast<int>(tc.borders.size())) {
        why = "no border selected";
        return false;
    }
    const Border& b = tc.borders[borderIdx];
    if (newTrack != b.trackId) {
        why = "a border can only move along its own track";
        return false;
    }
    const double tol = sameFracTol(polys, b.trackId);
    if (std::abs(newFrac - b.frac) <= tol) {
        why = "already there";
        return false;
    }
    // The nearest other border on this track, on the side we are moving toward, bounds the
    // move: crossing it would invert the section between them.
    const bool up = newFrac > b.frac;
    for (std::size_t i = 0; i < tc.borders.size(); ++i) {
        if (static_cast<int>(i) == borderIdx || tc.borders[i].trackId != b.trackId) continue;
        const double o = tc.borders[i].frac;
        if (up ? (o > b.frac && newFrac >= o - tol) : (o < b.frac && newFrac <= o + tol)) {
            why = "would cross the next border on this track";
            return false;
        }
    }
    return true;
}

int moveBorderFrac(TrackCircuits& tc, const std::vector<TrackPoly>& polys,
                   std::uint32_t trackId, double oldFrac, double newFrac) {
    const double tol = sameFracTol(polys, trackId);
    auto at = [&](std::uint32_t t, double f) {
        return t == trackId && std::abs(f - oldFrac) <= tol;
    };
    int n = 0;
    for (Border& b : tc.borders)
        if (at(b.trackId, b.frac)) { b.frac = newFrac; ++n; }
    for (Section& s : tc.sections)
        for (SectionInterval& iv : s.parts) {
            if (at(iv.trackId, iv.from)) { iv.from = newFrac; ++n; }
            if (at(iv.trackId, iv.to)) { iv.to = newFrac; ++n; }
        }
    return n;
}

bool projectOnTrack(const std::vector<TrackPoly>& polys, std::uint32_t trackId,
                    glm::dvec2 p, double& frac, double& dist) {
    const std::vector<glm::dvec3>* pts = findPoly(polys, trackId);
    if (!pts || pts->size() < 2) return false;
    projFrac(*pts, p, frac, dist);
    return true;
}

SectionResult floodSection(const std::vector<TrackPoly>& polys,
                           const std::vector<Border>& borders,
                           std::uint32_t seedTrack, double seedFrac) {
    SectionResult r;
    std::unordered_map<std::uint32_t, int> idx;
    for (int i = 0; i < static_cast<int>(polys.size()); ++i) idx[polys[i].id] = i;
    if (idx.find(seedTrack) == idx.end()) return r;

    std::unordered_map<std::uint32_t, std::vector<double>> bfr; // sorted border fracs
    for (const Border& b : borders) bfr[b.trackId].push_back(b.frac);
    for (auto& kv : bfr) std::sort(kv.second.begin(), kv.second.end());
    const auto conns = buildConns(polys);

    // The border-bounded interval [lo,hi] of `track` containing `frac`.
    auto intervalAt = [&](std::uint32_t track, double frac,
                          double& lo, double& hi, bool& loBorder, bool& hiBorder) {
        lo = 0.0; hi = 1.0; loBorder = hiBorder = false;
        auto it = bfr.find(track);
        if (it == bfr.end()) return;
        for (double bf : it->second) {
            if (bf <= frac && bf >= lo) { lo = bf; loBorder = true; }
            if (bf >= frac && bf <= hi) { hi = bf; hiBorder = true; }
        }
    };

    std::deque<std::pair<std::uint32_t, double>> q;
    q.push_back({seedTrack, seedFrac});
    std::vector<char> done(polys.size(), 0); // one interval per track reached (v1)

    while (!q.empty()) {
        auto [track, frac] = q.front(); q.pop_front();
        const int ti = idx[track];
        if (done[ti]) continue;
        double lo, hi; bool loB, hiB;
        intervalAt(track, frac, lo, hi, loB, hiB);
        done[ti] = 1;
        r.parts.push_back({track, lo, hi});
        r.lengthM += polyLength(polys[ti].pts) * (hi - lo);

        // Junctions anywhere within [lo,hi] cross into the connected track.
        bool loJoin = false, hiJoin = false;
        auto ci = conns.find(track);
        if (ci != conns.end())
            for (const Conn& c : ci->second) {
                if (c.here < lo - 1e-6 || c.here > hi + 1e-6) continue;
                if (std::abs(c.here - lo) < 1e-6) loJoin = true;
                if (std::abs(c.here - hi) < 1e-6) hiJoin = true;
                auto oi = idx.find(c.other);
                if (oi != idx.end() && !done[oi->second]) {
                    // nudge inward so intervalAt lands inside the neighbour's interval.
                    const double f = c.there <= 1e-6 ? 1e-6
                                     : c.there >= 1.0 - 1e-6 ? 1.0 - 1e-6 : c.there;
                    q.push_back({c.other, f});
                }
            }

        // Classify each interval end: a border is a boundary; a bare track terminus
        // with no junction is a real dead end (buffer).
        if (loB) ++r.borderEnds; else if (lo <= 1e-6 && !loJoin) ++r.deadEnds;
        if (hiB) ++r.borderEnds; else if (hi >= 1.0 - 1e-6 && !hiJoin) ++r.deadEnds;
    }
    r.enclosed = r.borderEnds >= 1;
    return r;
}

// ------------------------------------------------------------- signal routing
namespace {
// World unit tangent at `frac` along `track`, pointing toward +frac when dir=+1.
glm::dvec2 tangentAt(const std::vector<TrackPoly>& polys, std::uint32_t track,
                     double frac, int dir) {
    constexpr double e = 1e-3;
    const glm::dvec3 a = fracToWorld(polys, track, std::clamp(frac - e, 0.0, 1.0));
    const glm::dvec3 b = fracToWorld(polys, track, std::clamp(frac + e, 0.0, 1.0));
    glm::dvec2 t(b.x - a.x, b.y - a.y);
    const double L = glm::length(t);
    t = L > 1e-9 ? t / L : glm::dvec2(1.0, 0.0);
    return t * static_cast<double>(dir);
}

struct RouteCtx {
    const std::vector<TrackPoly>* polys;
    const std::unordered_map<std::uint32_t, std::vector<Conn>>* conns;
    std::uint32_t endTrack;
    double endFrac;
    std::vector<std::vector<SectionInterval>>* results; // capped at 2
    int cap = 2;   // routes to collect before de-duplication
    int budget; // node-expansion budget; guards against combinatorial blow-up
};

constexpr int kRouteRawCap = 16; // raw walks collected before de-duplication
constexpr int kRouteMaxTracks = 8; // a "mini" path spans few tracks; bounds the search

// Walk `track` from `entryFrac` toward `dir`; at each junction ahead either finish (if the
// destination lies ahead on the end track) or cross onto a connected track, but only via
// forward (non-reversing) moves. Records one directed interval per track traversed.
void routeDFS(RouteCtx& ctx, std::uint32_t track, double entryFrac, int dir,
              std::vector<std::uint32_t>& visited, std::vector<SectionInterval>& route,
              int depth) {
    if (ctx.results->size() >= (size_t)ctx.cap || depth > kRouteMaxTracks || --ctx.budget < 0) return;
    // Finish: destination on this track, ahead in the travel direction.
    if (track == ctx.endTrack && dir * (ctx.endFrac - entryFrac) > 1e-6) {
        route.push_back({track, entryFrac, ctx.endFrac});
        ctx.results->push_back(route);
        route.pop_back();
        // fall through: a junction before the destination may yield another route
    }
    const auto it = ctx.conns->find(track);
    if (it == ctx.conns->end()) return;
    for (const Conn& c : it->second) {
        if (dir * (c.here - entryFrac) <= 1e-6) continue; // not ahead of where we are
        if (track == ctx.endTrack && dir * (c.here - ctx.endFrac) > 1e-6)
            continue; // past the destination border on the end track
        if (std::find(visited.begin(), visited.end(), c.other) != visited.end())
            continue; // simple path (no track revisit)
        // Forward continuation onto `other` at `there`: pick the direction aligned with our
        // heading at the junction; reject if it would reverse (illegal turn at a turnout).
        const glm::dvec2 headIn = tangentAt(*ctx.polys, track, c.here, dir);
        const glm::dvec2 tPlus = tangentAt(*ctx.polys, c.other, c.there, +1);
        const int ndir = glm::dot(tPlus, headIn) >= 0.0 ? +1 : -1;
        const glm::dvec2 headOut = tangentAt(*ctx.polys, c.other, c.there, ndir);
        if (glm::dot(headOut, headIn) <= 0.0) continue; // reversing -> not a valid move
        route.push_back({track, entryFrac, c.here});
        visited.push_back(c.other);
        routeDFS(ctx, c.other, c.there, ndir, visited, route, depth + 1);
        visited.pop_back();
        route.pop_back();
        if (ctx.results->size() >= (size_t)ctx.cap) return;
    }
}
} // namespace

int findSignalRoute(const std::vector<TrackPoly>& polys, const Border& start,
                    const Border& end, std::vector<SectionInterval>& out) {
    out.clear();
    const auto conns = buildConns(polys);
    // Collect the raw walks, then fold away the ones that are the same road. Where two
    // track ends nearly coincide, buildConns records that single node twice (each end
    // projected onto the other track), so the search can hop across at either record and
    // report one physical road as several routes differing by a few metres of stub.
    std::vector<std::vector<SectionInterval>> raw;
    RouteCtx ctx{&polys, &conns, end.trackId, end.frac, &raw, kRouteRawCap, 500000};
    for (int dir = 1; dir >= -1; dir -= 2) { // leave the start border either way
        std::vector<std::uint32_t> visited{start.trackId};
        std::vector<SectionInterval> route;
        routeDFS(ctx, start.trackId, start.frac, dir, visited, route, 0);
        if (static_cast<int>(raw.size()) >= kRouteRawCap) break;
    }
    // A route's identity is the *real* tracks it runs along, in order. Two things are
    // deliberately not part of it:
    //   - intervals shorter than the join tolerance: those are the coincident-node stubs
    //     above, and say nothing about the road taken;
    //   - editor-added slip connectors (id >= kRailIdBase): these are the short links that
    //     let a train transfer through a diamond, not roads in their own right. The same
    //     transfer can be walked either over the connector or across the direct junction
    //     beside it, which is one road described two ways.
    auto signature = [&](const std::vector<SectionInterval>& r) {
        std::vector<std::uint32_t> sig;
        for (const SectionInterval& iv : r) {
            if (iv.trackId >= kRailIdBase) continue; // a slip link, not a road
            const std::vector<glm::dvec3>* pts = findPoly(polys, iv.trackId);
            const double m = pts ? polyLength(*pts) * std::abs(iv.to - iv.from) : 0.0;
            if (m > kJoinTol && (sig.empty() || sig.back() != iv.trackId))
                sig.push_back(iv.trackId);
        }
        return sig;
    };
    std::vector<std::vector<std::uint32_t>> seen;
    std::vector<std::vector<SectionInterval>> distinct;
    for (const auto& r : raw) {
        const std::vector<std::uint32_t> sig = signature(r);
        if (std::find(seen.begin(), seen.end(), sig) != seen.end()) continue;
        seen.push_back(sig);
        distinct.push_back(r);
    }
    if (distinct.size() == 1) out = distinct[0];
    return static_cast<int>(distinct.size());
}

