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

std::unordered_map<std::uint32_t, std::vector<Conn>>
buildConns(const std::vector<TrackPoly>& polys) {
    std::unordered_map<std::uint32_t, std::vector<Conn>> conns;
    for (const TrackPoly& A : polys) {
        if (A.pts.size() < 2) continue;
        for (int endBit = 0; endBit < 2; ++endBit) {
            const glm::dvec3 e = endBit ? A.pts.back() : A.pts.front();
            const double aFrac = endBit ? 1.0 : 0.0;
            for (const TrackPoly& B : polys) {
                if (B.id == A.id || B.pts.size() < 2) continue;
                double bFrac, d;
                projFrac(B.pts, glm::dvec2(e.x, e.y), bFrac, d);
                if (d <= kJoinTol) {
                    conns[A.id].push_back({aFrac, B.id, bFrac});
                    conns[B.id].push_back({bFrac, A.id, aFrac});
                }
            }
        }
    }
    return conns;
}
} // namespace

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
