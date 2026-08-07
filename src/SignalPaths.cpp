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

#include "SignalPaths.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <unordered_map>

namespace fs = std::filesystem;

namespace {
std::string pathsFile(const std::string& root) {
    return root + "/overlay/signal-paths.txt";
}

// Parse a `<trackHex>:<frac>` border token.
bool parseBorder(const std::string& tok, Border& b) {
    const auto c = tok.find(':');
    if (c == std::string::npos) return false;
    b.trackId = static_cast<std::uint32_t>(std::strtoul(tok.substr(0, c).c_str(), nullptr, 16));
    b.frac = std::atof(tok.substr(c + 1).c_str());
    return true;
}
} // namespace

bool borderMoveBreaksPath(const std::vector<SignalPath>& paths,
                          const std::vector<TrackPoly>& polys, std::uint32_t trackId,
                          double oldFrac, double newFrac, std::string& why) {
    const double tol = sameFracTol(polys, trackId);
    auto at = [&](std::uint32_t t, double f) {
        return t == trackId && std::abs(f - oldFrac) <= tol;
    };
    for (const SignalPath& p : paths)
        for (const SectionInterval& iv : p.parts) {
            const double from = at(iv.trackId, iv.from) ? newFrac : iv.from;
            const double to = at(iv.trackId, iv.to) ? newFrac : iv.to;
            if (from == iv.from && to == iv.to) continue; // untouched by this move
            const std::string nm = p.name.empty() ? "?" : p.name;
            if (std::abs(to - from) <= tol) {
                why = "would collapse route " + nm;
                return true;
            }
            // Direction must survive: a flipped sign means the border was dragged past a
            // junction this route uses, turning the route and its signal around.
            if ((to - from) * (iv.to - iv.from) < 0.0) {
                why = "would reverse route " + nm + " (past a junction it uses)";
                return true;
            }
        }
    return false;
}

int moveBorderFrac(std::vector<SignalPath>& paths, const std::vector<TrackPoly>& polys,
                   std::uint32_t trackId, double oldFrac, double newFrac) {
    const double tol = sameFracTol(polys, trackId);
    auto at = [&](std::uint32_t t, double f) {
        return t == trackId && std::abs(f - oldFrac) <= tol;
    };
    int n = 0;
    for (SignalPath& p : paths) {
        if (at(p.start.trackId, p.start.frac)) { p.start.frac = newFrac; ++n; }
        if (at(p.end.trackId, p.end.frac)) { p.end.frac = newFrac; ++n; }
        for (SectionInterval& iv : p.parts) {
            if (at(iv.trackId, iv.from)) { iv.from = newFrac; ++n; }
            if (at(iv.trackId, iv.to)) { iv.to = newFrac; ++n; }
        }
    }
    return n;
}

std::vector<SignalPath> loadSignalPaths(const std::string& datasetRoot) {
    std::vector<SignalPath> out;
    std::ifstream f(pathsFile(datasetRoot));
    if (!f) return out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string kind, name, startTok, endTok;
        SignalPath p;
        is >> kind >> p.id >> name >> startTok >> endTok;
        if (kind != "path" || startTok.empty() || endTok.empty()) continue;
        p.name = name;
        if (!parseBorder(startTok, p.start) || !parseBorder(endTok, p.end)) continue;
        std::string tok; // optional `via <border>` entries, then trackHex:from:to intervals
        while (is >> tok) {
            if (tok == "via") {
                std::string vt;
                Border vb;
                if ((is >> vt) && parseBorder(vt, vb)) p.vias.push_back(vb);
                continue;
            }
            const auto c1 = tok.find(':');
            const auto c2 = tok.rfind(':');
            if (c1 == std::string::npos || c2 == c1) continue;
            SectionInterval iv;
            iv.trackId = static_cast<std::uint32_t>(
                std::strtoul(tok.substr(0, c1).c_str(), nullptr, 16));
            iv.from = std::atof(tok.substr(c1 + 1, c2 - c1 - 1).c_str());
            iv.to = std::atof(tok.substr(c2 + 1).c_str());
            p.parts.push_back(iv);
        }
        if (!p.parts.empty()) out.push_back(std::move(p));
    }
    return out;
}

bool writeSignalPaths(const std::string& datasetRoot,
                      const std::vector<SignalPath>& paths) {
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(pathsFile(datasetRoot), std::ios::trunc);
    if (!f) return false;
    f << "# ebaner mini signal paths (directional routes, border -> border).\n"
         "# path <id> <name> <startTrackHex>:<frac> <endTrackHex>:<frac> "
         "[via <trackHex>:<frac>]... <trackHex>:<from>:<to> ...\n";
    for (const SignalPath& p : paths) {
        f << "path " << p.id << ' ' << (p.name.empty() ? "-" : p.name) << ' ' << std::hex
          << p.start.trackId << std::dec << ':' << p.start.frac << ' ' << std::hex
          << p.end.trackId << std::dec << ':' << p.end.frac;
        for (const Border& v : p.vias)
            f << " via " << std::hex << v.trackId << std::dec << ':' << v.frac;
        for (const SectionInterval& iv : p.parts)
            f << ' ' << std::hex << iv.trackId << std::dec << ':' << iv.from << ':' << iv.to;
        f << '\n';
    }
    return static_cast<bool>(f);
}

std::vector<SignalPlacement> signalPlacements(const std::vector<SignalPath>& paths,
                                              const std::vector<TrackPoly>& polys) {
    std::vector<SignalPlacement> out;
    std::unordered_map<std::string, int> seen; // dedupe key -> placement index
    for (std::size_t pi = 0; pi < paths.size(); ++pi) {
        const SignalPath& p = paths[pi];
        if (p.parts.empty()) continue;
        const SectionInterval& iv0 = p.parts.front();
        const glm::dvec3 a = fracToWorld(polys, iv0.trackId, iv0.from);
        // A small step toward `to` gives the travel direction leaving the border.
        const double f1 = iv0.from + (iv0.to - iv0.from) * 0.02;
        const glm::dvec3 b = fracToWorld(polys, iv0.trackId, f1);
        if (a.x == 0.0 && a.y == 0.0) continue; // stale/missing track
        glm::dvec2 fwd(b.x - a.x, b.y - a.y);
        const double L = glm::length(fwd);
        if (L < 1e-6) continue;
        fwd /= L;
        // Dedupe: same start border (trackId + frac) and same rough direction -> one signal.
        char key[64];
        std::snprintf(key, sizeof(key), "%x:%d:%d:%d", p.start.trackId,
                      static_cast<int>(std::lround(p.start.frac * 1000.0)),
                      static_cast<int>(std::lround(fwd.x * 8.0)),
                      static_cast<int>(std::lround(fwd.y * 8.0)));
        // Paths sharing a start + direction share one signal, which governs them all.
        const auto it = seen.find(key);
        if (it != seen.end()) {
            out[it->second].paths.push_back(static_cast<int>(pi));
            continue;
        }
        seen.emplace(key, static_cast<int>(out.size()));
        SignalPlacement sp;
        sp.world = a;
        sp.forward = fwd;
        sp.paths.push_back(static_cast<int>(pi));
        out.push_back(std::move(sp));
    }
    return out;
}

// ------------------------------------------------------------ aspect evaluation
namespace {
constexpr double kAtTurnout = 3.0; // m; how close a point must be to sit at a turnout
constexpr double kEdgeSlack = 0.25; // m; rounding margin at an interval's own ends

double planarDist(const glm::dvec3& a, const glm::dvec3& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}
} // namespace

std::vector<PathSwitch> pathSwitchRequirements(const SignalPath& p, const SwitchNetwork& net,
                                               const std::vector<TrackPoly>& polys) {
    std::vector<PathSwitch> reqs;
    const std::vector<Turnout>& tos = net.turnouts();
    for (std::size_t i = 0; i < tos.size(); ++i) {
        if (tos[i].mainPath < 0) continue; // inert crossing: nothing to set
        bool needSet = false;
        SwitchState need = SwitchState::Straight;
        // (a) The route crosses from one track to another at this turnout: diverging if
        // either side is the branch, else it runs straight over a joint here.
        for (std::size_t k = 0; k + 1 < p.parts.size(); ++k) {
            const glm::dvec3 w = fracToWorld(polys, p.parts[k].trackId, p.parts[k].to);
            if (w.x == 0.0 && w.y == 0.0) continue;
            if (planarDist(w, tos[i].world) > kAtTurnout) continue;
            need = (p.parts[k].trackId == tos[i].sidingTrack ||
                    p.parts[k + 1].trackId == tos[i].sidingTrack)
                       ? SwitchState::Diverging
                       : SwitchState::Straight;
            needSet = true;
        }
        // (b) Otherwise the route may run straight through the turnout inside one leg.
        if (!needSet) {
            for (const SectionInterval& iv : p.parts) {
                double frac = 0.0, dist = 0.0;
                if (!projectOnTrack(polys, iv.trackId,
                                    glm::dvec2(tos[i].world.x, tos[i].world.y), frac, dist))
                    continue;
                if (dist > kAtTurnout) continue; // this leg doesn't reach the turnout
                const double lo = std::min(iv.from, iv.to), hi = std::max(iv.from, iv.to);
                // Only an interior crossing counts, so a route that merely ends at the
                // turnout imposes no requirement. The margin has to stay well under a
                // border-to-turnout spacing (which can be ~1 m), so it is only wide
                // enough to absorb rounding, not to swallow a real crossing.
                const std::vector<glm::dvec3>* pts = nullptr;
                for (const TrackPoly& tp : polys)
                    if (tp.id == iv.trackId) { pts = &tp.pts; break; }
                const double len = pts ? polyLength(*pts) : 0.0;
                const double eps = len > 1.0 ? kEdgeSlack / len : 1e-6;
                if (frac > lo + eps && frac < hi - eps) {
                    need = SwitchState::Straight;
                    needSet = true;
                    break;
                }
            }
        }
        if (needSet) reqs.push_back({static_cast<int>(i), need});
    }
    return reqs;
}

bool pathSwitchesAligned(const SignalPath& p, const SwitchNetwork& net,
                         const std::vector<TrackPoly>& polys) {
    for (const PathSwitch& ps : pathSwitchRequirements(p, net, polys))
        if (net.state(ps.turnout) != ps.need) return false;
    return true;
}

std::vector<int> pathSections(const SignalPath& p, const TrackCircuits& circuits) {
    std::vector<int> ids;
    for (const Section& s : circuits.sections) {
        bool hit = false;
        for (const SectionInterval& si : s.parts) {
            for (const SectionInterval& pi : p.parts) {
                if (si.trackId != pi.trackId) continue;
                const double plo = std::min(pi.from, pi.to), phi = std::max(pi.from, pi.to);
                const double slo = std::min(si.from, si.to), shi = std::max(si.from, si.to);
                // Positive overlap only: merely touching at a shared border (the section
                // behind the signal) is not part of the route.
                if (std::min(phi, shi) - std::max(plo, slo) > 1e-6) { hit = true; break; }
            }
            if (hit) break;
        }
        if (hit) ids.push_back(s.id);
    }
    return ids;
}

bool updateSignalAspects(std::vector<SignalPlacement>& placements,
                         const std::vector<SignalPath>& paths, const SwitchNetwork& net,
                         const std::vector<TrackPoly>& polys,
                         const TrackCircuits& circuits,
                         const std::vector<char>& secOccupied,
                         const std::vector<char>& routeSet) {
    bool changed = false;
    for (SignalPlacement& sp : placements) {
        SignalAspect want = SignalAspect::Stop;
        for (int pi : sp.paths) {
            if (pi < 0 || pi >= static_cast<int>(paths.size())) continue;
            // A route set from this signal shows clear (it is dropped the moment a train
            // enters its circuits, so "set" already implies the road ahead is empty).
            if (pi < static_cast<int>(routeSet.size()) && routeSet[pi]) {
                want = SignalAspect::Clear;
                break;
            }
            const SignalPath& p = paths[pi];
            if (!pathSwitchesAligned(p, net, polys)) continue; // not this signal's route
            bool occupied = false;
            for (int id : pathSections(p, circuits))
                for (std::size_t si = 0; si < circuits.sections.size(); ++si)
                    if (circuits.sections[si].id == id && si < secOccupied.size() &&
                        secOccupied[si])
                        occupied = true;
            if (occupied) want = SignalAspect::TrainOnTrack; // keep looking for a set route
        }
        if (sp.aspect != want) { sp.aspect = want; changed = true; }
    }
    return changed;
}
