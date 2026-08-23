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
std::string exitsFile(const std::string& root) {
    return root + "/overlay/exit-signals.txt";
}

std::string pathsFile(const std::string& root) {
    return root + "/overlay/signal-paths.txt";
}

std::string exitRoutesFile(const std::string& root) {
    return root + "/overlay/exit-routes.txt";
}

std::string entriesFile(const std::string& root) {
    return root + "/overlay/entry-signals.txt";
}

std::string entryApproachesFile(const std::string& root) {
    return root + "/overlay/entry-approaches.txt";
}

std::string distantsFile(const std::string& root) {
    return root + "/overlay/distant-signals.txt";
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

namespace {
// Mini signal paths and exit signals hold the same thing - a directional route between two
// borders - so they share one grammar, differing only in file name and record keyword.
std::vector<SignalPath> loadRoutes(const std::string& file, const char* keyword) {
    std::vector<SignalPath> out;
    std::ifstream f(file);
    if (!f) return out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string kind, startTok, endTok;
        SignalPath p;
        is >> kind >> p.id;
        if (kind != keyword) continue;
        readName(is, p.name);
        is >> startTok >> endTok;
        if (startTok.empty() || endTok.empty()) continue;
        if (!parseBorder(startTok, p.start) || !parseBorder(endTok, p.end)) continue;
        // Optional keyword entries (`via`, `signal`, `type`), then the trackHex:from:to
        // intervals. Each keyword was added after the fact and is skipped harmlessly by a
        // reader that predates it, since neither it nor its argument parses as an interval.
        std::string tok;
        while (is >> tok) {
            if (tok == "via") {
                std::string vt;
                Border vb;
                if ((is >> vt) && parseBorder(vt, vb)) p.vias.push_back(vb);
                continue;
            }
            if (tok == "signal") { // exit routes: the exit signal this route leads to
                is >> p.exitId;
                continue;
            }
            if (tok == "type") { // exit routes: the authority a set route grants
                std::string tt;
                if (is >> tt) p.type = tt == "C2" ? RouteType::C2 : RouteType::C1;
                continue;
            }
            if (tok == "distant") { // main signals: a distant hangs on this mast
                p.distant = true;
                continue;
            }
            if (tok == "twolamp") { // main signals: red over green, no second green
                p.twoLamp = true;
                continue;
            }
            if (tok == "left") { // the mast stands left of the way it is read from
                p.side = -1;
                continue;
            }
            if (tok == "right") { // ...which is the default, so this is only ever explicit
                p.side = 1;
                continue;
            }
            if (tok == "station") { // which place works this route, overruling geometry
                readName(is, p.station);
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

// `withType` writes the C1/C2 token: a main-signal route always carries one, while a mini
// path has no such thing to say. `withMastFlags` is the same for the keywords that describe
// how a mast is *built* - the distant hanging on it and the two-lamp head - which only a
// main *signal* can have: not a mini path, and not an exit route, whose start is inside the
// station where no mast stands.
//
// `withSide` is a third grouping again, and deliberately not the same as either: which side
// of the track a post stands on is something a dwarf's mini path has to say as much as a
// main signal does, and something a road *leading up to* a mast has no business saying at
// all. Written only when the answer is left, so every file that means right - which is all
// of them today - comes back out of a round trip exactly as it went in.
bool writeRoutes(const std::string& datasetRoot, const std::string& file,
                 const char* keyword, const char* header,
                 const std::vector<SignalPath>& routes, bool withType = false,
                 bool withMastFlags = false, bool withSide = false) {
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(file, std::ios::trunc);
    if (!f) return false;
    f << header << "# " << keyword
      << " <id> \"<name>\" <startTrackHex>:<frac> <endTrackHex>:<frac> "
         "[signal <exitId> type <C1|C2>] "
      << (withMastFlags ? "[distant] [twolamp] " : "")
      << (withSide ? "[left] " : "")
      << "[station \"<name>\"] [via <trackHex>:<frac>]... "
         "<trackHex>:<from>:<to> ...\n";
    for (const SignalPath& p : routes) {
        f << keyword << ' ' << p.id << ' ' << quoteName(p.name) << ' '
          << std::hex << p.start.trackId << std::dec << ':' << p.start.frac << ' '
          << std::hex << p.end.trackId << std::dec << ':' << p.end.frac;
        // An exit route names the signal it leads to; every main-signal route states its
        // type outright, so the file stays self-documenting rather than leaning on the
        // C1 default.
        if (p.exitId != 0) f << " signal " << p.exitId;
        if (withType) f << " type " << (p.type == RouteType::C2 ? "C2" : "C1");
        if (withMastFlags && p.distant) f << " distant";
        if (withMastFlags && p.twoLamp) f << " twolamp";
        if (withSide && p.side < 0) f << " left";
        if (!p.station.empty()) f << " station " << quoteName(p.station);
        for (const Border& v : p.vias)
            f << " via " << std::hex << v.trackId << std::dec << ':' << v.frac;
        for (const SectionInterval& iv : p.parts)
            f << ' ' << std::hex << iv.trackId << std::dec << ':' << iv.from << ':' << iv.to;
        f << '\n';
    }
    return static_cast<bool>(f);
}
} // namespace

std::vector<SignalPath> loadSignalPaths(const std::string& datasetRoot) {
    return loadRoutes(pathsFile(datasetRoot), "path");
}

bool writeSignalPaths(const std::string& datasetRoot,
                      const std::vector<SignalPath>& paths) {
    return writeRoutes(datasetRoot, pathsFile(datasetRoot), "path",
                       "# ebaner mini signal paths (directional routes, border -> border).\n",
                       paths, /*withType=*/false, /*withMastFlags=*/false,
                       /*withSide=*/true);
}

std::vector<SignalPath> loadExitRoutes(const std::string& datasetRoot) {
    return loadRoutes(exitRoutesFile(datasetRoot), "route");
}

bool writeExitRoutes(const std::string& datasetRoot,
                     const std::vector<SignalPath>& routes) {
    return writeRoutes(datasetRoot, exitRoutesFile(datasetRoot), "route",
                       "# ebaner exit routes: the authority to move from a border inside the\n"
                       "# station up to an exit signal, which the signal then displays.\n",
                       routes, true);
}

std::vector<SignalPath> loadEntryApproaches(const std::string& datasetRoot) {
    return loadRoutes(entryApproachesFile(datasetRoot), "approach");
}

bool writeEntryApproaches(const std::string& datasetRoot,
                          const std::vector<SignalPath>& approaches) {
    return writeRoutes(datasetRoot, entryApproachesFile(datasetRoot), "approach",
                       "# ebaner entry approaches: the road leading up to an entry signal,\n"
                       "# which is what an exit route is on the other side of the station.\n"
                       "# It names no signal - it ends on the mast's own border, facing the\n"
                       "# way the mast faces, and that is what says which mast it is for.\n"
                       "# An entry signal with no approach begins its authority at the mast,\n"
                       "# which is what nearly all of them do.\n",
                       approaches);
}

std::vector<SignalPath> loadEntrySignals(const std::string& datasetRoot) {
    return loadRoutes(entriesFile(datasetRoot), "entry");
}

bool writeEntrySignals(const std::string& datasetRoot,
                       const std::vector<SignalPath>& entries) {
    return writeRoutes(datasetRoot, entriesFile(datasetRoot), "entry",
                       "# ebaner entry signals (main signals). The signal stands on the start\n"
                       "# border and the record is the whole route into the station; several\n"
                       "# sharing a start border are one signal governing them all.\n",
                       entries, /*withType=*/true, /*withMastFlags=*/true,
                       /*withSide=*/true);
}

std::vector<SignalPath> loadExitSignals(const std::string& datasetRoot) {
    return loadRoutes(exitsFile(datasetRoot), "exit");
}

bool writeExitSignals(const std::string& datasetRoot,
                      const std::vector<SignalPath>& exits) {
    return writeRoutes(datasetRoot, exitsFile(datasetRoot), "exit",
                       "# ebaner exit signals (main signals). The signal stands on the start\n"
                       "# border and protects the route to the destination border.\n",
                       exits, /*withType=*/false, /*withMastFlags=*/true,
                       /*withSide=*/true);
}

bool routeStartPose(const SignalPath& p, const std::vector<TrackPoly>& polys,
                    glm::dvec3& world, glm::dvec2& fwd) {
    if (p.parts.empty()) return false;
    const SectionInterval& iv0 = p.parts.front();
    const glm::dvec3 a = fracToWorld(polys, iv0.trackId, iv0.from);
    if (a.x == 0.0 && a.y == 0.0) return false; // stale/missing track
    // A small step toward `to` gives the travel direction leaving the border.
    const double f1 = iv0.from + (iv0.to - iv0.from) * 0.02;
    const glm::dvec3 b = fracToWorld(polys, iv0.trackId, f1);
    glm::dvec2 d(b.x - a.x, b.y - a.y);
    const double L = glm::length(d);
    if (L < 1e-6) return false;
    world = a;
    fwd = d / L;
    return true;
}

// Where a route *ends* and the direction it is travelling as it arrives there - the mirror
// of routeStartPose, taken from the last interval's `to`.
namespace {
bool routeEndPose(const SignalPath& p, const std::vector<TrackPoly>& polys,
                  glm::dvec3& world, glm::dvec2& fwd) {
    if (p.parts.empty()) return false;
    const SectionInterval& ivN = p.parts.back();
    const glm::dvec3 a = fracToWorld(polys, ivN.trackId, ivN.to);
    if (a.x == 0.0 && a.y == 0.0) return false;
    const double f1 = ivN.to + (ivN.from - ivN.to) * 0.02; // a bit upstream
    const glm::dvec3 b = fracToWorld(polys, ivN.trackId, f1);
    glm::dvec2 d(a.x - b.x, a.y - b.y); // upstream -> end = the arrival direction
    const double L = glm::length(d);
    if (L < 1e-6) return false;
    world = a;
    fwd = d / L;
    return true;
}
} // namespace

int routeTargetSignal(const SignalPath& route, const std::vector<SignalPath>& signals,
                      const std::vector<TrackPoly>& polys) {
    glm::dvec3 endW(0.0);
    glm::dvec2 arrive(0.0);
    if (!routeEndPose(route, polys, endW, arrive)) return -1;
    for (std::size_t i = 0; i < signals.size(); ++i) {
        const SignalPath& e = signals[i];
        if (e.start.trackId != route.end.trackId) continue;
        if (std::abs(e.start.frac - route.end.frac) > sameFracTol(polys, e.start.trackId))
            continue;
        glm::dvec3 sigW(0.0);
        glm::dvec2 face(0.0);
        if (!routeStartPose(e, polys, sigW, face)) continue;
        if (glm::dot(arrive, face) <= 0.0) continue; // would arrive behind the signal
        return static_cast<int>(i);
    }
    return -1;
}

std::vector<DistantSignal> loadDistantSignals(const std::string& datasetRoot) {
    std::vector<DistantSignal> out;
    std::ifstream f(distantsFile(datasetRoot));
    if (!f) return out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string kind, atTok, dirTok;
        DistantSignal d;
        is >> kind >> d.id;
        if (kind != "distant") continue;
        readName(is, d.name);
        is >> atTok >> dirTok;
        Border b;
        if (atTok.empty() || !parseBorder(atTok, b)) continue;
        d.trackId = b.trackId;
        d.frac = b.frac;
        d.dir = dirTok == "-" ? -1 : 1;
        // Optional side flag. Peeked rather than consumed, so a file that has not got one
        // is left exactly where the reader found it.
        const std::streampos after = is.tellg();
        std::string sideTok;
        if (is >> sideTok) {
            if (sideTok == "left") d.side = -1;
            else if (sideTok == "right") d.side = 1;
            else { is.clear(); is.seekg(after); }
        }
        out.push_back(std::move(d));
    }
    return out;
}

bool writeDistantSignals(const std::string& datasetRoot,
                         const std::vector<DistantSignal>& ds) {
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(distantsFile(datasetRoot), std::ios::trunc);
    if (!f) return false;
    f << "# ebaner distant signals (forsignal). Each stands at a plain point along a track,\n"
         "# not on a border, and repeats what the first main signal ahead is showing.\n"
         "# distant <id> \"<name>\" <trackHex>:<frac> <+|-> [left]\n"
         "# + reads toward increasing frac; the post stands right of that unless it says\n"
         "# left, right being the convention and so the silent default.\n";
    for (const DistantSignal& d : ds) {
        f << "distant " << d.id << ' ' << quoteName(d.name) << ' ' << std::hex << d.trackId
          << std::dec << ':' << d.frac << ' ' << (d.dir < 0 ? '-' : '+');
        if (d.side < 0) f << " left";
        f << '\n';
    }
    return static_cast<bool>(f);
}

bool signalGivesAuthority(const SignalPlacement& sp) {
    auto proceed = [](SignalAspect a) {
        return a == SignalAspect::Clear || a == SignalAspect::ClearReduced;
    };
    if (proceed(sp.aspect)) return true;
    // A dwarf on the same pole is a separate signal with a separate authority: a shunt
    // move over the same rails is authorised by the dwarf whatever the main head shows.
    return sp.withDwarf && proceed(sp.dwarfAspect);
}

std::vector<SignalPlacement> signalPlacements(const std::vector<SignalPath>& paths,
                                              const std::vector<TrackPoly>& polys,
                                              SignalKind kind) {
    std::vector<SignalPlacement> out;
    std::unordered_map<std::string, int> seen; // dedupe key -> placement index
    for (std::size_t pi = 0; pi < paths.size(); ++pi) {
        const SignalPath& p = paths[pi];
        glm::dvec3 a(0.0);
        glm::dvec2 fwd(0.0);
        if (!routeStartPose(p, polys, a, fwd)) continue;
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
            // A mast is built one way or the other, so any one record saying how settles it
            // for the signal all of them share.
            if (p.distant) out[it->second].withDistant = true;
            if (p.twoLamp) out[it->second].twoLamp = true;
            if (p.side < 0) out[it->second].side = -1;
            continue;
        }
        seen.emplace(key, static_cast<int>(out.size()));
        SignalPlacement sp;
        sp.kind = kind;
        sp.world = a;
        sp.forward = fwd;
        sp.at = p.start;
        sp.withDistant = p.distant;
        sp.twoLamp = p.twoLamp;
        sp.side = p.side;
        sp.paths.push_back(static_cast<int>(pi));
        out.push_back(std::move(sp));
    }
    return out;
}

std::vector<SignalPlacement> mergeSignals(const std::vector<SignalPlacement>& dwarfs,
                                          const std::vector<SignalPlacement>& mains) {
    constexpr double kSamePlace = 1.0; // m
    std::vector<SignalPlacement> out;
    std::vector<char> used(dwarfs.size(), 0);
    for (const SignalPlacement& e : mains) {
        SignalPlacement p = e; // already tagged Exit or Entry by signalPlacements
        for (std::size_t i = 0; i < dwarfs.size(); ++i) {
            if (used[i]) continue;
            const SignalPlacement& d = dwarfs[i];
            if (std::hypot(d.world.x - e.world.x, d.world.y - e.world.y) > kSamePlace) continue;
            if (glm::dot(d.forward, e.forward) < 0.9) continue; // faces the other way
            used[i] = 1;
            p.withDwarf = true;
            p.dwarfAspect = d.aspect;
            p.dwarfPaths = d.paths;
            break;
        }
        out.push_back(std::move(p));
    }
    for (std::size_t i = 0; i < dwarfs.size(); ++i)
        if (!used[i]) out.push_back(dwarfs[i]); // a dwarf on its own post
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

    // What each of the route's legs needs, worked out once. Every lookup here walks the
    // whole track list, and there are thousands of turnouts to test below - doing this
    // inside that loop is what made a train entering a circuit stall the sim for the best
    // part of a second.
    struct Leg {
        const std::vector<glm::dvec3>* pts = nullptr; // the track's polyline, or null
        double lo = 0.0, hi = 1.0;                    // the interval, ordered
        double eps = 1e-6;                            // rounding margin at its own ends
        glm::dvec3 endWorld{0.0};                     // where `to` lands, for case (a)
        bool haveEnd = false;
    };
    std::vector<Leg> legs(p.parts.size());
    // The box the route occupies. A turnout outside it, by more than the distance that
    // counts as standing at one, cannot satisfy either test below - so it can be rejected
    // before any of the expensive geometry. The box is over the surveyed points *inside*
    // each interval as well as its two ends, since a polyline's extent is its vertices'
    // and a curve may bulge well outside the chord between them.
    double bx0 = 1e30, by0 = 1e30, bx1 = -1e30, by1 = -1e30;
    auto cover = [&](const glm::dvec3& w) {
        bx0 = std::min(bx0, w.x); bx1 = std::max(bx1, w.x);
        by0 = std::min(by0, w.y); by1 = std::max(by1, w.y);
    };
    for (std::size_t k = 0; k < p.parts.size(); ++k) {
        const SectionInterval& iv = p.parts[k];
        Leg& leg = legs[k];
        leg.lo = std::min(iv.from, iv.to);
        leg.hi = std::max(iv.from, iv.to);
        for (const TrackPoly& tp : polys)
            if (tp.id == iv.trackId) { leg.pts = &tp.pts; break; }
        if (!leg.pts || leg.pts->size() < 2) continue; // stale overlay: no such track
        const double len = polyLength(*leg.pts);
        leg.eps = len > 1.0 ? kEdgeSlack / len : 1e-6;
        const glm::dvec3 a = fracToWorld(polys, iv.trackId, iv.from);
        const glm::dvec3 b = fracToWorld(polys, iv.trackId, iv.to);
        leg.endWorld = b;
        leg.haveEnd = !(b.x == 0.0 && b.y == 0.0);
        cover(a);
        cover(b);
        double acc = 0.0;
        for (std::size_t j = 0; j < leg.pts->size(); ++j) {
            if (j) acc += std::hypot((*leg.pts)[j].x - (*leg.pts)[j - 1].x,
                                     (*leg.pts)[j].y - (*leg.pts)[j - 1].y);
            const double f = len > 1e-9 ? acc / len : 0.0;
            if (f > leg.lo && f < leg.hi) cover((*leg.pts)[j]);
        }
    }
    if (bx0 > bx1) return reqs; // nothing locatable in the route at all
    bx0 -= kAtTurnout; by0 -= kAtTurnout; bx1 += kAtTurnout; by1 += kAtTurnout;

    for (std::size_t i = 0; i < tos.size(); ++i) {
        if (tos[i].mainPath < 0) continue; // inert crossing: nothing to set
        const glm::dvec3& tw = tos[i].world;
        if (tw.x < bx0 || tw.x > bx1 || tw.y < by0 || tw.y > by1) continue; // not on the road
        bool needSet = false;
        SwitchState need = SwitchState::Straight;
        // (a) The route crosses from one track to another at this turnout: diverging if
        // either side is the branch, else it runs straight over a joint here.
        for (std::size_t k = 0; k + 1 < p.parts.size(); ++k) {
            if (!legs[k].haveEnd) continue;
            if (planarDist(legs[k].endWorld, tw) > kAtTurnout) continue;
            need = (p.parts[k].trackId == tos[i].sidingTrack ||
                    p.parts[k + 1].trackId == tos[i].sidingTrack)
                       ? SwitchState::Diverging
                       : SwitchState::Straight;
            needSet = true;
        }
        // (b) Otherwise the route may run straight through the turnout inside one leg.
        if (!needSet) {
            for (std::size_t k = 0; k < p.parts.size(); ++k) {
                const Leg& leg = legs[k];
                if (!leg.pts) continue;
                double frac = 0.0, dist = 0.0;
                if (!projectOnTrack(polys, p.parts[k].trackId, glm::dvec2(tw.x, tw.y), frac,
                                    dist))
                    continue;
                if (dist > kAtTurnout) continue; // this leg doesn't reach the turnout
                // Only an interior crossing counts, so a route that merely ends at the
                // turnout imposes no requirement. The margin has to stay well under a
                // border-to-turnout spacing (which can be ~1 m), so it is only wide
                // enough to absorb rounding, not to swallow a real crossing.
                if (frac > leg.lo + leg.eps && frac < leg.hi - leg.eps) {
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

SignalPath departureRoute(const SignalPath& exitRoute, const SignalPath& exitSignal) {
    SignalPath d;
    d.id = exitRoute.id;
    d.name = exitRoute.name;
    d.start = exitRoute.start;
    d.end = exitSignal.end;
    d.exitId = exitRoute.exitId;
    d.type = exitRoute.type;
    d.parts = exitRoute.parts;
    for (const SectionInterval& iv : exitSignal.parts) {
        // The two halves meet at the signal's border: where that join runs on through one
        // track in one direction it is a single interval, not two that happen to touch.
        if (!d.parts.empty()) {
            SectionInterval& last = d.parts.back();
            if (last.trackId == iv.trackId && last.to == iv.from &&
                (last.to - last.from) * (iv.to - iv.from) > 0.0) {
                last.to = iv.to;
                continue;
            }
        }
        d.parts.push_back(iv);
    }
    return d;
}

bool routeContains(const SignalPath& whole, const SignalPath& part) {
    if (part.parts.empty()) return false;
    for (const SectionInterval& p : part.parts) {
        const double lo = std::min(p.from, p.to), hi = std::max(p.from, p.to);
        bool inside = false;
        for (const SectionInterval& w : whole.parts) {
            if (w.trackId != p.trackId) continue;
            if ((w.to - w.from) * (p.to - p.from) <= 0.0) continue; // runs the other way
            const double wlo = std::min(w.from, w.to), whi = std::max(w.from, w.to);
            const double tol = 1e-9;
            if (lo >= wlo - tol && hi <= whi + tol) { inside = true; break; }
        }
        if (!inside) return false;
    }
    return true;
}

bool routesOppose(const SignalPath& a, const SignalPath& b) {
    for (const SectionInterval& x : a.parts) {
        for (const SectionInterval& y : b.parts) {
            if (x.trackId != y.trackId) continue;
            const double xlo = std::min(x.from, x.to), xhi = std::max(x.from, x.to);
            const double ylo = std::min(y.from, y.to), yhi = std::max(y.from, y.to);
            // Positive overlap only: two routes meeting end to end at a border share a
            // point, not a length of rail, and that is exactly the join a through move
            // is made of.
            if (std::min(xhi, yhi) - std::max(xlo, ylo) <= 1e-6) continue;
            if ((x.to > x.from) != (y.to > y.from)) return true;
        }
    }
    return false;
}

namespace {
bool routeDiverges(const SignalPath& p, const SwitchNetwork& net,
                   const std::vector<TrackPoly>& polys) {
    for (const PathSwitch& ps : pathSwitchRequirements(p, net, polys))
        if (ps.need == SwitchState::Diverging) return true;
    return false;
}
} // namespace

RouteType defaultRouteType(const SignalPath& route, const SignalPath& exit,
                           const SwitchNetwork& net, const std::vector<TrackPoly>& polys) {
    return (routeDiverges(route, net, polys) || routeDiverges(exit, net, polys))
               ? RouteType::C2
               : RouteType::C1;
}

RouteType defaultRouteType(const SignalPath& route, const SwitchNetwork& net,
                           const std::vector<TrackPoly>& polys) {
    return routeDiverges(route, net, polys) ? RouteType::C2 : RouteType::C1;
}

// ------------------------------------------------------- what a distant can see
namespace {
// The turnout standing at a world point, or -1. Same tolerance the route requirements use.
int turnoutAt(const SwitchNetwork& net, const glm::dvec3& w) {
    const std::vector<Turnout>& tos = net.turnouts();
    for (std::size_t i = 0; i < tos.size(); ++i) {
        if (tos[i].mainPath < 0) continue; // inert crossing: nothing is set here
        if (planarDist(w, tos[i].world) <= kAtTurnout) return static_cast<int>(i);
    }
    return -1;
}
} // namespace

void walkAhead(const std::vector<TrackPoly>& polys, const TrackJunctions& junctions,
               const SwitchNetwork& net, std::uint32_t trackId, double frac, int dir,
               double maxM, const WalkSpan& onSpan) {
    double gone = 0.0;
    std::uint32_t track = trackId;
    double at = frac;
    int d = dir;
    // Simple path: a road that returns to a track it has already used is a loop, and
    // nothing reading down the line wants to go round one.
    std::vector<std::uint32_t> visited{track};
    for (int step = 0; step < 256; ++step) {
        double lenM = 0.0;
        for (const TrackPoly& tp : polys)
            if (tp.id == track) lenM = polyLength(tp.pts);
        if (lenM <= 0.0) return;
        // The nearest junction ahead on this track bounds the span we can see along it.
        double stopFrac = d > 0 ? 1.0 : 0.0;
        const auto ji = junctions.find(track);
        if (ji != junctions.end())
            for (const TrackJunction& c : ji->second) {
                if (d * (c.here - at) <= 1e-9) continue;      // behind us
                if (d * (c.here - stopFrac) < 0.0) stopFrac = c.here;
            }
        // Would the span run past the reach? Then look only as far as the reach allows.
        const double spanM = lenM * std::abs(stopFrac - at);
        double limit = stopFrac;
        bool ranOut = false;
        if (gone + spanM > maxM) {
            limit = at + d * (maxM - gone) / std::max(lenM, 1e-9);
            ranOut = true;
        }
        if (onSpan(track, at, limit, d)) return; // the caller found what it was after
        if (ranOut) return; // nothing within the reach
        gone += spanM;
        // At the junction: every continuation that does not reverse is a candidate. Running
        // straight on along this same track is one of them - at a turnout that is the
        // through leg, and leaving it out would silently divert the walk onto the branch.
        struct Next { std::uint32_t track; double frac; int dir; };
        std::vector<Next> cand;
        const glm::dvec2 headIn = trackTangent(polys, track, stopFrac, d);
        if (d > 0 ? stopFrac < 1.0 - 1e-9 : stopFrac > 1e-9)
            cand.push_back({track, stopFrac, d});
        if (ji != junctions.end())
            for (const TrackJunction& c : ji->second) {
                if (std::abs(c.here - stopFrac) > 1e-9) continue; // a different junction
                const glm::dvec2 tPlus = trackTangent(polys, c.other, c.there, +1);
                const int nd = glm::dot(tPlus, headIn) >= 0.0 ? +1 : -1;
                if (glm::dot(trackTangent(polys, c.other, c.there, nd), headIn) <= 0.0)
                    continue; // reversing: not a move a train can make
                cand.push_back({c.other, c.there, nd});
            }
        if (cand.empty()) return; // a dead end, or nothing legal to take
        std::size_t take = 0;
        if (cand.size() > 1) {
            // A turnout: follow the leg it is set to. Anything the switches cannot resolve
            // - an unregistered fan, a broken point - ends the walk.
            const int t = turnoutAt(net, fracToWorld(polys, track, stopFrac));
            if (t < 0 || net.state(t) == SwitchState::Broken) return;
            const std::uint32_t siding = net.turnouts()[t].sidingTrack;
            int pick = -1;
            if (net.state(t) == SwitchState::Diverging) {
                // Set to the branch: only the branch will do. If it is not among the roads
                // out of here, this turnout is set away from us and there is nowhere to go.
                for (std::size_t i = 0; i < cand.size(); ++i)
                    if (cand[i].track == siding) pick = static_cast<int>(i);
            } else {
                // Set straight: the through leg. Carrying on along the track we are already
                // on is that leg wherever the turnout sits on it - which is the usual case,
                // and the one that matters where a slip connector branches off the main.
                // Approaching from the branch instead, the through leg is the one road out
                // that is not the branch.
                for (std::size_t i = 0; i < cand.size(); ++i)
                    if (cand[i].track == track) pick = static_cast<int>(i);
                if (pick < 0)
                    for (std::size_t i = 0; i < cand.size(); ++i)
                        if (cand[i].track != siding) {
                            if (pick >= 0) { pick = -1; break; } // several: do not guess
                            pick = static_cast<int>(i);
                        }
            }
            if (pick < 0) return;
            take = static_cast<std::size_t>(pick);
        }
        // Carrying on along the same track is not a revisit; coming back to it by another
        // road is, and a distant reading round a loop would repeat a signal already passed.
        if (cand[take].track != track) {
            if (std::find(visited.begin(), visited.end(), cand[take].track) != visited.end())
                return;
            visited.push_back(cand[take].track);
        }
        track = cand[take].track;
        at = cand[take].frac;
        d = cand[take].dir;
    }
}

int firstMainSignalAhead(const std::vector<TrackPoly>& polys, const TrackJunctions& junctions,
                         const SwitchNetwork& net,
                         const std::vector<SignalPlacement>& placements,
                         std::uint32_t trackId, double frac, int dir, double maxM,
                         std::vector<SectionInterval>* walked) {
    if (walked) walked->clear();
    int hit = -1;
    // The first main signal standing on this span, facing the way we are going. A signal
    // facing the other way protects opposing moves: we step over its back.
    walkAhead(polys, junctions, net, trackId, frac, dir, maxM,
              [&](std::uint32_t track, double from, double to, int d) {
                  int best = -1;
                  double bestAt = 0.0;
                  for (std::size_t k = 0; k < placements.size(); ++k) {
                      const SignalPlacement& sp = placements[k];
                      if (sp.kind == SignalKind::Dwarf || sp.kind == SignalKind::Distant)
                          continue;
                      if (sp.at.trackId != track) continue;
                      const double f = sp.at.frac;
                      // Strictly ahead: a signal standing at the point the walk starts
                      // from is the one being read from, not one to read.
                      if (d * (f - from) <= 1e-9 || d * (to - f) < -1e-9) continue;
                      if (glm::dot(sp.forward, trackTangent(polys, track, f, d)) <= 0.0)
                          continue;
                      if (best < 0 || d * (f - bestAt) < 0.0) {
                          best = static_cast<int>(k);
                          bestAt = f;
                      }
                  }
                  if (walked)
                      walked->push_back({track, from, best >= 0 ? bestAt : to});
                  if (best < 0) return false;
                  hit = best;
                  return true;
              });
    return hit;
}

bool updateDistantAspects(std::vector<SignalPlacement>& placements,
                          const std::vector<TrackPoly>& polys,
                          const TrackJunctions& junctions, const SwitchNetwork& net,
                          double maxM) {
    bool changed = false;
    for (std::size_t k = 0; k < placements.size(); ++k) {
        SignalPlacement& sp = placements[k];
        const bool onItsOwnPost = sp.kind == SignalKind::Distant;
        if (!onItsOwnPost && !sp.withDistant) continue;
        // A distant on a main's mast goes out with that main: under a signal at danger
        // there is nothing ahead worth warning about, and a lit repeat would be read as
        // permission the main is not giving.
        SignalAspect want = SignalAspect::Dark;
        if (onItsOwnPost || sp.aspect != SignalAspect::Stop) {
            const int dir =
                glm::dot(sp.forward, trackTangent(polys, sp.at.trackId, sp.at.frac, +1)) >= 0.0
                    ? +1
                    : -1;
            const int hit = firstMainSignalAhead(polys, junctions, net, placements,
                                                 sp.at.trackId, sp.at.frac, dir, maxM);
            // Nothing reachable warns exactly as a signal at danger does.
            want = hit < 0 ? SignalAspect::Stop : placements[hit].aspect;
        }
        SignalAspect& shows = onItsOwnPost ? sp.aspect : sp.distantAspect;
        if (shows != want) { shows = want; changed = true; }
    }
    return changed;
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
                         const std::vector<char>& routeSet,
                         const std::vector<SignalAspect>& exitAspects) {
    // What a dwarf governing `governed` should display.
    auto dwarfAspectFor = [&](const std::vector<int>& governed) {
        SignalAspect want = SignalAspect::Stop;
        for (int pi : governed) {
            if (pi < 0 || pi >= static_cast<int>(paths.size())) continue;
            // A route set from this signal shows clear (it is dropped the moment a train
            // enters its circuits, so "set" already implies the road ahead is empty).
            if (pi < static_cast<int>(routeSet.size()) && routeSet[pi]) return SignalAspect::Clear;
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
        return want;
    };
    bool changed = false;
    for (std::size_t k = 0; k < placements.size(); ++k) {
        SignalPlacement& sp = placements[k];
        if (sp.kind != SignalKind::Dwarf) {
            // What the main head shows is decided by the interlocking, not here: danger
            // unless a locked route says otherwise. A dwarf sharing the pole keeps its own
            // indication, which is still this function's business.
            const SignalAspect w = k < exitAspects.size() ? exitAspects[k] : SignalAspect::Stop;
            if (sp.aspect != w) { sp.aspect = w; changed = true; }
            if (sp.withDwarf) {
                const SignalAspect d = dwarfAspectFor(sp.dwarfPaths);
                if (sp.dwarfAspect != d) { sp.dwarfAspect = d; changed = true; }
            }
            continue;
        }
        const SignalAspect want = dwarfAspectFor(sp.paths);
        if (sp.aspect != want) { sp.aspect = want; changed = true; }
    }
    return changed;
}
