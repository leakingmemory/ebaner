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

#include "SwitchTypes.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

namespace {
constexpr double kMatchTol = 3.0; // m; same tolerance SwitchNetwork uses to dedup turnouts

std::string typesFile(const std::string& root) {
    return root + "/overlay/switch-types.txt";
}
} // namespace

// --------------------------------------------------------------------------- IO
std::vector<SwitchTypeOverride> loadSwitchTypes(const std::string& datasetRoot) {
    std::vector<SwitchTypeOverride> out;
    std::ifstream f(typesFile(datasetRoot));
    if (!f) return out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string kind, idhex, ty;
        double x = 0.0, y = 0.0;
        is >> kind >> idhex >> x >> y >> ty;
        if (kind != "switch" || idhex.empty()) continue;
        SwitchTypeOverride o;
        o.sidingTrack = static_cast<std::uint32_t>(std::strtoul(idhex.c_str(), nullptr, 16));
        o.world = glm::dvec2(x, y);
        o.type = SwitchType::Motor; // only non-default (motor) switches are stored
        // Optional locking set: `... motor lock <id> <id> ...` (may be empty).
        std::string tok;
        if (is >> tok && tok == "lock") {
            o.hasLock = true;
            int id = 0;
            while (is >> id) o.lock.push_back(id);
        }
        out.push_back(o);
    }
    return out;
}

std::vector<SwitchSuppression> loadSwitchSuppressions(const std::string& datasetRoot) {
    std::vector<SwitchSuppression> out;
    std::ifstream f(typesFile(datasetRoot));
    if (!f) return out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string kind;
        SwitchSuppression r;
        is >> kind >> r.world.x >> r.world.y >> r.radius;
        if (kind != "noswitch" || !is) continue;
        // Optional: a branch id to narrow it to, and `all` to take drawn track too.
        std::string tok;
        while (is >> tok) {
            if (tok == "all") r.includeDrawn = true;
            else r.sidingTrack = static_cast<std::uint32_t>(
                     std::strtoul(tok.c_str(), nullptr, 16));
        }
        out.push_back(r);
    }
    return out;
}

bool writeSwitchTypes(const std::string& datasetRoot,
                      const std::vector<SwitchTypeOverride>& overrides,
                      const std::vector<SwitchSuppression>& suppress) {
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(typesFile(datasetRoot), std::ios::trunc);
    if (!f) return false;
    f << "# ebaner switch types (manual is default; only motor switches are listed).\n"
         "# switch   <trackIdHex> <x> <y> motor [lock <sectionId> ...]\n"
         "# noswitch <x> <y> <radius> [<trackIdHex>] [all]  - not really a switch\n";
    f << std::fixed << std::setprecision(3);
    for (const SwitchTypeOverride& o : overrides) {
        if (o.type != SwitchType::Motor) continue;
        f << "switch " << std::hex << o.sidingTrack << std::dec << ' ' << o.world.x << ' '
          << o.world.y << " motor";
        if (o.hasLock) {
            f << " lock";
            for (int id : o.lock) f << ' ' << id;
        }
        f << '\n';
    }
    for (const SwitchSuppression& r : suppress) {
        f << "noswitch " << r.world.x << ' ' << r.world.y << ' ' << r.radius;
        if (r.sidingTrack != 0) f << ' ' << std::hex << r.sidingTrack << std::dec;
        if (r.includeDrawn) f << " all";
        f << '\n';
    }
    return static_cast<bool>(f);
}

// -------------------------------------------------------------- apply / collect
void applySwitchTypes(SwitchNetwork& net,
                      const std::vector<SwitchTypeOverride>& overrides) {
    const std::vector<Turnout>& tos = net.turnouts();
    for (const SwitchTypeOverride& o : overrides) {
        int best = -1;
        double bestD = kMatchTol;
        for (std::size_t i = 0; i < tos.size(); ++i) {
            if (tos[i].sidingTrack != o.sidingTrack) continue;
            const double d = std::hypot(tos[i].world.x - o.world.x, tos[i].world.y - o.world.y);
            if (d < bestD) { bestD = d; best = static_cast<int>(i); }
        }
        if (best >= 0) net.setType(best, o.type);
    }
}

std::vector<SwitchTypeOverride> collectSwitchOverrides(const SwitchNetwork& net) {
    std::vector<SwitchTypeOverride> out;
    const std::vector<Turnout>& tos = net.turnouts();
    for (std::size_t i = 0; i < tos.size(); ++i) {
        if (tos[i].mainPath < 0) continue; // inert crossing: no working switch
        if (net.type(static_cast<int>(i)) != SwitchType::Motor) continue;
        SwitchTypeOverride o;
        o.sidingTrack = tos[i].sidingTrack;
        o.world = glm::dvec2(tos[i].world.x, tos[i].world.y);
        o.type = SwitchType::Motor;
        o.hasLock = true;                       // persist the resolved locking set
        o.lock = net.lock(static_cast<int>(i));
        out.push_back(o);
    }
    return out;
}

// ------------------------------------------------------------- lock resolution
std::vector<int> defaultLockSections(const Turnout& t, const TrackCircuits& circuits,
                                     const std::vector<TrackPoly>& polys) {
    // "The circuits the switch is located within": sections whose track polyline passes
    // within a few metres of the turnout's world point. A switch lies exactly on the
    // through track (and the branch leaves from the same point), so a small tolerance
    // catches its own section(s) - including both when it sits on a section border -
    // without grabbing a parallel siding a track-spacing away.
    constexpr double kNearTol = 3.0; // m
    const glm::dvec2 p(t.world.x, t.world.y);
    std::vector<int> ids;
    for (const Section& s : circuits.sections) {
        bool near = false;
        for (const SectionInterval& iv : s.parts) {
            const glm::dvec3 a0 = fracToWorld(polys, iv.trackId, iv.from);
            if (a0.x == 0.0 && a0.y == 0.0) continue; // stale/missing track
            constexpr int kSteps = 32;
            for (int k = 0; k <= kSteps && !near; ++k) {
                const double f = iv.from + (iv.to - iv.from) * k / kSteps;
                const glm::dvec3 w = fracToWorld(polys, iv.trackId, f);
                if (std::hypot(w.x - p.x, w.y - p.y) <= kNearTol) near = true;
            }
            if (near) break;
        }
        if (near) ids.push_back(s.id);
    }
    return ids;
}

void applySwitchLocks(SwitchNetwork& net,
                      const std::vector<SwitchTypeOverride>& overrides,
                      const TrackCircuits& circuits,
                      const std::vector<TrackPoly>& polys) {
    const std::vector<Turnout>& tos = net.turnouts();
    for (std::size_t i = 0; i < tos.size(); ++i) {
        if (tos[i].mainPath < 0) continue;
        if (net.type(static_cast<int>(i)) != SwitchType::Motor) continue;
        // Find this switch's override (match on sidingTrack + world, like applySwitchTypes).
        const SwitchTypeOverride* match = nullptr;
        double bestD = kMatchTol;
        for (const SwitchTypeOverride& o : overrides) {
            if (o.sidingTrack != tos[i].sidingTrack) continue;
            const double d = std::hypot(tos[i].world.x - o.world.x, tos[i].world.y - o.world.y);
            if (d < bestD) { bestD = d; match = &o; }
        }
        if (match && match->hasLock)
            net.setLock(static_cast<int>(i), match->lock);           // authored set
        else
            net.setLock(static_cast<int>(i),
                        defaultLockSections(tos[i], circuits, polys)); // default set
    }
}
