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
        out.push_back(o);
    }
    return out;
}

bool writeSwitchTypes(const std::string& datasetRoot,
                      const std::vector<SwitchTypeOverride>& overrides) {
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(typesFile(datasetRoot), std::ios::trunc);
    if (!f) return false;
    f << "# ebaner switch types (manual is default; only motor switches are listed).\n"
         "# switch <trackIdHex> <x> <y> motor\n";
    f << std::fixed << std::setprecision(3);
    for (const SwitchTypeOverride& o : overrides) {
        if (o.type != SwitchType::Motor) continue;
        f << "switch " << std::hex << o.sidingTrack << std::dec << ' ' << o.world.x << ' '
          << o.world.y << " motor\n";
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
        out.push_back(o);
    }
    return out;
}
