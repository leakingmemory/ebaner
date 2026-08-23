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

#include "AvalancheSignals.h"

#include "TrackCircuits.h" // readName / quoteName

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace {

std::string signalsFile(const std::string& root) {
    return root + "/overlay/avalanche-signals.txt";
}

// `<trackHex>:<frac>`, as the rest of the overlay anchors things.
bool parseAt(const std::string& tok, std::uint32_t& track, double& frac) {
    const auto c = tok.find(':');
    if (c == std::string::npos) return false;
    track = static_cast<std::uint32_t>(
        std::strtoul(tok.substr(0, c).c_str(), nullptr, 16));
    frac = std::atof(tok.substr(c + 1).c_str());
    return true;
}

} // namespace

std::vector<AvalancheSignal> loadAvalancheSignals(const std::string& datasetRoot) {
    std::vector<AvalancheSignal> out;
    std::ifstream f(signalsFile(datasetRoot));
    if (!f) return out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string kind, atTok, dirTok, sideTok;
        AvalancheSignal s;
        is >> kind >> s.id;
        if (kind != "avalanche") continue;
        readName(is, s.name);
        is >> atTok >> dirTok >> sideTok;
        if (atTok.empty() || !parseAt(atTok, s.trackId, s.frac)) continue;
        s.dir = dirTok == "-" ? -1 : 1;
        s.side = sideTok == "L" ? -1 : 1;
        out.push_back(std::move(s));
    }
    return out;
}

bool writeAvalancheSignals(const std::string& datasetRoot,
                           const std::vector<AvalancheSignal>& signals) {
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(signalsFile(datasetRoot), std::ios::trunc);
    if (!f) return false;
    f << "# ebaner avalanche warning signals: three lamps in a column, red over white\n"
         "# over red, protecting a stretch where the mountain can come down on the line.\n"
         "# At rest the white flashes - the watch is kept and has nothing to report. On a\n"
         "# warning both reds flash and the white goes out. They govern nothing: no route\n"
         "# runs through them and no interlocking reads them.\n"
         "# +/- is the direction of travel the head is read from; R/L is which side of the\n"
         "# track the post stands on, and is independent of it.\n"
         "# avalanche <id> \"<name>\" <trackHex>:<frac> <+|-> <R|L>\n";
    for (const AvalancheSignal& s : signals) {
        f << "avalanche " << s.id << ' ' << quoteName(s.name) << ' ' << std::hex
          << s.trackId << std::dec << ':' << s.frac << ' ' << (s.dir < 0 ? '-' : '+')
          << ' ' << (s.side < 0 ? 'L' : 'R') << '\n';
    }
    return static_cast<bool>(f);
}
