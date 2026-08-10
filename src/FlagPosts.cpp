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

#include "FlagPosts.h"

#include "TrackCircuits.h" // readName / quoteName

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace {

std::string postsFile(const std::string& root) {
    return root + "/overlay/flag-posts.txt";
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

std::vector<FlagPost> loadFlagPosts(const std::string& datasetRoot) {
    std::vector<FlagPost> out;
    std::ifstream f(postsFile(datasetRoot));
    if (!f) return out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string kind, atTok, sideTok;
        FlagPost p;
        is >> kind >> p.id;
        if (kind != "flag") continue;
        readName(is, p.name);
        is >> atTok >> sideTok;
        if (atTok.empty() || !parseAt(atTok, p.trackId, p.frac)) continue;
        p.side = sideTok == "L" ? -1 : 1;
        readName(is, p.station); // optional; absent leaves it empty = nearest wins
        out.push_back(std::move(p));
    }
    return out;
}

bool writeFlagPosts(const std::string& datasetRoot, const std::vector<FlagPost>& posts) {
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(postsFile(datasetRoot), std::ios::trunc);
    if (!f) return false;
    f << "# ebaner flag posts: the hand signal a station's TXP hangs out. Red is stop,\n"
         "# green is pass through, an empty fixture is neither (or the station is\n"
         "# unmanned). R/L is which side of the track the post stands on - the flag\n"
         "# hangs and reads from either direction, so there is no facing.\n"
         "# flag <id> \"<name>\" <trackHex>:<frac> <R|L> [\"<station>\"]\n";
    for (const FlagPost& p : posts) {
        f << "flag " << p.id << ' ' << quoteName(p.name) << ' ' << std::hex << p.trackId
          << std::dec << ':' << p.frac << ' ' << (p.side < 0 ? 'L' : 'R');
        if (!p.station.empty()) f << ' ' << quoteName(p.station);
        f << '\n';
    }
    return static_cast<bool>(f);
}
