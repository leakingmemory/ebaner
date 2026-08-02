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

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

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
        std::string tok; // remaining trackHex:from:to interval tokens
        while (is >> tok) {
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
         "<trackHex>:<from>:<to> ...\n";
    for (const SignalPath& p : paths) {
        f << "path " << p.id << ' ' << (p.name.empty() ? "-" : p.name) << ' ' << std::hex
          << p.start.trackId << std::dec << ':' << p.start.frac << ' ' << std::hex
          << p.end.trackId << std::dec << ':' << p.end.frac;
        for (const SectionInterval& iv : p.parts)
            f << ' ' << std::hex << iv.trackId << std::dec << ':' << iv.from << ':' << iv.to;
        f << '\n';
    }
    return static_cast<bool>(f);
}
