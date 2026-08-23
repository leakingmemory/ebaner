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

#include "SimpleEntrySignals.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace {

std::string signalsFile(const std::string& root) {
    return root + "/overlay/simple-entry-signals.txt";
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

std::vector<SimpleEntrySignal> loadSimpleEntrySignals(const std::string& datasetRoot) {
    std::vector<SimpleEntrySignal> out;
    std::ifstream f(signalsFile(datasetRoot));
    if (!f) return out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string kind, atTok, dirTok;
        SimpleEntrySignal s;
        is >> kind >> s.id;
        if (kind != "entry") continue;
        readName(is, s.name);
        is >> atTok >> dirTok;
        if (atTok.empty() || !parseAt(atTok, s.trackId, s.frac)) continue;
        s.dir = dirTok == "-" ? -1 : 1;
        // Optional side flag, ahead of the optional station. Peeked rather than consumed:
        // readName takes a bare token as a name, so a line ending in `left` and no station
        // would otherwise come back as a signal belonging to a station called "left".
        const std::streampos after = is.tellg();
        std::string sideTok;
        if (is >> sideTok) {
            if (sideTok == "left") s.side = -1;
            else if (sideTok == "right") s.side = 1;
            else { is.clear(); is.seekg(after); }
        }
        readName(is, s.station); // optional; absent leaves it empty = nearest wins
        out.push_back(std::move(s));
    }
    return out;
}

bool writeSimpleEntrySignals(const std::string& datasetRoot,
                             const std::vector<SimpleEntrySignal>& sigs) {
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(signalsFile(datasetRoot), std::ios::trunc);
    if (!f) return false;
    f << "# ebaner simple entry signals. Red or green, no track circuits and no routes;\n"
         "# the only interlocking is that a station shows one green at a time.\n"
         "# entry <id> \"<name>\" <trackHex>:<frac> <+|-> [left] [\"<station>\"]\n"
         "# + governs movements toward increasing frac, and the post stands right of that\n"
         "# unless it says left - right being the convention and so the silent default.\n"
         "# The station is optional and normally left out: the nearest one is used, which\n"
         "# cannot go stale.\n";
    for (const SimpleEntrySignal& s : sigs) {
        f << "entry " << s.id << ' ' << quoteName(s.name) << ' ' << std::hex << s.trackId
          << std::dec << ':' << s.frac << ' ' << (s.dir < 0 ? '-' : '+');
        if (s.side < 0) f << " left";
        if (!s.station.empty()) f << ' ' << quoteName(s.station);
        f << '\n';
    }
    return static_cast<bool>(f);
}
