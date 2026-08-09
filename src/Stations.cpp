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

#include "Stations.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <unordered_set>

namespace fs = std::filesystem;

namespace {

// Enough of a JSON reader for one array of flat objects, in the spirit of the binary
// parsers next door: the file is written by the exporter, not by hand, so the shape is
// known. Anything unexpected yields no stations rather than a wrong one.

// The quoted string that follows "key": inside [p, end). Empty if absent.
std::string strField(const char* p, const char* end, const char* key) {
    const std::string pat = std::string("\"") + key + "\":";
    const char* k = std::search(p, end, pat.begin(), pat.end());
    if (k == end) return {};
    k += pat.size();
    while (k < end && (*k == ' ' || *k == '\t')) ++k;
    if (k >= end || *k != '"') return {};
    ++k;
    std::string out;
    while (k < end && *k != '"') {
        if (*k == '\\' && k + 1 < end) { // the exporter writes UTF-8 raw, but be safe
            ++k;
            switch (*k) {
                case 'n': out.push_back('\n'); break;
                case 't': out.push_back('\t'); break;
                // An escaped quote or backslash keeps the character that follows.
                default: out.push_back(*k); break;
            }
            ++k;
            continue;
        }
        out.push_back(*k++);
    }
    return out;
}

// The number that follows "key":. `ok` reports whether it was there at all, so a missing
// coordinate is not silently read as the origin.
double numField(const char* p, const char* end, const char* key, bool& ok) {
    const std::string pat = std::string("\"") + key + "\":";
    const char* k = std::search(p, end, pat.begin(), pat.end());
    ok = false;
    if (k == end) return 0.0;
    k += pat.size();
    char* stop = nullptr;
    const double v = std::strtod(k, &stop);
    ok = stop != k;
    return v;
}

// Fold a name to a comparable key: lower case, and the Norwegian letters reduced to the
// ASCII shapes people reach for at a shell prompt.
std::string fold(const std::string& s) {
    std::string out;
    for (std::size_t i = 0; i < s.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        if (c == 0xC3 && i + 1 < s.size()) { // two-byte Latin-1 supplement
            const unsigned char d = static_cast<unsigned char>(s[++i]);
            switch (d) {
                case 0xA6: case 0x86: out += "ae"; break; // æ Æ
                case 0xB8: case 0x98: out += "o"; break;  // ø Ø
                case 0xA5: case 0x85: out += "a"; break;  // å Å
                default: out.push_back('?'); break;
            }
            continue;
        }
        out.push_back(static_cast<char>(std::tolower(c)));
    }
    return out;
}

// Levenshtein, capped - only used to suggest what someone meant.
std::size_t editDistance(const std::string& a, const std::string& b) {
    std::vector<std::size_t> prev(b.size() + 1), cur(b.size() + 1);
    for (std::size_t j = 0; j <= b.size(); ++j) prev[j] = j;
    for (std::size_t i = 1; i <= a.size(); ++i) {
        cur[0] = i;
        for (std::size_t j = 1; j <= b.size(); ++j)
            cur[j] = std::min({prev[j] + 1, cur[j - 1] + 1,
                               prev[j - 1] + (a[i - 1] == b[j - 1] ? 0 : 1)});
        prev = cur;
    }
    return prev[b.size()];
}

} // namespace

std::vector<Station> loadStations(const std::string& datasetRoot) {
    std::vector<Station> out;
    std::unordered_set<std::string> seen;

    for (int lod = 0; lod < 4; ++lod) {
        const std::string lodDir = datasetRoot + "/tiles/" + std::to_string(lod);
        std::error_code ec;
        if (!fs::is_directory(lodDir, ec)) continue;
        for (const fs::directory_entry& e : fs::directory_iterator(lodDir, ec)) {
            if (!e.is_directory()) continue;
            std::ifstream f(e.path() / "meta.json", std::ios::binary);
            if (!f) continue;
            const std::string text((std::istreambuf_iterator<char>(f)),
                                   std::istreambuf_iterator<char>());

            const std::string key = "\"stations\":";
            auto at = std::search(text.begin(), text.end(), key.begin(), key.end());
            if (at == text.end()) continue;
            const char* p = text.data() + (at - text.begin()) + key.size();
            const char* end = text.data() + text.size();
            while (p < end && *p != '[') ++p;
            if (p == end) continue;

            // Each {...} up to the closing ]. The objects are flat, so the first '}'
            // ends one.
            for (++p; p < end && *p != ']';) {
                if (*p != '{') { ++p; continue; }
                const char* obj = p + 1;
                const char* objEnd = static_cast<const char*>(
                    ::memchr(obj, '}', static_cast<std::size_t>(end - obj)));
                if (!objEnd) break;

                Station s;
                s.name = strField(obj, objEnd, "name");
                bool okX = false, okY = false, okZ = false;
                s.world.x = numField(obj, objEnd, "x", okX);
                s.world.y = numField(obj, objEnd, "y", okY);
                s.world.z = numField(obj, objEnd, "z", okZ);
                const std::string ty = strField(obj, objEnd, "type");
                s.type = ty.empty() ? 'S' : ty[0];
                s.line = strField(obj, objEnd, "line");

                if (!s.name.empty() && okX && okY) {
                    // The same station is written into every tile it touches.
                    char id[160];
                    std::snprintf(id, sizeof(id), "%s|%lld|%lld", s.name.c_str(),
                                  static_cast<long long>(std::llround(s.world.x)),
                                  static_cast<long long>(std::llround(s.world.y)));
                    if (seen.insert(id).second) {
                        if (!okZ) s.world.z = 0.0;
                        out.push_back(std::move(s));
                    }
                }
                p = objEnd + 1;
            }
        }
    }

    std::sort(out.begin(), out.end(), [](const Station& a, const Station& b) {
        if (a.line != b.line) return a.line < b.line;
        return a.name < b.name;
    });
    return out;
}

const Station* findStation(const std::vector<Station>& all, const std::string& name) {
    const std::string want = fold(name);
    for (const Station& s : all)
        if (fold(s.name) == want) return &s;
    return nullptr;
}

const Station* pickStation(const std::vector<Station>& all, const std::string& name) {
    if (all.empty()) {
        std::fprintf(stderr, "no stations found in the dataset\n");
        return nullptr;
    }
    const std::string want = name.empty() ? "Bodø" : name;
    if (const Station* s = findStation(all, want)) return s;

    std::fprintf(stderr, "no station called \"%s\". Did you mean:\n", name.c_str());
    for (const std::string& n : nearestNames(all, want)) 
        std::fprintf(stderr, "    %s\n", n.c_str());
    return nullptr;
}

std::vector<std::string> nearestNames(const std::vector<Station>& all,
                                      const std::string& name, std::size_t count) {
    const std::string want = fold(name);
    std::vector<std::pair<std::size_t, const Station*>> scored;
    scored.reserve(all.size());
    for (const Station& s : all) {
        const std::string f = fold(s.name);
        // A prefix match is what someone half-typing meant, whatever the edit distance.
        const std::size_t d = f.rfind(want, 0) == 0 ? 0 : editDistance(want, f);
        scored.push_back({d, &s});
    }
    std::sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) return a.first < b.first;
        return a.second->name < b.second->name;
    });
    std::vector<std::string> out;
    for (std::size_t i = 0; i < scored.size() && i < count; ++i)
        out.push_back(scored[i].second->name + " (" + scored[i].second->line + ")");
    return out;
}
