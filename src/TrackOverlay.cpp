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

#include "TrackOverlay.h"

#include "TerrainData.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>

namespace fs = std::filesystem;

namespace {
constexpr double kSnapTol = 8.0;             // m; snap an edit end to a real endpoint
std::uint32_t kConnectorIdBase = 0xF0000000; // synthetic connector trackIds
// kRailIdBase (0xE0000000) is declared in TrackOverlay.h so the switch detector can
// recognise editor-added connecting rails.

std::string overlayFile(const std::string& root) {
    return root + "/overlay/track-edits.txt";
}

// Nearest track-segment endpoint to a world point; returns the endpoint and its track
// type, or false if none within tol.
bool nearestEndpoint(const std::vector<TrackSegment>& segs, const glm::dvec3& q,
                     double tol, glm::dvec3& out, std::uint8_t& trackType) {
    double best = tol;
    bool found = false;
    for (const TrackSegment& s : segs) {
        if (s.pts.size() < 2) continue;
        for (const glm::dvec3& e : {s.pts.front(), s.pts.back()}) {
            const double d = std::hypot(e.x - q.x, e.y - q.y);
            if (d < best) {
                best = d;
                out = e;
                trackType = s.trackType;
                found = true;
            }
        }
    }
    return found;
}

// Nearest track *vertex* (any point, not just endpoints) to a world (x,y). Returns
// segment/vertex indices to mutate, or false if none in tol. If trackId != 0, only
// vertices of that track are considered, so an override can target one of several
// coincident siding points.
bool nearestVertex(const std::vector<TrackSegment>& segs, double qx, double qy,
                   double tol, int& segIdx, int& vertIdx, std::uint32_t trackId = 0) {
    double best = tol;
    bool found = false;
    for (int si = 0; si < static_cast<int>(segs.size()); ++si) {
        if (trackId != 0 && segs[si].trackId != trackId) continue;
        const std::vector<glm::dvec3>& p = segs[si].pts;
        for (int vi = 0; vi < static_cast<int>(p.size()); ++vi) {
            const double d = std::hypot(p[vi].x - qx, p[vi].y - qy);
            if (d < best) {
                best = d;
                segIdx = si; vertIdx = vi;
                found = true;
            }
        }
    }
    return found;
}
} // namespace

std::vector<TrackEdit> loadTrackOverlay(const std::string& datasetRoot) {
    std::vector<TrackEdit> edits;
    std::ifstream f(overlayFile(datasetRoot));
    if (!f) return edits;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        char kind[32] = {};
        TrackEdit e;
        const int n = std::sscanf(line.c_str(), "%31s %lf %lf %lf %lf %lf %lf", kind,
                                  &e.a.x, &e.a.y, &e.a.z, &e.b.x, &e.b.y, &e.b.z);
        if (n == 7 && std::string(kind) == "link") {
            e.kind = TrackEdit::Link;
            edits.push_back(e);
        } else if (n == 7 && std::string(kind) == "move") {
            e.kind = TrackEdit::Move; // a = old pos, b = new pos
            edits.push_back(e);
        } else if (n == 7 && std::string(kind) == "rail") {
            e.kind = TrackEdit::Rail; // a, b = the new connecting rail's ends
            edits.push_back(e);
        } else if ((n == 4 || n == 5) && std::string(kind) == "elev") {
            e.kind = TrackEdit::Elev; // a = {x, y, newZ}
            // Optional 4th number (parsed into b.x) is the track id to disambiguate
            // coincident points; absent (legacy) => 0 = match any track.
            if (n == 5) e.track = static_cast<std::uint32_t>(std::llround(e.b.x));
            e.b = glm::dvec3(0.0);
            edits.push_back(e);
        }
    }
    return edits;
}

void applyTrackOverlay(std::vector<TrackSegment>& segs,
                       const std::vector<TrackEdit>& edits) {
    if (edits.empty()) return;
    constexpr double kVertexTol = 2.0; // m; snap an elev override to a real vertex
    int elev = 0, links = 0, rails = 0;

    int moves = 0;
    // Added connecting rails: a new surface segment whose ends sit on the tracks
    // there, so the turnout detection makes a switch at each end (a crossover the
    // export omitted). Added first, so elev/move can target it too if needed.
    for (std::size_t i = 0; i < edits.size(); ++i) {
        if (edits[i].kind != TrackEdit::Rail) continue;
        if (std::hypot(edits[i].a.x - edits[i].b.x, edits[i].a.y - edits[i].b.y) < 1.0)
            continue; // degenerate
        TrackSegment r;
        r.trackId = kRailIdBase + static_cast<std::uint32_t>(i);
        r.trackType = 1;  // siding
        r.medium = 0x20;  // surface (drawn as a real rail, not carved)
        r.pts = {edits[i].a, edits[i].b};
        segs.push_back(std::move(r));
        ++rails;
    }
    // Elevation overrides first, so a link's endpoint reflects any regraded z.
    for (const TrackEdit& e : edits) {
        if (e.kind != TrackEdit::Elev) continue;
        int si = 0, vi = 0;
        if (nearestVertex(segs, e.a.x, e.a.y, kVertexTol, si, vi, e.track)) {
            segs[si].pts[vi].z = e.a.z;
            ++elev;
        }
    }
    // Vertex moves (e.g. snapping a siding end onto the track it crosses).
    for (const TrackEdit& e : edits) {
        if (e.kind != TrackEdit::Move) continue;
        int si = 0, vi = 0;
        if (nearestVertex(segs, e.a.x, e.a.y, kVertexTol, si, vi, e.track)) {
            segs[si].pts[vi] = e.b;
            ++moves;
        }
    }

    for (std::size_t i = 0; i < edits.size(); ++i) {
        if (edits[i].kind != TrackEdit::Link) continue;
        glm::dvec3 ea, eb;
        std::uint8_t ta = 0, tb = 0;
        if (!nearestEndpoint(segs, edits[i].a, kSnapTol, ea, ta)) continue;
        if (!nearestEndpoint(segs, edits[i].b, kSnapTol, eb, tb)) continue;
        if (std::hypot(ea.x - eb.x, ea.y - eb.y) < 1.0) continue; // same point

        // A synthetic connector segment joining the two ends. medium = tunnel so the
        // terrain carve leaves it (the gap is typically inside a mountain anyway),
        // and a unique high trackId so it survives buildTrackPaths' dedup.
        TrackSegment c;
        c.trackId = kConnectorIdBase + static_cast<std::uint32_t>(i);
        c.trackType = ta;      // both ends share type (else the join won't chain)
        c.medium = 0x55;       // tunnel
        c.pts = {ea, eb};
        segs.push_back(std::move(c));
        ++links;
    }
    if (elev > 0 || links > 0 || moves > 0 || rails > 0)
        std::printf("[TrackOverlay] applied %d elev + %d move + %d link + %d rail edit(s)\n",
                    elev, moves, links, rails);
}

namespace {
// Write one overlay line per edit. Full precision: default ostream is 6 significant
// figures, which mangles the ~7-digit UTM coordinates (e.g. 7463588 -> "7.46359e+06")
// so the reload snap fails; fixed with 3 decimals round-trips the float coords well
// within tolerance.
void writeEdits(std::ofstream& f, const std::vector<TrackEdit>& edits) {
    f << std::fixed << std::setprecision(3);
    for (const TrackEdit& e : edits) {
        if (e.kind == TrackEdit::Elev) {
            f << "elev " << e.a.x << ' ' << e.a.y << ' ' << e.a.z;
            if (e.track != 0) f << ' ' << e.track; // disambiguate coincident points
            f << '\n';
        } else {
            const char* kw = e.kind == TrackEdit::Move ? "move"
                             : e.kind == TrackEdit::Rail ? "rail" : "link";
            f << kw << ' ' << e.a.x << ' ' << e.a.y << ' ' << e.a.z << ' ' << e.b.x
              << ' ' << e.b.y << ' ' << e.b.z << '\n';
        }
    }
}
} // namespace

bool appendTrackEdits(const std::string& datasetRoot,
                      const std::vector<TrackEdit>& edits) {
    if (edits.empty()) return true;
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(overlayFile(datasetRoot), std::ios::app);
    if (!f) return false;
    writeEdits(f, edits);
    return static_cast<bool>(f);
}

bool writeTrackOverlay(const std::string& datasetRoot,
                       const std::vector<TrackEdit>& edits) {
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(overlayFile(datasetRoot), std::ios::trunc);
    if (!f) return false;
    writeEdits(f, edits);
    return static_cast<bool>(f);
}
