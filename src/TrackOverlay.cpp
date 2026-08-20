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
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
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

// Nearest track-segment endpoint to a world point; returns the endpoint with its track
// type and medium, or false if none within tol.
bool nearestEndpoint(const std::vector<TrackSegment>& segs, const glm::dvec3& q,
                     double tol, glm::dvec3& out, std::uint8_t& trackType,
                     std::uint8_t& medium) {
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
                medium = s.medium;
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
                   double tol, int& segIdx, int& vertIdx, std::uint32_t trackId = 0,
                   const double* fromZ = nullptr) {
    // With `fromZ` the match is on height among everything within the tolerance, not on
    // position: two vertices at one spot are equally near it, so position cannot choose
    // and whichever came first would always win.
    double best = fromZ ? 1e30 : tol;
    bool found = false;
    for (int si = 0; si < static_cast<int>(segs.size()); ++si) {
        if (trackId != 0 && segs[si].trackId != trackId) continue;
        const std::vector<glm::dvec3>& p = segs[si].pts;
        for (int vi = 0; vi < static_cast<int>(p.size()); ++vi) {
            const double d = std::hypot(p[vi].x - qx, p[vi].y - qy);
            if (d >= tol) continue;
            const double score = fromZ ? std::abs(p[vi].z - *fromZ) : d;
            if (score < best) {
                best = score;
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
        // A drawn track carries a point list of no fixed length, so it is read off a
        // token stream rather than through the shared sscanf below.
        if (line.compare(0, 6, "track ") == 0) {
            std::istringstream is(line);
            std::string tok, idTok, typeTok;
            is >> tok >> idTok >> typeTok;
            if (!is) continue;
            e.kind = TrackEdit::Track;
            e.track = static_cast<std::uint32_t>(
                std::strtoul(idTok.c_str(), nullptr, 16)); // strtoul: a bad id is 0, not a throw
            e.trackType = typeTok == "yard" ? 2 : 1;
            glm::dvec3 p(0.0);
            while (is >> p.x >> p.y >> p.z) e.pts.push_back(p);
            if (e.pts.size() >= 2) edits.push_back(e);
            continue;
        }
        const int n = std::sscanf(line.c_str(), "%31s %lf %lf %lf %lf %lf %lf", kind,
                                  &e.a.x, &e.a.y, &e.a.z, &e.b.x, &e.b.y, &e.b.z);
        if (n == 7 && std::string(kind) == "link") {
            e.kind = TrackEdit::Link;
            // Anything after the six numbers names the connector's medium. sscanf stops
            // at the last one it was asked for and ignores the rest, so the tail is read
            // separately rather than by widening the format.
            std::istringstream is(line);
            std::string tok;
            for (int k = 0; k < 7 && (is >> tok); ++k) {} // the keyword and six numbers
            while (is >> tok) {
                if (tok == "tunnel") e.medium = 0x55;
                else if (tok == "surface") e.medium = 0x20;
            }
            edits.push_back(e);
        } else if (n == 7 && std::string(kind) == "move") {
            e.kind = TrackEdit::Move; // a = old pos, b = new pos
            // An optional track id after the six numbers, read off the tail the way a
            // link's medium is: which track's vertex is meant, where several tracks meet
            // at one spot. Absent (legacy) = whichever track is nearest. Written in
            // decimal, as an elev's is: the two are the vertex edits and end up side by
            // side naming the same points, and one number for a track is enough.
            std::istringstream is(line);
            std::string tok;
            for (int k = 0; k < 7 && (is >> tok); ++k) {}
            if (is >> tok)
                e.track = static_cast<std::uint32_t>(std::strtoul(tok.c_str(), nullptr, 10));
            edits.push_back(e);
        } else if (n == 7 && std::string(kind) == "rail") {
            e.kind = TrackEdit::Rail; // a, b = the new connecting rail's ends
            edits.push_back(e);
        } else if (n >= 4 && n <= 6 && std::string(kind) == "elev") {
            e.kind = TrackEdit::Elev; // a = {x, y, newZ}
            // Optional 4th number (parsed into b.x) is the track id to disambiguate
            // coincident points; absent (legacy) => 0 = match any track. An optional 5th
            // (b.y) is the height the vertex has now, which is what separates two of them
            // at the same spot on the same track.
            if (n >= 5) e.track = static_cast<std::uint32_t>(std::llround(e.b.x));
            if (n == 6) { e.hasFromZ = true; e.fromZ = e.b.y; }
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
    int elev = 0, links = 0, rails = 0, tracks = 0;

    int moves = 0;
    // Whole drawn roads. Added before the elev and move edits so those can regrade and
    // nudge their points afterwards, which is how a drawn siding gets its elevation:
    // the mode that draws it lays it out flat at one height on purpose.
    for (const TrackEdit& e : edits) {
        if (e.kind != TrackEdit::Track || e.pts.size() < 2) continue;
        TrackSegment t;
        t.trackId = e.track;
        t.trackType = e.trackType;
        t.medium = 0x20; // surface
        t.pts = e.pts;
        segs.push_back(std::move(t));
        ++tracks;
    }
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
    // The vertex edits, in the order they are written down - both kinds in one pass.
    // Each names a vertex by where it is *now*, so a move made after a regrade has to
    // see the regraded height and a regrade made after a move has to see the moved
    // position. Two passes by kind can only ever satisfy one of those, and the editor
    // authors both against what is on screen, which is the state after everything
    // above the new line. Done before the links, so a link's endpoint picks up any
    // vertex the edits above it have shifted.
    for (const TrackEdit& e : edits) {
        int si = 0, vi = 0;
        if (e.kind == TrackEdit::Elev) {
            if (nearestVertex(segs, e.a.x, e.a.y, kVertexTol, si, vi, e.track,
                              e.hasFromZ ? &e.fromZ : nullptr)) {
                segs[si].pts[vi].z = e.a.z;
                ++elev;
            }
        } else if (e.kind == TrackEdit::Move) {
            // A move carries the whole old position, so its z separates two vertices
            // sharing a spot without a `fromz` of its own - the same problem an elev
            // needs that keyword for.
            if (nearestVertex(segs, e.a.x, e.a.y, kVertexTol, si, vi, e.track, &e.a.z)) {
                segs[si].pts[vi] = e.b;
                ++moves;
            }
        }
    }

    for (std::size_t i = 0; i < edits.size(); ++i) {
        if (edits[i].kind != TrackEdit::Link) continue;
        glm::dvec3 ea, eb;
        std::uint8_t ta = 0, tb = 0, ma = 0x20, mb = 0x20;
        if (!nearestEndpoint(segs, edits[i].a, kSnapTol, ea, ta, ma)) continue;
        if (!nearestEndpoint(segs, edits[i].b, kSnapTol, eb, tb, mb)) continue;
        if (std::hypot(ea.x - eb.x, ea.y - eb.y) < 1.0) continue; // same point

        // A synthetic connector segment joining the two ends, with a unique high trackId
        // so it survives buildTrackPaths' dedup.
        //
        // It takes the medium of what it joins. A gap at a tunnel mouth, or inside the
        // mountain, is tunnel: the terrain carve leaves it alone and the bore runs
        // through. A gap between two surface tracks is surface - call that one tunnel
        // and a bore grows along it, leaving a length of rock tube lying in the open
        // where the missing embankment should be.
        auto underground = [](std::uint8_t m) { return m == 0x55 || m == 0x54; };
        TrackSegment c;
        c.trackId = kConnectorIdBase + static_cast<std::uint32_t>(i);
        c.trackType = ta;      // both ends share type (else the join won't chain)
        c.medium = edits[i].medium ? edits[i].medium
                                   : ((underground(ma) || underground(mb)) ? 0x55 : 0x20);
        c.pts = {ea, eb};
        segs.push_back(std::move(c));
        ++links;
    }
    if (elev > 0 || links > 0 || moves > 0 || rails > 0 || tracks > 0)
        std::printf("[TrackOverlay] applied %d elev + %d move + %d link + %d rail + "
                    "%d track edit(s)\n",
                    elev, moves, links, rails, tracks);
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
            if (e.track != 0 || e.hasFromZ) f << ' ' << e.track; // coincident points
            if (e.hasFromZ) f << ' ' << e.fromZ;                 // ...on the same track
            f << '\n';
        } else if (e.kind == TrackEdit::Track) {
            f << "track " << std::hex << e.track << std::dec << ' '
              << (e.trackType == 2 ? "yard" : "siding");
            for (const glm::dvec3& p : e.pts)
                f << ' ' << p.x << ' ' << p.y << ' ' << p.z;
            f << '\n';
        } else {
            const char* kw = e.kind == TrackEdit::Move ? "move"
                             : e.kind == TrackEdit::Rail ? "rail" : "link";
            f << kw << ' ' << e.a.x << ' ' << e.a.y << ' ' << e.a.z << ' ' << e.b.x
              << ' ' << e.b.y << ' ' << e.b.z;
            if (e.kind == TrackEdit::Move && e.track != 0)
                f << ' ' << e.track; // which track's vertex, as an elev names it
            if (e.kind == TrackEdit::Link && e.medium == 0x55) f << " tunnel";
            else if (e.kind == TrackEdit::Link && e.medium == 0x20) f << " surface";
            f << '\n';
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
