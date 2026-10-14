// ebaner - a Norwegian railway simulator.
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

#include "RouteQueue.h"

#include <algorithm>
#include <unordered_set>

namespace {
bool shareAny(const std::vector<int>& a, const std::vector<int>& b) {
    const std::unordered_set<int> in(a.begin(), a.end());
    return std::any_of(b.begin(), b.end(), [&](int id) { return in.count(id) != 0; });
}
} // namespace

std::vector<int> routeQueueReady(const std::vector<std::vector<int>>& sections,
                                 const std::vector<char>& blocked,
                                 const std::vector<std::vector<int>>& beyond,
                                 const std::function<bool(int, int)>& opposed) {
    std::vector<int> out;
    const std::size_t n = std::min(sections.size(), blocked.size());
    const bool byRule = beyond.size() >= n && static_cast<bool>(opposed);
    // Whether route `a`, armed before `b`, holds it back. Every route earlier in the queue
    // is asked, whichever of the three things it is doing - going this step, refused by
    // the interlocking, or held back itself by one in front of it. That last case is the
    // one that is easy to get wrong: a route waiting its turn still wants its road, and if
    // it stopped holding it the route armed after it would take it and go first, which is
    // precisely the order the operator armed them in, reversed.
    auto holds = [&](std::size_t a, std::size_t b) {
        if (!byRule) return shareAny(sections[a], sections[b]);
        if (shareAny(beyond[a], beyond[b])) return true;
        return shareAny(sections[a], sections[b]) &&
               opposed(static_cast<int>(a), static_cast<int>(b));
    };
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (blocked[i]) continue;
        bool held = false;
        for (std::size_t j = 0; j < i && !held; ++j) held = holds(j, i);
        if (!held) out.push_back(static_cast<int>(i));
    }
    return out;
}
