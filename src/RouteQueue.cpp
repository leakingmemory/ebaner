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

std::vector<int> routeQueueReady(const std::vector<std::vector<int>>& sections,
                                 const std::vector<char>& blocked) {
    std::vector<int> out;
    // The circuits spoken for by something earlier in the queue. Every route adds its
    // own, whichever of the three things it is doing - going this step, refused by the
    // interlocking, or held back here by one in front of it. That last case is the one
    // that is easy to get wrong and the reason this is a single set rather than a test
    // against the blocked ones: a route waiting its turn still wants its road, and if it
    // stopped holding it the route armed after it would take it and go first, which is
    // precisely the order the operator armed them in, reversed.
    std::unordered_set<int> held;
    const std::size_t n = std::min(sections.size(), blocked.size());
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const std::vector<int>& want = sections[i];
        const bool taken = std::any_of(want.begin(), want.end(),
                                       [&](int id) { return held.count(id) != 0; });
        if (!blocked[i] && !taken) out.push_back(static_cast<int>(i));
        held.insert(want.begin(), want.end());
    }
    return out;
}
