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

// Arming: which of the queued routes are handed to the interlocking this step, and in
// what order.
//
// No dataset. The thing under test takes circuit ids and a blocked flag per queued route
// and answers with indices, so a handful of small integers is the whole world here - and
// the question it answers is about order alone, never about whether a route may be set.

#include "RouteQueue.h"

#include <cstdio>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  %-66s %s\n", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

std::string show(const std::vector<int>& v) {
    std::string s = "[";
    for (std::size_t i = 0; i < v.size(); ++i)
        s += (i ? " " : "") + std::to_string(v[i]);
    return s + "]";
}

void same(const std::vector<int>& got, const std::vector<int>& want, const std::string& what) {
    const bool ok = got == want;
    std::printf("  %-66s %s", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) std::printf(" (got %s, want %s)", show(got).c_str(), show(want).c_str());
    std::putchar('\n');
    if (!ok) ++failures;
}

} // namespace

int main() {
    std::printf("route queue: the order armed routes are taken in\n\n");

    // Three routes over quite separate roads.
    const std::vector<std::vector<int>> apart = {{1, 2}, {3, 4}, {5, 6}};

    std::printf("nothing in the way:\n");
    same(routeQueueReady(apart, {0, 0, 0}), {0, 1, 2},
         "all three free -> all tried, in arm order");

    std::printf("\nan armed route waiting holds the road against what comes after it:\n");
    {
        // 0 and 1 share circuit 2; 2 shares nothing with either.
        const std::vector<std::vector<int>> shared = {{1, 2}, {2, 3}, {7, 8}};
        same(routeQueueReady(shared, {1, 0, 0}), {2},
             "head blocked -> the one sharing its road is held, the other goes");
        same(routeQueueReady(shared, {0, 0, 0}),  {0, 2},
             "none blocked but two share road -> only the earlier of the two");
        same(routeQueueReady(shared, {1, 1, 0}), {2},
             "both of the pair blocked -> the unrelated one still goes");
        same(routeQueueReady(shared, {0, 1, 0}), {0, 2},
             "a blocked route holds nothing from the one armed before it");
    }

    std::printf("\nthe case that separates this from a strict queue:\n");
    same(routeQueueReady(apart, {1, 0, 0}), {1, 2},
         "head stuck on its own road -> the others are not held up by it");
    same(routeQueueReady(apart, {1, 1, 0}), {2},
         "two stuck -> the third, sharing nothing, still goes");

    std::printf("\na blocked route in the middle holds only what it touches:\n");
    {
        //      0:{1,2}   1:{4,5} blocked   2:{5,6} shares 5   3:{9} shares nothing
        const std::vector<std::vector<int>> mid = {{1, 2}, {4, 5}, {5, 6}, {9}};
        same(routeQueueReady(mid, {0, 1, 0, 0}), {0, 3},
             "only the one sharing with it is held back");
    }

    std::printf("\nchains hold transitively, because each waiting route adds its own road:\n");
    {
        //  0:{1} blocked   1:{1,2} held by 0   2:{2,3} held by 1   3:{4} free
        const std::vector<std::vector<int>> chain = {{1}, {1, 2}, {2, 3}, {4}};
        same(routeQueueReady(chain, {1, 0, 0, 0}), {3},
             "a route held back goes on holding the road for the next one");
    }

    std::printf("\nedges:\n");
    same(routeQueueReady({}, {}), {}, "an empty queue asks for nothing");
    same(routeQueueReady(apart, {1, 1, 1}), {}, "everything blocked -> nothing tried");
    same(routeQueueReady({{}, {}}, {0, 0}), {0, 1},
         "routes with no circuits at all never hold each other");
    same(routeQueueReady({{1, 2}}, {}), {},
         "a blocked-flag list shorter than the queue is read only as far as it goes");
    {
        const std::vector<std::vector<int>> dup = {{1, 1, 2}, {2}};
        same(routeQueueReady(dup, {0, 0}), {0},
             "a repeated circuit id is still just that circuit");
    }

    std::printf("\n%s\n", failures ? "FAILURES" : "all ok");
    return failures ? 1 : 0;
}
