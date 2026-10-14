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

#pragma once

// Arming a route: the order blocked routes are taken in once their road frees.
//
// A route the operator arms is one the interlocking refuses right now - the platform is
// occupied, a switch is held by another departure, the line is claimed the other way. The
// arming says "take it the moment you can", and arming several says "take them in this
// order". Both are the operator's intent about *timing*, which is the only thing decided
// here.
//
// Nothing in this file decides whether a route may be set. That stays where it already
// is: the interlocking asks exitRouteBlocked, and a route released by the ordering below
// still goes through trySetExitRoute - the same call the operator's own keypress makes,
// with every occupancy, conflict, line-block and switch-lock check it has always run. A
// queue that granted anything would be a second interlocking, and two of those disagreeing
// is exactly the fault worth never having.

#include <functional>
#include <vector>

// Which armed routes to try this step, as indices into the queue, in arm order.
//
// A route is tried unless something armed ahead of it, and still waiting, contends with it
// for the same road - by the interlocking's own rule when `beyond` and `opposed` are given:
// a circuit lying beyond both their signals, or any circuit shared by two that run toward
// each other. Two routes set end to end through a station share the platform road without
// contending - an entry finishing on the road a departure from it begins on - and those are
// not held one behind the other.
//
// They used to be, on the reasoning that holding the second back cost a step at most. It
// cost a deadlock: an entry into a platform armed first, and the departure off it armed
// after. The entry waits for the platform to clear, the platform is the train waiting to
// depart, and the departure waited behind the entry. Bergsgrav stalled that way. Without
// `beyond` the test is still the old coarse one, any circuit shared, which is the only
// thing a caller with nothing else to go on can safely use.
//
// Holding a route back is safe in one direction only, and this is that direction: the
// test can delay a route, never permit one - whatever it answers, the interlocking still
// decides.
//
// Every route in the queue holds its circuits against those armed after it, whether it is
// going this step, refused by the interlocking, or held back here by one in front - a
// route waiting its turn that stopped holding its road would be overtaken by the very
// route it is meant to go before. `sections` and `blocked` are parallel to the queue;
// `sections[i]` is the track-circuit ids route i runs through and `blocked[i]` whether the
// interlocking refuses it as things stand.
// `beyond[i]` is the circuits past route i's signal; `opposed(a, b)` whether queued routes a
// and b run toward each other over what they share.
std::vector<int> routeQueueReady(const std::vector<std::vector<int>>& sections,
                                 const std::vector<char>& blocked,
                                 const std::vector<std::vector<int>>& beyond = {},
                                 const std::function<bool(int, int)>& opposed = {});
