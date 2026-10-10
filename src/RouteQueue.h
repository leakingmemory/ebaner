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

#include <vector>

// Which armed routes to try this step, as indices into the queue, in arm order.
//
// A route is tried unless something armed ahead of it, and still waiting, wants a circuit
// it wants too. Sharing a circuit is the whole test, and it is deliberately coarser than
// the conflict rule the interlocking itself applies: two routes set end to end through a
// station share the platform road without conflicting, and this holds the second of them
// back for a step anyway. That costs nothing - it is tried again as soon as the first is
// set - and it buys the property actually wanted, which is that two moves over the same
// rails are taken in the order they were armed whatever the reason the first is waiting.
//
// Being coarse is safe in one direction only, and this is that direction: the test can
// delay a route, never permit one.
//
// Every route in the queue holds its circuits against those armed after it, whether it is
// going this step, refused by the interlocking, or held back here by one in front - a
// route waiting its turn that stopped holding its road would be overtaken by the very
// route it is meant to go before. `sections` and `blocked` are parallel to the queue;
// `sections[i]` is the track-circuit ids route i runs through and `blocked[i]` whether the
// interlocking refuses it as things stand.
std::vector<int> routeQueueReady(const std::vector<std::vector<int>>& sections,
                                 const std::vector<char>& blocked);
