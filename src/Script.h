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

#pragma once

#include <string>

// Behaviour authored alongside the data.
//
// Everything in the overlay so far is a fact: a crossing is a line in a file, a TXP
// position is a line in a file. Some of what a railway needs is not a fact but a rule -
// a timetable, a train that runs itself, when a station is worked - and there is nowhere
// to write a rule down. This is that place: `<datasetRoot>/overlay/overlay.lua`, run once
// at startup.
//
// Nothing of the simulator is exposed to it yet. This is the path from the library being
// found at build time to a script's output reaching the terminal, and no more.
//
// Lua is optional. Built without it, a dataset carrying a script is told so rather than
// left wondering why nothing happened - which is the one way this differs from the audio
// it is otherwise modelled on, where silence is a reasonable thing to leave unsaid.
class Script {
public:
    Script() = default;
    ~Script();
    Script(const Script&) = delete;
    Script& operator=(const Script&) = delete;

    // Run the dataset's script, if it has one. False when nothing ran - no file, no Lua
    // in the build, or the script failed - none of which is fatal and only the last of
    // which is an error.
    bool run(const std::string& datasetRoot);

    // Whether the script left a global of this name behind. The state outlives `run`, so
    // what a script defines stays defined; this is how the hooks it will later be asked
    // for get found.
    bool hasGlobal(const std::string& name) const;

    // Close the interpreter. Called from the destructor too, so a caller that forgets
    // leaks nothing.
    void shutdown();

    // Whether this build can run a script at all.
    static bool available();

private:
    void* L_ = nullptr; // lua_State* (opaque here, so no Lua headers escape the .cpp)
};
