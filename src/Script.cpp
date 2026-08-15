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

#include "Script.h"

#include <cstdio>
#include <fstream>

#if HAVE_LUA
#include <lua.hpp>
#endif

namespace {

std::string scriptFile(const std::string& root) {
    return root + "/overlay/overlay.lua";
}

// Whether the dataset carries a script at all. Read rather than stat'd, to match how
// every other overlay decides the same thing.
bool haveScript(const std::string& root) {
    std::ifstream f(scriptFile(root));
    return static_cast<bool>(f);
}

} // namespace

bool Script::available() {
#if HAVE_LUA
    return true;
#else
    return false;
#endif
}

Script::~Script() { shutdown(); }

#if HAVE_LUA

bool Script::run(const std::string& datasetRoot) {
    const std::string path = scriptFile(datasetRoot);
    if (!haveScript(datasetRoot)) return false; // no script: the ordinary case, said quietly

    if (!L_) {
        L_ = luaL_newstate();
        if (!L_) {
            std::fprintf(stderr, "[Script] out of memory opening Lua\n");
            return false;
        }
        // The whole standard library, io and os included. The script is in the user's own
        // dataset and written by them, so a sandbox here would keep nobody out and would
        // rule out the file work a scripted authoring task wants to do.
        luaL_openlibs(static_cast<lua_State*>(L_));
    }
    lua_State* L = static_cast<lua_State*>(L_);

    // Loading and calling are separate failures worth telling apart: one is a script that
    // does not parse, the other one that parsed and then went wrong.
    if (luaL_loadfile(L, path.c_str()) != LUA_OK) {
        std::fprintf(stderr, "[Script] overlay.lua: %s\n", lua_tostring(L, -1));
        lua_pop(L, 1);
        return false;
    }
    if (lua_pcall(L, 0, 0, 0) != LUA_OK) {
        std::fprintf(stderr, "[Script] overlay.lua: %s\n", lua_tostring(L, -1));
        lua_pop(L, 1);
        return false;
    }
    std::printf("[Script] ran overlay.lua\n");
    return true;
}

bool Script::hasGlobal(const std::string& name) const {
    if (!L_) return false;
    lua_State* L = static_cast<lua_State*>(L_);
    lua_getglobal(L, name.c_str());
    const bool set = !lua_isnil(L, -1);
    lua_pop(L, 1);
    return set;
}

void Script::shutdown() {
    if (!L_) return;
    lua_close(static_cast<lua_State*>(L_));
    L_ = nullptr;
}

#else // no Lua in this build

bool Script::run(const std::string& datasetRoot) {
    // Only worth a word when there was something to run. A dataset without a script is
    // not missing anything, but one with a script that never runs would otherwise look
    // like the script itself was at fault.
    if (haveScript(datasetRoot))
        std::fprintf(stderr,
                     "script: overlay.lua present but built without Lua; not run\n");
    return false;
}

bool Script::hasGlobal(const std::string&) const { return false; }
void Script::shutdown() {}

#endif
