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

#include "Paths.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>

#ifdef _WIN32
// Both before windows.h or it defines min/max as macros and brings in half the SDK,
// either of which breaks <filesystem> above in ways that are tedious to read. Guarded
// because they are not ours alone: the build defines them too, and mingw's libstdc++
// has already set NOMINMAX by the time any of this is read.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <unistd.h> // access(), for the X_OK that decides a PATH hit
#endif

// Set by CMake: the install prefix's share/ebaner, as an absolute path.
#ifndef EBANER_DATADIR
#define EBANER_DATADIR ""
#endif

namespace fs = std::filesystem;

namespace Paths {
namespace {

std::string g_argv0;
std::vector<std::string> g_searched;

// An environment variable, or "" - including when it is set but empty, which is how a
// shell unsets one in passing and is never a path anybody means.
std::string env(const char* name) {
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : std::string();
}

// Add `p` to the list unless it is empty or already there. Candidates are gathered
// before any of them is tested, so the list of places looked is complete whichever one
// wins - and the duplicate is worth dropping, because the configured datadir is very
// often one of the XDG directories as well and a list that says /usr/local/share/ebaner
// twice reads as a bug in the search.
void add(std::vector<std::string>& out, const std::string& p) {
    if (p.empty()) return;
    for (const std::string& have : out)
        if (have == p) return;
    out.push_back(p);
}

#ifndef _WIN32
// Split a PATH-style list on ':'. This is for XDG_DATA_DIRS, which is a Unix idea; the
// PATH walk below does its own splitting, because it has a separator and an extension
// list of its own to think about.
void addList(std::vector<std::string>& out, const std::string& list, const char* suffix) {
    std::size_t i = 0;
    while (i <= list.size()) {
        const std::size_t j = list.find(':', i);
        const std::string one = list.substr(i, j == std::string::npos ? j : j - i);
        add(out, one.empty() ? std::string() : one + suffix);
        if (j == std::string::npos) break;
        i = j + 1;
    }
}
#endif

bool isDir(const std::string& p) {
    if (p.empty()) return false;
    std::error_code ec; // the throwing overloads would turn a stale mount into a crash
    return fs::is_directory(p, ec);
}

// The first candidate that is a directory, or "".
std::string firstDir(const std::vector<std::string>& cand) {
    for (const std::string& c : cand)
        if (isDir(c)) return c;
    return {};
}

// Whether this is a file that could be run - which is what makes a PATH entry the
// answer rather than merely a directory that has something of the right name in it.
bool isExecutableFile(const fs::path& p) {
    std::error_code ec;
    if (!fs::is_regular_file(p, ec)) return false;
#ifdef _WIN32
    return true; // the extension already decided it; Windows has no execute bit
#else
    return ::access(p.c_str(), X_OK) == 0;
#endif
}

// Find `name` on PATH, the way the shell that launched us did, and give back the
// directory it was found in.
//
// This is how a program invoked as a bare name knows where it is. It is the same search
// execvp performs, against the same PATH the caller used, and it finds the same file -
// the one actually running. The cases where it can differ are ones nothing else would
// survive either: the binary renamed or deleted since it started, or PATH rewritten by
// the program itself before this is called.
std::string dirOnPath(const std::string& name) {
    const std::string path = env("PATH");
    if (path.empty()) return {};
#ifdef _WIN32
    const char sep = ';';
    // Each name is tried bare and then with every extension Windows considers
    // executable, since argv[0] there is as often "ebaner" as "ebaner.exe".
    std::vector<std::string> exts{""};
    const std::string pathext = env("PATHEXT");
    for (std::size_t i = 0; i <= pathext.size();) {
        const std::size_t j = pathext.find(';', i);
        const std::string e = pathext.substr(i, j == std::string::npos ? j : j - i);
        if (!e.empty()) exts.push_back(e);
        if (j == std::string::npos) break;
        i = j + 1;
    }
#else
    const char sep = ':';
    const std::vector<std::string> exts{""};
#endif
    for (std::size_t i = 0; i <= path.size();) {
        const std::size_t j = path.find(sep, i);
        std::string dir = path.substr(i, j == std::string::npos ? j : j - i);
        // An empty entry means the working directory - "::" and a leading or trailing
        // separator are all ways of writing it, and all of them are in real PATHs.
        if (dir.empty()) dir = ".";
        for (const std::string& e : exts) {
            const fs::path cand = fs::path(dir) / (name + e);
            if (isExecutableFile(cand)) {
                std::error_code ec;
                const fs::path abs = fs::absolute(cand, ec);
                // Normalised only so that the "." an empty PATH entry stands for does
                // not come back glued to the end of an otherwise good directory.
                return (ec ? cand : abs).parent_path().lexically_normal().string();
            }
        }
        if (j == std::string::npos) break;
        i = j + 1;
    }
    return {};
}

} // namespace

void init(const char* argv0) { g_argv0 = argv0 ? argv0 : ""; }

std::string exeDir() {
#ifdef _WIN32
    // Not testable on the machine this was written on: Win32 is a later port and this
    // is the one piece of it written blind. GetModuleFileNameW with a null module is
    // the running .exe; it does not null-terminate on truncation, hence the grow loop.
    //
    // The wide path is narrowed on the way out because every path in the program is a
    // std::string, loaders included. That is the existing shape of the code rather
    // than a decision made here, and it means an install under a directory with
    // characters outside the active code page will not be found. Worth fixing when
    // there is a Windows machine to fix it against; not worth guessing at now.
    std::wstring buf(MAX_PATH, L'\0');
    for (;;) {
        const DWORD n = GetModuleFileNameW(nullptr, buf.data(),
                                           static_cast<DWORD>(buf.size()));
        if (n == 0) break;                   // failed outright; fall through to argv[0]
        if (n < buf.size()) {                // fits, so it is terminated and complete
            buf.resize(n);
            std::error_code ec;
            const fs::path exe(buf);
            return exe.parent_path().string();
        }
        if (buf.size() > 32768) break;       // past any real Windows path length
        buf.resize(buf.size() * 2);
    }
#else
    std::error_code ec;
    const fs::path self = fs::read_symlink("/proc/self/exe", ec);
    if (!ec && !self.empty()) return self.parent_path().string();
#endif
    // No OS answer, so fall back to how we were invoked. A path - absolute or relative -
    // names the file directly. A bare name means the shell found it on PATH, so look
    // there for it the same way the shell did.
    if (g_argv0.empty()) return {};
    const fs::path a(g_argv0);
    if (a.has_parent_path()) {
        std::error_code ec2;
        const fs::path abs = fs::absolute(a, ec2);
        return (ec2 ? a : abs).parent_path().string();
    }
    return dirOnPath(a.filename().string());
}

bool looksLikeDataset(const std::string& dir) {
    return isDir(dir) && isDir((fs::path(dir) / "tiles").string());
}

std::string shaderDir() {
    std::vector<std::string> cand;
    add(cand, env("EBANER_SHADERS"));
    const std::string exe = exeDir();
    if (!exe.empty()) {
        // Beside the binary, which is both the Windows install layout and - without any
        // special case for it - the build tree, where CMake puts the compiled .spv
        // files in build/shaders next to build/ebaner. Then the GNU install, which
        // splits them across bin/ and share/ebaner/.
        add(cand, (fs::path(exe) / "shaders").string());
        add(cand, (fs::path(exe) / ".." / "share" / "ebaner" / "shaders").string());
    }
    if (*EBANER_DATADIR) add(cand, (fs::path(EBANER_DATADIR) / "shaders").string());
    const std::string found = firstDir(cand);
    if (!found.empty()) return found;
    // A broken install. Say where it looked - the renderer can only report the one file
    // it failed to open - and hand back the likeliest of them so that it names a real
    // path when it does.
    std::fprintf(stderr, "ebaner: no compiled shaders found. Looked in:\n");
    for (const std::string& c : cand) std::fprintf(stderr, "  %s\n", c.c_str());
    return cand.empty() ? std::string("shaders") : cand.front();
}

std::string datasetRoot(const char* fromArgv) {
    g_searched.clear();
    // Given on the command line: taken as given. Searching on from a path the user
    // actually typed would load a different dataset than the one they asked for.
    if (fromArgv && *fromArgv) return fromArgv;

    add(g_searched, env("EBANER_DATA"));
#ifndef _WIN32
    // The user's own, which beats anything installed system-wide.
    const std::string home = env("HOME");
    const std::string xdgHome = env("XDG_DATA_HOME");
    if (!xdgHome.empty())
        add(g_searched, (fs::path(xdgHome) / "ebaner").string());
    else if (!home.empty())
        add(g_searched, (fs::path(home) / ".local" / "share" / "ebaner").string());
#endif
    const std::string exe = exeDir();
    if (!exe.empty()) {
        // Beside the binary - the Windows layout, where there is nowhere else to look -
        // and then the relocatable Unix one, bin/../share/ebaner, which finds an install
        // under any prefix without the prefix having been compiled in.
        add(g_searched, (fs::path(exe) / "data").string());
        add(g_searched, (fs::path(exe) / ".." / "share" / "ebaner").string());
    }
#ifndef _WIN32
    const std::string xdgDirs = env("XDG_DATA_DIRS");
    addList(g_searched, xdgDirs.empty() ? "/usr/local/share:/usr/share" : xdgDirs,
            "/ebaner");
#endif
    if (*EBANER_DATADIR) add(g_searched, EBANER_DATADIR);

    for (const std::string& c : g_searched)
        if (looksLikeDataset(c)) return c;
    return {};
}

const std::vector<std::string>& searchedForDataset() { return g_searched; }

void reportMissingDataset(const char* programName) {
    std::fprintf(stderr,
                 "%s: no terrainmapper export found. It is installed separately - the\n"
                 "tiles run to tens of gigabytes and are not part of this repository.\n\n"
                 "Give one on the command line, set EBANER_DATA, or put one where it\n"
                 "will be found. Looked in:\n",
                 programName);
    for (const std::string& c : g_searched) {
        std::error_code ec;
        const fs::path abs = fs::weakly_canonical(c, ec);
        std::fprintf(stderr, "  %s%s\n", (ec ? fs::path(c) : abs).string().c_str(),
                     isDir(c) ? "  (exists, but holds no tiles/)" : "");
    }
}

} // namespace Paths
