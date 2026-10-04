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
#include <vector>

// Where the program's files are, once it is installed somewhere rather than run out of
// the tree it was built in.
//
// Two things have to be found and they are found differently. The SHADERS are built
// with the program and installed beside it, so they are always somewhere relative to the
// binary. The DATASET is a terrainmapper export of tens of gigabytes, installed by hand
// and separately - it cannot go in a repository and it does not belong in a package - so
// the program has to go looking for it in the places a Unix system keeps such a thing.
//
// Everything here is a SEARCH: a list of candidates, first one that exists wins, and the
// list is kept so a failure can say where it looked. Nothing is created and nothing is
// written - this is the reading side only.
//
// No development path is searched and none is compiled in. Working on the program means
// saying where the data is, on the command line or in EBANER_DATA; the shaders need no
// such help, because the build tree puts them beside the binary exactly as a Windows
// install does. A guessed-at tree is one more place a wrong dataset can be found by
// accident, and it only ever shortened a command that is typed by the person who knows
// the answer anyway.
namespace Paths {

// Remember argv[0]. Worth calling first thing in main, though not required: the
// executable's own directory is read from the OS where that is possible (/proc/self/exe
// on Linux, GetModuleFileName on Windows) and argv[0] is only the fallback for the
// platforms where it is not.
void init(const char* argv0);

// The directory the running executable is in, or "" if it cannot be determined.
//
// Asked of the OS first. Failing that, argv[0]: a path is used as given, and a bare name
// is looked up on PATH the same way the shell that launched us did - same search, same
// PATH, same file.
std::string exeDir();

// Where the compiled .spv shaders are. $EBANER_SHADERS, then beside the binary - which
// is the build tree and a Windows install both - then the GNU share/ebaner/shaders.
std::string shaderDir();

// The root of a terrainmapper export - the directory holding `tiles/`.
//
// `fromArgv` is the path given on the command line, or nullptr. An explicit path always
// wins and is returned whether or not it exists, so that a wrong path on the command
// line is reported as the wrong path it is rather than silently replaced by some other
// dataset that happens to be installed.
//
// Returns "" when nothing was found; ask searchedForDataset() what was tried.
std::string datasetRoot(const char* fromArgv);

// Every place datasetRoot() looked, in the order it looked, for the error message. Only
// meaningful after a call to it.
const std::vector<std::string>& searchedForDataset();

// Print to stderr what datasetRoot() tried and what a dataset is, for the three
// programs that cannot run without one. Here rather than in each of them so the list of
// places and the advice stay the same wherever it is reported from.
void reportMissingDataset(const char* programName);

// Whether this directory looks like an export rather than merely existing. The tiles are
// what every loader needs, so their presence is the test.
bool looksLikeDataset(const std::string& dir);

} // namespace Paths
