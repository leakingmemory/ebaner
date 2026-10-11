# Installing ebaner on Linux

ebaner comes in two parts that are installed separately:

1. **The program** — `ebaner`, `ebaner-trackedit` and `ebaner-dumptrack`, their
   compiled shaders, and the desktop entries. Built from this repository.
2. **The dataset** — the `norway-rails` terrainmapper export the simulator drives over
   (about 1.5 GB compressed), plus the **overlay** of signalling, interlocking and track
   edits that is authored on top of it in its own repository. Neither is in this
   repository, and `cmake --install` does not install them.

The program will not start without the dataset, so both steps are needed.

On Gentoo, an unofficial third-party overlay can do both — see [Gentoo](#gentoo) at the
end.

## 1. Dependencies

What the build needs:

| | |
|---|---|
| CMake ≥ 3.20, a C++20 compiler, pkg-config | build |
| Vulkan loader and headers | required |
| GLFW 3 | required |
| GLM | required (headers only) |
| `glslc`, from shaderc | required, to compile the shaders |
| PulseAudio (`libpulse-simple`) or PortAudio | optional — without either the sim is silent |
| Lua **5.4** | optional — without it the overlay's `overlay.lua` is not run |

Lua has to be 5.4. LuaJIT is not accepted even when it is installed, since it would
give 5.1 semantics. A working Vulkan driver for your GPU (Mesa's `radv`/`anv`, or the
vendor driver) is also needed at run time.

**Debian / Ubuntu**

```sh
sudo apt install build-essential cmake pkg-config git \
    libvulkan-dev glslc libglfw3-dev libglm-dev \
    libpulse-dev portaudio19-dev liblua5.4-dev
```

**Fedora**

```sh
sudo dnf install gcc-c++ cmake pkgconf git \
    vulkan-loader-devel vulkan-headers glslc glfw-devel glm-devel \
    pulseaudio-libs-devel portaudio-devel lua-devel
```

**Arch Linux**

```sh
sudo pacman -S --needed base-devel cmake git \
    vulkan-icd-loader vulkan-headers shaderc glfw glm \
    libpulse portaudio lua
```

The CMake summary says which of the optional parts it found — look for the `Lua found`
line and the audio backend before building, rather than discovering a silent sim
afterwards.

## 2. Build and install the program

```sh
git clone https://github.com/leakingmemory/ebaner.git
cd ebaner
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j
sudo cmake --install build
```

This installs:

| | |
|---|---|
| `$prefix/bin` | `ebaner`, `ebaner-trackedit`, `ebaner-dumptrack` |
| `$prefix/share/ebaner/shaders` | the compiled `.spv` shaders |
| `$prefix/share/applications` | `ebaner.desktop`, `ebaner-trackedit.desktop` |
| `$prefix/share/icons/hicolor/scalable/apps` | the two icons |
| `$prefix/share/metainfo` | `org.radiotube.ebaner.metainfo.xml` |
| `$prefix/share/doc/ebaner` | `README.md`, `LICENSE` |

For a package, stage it with `DESTDIR=... cmake --install build` and set the prefix to
`/usr`. If the menu entries do not show up straight away, refresh the caches:

```sh
sudo update-desktop-database
sudo gtk-update-icon-cache -f /usr/local/share/icons/hicolor
```

Installing is optional: `./build/ebaner` runs from the build tree, and finds its
shaders in `build/shaders` with no configuration.

## 3. Install the dataset

The export is a tarball **with no top-level directory** — `tiles/`, `manifest.json` and
the rest sit at its root — so make the directory first and unpack into it. The overlay
is cloned into `overlay/` inside it.

System-wide, beside the program's shaders:

```sh
sudo mkdir -p /usr/local/share/ebaner/norway-rails
curl -LO https://ebaner.radiotube.org/norway-rails-2026.01.tar.bz2
sudo tar -xjf norway-rails-2026.01.tar.bz2 -C /usr/local/share/ebaner/norway-rails
sudo git clone --depth 1 https://github.com/leakingmemory/ebaner-overlay.git \
    /usr/local/share/ebaner/norway-rails/overlay
```

Or for one user, with no root needed:

```sh
mkdir -p ~/.local/share/ebaner/norway-rails
tar -xjf norway-rails-2026.01.tar.bz2 -C ~/.local/share/ebaner/norway-rails
git clone https://github.com/leakingmemory/ebaner-overlay.git \
    ~/.local/share/ebaner/norway-rails/overlay
```

The per-user copy is the one to use if you will edit the overlay with
`ebaner-trackedit`, since the editor writes back into it. A `git pull` in `overlay/`
later picks up new signalling without fetching the tiles again.

The archive's checksum (SHA-512) is

```
b504956ea4bf5904b0102dead8c7ebcc3ac4e64e0e52b24bcd9a26de31a841b34b4545ea06e40da406656ba5d8738cca06b4d9eab6c15b52a065aa8a022f7466
```

and can be checked with `sha512sum norway-rails-2026.01.tar.bz2` before unpacking.

### Where the program looks

Either location above is found without any configuration. In full, when no dataset is
named on the command line, the first of these holding a `tiles/` directory is used:

1. `$EBANER_DATA`
2. `$XDG_DATA_HOME/ebaner` (by default `~/.local/share/ebaner`)
3. `<binary dir>/data`
4. `<binary dir>/../share/ebaner`
5. each of `$XDG_DATA_DIRS` + `/ebaner` (by default `/usr/local/share`, then `/usr/share`)
6. `$prefix/share/ebaner` as configured at build time

Each of 2–6 is also tried with `norway-rails` appended. Anywhere else, set `EBANER_DATA`
or pass the path as the first argument. If nothing is found, the program lists every
place it looked. A path given on the command line is used as given.

The shaders are found the same way: `$EBANER_SHADERS`, then `<binary dir>/shaders` (as
in the build tree), then `<binary dir>/../share/ebaner/shaders`, then the configured
`$prefix/share/ebaner/shaders`. `<binary dir>` is the directory of the running executable.

## 4. Run

```sh
ebaner                    # choose the station on the start screen
ebaner "" Fauske          # or name one and skip the start screen
ebaner /path/to/norway-rails Bodø
ebaner-trackedit          # the track and signalling editor
```

The station name is matched ignoring case and Norwegian letters, so `Bodo` finds `Bodø`.
The first console lines, `[Paths]`, say which dataset and which shaders were picked up —
the first thing to check if it will not start. See the [README](README.md#controls) for
the controls.

## Running the tests

The tests need the dataset too. They look for it at `../norway-rails` relative to the
source tree, or wherever `-DEBANER_TEST_DATASET` points:

```sh
cmake -S . -B build -DEBANER_TEST_DATASET=$HOME/.local/share/ebaner/norway-rails
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Without a dataset they report as skipped rather than failing.

## Gentoo

ebaner is not in the official Gentoo repository. The unofficial, third-party
`leakingmemory` overlay carries a live ebuild that builds the program from git and
installs the dataset and its overlay to `/usr/share/ebaner/norway-rails`:

```sh
sudo eselect repository add leakingmemory git https://github.com/leakingmemory/gentoo-overlay.git
sudo emaint sync -r leakingmemory
echo 'games-simulation/ebaner **' | sudo tee /etc/portage/package.accept_keywords/ebaner
sudo emerge -av games-simulation/ebaner
```

Expect a long fetch: the dataset tarball is fetched as the package's distfile. With
`USE=test`, the tests run against the dataset it has just unpacked.
