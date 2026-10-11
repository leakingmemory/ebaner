# Windows

## Running the Windows build

`ebaner-windows-x64.zip` holds the executables in `bin\` and the shaders in
`share\ebaner\shaders\`. It is statically linked and needs no DLLs or MSVC
redistributable; `vulkan-1.dll` comes with the graphics driver.

The dataset is installed separately (see [INSTALL.md](../INSTALL.md#3-install-the-dataset)):
put it in `data\` beside the executables, or point `EBANER_DATA` at it.

## How it is built

`.github/workflows/windows.yml` builds x64 executables with MSVC and vcpkg. It runs only
when **dispatched by hand** or on a pushed **`v*` tag**.

1. Installs the Vulkan SDK from LunarG (headers, `vulkan-1.lib`, `glslc`).
2. Takes glfw3, glm, lua and portaudio from `vcpkg.json`, for the `x64-windows-static`
   triplet.
3. Builds, and runs `ctest`. The tests that need a dataset report as skipped.
4. Packs the `cmake --install` output into `ebaner-windows-x64.zip`, which a tag attaches
   to the release.

`vcpkg.json` has no effect on a Linux build: vcpkg's manifest mode is used only when its
toolchain file is passed.

## Checking for Windows locally

Before spending a run, the tree can be syntax-checked for Windows with the mingw-w64
cross compiler. It isn't MSVC, but it catches POSIX-only code:

```sh
mkdir -p /tmp/wininc && ln -sfn /usr/include/{vulkan,vk_video,GLFW,glm} /tmp/wininc/
for f in src/*.cpp tools/*.cpp tests/*.cpp; do
    x86_64-w64-mingw32-g++ -std=c++20 -fsyntax-only -I src -I /tmp/wininc \
        -DEBANER_DATADIR='"C:/x"' "$f" || echo "FAILED: $f"
done
```
