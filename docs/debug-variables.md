# Environment variables

Every variable ebaner reads. Most exist so that a scenario can be set up, run and
screenshotted without a keyboard.

## Paths

| Variable | |
|---|---|
| `EBANER_DATA` | the dataset, ahead of every other place searched. A path on the command line still wins. See [INSTALL.md](../INSTALL.md#where-the-program-looks) |
| `EBANER_SHADERS` | the directory with the compiled `.spv` shaders, ahead of every other place searched |

## Starting a scenario

| Variable | |
|---|---|
| `EBANER_STATION=<name>` | start at this station, skipping the station picker |
| `EBANER_VEHICLE=<n>` | skip the start screen and take this vehicle, by its number on the start screen |
| `EBANER_TRAINS="A,B"` | place further trains at these stations |
| `EBANER_CAB=<n>` | seat the driver in cab `n` |
| `EBANER_CHASE=1` | start in the chase camera |
| `EBANER_CAM="x,y,z,yaw,pitch"` | place the camera, in scene-relative metres and degrees |
| `EBANER_AUTO=1` | hand the train to the auto-driver and log what it does once a second |
| `EBANER_ROLL=<m/s>` | start the train at this speed with the brake released, so it coasts |
| `EBANER_MENU=1` / `train` / `station` / `devstation` / `vehicle` | open the Esc menu, the train picker, the station list, the stations still under development or the vehicle list |
| `EBANER_FULLSCREEN=1` | start in full screen, as the Esc menu's Full screen item puts it |

## Signalling

| Variable | |
|---|---|
| `EBANER_MAP=1` / `<station>` | open the traffic manager, optionally at a station |
| `EBANER_PANEL=1` / `dest` / `type` / `route` / `entry` / `exit` / `line` | open the station panel, at a dispatch step, the `R` route picker or the `G` line picker |
| `EBANER_TXP_OPEN=A,B,C` | man these stations |
| `EBANER_ROUTE=<n>[,<n>…]` | set the worked station's routes, in order. A bare number indexes everything `EBANER_ROUTES` prints; `e3` / `x2` index the in / out lists |
| `EBANER_ARM=<n>[,<n>…]` | arm routes, written as for `EBANER_ROUTE` and armed in that order, after it |
| `EBANER_ROUTES=1` | print every route the interlocking knows at load |
| `EBANER_GREEN="OTR 1"` | clear a simple entry signal by name, manning its station |
| `EBANER_THROW="x,y;x,y"` | throw the switch nearest each world point to diverging |

## World

| Variable | |
|---|---|
| `EBANER_RADIUS=<m>` | the half-width of world loaded at the start (default 20000). Smaller starts faster |
| `EBANER_NOSTREAM=1` | don't stream the world as the camera moves |
| `EBANER_NOSTITCH=1` | skip stitching the terrain tile seams |
| `EBANER_NOCARVE=1` | don't cut railway cuttings into the terrain |
| `EBANER_NOOVERLAY=1` | ignore the `overlay/` track edits |

## Screenshots and profiling

| Variable | |
|---|---|
| `EBANER_SCREENSHOT=<file.ppm>` | write a frame to this file and exit. Sound is off |
| `EBANER_SHOTFRAME=<n>` | take the screenshot at frame `n` (default 20), to give the world time to stream in |
| `EBANER_PROFILE=1` | print per-frame timings once a second. See [The world and its rendering](world-and-rendering.md#profiling) |

## Sound

| Variable | |
|---|---|
| `EBANER_AUDIO_DEVICE=<index or name>` | the PortAudio output device |
| `EBANER_AUDIO_TEST=1` | play a steady 220 Hz tone instead of the train, to check the sound output |
| `EBANER_AUDIO_DUMP…`, `EBANER_AUDIO_ENGINE` | render a sound to a WAV file and exit. See [Sound](audio.md#checking-it) |

## Track editor

| Variable | |
|---|---|
| `EBANER_EDMODE="<mode>"` | start in this mode, by its menu name |
