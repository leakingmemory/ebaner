# The track editor

`ebaner-trackedit` is the authoring side of ebaner: the editor that fixes the track
geometry and lays out the signalling, interlocking and lineside equipment over the
exported dataset.

```sh
./build/ebaner-trackedit ../norway-rails          # or: ... ../norway-rails Rognan
```

It renders the same scene as the simulator, with the terrain, rails and buildings
**ghosted**, and on top of them the **raw surveyed points** of every track and the links
between them, drawn through everything so nothing is hidden. The points are coloured by
track type:

| Colour | |
|---|---|
| amber | main line |
| cyan | siding |
| magenta | yard |
| red | a dead end (a loose end of a broken link) |
| white | selected |

You move around with the simulator's free-fly controls (`W` `A` `S` `D`, `Q` / `E`, the
mouse, `Shift`, `Tab` to free the cursor). The HUD shows the track, point and dead-end
counts and the camera position. **Esc** opens the menu of modes.

## Saving: the overlay

Edits preview at once, and are written only on **Ctrl+S**, to the **overlay**:
`<dataset>/overlay/`, a directory the terrainmapper export never touches. The simulator
and the editor both apply the overlay over the exported tiles at load, so a new export
can be dropped in without losing the edits. Unsaved edits are lost on quit, and the HUD
shows how many there are. `EBANER_NOOVERLAY=1` loads without the overlay.

| File | Written by |
|---|---|
| `track-edits.txt` | Geometry edit, New sidings, Move track points (`elev`, `move`, `rail`, `link`, `track`, `drop`) |
| `track-circuits.txt` | Track circuits |
| `switch-types.txt` | Switches (motor / manual, `noswitch`) |
| `signal-paths.txt` | Signal paths |
| `exit-signals.txt`, `exit-routes.txt` | Exit signals |
| `entry-signals.txt`, `entry-approaches.txt` | Entry signals |
| `distant-signals.txt` | Distant signals |
| `block-signals.txt` | Block signals |
| `simple-entry-signals.txt` | Simple entry signals |
| `level-crossings.txt` | Level crossings |
| `flag-posts.txt` | Flag posts |
| `txp-positions.txt` | TXP positions |
| `avalanche-signals.txt` | Avalanche signals |

Track positions in these files are written `<trackHex>:<frac>`: a track id and a fraction
along it.

## Editing the track (Geometry edit)

### Selecting

- **Tab** frees the cursor. **Click** a point to select it; **Ctrl+click** adds or
  removes points; a click on empty space clears the selection.
- **Right** / **Left** extend the selection forward / backward along the line, one
  point at a time, **Shift** ten, **Ctrl** fifty. It follows the route rather than a
  single track, taking the straightest way at a junction.
- **N** cycles the selection through points that share one spot (where tracks meet).

The point under the cursor is labelled with its elevation. The HUD shows the selected
point's elevation, or the range for several, and the length selected.

### Heights

- **Up** / **Down** raise or lower the selection by 0.1 m, **Shift** 1 m, **Ctrl**
  10 m. Every point moves by the same amount, so the grade through the selection is kept.
- **G** straightens the grade: every point between the first and last selected is put on
  a straight line between those two. Use it to take out a bump.

Both write `elev` lines. Only the elevation changes.

### Joining track

| Key | Select | Does |
|---|---|---|
| **J** | a dead end that overshoots another track | moves the end onto the track it crosses (a `move` line) |
| **R** | two points on different tracks | lays a straight connecting rail between them, which makes a switch at each end (a `rail` line). Keep the angle shallow, or it becomes a crossing rather than a switch |
| **C** | one point on each of two parallel tracks, opposite each other | lays a scissors crossover: two diagonal rails crossing in the middle, with a switch on each track at each end |
| **K** | one point in the middle of a diamond crossing | lays the two diagonal rails of a slip. Skipped if the tracks are nearly parallel (use `C`) or cross too steeply for a switch |
| **Enter**, **Enter**, **L** | aim the crosshair at one red dead end, then the other | links the two into one continuous track (a `link` line). **X** clears the pick |

**P** renders the real track, terrain and switch stands, to see how the edits look.
Switch stands for new switches appear on reload and in the simulator.

### Drawing a new siding (New sidings)

1. **Click a point on an existing track.** That is where the road starts, and its rail
   height becomes the height of the whole road.
2. **Click on** to lay it out point by point.
3. Finish it with a click **on another track**, which joins it there (a switch forms),
   or with **Enter**, which leaves a buffer stop.

**X** takes back the last point, or abandons the road at its first. **Y** makes it a
**yard** track instead of a siding. **Right-click** selects a drawn road for **Y** or
**X**. Each road is saved as a `track` line with a track id of its own, so anything
anchored to it stays put.

A road is drawn level at its starting height; set the grade afterwards in Geometry edit.
The HUD shows the ground height under the cursor while you draw, and the angle the road
leaves its parent track at. A switch forms only between **8° and 35°**, and the HUD says
when the angle is outside that.

### Moving points (Move track points)

Click a point and drag it, or nudge it with the **arrow keys**: 0.1 m a tap, **Shift**
1 m, **Ctrl** 10 m, in the direction you are looking. The height never changes.

This works on exported track and drawn roads alike. A drawn road's own `track` line is
updated. An exported point is corrected with a `move` line, which finds the point by its
old position:

```
move <oldX> <oldY> <oldZ> <newX> <newY> <newZ> [<trackId>]
```

The optional track id (decimal) says which track's point is meant where several meet.

- The HUD shows how far the point sits **off the straight line** between its two
  neighbours, which is the size of the bump in the ride.
- Where tracks meet end to end, the meeting point exists once on each track. They move
  together, one `move` line per track, and the HUD says how many tracks share the point.
- An **end** let go within **2 m** of another track snaps onto it and takes its height,
  so the switch forms. 2 m is the switch detector's tolerance.
- The HUD shows what both ends of the track stand on, and at what angle.

`elev` and `move` lines are applied in the order they are written, so each finds its
point where the lines above it left it.

### Dropping a duplicate track (`drop`)

A dataset merged from two exports can carry the same railway twice. Two tracks in one
place are two roads to the signalling and two sets of turnouts, so one has to go:

```
drop 3459
```

The track id is decimal. An id the export doesn't have is ignored. Drops are applied
before every other edit, whatever their place in the file. Nothing is reconnected: an
end that switched onto the dropped track is left loose, and a `move` onto the surviving
track restores the switch.

### Removing a false turnout (`noswitch`)

A switch is detected wherever a track end stands on another track's line. Where several
ends meet at one point, one real set of points can come out as two turnouts, and the
interlocking may then throw the wrong one. In **Switches** mode, select the false stand
and press **X**. It writes a line to `switch-types.txt`:

```
noswitch <x> <y> <radius> [<trackIdHex>] [all]
```

- With a track id, it removes only the turnout whose branch is that track. This is the
  form for a doubled set of points.
- Without one, it removes every turnout in the radius, except those on roads drawn in the
  editor; add `all` to remove those too.

Only remove a turnout that has been identified as false at that site.

## Track circuits

In **Track circuits** mode, **click** a track to drop a border (an insulated joint), and
**X** deletes one. Hover inside a stretch bounded by borders and press **Enter** to save
it as a section, named `S1`, `S2`, …. **Right-click** selects a section, and **F2**
renames it.

## Main signals and routes

Entry and exit signals are routes between track-circuit borders, with a **C1** (straight)
or **C2** (diverging) authority, and they need circuits through the station. How they are
worked is in [Using the traffic manager](traffic-manager.md).

- **Exit signals** are written in two parts: the signal and the road beyond it in
  `exit-signals.txt`, and the roads leading up to it in `exit-routes.txt`, one per line
  in the route list.
- **Entry approaches.** Where several roads lead to one entry signal, each is written in
  `entry-approaches.txt`. In Entry signals mode, right-click the entry route, press **B**,
  and click the start border out on the approach. The approach ends on the signal's
  border, facing its way. The list then offers one line per pair, `approach > destination`.
  The approach's circuits are locked with the route.
- **Which station works a route** is worked out from where its in-station end lies. A
  route far from the platforms, such as a branch to an industrial siding, can name its
  station on any of its route lines:

  ```
  entry 16 "NO MO INFN MINING" 63e:0.826377 d0000000:0.097737 type C2 station "Mo i Rana" …
  ```

  Where both halves of a movement name a station, they must agree; the approach wins.
- **A distant on the main signal's mast** (`F`, in Entry or Exit signals mode). The
  distant hangs 0.20 m in front of and 0.40 m below the main head, and repeats the next
  main signal along the road the points are set for, and is dark while the main it hangs
  on is at danger. It is set for the whole mast, as a `distant` keyword.
- **A two-lamp head** (`2`, in either mode): red over green, for a siding's signal. It
  shows its green for any clearance, C1 or C2, and doesn't change what the route
  authorises. It is written as a `twolamp` keyword.

## Block signals

A block signal is a main signal on the plain line between two stations, worked by the
line block. It divides the line into sections, so that a second train can follow the
first. See [Working a remote-controlled station](remote-controlled-station.md) for how
the line block works.

In **Block signals** mode, **click a track-circuit border**.

| Key | |
|---|---|
| right-click | select |
| **F** | turn it round, which also picks the border's other road |
| **B** | move the post to the other side of the line |
| **F2** | name |
| **X** | delete |

```
block <id> "<name>" <trackHex>:<frac> <+|-> [left]
```

A block signal has no route of its own. It opens the signal path already authored from
its border in its direction. One that resolves to no road is drawn orange and stays at
danger. The head is an exit signal's, but its red flashes. The two at one border stand
0.5 m back down their own approaches, back to back.

## Simple entry signals

A two-lamp head, red over green, with no circuits or routes behind it, for stations
without an interlocking. In **Simple entry signals** mode, **click any point on any
track**; right-click selects, **F** turns it round, **F2** names, **X** deletes.

Each signal belongs to the **nearest station**. The HUD names that station and its
distance, and flags anything beyond 4 km. Where two stations are close together, write
the station's name into the line. How these signals are worked is in
[Working a TXP station with simple signals](txp-station.md).

## Level crossings

In **Level crossings** mode, **click a point on a track**. **T** then a click adds another
track the crossing spans, **B** switches between lights only and lights with
half-barriers, **F2** names, **X** deletes. A crossing faces both ways, so there is no
turning it round.

What the editor derives, and shows on the HUD:

- **Circuits.** An inner circuit over the crossing, and an approach circuit each way. The
  approach length comes from the line speed: about 1.5 km at 130 km/h, a couple of
  hundred metres at 40. A distance written into the line overrides it. A crossing over
  several tracks has its own circuits and train heads on each, and its distances come
  from the fastest track.
- **Distant signals**, one each side, at four fifths of the braking distance, inside the
  approach circuit. They repeat the crossing's train indication, with the red shown
  **violet**. Where the crossing spans two roads, a distant repeats the road the points
  lead to, and warns when the points can't say.

Four heads, each red over white, flashing: two to the train, two to the road.

| | road | train |
|---|---|---|
| **idle** | white, slow | red, slow |
| **closing** (5 s) | red, fast | red, fast |
| **secured** | red, fast | white, fast |
| **opening** (5 s) | red, fast | red, fast |

- An approach circuit arms the crossing when it becomes occupied, and only from idle.
  The inner circuit arms it whenever it is occupied.
- It opens when the inner circuit clears: the train's lights go red at once, and the road
  opens 5 s later. A crossing armed by a train that turned back releases itself after a
  minute with everything clear.
- A **circuit-driven signal at danger** inside the approach, facing the crossing, cuts
  the approach circuit there, so a train held at it doesn't close the crossing. Simple
  entry signals don't.
- **Barriers** start down 7 s after the crossing activates and take 8 s, so they are down
  at 15 s. They start up as soon as the train leaves the inner circuit, and are still
  rising for about 3 s after the crossing is idle again. Written as a trailing `barriers`
  keyword.

## Flag posts

The TXP's hand signal to an arriving train: **red** stop, **green** pass through, or an
empty fixture. In **Flag posts** mode, **click a point on a track**; right-click selects,
**F** moves the post to the other side, **F2** names, **X** deletes. The flag hangs, so it
reads from both directions and has no facing.

Each post belongs to the nearest station, and is worked from that station's panel.

## TXP positions

Where the TXP stands to show a stopped train the departure signal, a green ring around a
white centre. In **TXP positions** mode, **click a point on a track**; right-click
selects, **F** turns it round (it faces against the direction of departure), **B** moves
it to the other side of the track, **F2** names, **X** deletes.

Place one wherever a train may stop and the TXP must be seen from: a long station needs
several. The editor draws a figure at every position.

The stations with positions make up the train-order network: each deals with the nearest
one on either side along the line. A junction, where a station would deal with three, and
a break in the line can't yet be expressed. Working the network is covered in
[Using the traffic manager](traffic-manager.md).

## Avalanche warning signals

Three lamps in a column, red over white over red. At rest the white flashes; on a warning
both reds flash and the white goes out. In **Avalanche signals** mode, **click a point
on a track**; right-click selects, **F** turns the head round, **B** moves the post across
the track, **Y** previews the warning, **F2** names, **X** deletes.

An avalanche signal governs nothing: no route or interlocking reads it. Nothing raises a
warning yet, and the `Y` preview is never saved.

## Startup

`EBANER_EDMODE="<mode>"` starts the editor in a mode, by its menu name, for example
`EBANER_EDMODE="Move track points"`.
