---
name: regrade-siding
description: Regrade a siding or yard track to match the main line beside it, by writing elev edits into the overlay's track-edits.txt. Use when asked to fix, level, or smooth the elevation of a siding at a station (e.g. "fix the elevation of the siding at Eiterstraum using the main line as reference").
---

# Regrading a siding against its main line

Sidings take their height from the DTM, one sample per vertex, so they arrive
noisy - gradients of several percent between neighbouring points, which no
railway has. The main line beside them comes from the survey profile and is
smooth. Regrading means giving the siding the main's height at each of its own
vertices.

## Read the applied geometry, never tracks.bin

**This is the whole trap.** `tracks.bin` holds the export's heights, and the
overlay then regrades them - often by tens of metres. At Eiterstraum the main
line `5b6` reads 80.2 m on disk and sits at 20.2 m once `track-edits.txt` is
applied, a flat 60 m difference. A profile derived from the raw file is wrong by
whatever the overlay moved, and wrong *consistently*, so it looks perfectly
self-consistent and passes every sanity check you throw at it.

Use the tool that loads the dataset the way the game does:

```sh
# what is near a place, nearest first
./build/ebaner-dumptrack ../norway-rails --near 419300 7287500 300

# every vertex of a track, in order (the engine logs to stdout too, hence grep)
./build/ebaner-dumptrack ../norway-rails 5b6 | grep '^PT ' > /tmp/main.txt
./build/ebaner-dumptrack ../norway-rails 3658 | grep '^PT ' > /tmp/sid.txt
```

The same applies to anything else you compute here: comparisons against the
terrain heightmap, gradients, "is this track anomalous". Do them on applied
geometry or not at all.

## Steps

1. **Find the station.** `--near` with the station's rough coordinates. The
   siding is the type-1 (or type-2 yard) track running alongside a type-0 main;
   a passing loop is typically 600-1100 m. If you only know the station by name,
   locate it from the overlay's route anchors (`grep 'DVT' exit-routes.txt`) or
   by its position between two known stations.

2. **Pick the reference main.** The type-0 track whose lateral offset stays
   small along the whole siding - check it, do not assume. Offsets of 0-5 m are
   right for a loop: near zero at the ends where the turnouts are, ~4.5 m in the
   middle. If one main track does not cover the full length, the siding spans
   two and you need both.

3. **Project.** For each siding vertex, the nearest point on the main polyline
   (point-to-segment, interpolating z along the segment). That z is the new
   height. Taking it per-vertex rather than as a constant offset keeps the ends
   meeting the main at exactly its own height, which is what lets turnouts join
   cleanly later.

4. **Write the edits**, one per siding vertex:

   ```
   elev <x> <y> <newz> <trackid> <fromz>
   ```

   - Coordinates and heights at **3 decimals** - matches `writeEdits`, and
     round-trips the float32 vertices well inside the 2.0 m snap tolerance.
   - `trackid` is **decimal** here. The signalling files write track ids in hex;
     `elev` and `move` do not. Getting this wrong silently targets another track.
   - `fromz` is the vertex's height *now*, which is what separates two vertices
     sharing a spot on the same track. Always include it.

5. **Append to the end of `track-edits.txt`. Never insert.** A `link` edit's
   synthetic connector id is `0xF0000000 + its edit index in this file`, and
   track circuits anchor to those ids. Inserting a line above a link slides every
   connector id by one and silently repoints whatever named it.

## Verify

- `[TrackOverlay] applied N elev` rises by exactly the number of lines written.
  Anything less means some did not match - usually a coordinate or `fromz` that
  does not land within tolerance.
- The new profile's gradient is plausible: a few ‰, not a few %. Compare against
  what it was; the before/after on that number is the clearest evidence the
  regrade did what was wanted.
- The siding's height now tracks the main along its whole length.
- `ctest` in `build/`.
- If the file has `link` edits after your insertion point, re-check the connector
  ids did not move.

## Notes

- Do not commit unless asked. `../norway-rails/overlay` is a separate repo; stage
  only the files in scope.
- Back up `track-edits.txt` to the scratchpad before appending, and confirm the
  tree was clean first, so the change stays reviewable.
- A siding whose ends are both loose is not yet a working loop - no turnout
  either end. Regrading it is still correct and makes a later `link` join at
  matching heights, but say so rather than implying the loop works.
