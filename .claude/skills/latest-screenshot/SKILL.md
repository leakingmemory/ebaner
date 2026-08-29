---
name: latest-screenshot
description: Find and open the most recent screenshot on this machine. Use whenever the user refers to "the latest screenshot", "the screenshot", "see the screenshot", or asks you to look at what they just captured.
---

# Finding the latest screenshot

Screenshots live in `/home/sigsegv/Pictures/Screenshots/`. Getting the newest one
looks trivial and is not: two separate traps here return a stale file *quietly*,
handing you a picture from years ago that you then reason about as if it were
current.

## The command

```sh
find ~/Pictures/Screenshots -maxdepth 1 -type f -printf '%T@ %p\n' \
  | sort -rn | head -1 | cut -d' ' -f2-
```

Then open it with the Read tool, passing the absolute path.

## Trap 1: sorting by name gives the wrong file

Filenames carry an ISO timestamp, so name order looks like time order. It is not,
because the capture tool changed case at some point: older files are
`Screenshot from ...`, newer ones `Screenshot From ...`. `F` is 0x46 and `f` is
0x66, so **every** capital-F file sorts before **every** lowercase one:

```
ls | sort | tail -1   ->  Screenshot from 2025-02-05 16-17-17.png   (wrong, and 18 months stale)
ls -t | head -1       ->  Screenshot From 2026-08-29 12-17-42.png   (right)
```

At last count 42 files were `From` and 9 were `from`. Always sort by mtime.

## Trap 2: the parent directory is full of decoys

`~/Pictures` holds ~116 entries: dated subdirectories, a `smplayer_screenshots`
directory, and **33 loose `Screenshot*.png` files** from 2018-2022. So:

- `~/Pictures/Screenshot*` matches the old loose files, not the current ones.
- `ls -t ~/Pictures | grep '^Screenshot' | head -1` returns `Screenshots` - the
  directory itself, not an image.
- Anything matching `*creen*` in `~/Pictures` hits those decoys first.

Search `~/Pictures/Screenshots/` specifically, with `-maxdepth 1 -type f`.

## Trap 3: the names contain spaces

`Screenshot From 2026-08-29 12-17-42.png` has three of them. Quote the path
everywhere, and do not pipe a bare `ls` into a loop that splits on whitespace.
`cut -d' ' -f2-` above is deliberate: it keeps everything after the timestamp
field rather than taking one token.

## Confirm before relying on it

State the filename and its timestamp in your reply. If the date is not close to
today, say so rather than pressing on - it almost certainly means one of the
traps above bit, and the user knows when they took the picture.
