#!/usr/bin/env python3
"""Re-anchor the overlay after an export has renumbered track ids.

The overlay pins every edit to <trackId>:<frac>. Main-line ids are stable - they
come from the national survey, ordered by banenavn and chainage - but siding ids
are a running counter over GPKG features in PBF file order, and an id is burned
even for a siding the elevation filter then drops. So any churn in the source
data shifts every siding id after it, and the overlay silently comes adrift:
some anchors dangle, and worse, some resolve onto an entirely different track.

Geometry is what actually survives a re-export, so that is what we match on. A
track is identified by its type, vertex count and the hash of its vertices; the
same ground exports to the same fingerprint whatever number it was given.

Usage:
    remap-track-ids.py --from <ref> --to <dataset> [--overlay <dir>]

<ref> is either a previous dataset root or a fingerprint file written by an
earlier run. Add --dry-run to report without touching anything.
"""

import argparse
import hashlib
import os
import re
import struct
import sys
from collections import defaultdict

# Ids at or above this are synthesised by the overlay itself, not the export.
SYNTHETIC_ID_MIN = 0xD0000000

# <trackhex>:<frac>[:<frac>] - only the first field is an id, the rest are
# fractions. Tokenising matters: a regex loose enough to scan the whole line
# happily reads the digits of "0.849372" as a track id.
ANCHOR_RE = re.compile(r'^([0-9a-f]{1,8})((?::[0-9.]+)+)$')
HEX_RE = re.compile(r'^[0-9a-f]{1,8}$')


def read_tracks(root):
    """{trackId: (type, [(x, y), ...])} across every tile of a dataset.

    A track is written whole into every tile it overlaps, so ids repeat; the
    first copy wins. Vertices are a contiguous x,y,z block, optionally followed
    by a separate uint16 speed block - hence the stride probe rather than an
    assumption.
    """
    out = {}
    tiles = os.path.join(root, 'tiles')
    if not os.path.isdir(tiles):
        sys.exit(f"no tiles/ under {root}")
    for lod in sorted(os.listdir(tiles)):
        lod_dir = os.path.join(tiles, lod)
        if not os.path.isdir(lod_dir):
            continue
        for tile in os.listdir(lod_dir):
            path = os.path.join(lod_dir, tile, 'tracks.bin')
            if not os.path.isfile(path):
                continue
            with open(path, 'rb') as fh:
                data = fh.read()
            if len(data) < 4:
                continue
            count = struct.unpack_from('<I', data, 0)[0]
            for stride in (14, 12):
                pos, ok = 4, True
                for _ in range(count):
                    if pos + 12 > len(data):
                        ok = False
                        break
                    nverts = struct.unpack_from('<I', data, pos + 8)[0]
                    pos += 12 + nverts * stride
                    if pos > len(data):
                        ok = False
                        break
                if not ok or pos != len(data):
                    continue
                pos = 4
                for _ in range(count):
                    tid = struct.unpack_from('<I', data, pos)[0]
                    ttype = data[pos + 4]
                    nverts = struct.unpack_from('<I', data, pos + 8)[0]
                    if tid not in out:
                        pts = [struct.unpack_from('<ff', data, pos + 12 + v * 12)
                               for v in range(nverts)]
                        out[tid] = (ttype, pts)
                    pos += 12 + nverts * stride
                break
            else:
                sys.exit(f"cannot parse {path}: no stride walks to EOF")
    return out


def fingerprint(ttype, pts):
    h = hashlib.blake2b(digest_size=16)
    for x, y in pts:
        # 0.1 m is far below any real vertex spacing, and keeps float noise out.
        h.update(b"%.1f,%.1f;" % (x, y))
    return (ttype, len(pts), h.hexdigest())


def fingerprints_of(tracks):
    return {tid: fingerprint(ttype, pts) for tid, (ttype, pts) in tracks.items()}


def write_fingerprint(path, prints):
    with open(path, 'w') as fh:
        fh.write("# trackId  type  vertices  geometryHash\n")
        fh.write("# Written by tools/remap-track-ids.py. Describes the export\n"
                 "# this overlay is anchored to, so the next one can be matched\n"
                 "# to it by geometry after the ids shift.\n")
        for tid in sorted(prints):
            ttype, nverts, h = prints[tid]
            fh.write(f"{tid:x} {ttype} {nverts} {h}\n")


def read_fingerprint(path):
    prints = {}
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            tid, ttype, nverts, h = line.split()
            prints[int(tid, 16)] = (int(ttype), int(nverts), h)
    return prints


def build_map(ref_prints, new_prints):
    """old id -> new id, by geometry. Returns (mapping, tiebroken, unresolved)."""
    by_print = defaultdict(list)
    for tid, fp in new_prints.items():
        by_print[fp].append(tid)
    ref_by_print = defaultdict(list)
    for tid, fp in ref_prints.items():
        ref_by_print[fp].append(tid)

    mapping, tiebroken, unresolved = {}, [], []
    for fp, olds in ref_by_print.items():
        news = by_print.get(fp, [])
        if len(olds) == 1 and len(news) == 1:
            mapping[olds[0]] = news[0]
        elif len(olds) == len(news) and news:
            # Duplicate geometry - the known duplicate turnout tracks. The group
            # is the same size in both exports, so pairing in id order is stable
            # and every member still lands on identical geometry. Recorded so an
            # unexpected tiebreak is visible rather than silent.
            for o, n in zip(sorted(olds), sorted(news)):
                mapping[o] = n
                if o != n:
                    tiebroken.append((o, n))
        else:
            unresolved.extend(olds)
    return mapping, tiebroken, unresolved


def ids_in_line(tokens):
    """[(token index, id)] for the id-bearing fields of one overlay line."""
    found = []
    for i, tok in enumerate(tokens):
        m = ANCHOR_RE.match(tok)
        if m:
            found.append((i, int(m.group(1), 16)))
    if not tokens:
        return found
    kw = tokens[0]
    if kw in ('border', 'switch') and len(tokens) > 1 and HEX_RE.match(tokens[1]):
        # switch <hex> <x> <y> motor [lock <sectionId> ...] - the lock ids are
        # decimal section ids and must not be touched.
        found.append((1, int(tokens[1], 16)))
    elif kw == 'noswitch':
        # noswitch <x> <y> <radius> [<hex>] [all]
        for i, tok in enumerate(tokens[4:], start=4):
            if tok != 'all' and HEX_RE.match(tok):
                found.append((i, int(tok, 16)))
    return [(i, v) for i, v in found if v < SYNTHETIC_ID_MIN]


def rewrite_line(line, mapping, stats):
    stripped = line.strip()
    if not stripped or stripped.startswith('#'):
        return line
    spans = [(m.start(), m.end(), m.group()) for m in re.finditer(r'\S+', line)]
    tokens = [s[2] for s in spans]
    edits = []
    for idx, old in ids_in_line(tokens):
        if old in mapping and mapping[old] != old:
            start, end, tok = spans[idx]
            new_tok = f"{mapping[old]:x}" + tok[len(f"{old:x}"):]
            edits.append((start, end, new_tok))
            stats['remapped'] += 1
        elif old in mapping:
            stats['unchanged'] += 1
        else:
            stats['unknown'] += 1
            stats['unknown_ids'].add(old)
    if not edits:
        return line
    # Apply back to front so earlier offsets stay valid; only the id fields move,
    # every other byte of the line is preserved exactly.
    for start, end, new_tok in sorted(edits, reverse=True):
        line = line[:start] + new_tok + line[end:]
    return line


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--from', dest='ref', required=True,
                    help='previous dataset root, or a fingerprint file')
    ap.add_argument('--to', dest='dataset', required=True, help='new dataset root')
    ap.add_argument('--overlay', help='overlay directory (default <to>/overlay)')
    ap.add_argument('--fingerprint', help='write the new fingerprint here '
                                          '(default <overlay>/track-fingerprint.txt)')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    overlay = args.overlay or os.path.join(args.dataset, 'overlay')
    if not os.path.isdir(overlay):
        sys.exit(f"no overlay directory at {overlay}")

    print(f"reading new export {args.dataset} ...")
    new_prints = fingerprints_of(read_tracks(args.dataset))
    if os.path.isfile(args.ref):
        print(f"reading reference fingerprint {args.ref} ...")
        ref_prints = read_fingerprint(args.ref)
    else:
        print(f"reading reference export {args.ref} ...")
        ref_prints = fingerprints_of(read_tracks(args.ref))
    print(f"  reference {len(ref_prints)} tracks, new {len(new_prints)} tracks")

    mapping, tiebroken, unresolved = build_map(ref_prints, new_prints)
    moved = sum(1 for o, n in mapping.items() if o != n)
    print(f"  matched {len(mapping)}/{len(ref_prints)} by geometry "
          f"({moved} renumbered, {len(mapping) - moved} unchanged)")
    if tiebroken:
        print(f"  {len(tiebroken)} matched via duplicate-geometry tiebreak")
    if unresolved:
        print(f"  {len(unresolved)} reference tracks have no counterpart: "
              + ' '.join(f"{i:x}" for i in sorted(unresolved)[:12]))

    stats = defaultdict(int)
    stats['unknown_ids'] = set()
    changed = []
    for name in sorted(os.listdir(overlay)):
        if not name.endswith('.txt') or name == 'track-fingerprint.txt':
            continue
        path = os.path.join(overlay, name)
        with open(path) as fh:
            lines = fh.readlines()
        out = [rewrite_line(l, mapping, stats) for l in lines]
        if out != lines:
            changed.append((path, name, sum(1 for a, b in zip(lines, out) if a != b)))
            if not args.dry_run:
                with open(path, 'w') as fh:
                    fh.writelines(out)

    print(f"\noverlay: {stats['remapped']} ids remapped, {stats['unchanged']} already correct, "
          f"{stats['unknown']} not found in the reference")
    for path, name, n in changed:
        print(f"  {n:5d} lines  {name}")
    if stats['unknown_ids']:
        print("  ids absent from the reference (left alone): "
              + ' '.join(f"{i:x}" for i in sorted(stats['unknown_ids'])))

    fp_path = args.fingerprint or os.path.join(overlay, 'track-fingerprint.txt')
    if args.dry_run:
        print(f"\n--dry-run: nothing written (would refresh {fp_path})")
    else:
        write_fingerprint(fp_path, new_prints)
        print(f"\nfingerprint of the new export written to {fp_path}")


if __name__ == '__main__':
    main()
