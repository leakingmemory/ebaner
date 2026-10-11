# Working a remote-controlled station

*Fjernstyrt stasjon med linjeblokk.*

On a remote-controlled stretch, the stations have no staff at all. Every station has its
own interlocking (track circuits, entry and exit signals, routes that lock their switches),
and the lines between the stations have a **line block** (*linjeblokk*).

The line block is the train detection and interlocking carried out from the stations
onto the line between them. The signalling knows whether a train is on the line and
which way the line is set, and **every signal that leads onto the line depends on it**:
the exit signal at one station, the entry signal at the other, and any block signals in
between. Together they protect a train from the moment it leaves one station until it
is inside the next.

**Block signals** are a separate thing, and an optional one. They divide a line that
already has a line block into several sections, so that more than one train can be on
it the same way at once. A line block with no block signals is one section from the
exit signal to the next station's entry signal.

So there is **no TXP and no train order** (*togmelding*) on a remote-controlled
stretch. The dispatcher works every station from the traffic manager by setting routes,
and the signalling decides whether each route is safe to give. If the line block says no,
the route is refused.

The one exception is the **border station**: a station with a line block on one side,
and on the other a line with no interlocking between the stations, worked by train
orders. A border station is **manned by a TXP**. On its line-block side the signalling
protects the line, exactly as for the other stations here. On its other side nothing
does, so the TXP works that line with train orders, as in
[Working a TXP station without line block](txp-station-without-line-block.md).

See [Using the traffic manager](traffic-manager.md) for the map itself.

## Where

**In the simulator today, a line block is only recognised on a line that has block
signals.** The simulator works out where a line block starts and ends from the block
signals authored on it (`overlay/block-signals.txt`); it can't yet represent a line
block with no block signals. At the time of writing that is:

- `LSM` – `NSK` – `MAJ` – `SGD`, the longest gap being Majavatn to Svenningdal (31 km)
- `AGL` – `GRG`

Each of these lines has one block signal each way, at a track-circuit border in the
middle. So each line is two block sections, and up to two trains can follow each other
along it.

## Nothing to man

A remote-controlled station has no TXP, no departure positions, no flag post and no
simple entry signals. Its station panel (`E`) has nothing of use in it, and its
**manned** switch does nothing to its signals: the interlocking is always working.
Everything is done from the map:

| | |
|---|---|
| **`R`** | Set or cancel a **main route**: into the station or out of it |
| **`Q`** in the `R` list | **Arm** a route, to set itself as soon as it can |
| **Click a signal**, then a ringed destination | A shunting road |
| **Click a switch** | Throw it, if nothing holds it |
| **`N`** / **`B`** | Next / previous station |

## The line block

You don't work the line block directly. What you set are the **exit routes** onto the
line and the **entry routes** off it. The line block decides whether an exit route may
be given, and it works any block signals by itself.

**The direction lock.** The first exit route onto a line **claims** it in its direction.
While the claim stands:

- an exit route the **other way**, from the station at the far end, is refused:
  `the line LSM-NSK is set the other way, by …` (shown in the route list as
  `line set`),
- another exit route the **same way** is allowed. That is a following train.

**The claim outlives the route.** A route is released circuit by circuit as the train
runs, and it is gone by the time the train is out on the line. The claim isn't: it is
released only when the line is **genuinely clear**, meaning no route into it and no
track circuit on it occupied. Cancelling a route doesn't release the claim either,
because a train may already have gone.

**Block signals.** On a line that has them, you never work them. They don't appear in
the traffic manager and can't be set, held or overridden. Each one shows green only when
the line is claimed its way **and** the section beyond it is clear. A second train
following the first is held at the block signal until the one in front has left the
section beyond, and then it goes, without anyone touching anything. The block signal's
red **flashes**.

**A train with no route.** A train that reached the line without a route (shunted out,
or there before the signalling was set up) gives the line no direction. While it is
there, **nothing is let out toward it** from either end: `a train stands on the line …
and no route claimed it` (`line occupied`). Get it off the line first.

**A line that doesn't make sense.** If the simulator can't work out from the signals on
a line that it runs between exactly two stations, one way each direction, every block
signal on it is held at danger and no claim is granted:
`the line block at … does not make sense - see the log` (`line unclear`).
That is a fault in the signalling data, not something to work around.

## Sending a train

1. **`R`** → **Out of the station** → the route from the train's track → **Enter**.
2. If it is set, the switches go over, the exit signal clears (after a few seconds, if
   it is tied to a level crossing), and the line is claimed your way. The train runs out
   onto the line. Where the line has a block signal, it clears ahead of the train if the
   section beyond is free.
3. If it is refused, the route list shows why. To have it set itself the moment the
   reason passes, **`Q`** to arm it. That is the normal way to send a train out behind an
   opposing one: arm its exit, and it sets itself once the opposing train has cleared
   the line.

There is no order to ask for and nothing to report. The signalling has done it.

## Receiving a train

**`R`** → **Into the station** → the track you want it on. An entry route needs its
**whole** road clear, so you can't let a train into an occupied track. To stop the
train here, leave the exit signal at the far end of that track red. To send it straight
through, set the exit route too: the two routes may share road, and the driver gets a
green at both.

An entry route can be **armed** with `Q` as well. Arm it before the train is near, and
the entry signal clears as soon as the track is free.

## Crossing two trains

Two trains meeting from opposite ends at one station, one waiting for the other:

1. **Set or arm both entry routes**, one into the loop (C2) and one into the main (C1).
   Both may stand at once if they don't conflict. If they do, arm the second.
2. Each train stops at the red exit signal at the far end of its track.
3. **Arm both exit routes.** Each one is refused while the opposing train is still on the
   line it will take (`line set` or `line occupied`). As each incoming train gets fully
   into the station, its line clears, its claim is released, and the waiting exit route
   in the other direction sets itself.

With both exits armed, the crossing runs by itself: neither train can be let out until the
other has left the line it needs.

## Following trains

Two trains the same way along a line. This needs block signals: on a line block
without them, the whole line is one section, and the second train can't go until the
first has arrived.

1. Set the first train's exit route. It claims the line.
2. Set the second train's exit route as soon as the station allows it. It is not refused,
   because the line is already claimed its way.
3. The block signal halfway along holds the second train until the first is clear of the
   section beyond, then clears by itself.

## What can still go wrong

- **A route nobody used.** A set route never releases road that no train used. If the
  train doesn't come, the route stays set and keeps its switches locked, until you
  cancel it in the `R` list or by clicking its signal.
- **A claim that won't go.** Cancelling an exit route leaves the line claimed until it
  is clear. If the claim seems stuck with no train on the line, look for an occupied
  track circuit on it. The map lists occupied sections by name beside the legend.
- **The border station's other side.** On the side without a line block, the
  signalling has no train detection beyond the station limit. A train sent that way
  needs a train order, and the interlocking won't ask for one.

## Startup options

| Variable | |
|---|---|
| `EBANER_MAP=<station>` | open the map at a station |
| `EBANER_PANEL=route` / `entry` / `exit` | open the `R` route picker, at its first step or straight into a list |
| `EBANER_ROUTE=<n>[,<n>…]` | set the station's routes at startup, in order (`e3`, `x2` index the in / out lists) |
| `EBANER_ROUTES=1` | print every route the interlocking knows at load |

The console logs the line block as it works: `[Block] line … claimed by …` and
`[Block] line … released (clear)`.
