# Using the traffic manager

The traffic manager is the signaller's view: a map of the line from above, where you
throw switches, set routes for trains, work a station's signals and flags, and pass
train orders between stations. Press **`O`** to open it and `O` again to go back to
the cab.

**The simulation keeps running while the map is open.** A train you were driving keeps
going on whatever the handles were set to, so stop it or hand it to the auto-driver
(`P`) first.

## Getting around

| Input | |
|---|---|
| `W` `A` `S` `D` | Pan |
| Mouse wheel, `Z` / `X` | Zoom in / out |
| `N` / `B`, or Right / Left | Next / previous station |
| `Esc` | Back out of a picker one step; with nothing open, the menu |

The map opens on the **station nearest your train**. Its name is in the title,
`TRAFFIC MANAGER - Fauske (12/40)`, and **everything the panels do acts on that
station**. `N` and `B` walk along the stations of the line, bringing the view with them.

To open it straight away: `EBANER_MAP=1 ebaner …`, or `EBANER_MAP=Fauske` to open it at
a named station.

## Reading the map

The legend is at the top left:

| | |
|---|---|
| Track | **main** amber, **siding** cyan, **yard** magenta |
| Switches (diamonds) | **green** straight, **orange** diverging, **red** broken. A **ring** marks a motor switch, which the map can throw |
| Track circuits | each section coloured. **Red** = occupied, and the names of occupied sections are listed beside the legend |
| Messages | the line under the legend. **Green** for something done, **orange** for something refused, with the reason |

Hover the mouse over something and it says what a click will do: *switch - click to
throw*, *signal - click to select route*, *cancel route*, *set route …*. The bottom of
the legend shows how many kilometres the view is across.

## Throwing a switch

**Click its diamond.** Only motor switches (ringed) can be thrown from here. A hand switch
says *Manual switch - hand-thrown in the cab*: it is thrown from the lineside, with `T`
and the crosshair on it.

A switch will not move when:

| Message | |
|---|---|
| `Locked by main route …` / `Locked by route …` | a set route runs over it. Cancel the route first |
| `BLOCKED: … occupied` | a train is standing on it |
| `Switch BROKEN - cannot be worked` | the switch has failed |

You rarely need to throw switches by hand. Setting a route throws every switch on it.

## Setting a main route (`R`)

A main route is a train movement past a main signal: into the station past the entry
signal, or out of it past the exit signal. Setting one throws its switches, opens the
shunting signals along it, locks its track circuits and clears the signal.

1. **`R`**, then choose **Into the station** or **Out of the station**. The count of
   each is shown.
2. Up / Down to a route, **Enter** to set it.

Each line shows the route name, its type and its state:

```
NO MO NB T2                C1  SET - cancel
NO MO INFN T2              C2  occupied
NO MO NB T3                C1  ARMED 1
```

- **C1** clears the signal for an unrestricted road. **C2** is a road over a turnout to
  the side, at reduced speed.
- **SET - cancel**: it is set. Enter on it again cancels it.
- **ARMED n**: it is waiting in the queue (see below).
- Anything else is why it can't be set now. The full reason appears in the message line
  when you try.

| Refused as | Meaning |
|---|---|
| `occupied` | a track circuit beyond the signal has a train in it. For an entry route, that is the whole route: no train is let into an occupied platform |
| `conflicts with …` | another set route already authorises a movement onto the same rails |
| `faces …` | another set route runs the other way over shared rails |
| `switch held` | a switch it needs is locked the other way by another route |
| `switch locked` | a switch it needs is occupied |
| `needs a hand-thrown switch` | a hand switch on it is the wrong way. Throw it in the cab |
| `switch BROKEN` | a switch on it has failed |
| line block | the plain line beyond is claimed by a train going the other way |

**The train you are setting the route for doesn't count.** An exit route only needs the
road beyond the signal to be clear, and the platform it starts from is where your train
is standing.

**To pass a train through a station without stopping**, set the entry route into the
platform and the exit route out of it. The two may share road, and the driver gets a
green at both signals.

### Arming a route that can't be set yet

Highlight a refused route and press **`Q`**. It is **armed** and joins a queue, and it
sets itself as soon as whatever blocked it clears: the train ahead gets clear, a
conflicting route ends. Where several armed routes want the same road, they are set in
the order they were armed. `Q` on an armed route disarms it.

Arming is only about timing. When its turn comes, the route is checked again exactly as
if you had pressed Enter, so arming cannot get past any of the refusals above. It only
waits for them to pass.

### How a route is given back

A set route releases each track circuit **as the train enters it**, and the route ends
when the train enters its last circuit. Behind the train, the road is free for the next
movement.

A route **never releases road that no train used**. If the train doesn't come, or takes
another way, the route stays set and keeps its switches locked until you cancel it, with
Enter on it in the `R` list or a click on its signal. A route that seems stuck after a
train has passed is a fault in the signalling data, not something to wait out.

Cancelling a route does **not** release the line block beyond it. The line is released
when it is actually clear of trains.

## Shunting routes (clicking signals)

Shunting signals (dwarfs) and the short roads between them are worked with the mouse:

1. **Click the signal.** It is armed, and its possible destinations are **ringed** and
   labelled.
2. **Click a ringed destination** to set the road. It throws the switches and clears the
   signal.

Click the same signal again to disarm it, or empty space to disarm it. Click a signal
that has a road set to **cancel** it. A road that is occupied is refused with the
section's name.

Block signals on the plain line between stations don't appear here: they work by
themselves.

## The station panel (`E`)

**`E`** opens the panel for the station being worked. Move with Up / Down and act with
**Enter**. A station's panel looks something like this:

```
STATION - Fauske  (Up/Down, Enter, Esc)
Request dispatch                    >
Train on track: -> Rognan (cargo)   >
Switch station off (unmanned)      ON
All red
Fauske N                         GREEN
Fauske S                           red
Flag: Fauske platform         no flag
Depart: Fauske T1                    -
```

**Switch station on / off.** An **unmanned** station has its simple entry signals
dark, and trains run through without reference to them. A **manned** station shows red
on all of them until you clear one. Manning a station also connects it to the
train-order network (below), and the neighbours can refuse.

**All red** puts every simple entry signal at this station to danger.

**The signals** (one line each). Enter clears that one. A station clears **one at a
time**, so clearing one puts the others back to red. A signal holds whatever it was last
given: no train passing and no timer resets it, so put it back yourself with *All red*.
Clearing a signal at an unmanned station mans it.

**Flags** (`Flag: …`). Enter cycles **no flag → red → green → no flag**. A flag on a
post is the station staff's hand signal: red stop, green pass through. Each post is
independent of the others and of the signals, so a green flag against a red signal is
allowed.

**Departure positions** (`Depart: …`). Enter shows or withdraws the station staff's
permission to leave at that position. Only one can be shown at a station at a time.

## Train orders between stations

[Working a TXP station](txp-station.md) walks through a whole station worked this way,
including crossing two trains, and
[Working a TXP station without line block](txp-station-without-line-block.md) does the
same for a station with its own interlocking.

Where the line has no fixed signalling between stations, trains run on **train orders**
agreed between the manned stations at each end of a section. Which stations can deal with
each other is worked out from where departure positions have been placed
(`overlay/txp-positions.txt`): each such station deals with the nearest one either side.

**Manning a station puts it on the network.** It connects to the nearest manned station
on each side and takes over its share of the line. It is refused only if a train order is
already booked on a section it would take over. A train physically standing on the line is
not a reason to refuse. Unmanning hands the sections back to the neighbours.

**Sending a train:**

1. At the departure station: **`E`** → **Request dispatch** → choose the station at the
   other end → choose **Passenger**, **Cargo** or **Other**.
2. The far end answers by itself. If the section is clear in the books, you get *line
   clear*. It is refused if the section is already booked.
3. When the train has left, Enter on **Train on track: -> …** to report it.
   Until then you may instead **Withdraw order**.
4. At the **far** station (`N` / `B` to it, then `E`), when the train has arrived,
   Enter on **Train arrived: from …**. That releases the section. Only the far end can
   report the arrival, because the station that sent the train cannot see it get there.

While a section is held, nothing else can have it. Another request from either end is
refused, a station between them cannot be manned, and the two stations holding it cannot
be unmanned.

The exchange is printed to the console as it happens (`[TXP] CONNECT`, `ACCEPT`,
`REJECT`).

## Startup options

| Variable | |
|---|---|
| `EBANER_MAP=1` / `=<station>` | open the traffic manager, optionally at a station |
| `EBANER_PANEL=1` | also open the station panel |
| `EBANER_TXP_OPEN=A,B,C` | man these stations at startup |
| `EBANER_ROUTE=<n>[,<n>…]` | set the worked station's routes at startup, in order |
| `EBANER_ROUTES=1` | print every route the interlocking knows at load |
