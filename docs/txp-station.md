# Working a TXP station with simple signals

Many stations on the line have no interlocking: no track circuits, no routes, no
signalling worked from a panel. Instead they have a **togekspeditør (TXP)**, the
station's traffic controller, and three simple tools:

- **Simple entry signals**: one at each end of the station, two lamps, **red over
  green**. They say only *stop* or *come in*.
- **A flag post**: the TXP's hand signal to an arriving train. **Red** means stop at the
  station, **green** means pass through.
- **Departure positions**: spots beside each track where the TXP stands and shows a
  stopped train the departure signal, **a green ring around a white centre**, which
  gives it permission to leave.

Between stations, the line is worked by **train orders** agreed with the next manned
station each way.

This guide walks through working such a station: manning it, letting trains in, stopping
or passing them, crossing two trains, and sending them on. Everything is done from the
traffic manager (**`O`**, then **`E`** for the station panel). See
[Using the traffic manager](traffic-manager.md) for the map itself, and
[Working a TXP station without line block](txp-station-without-line-block.md) for a
station with its own interlocking.

## Which stations

A TXP station is a station with a TXP. The ones worked as described here have simple
entry signals and a flag post. Their signals are named with the station's short name,
for example `OTR 1` and `OTR 2` at Oteråga, and `NO FSK 1` and `NO FSK 2` at Fauske. The
map's title names the station you are working.

## The station panel

Open the map with **`O`**, use **`N`** / **`B`** until the title names your station, and
press **`E`**:

```
STATION - Oteråga  (Up/Down, Enter, Esc)
Request dispatch                    >
Switch station on (manned)        OFF
All red
OTR 2                            dark
OTR 1                            dark
Flag: OTR FLAG                no flag
Depart: NO OTR TRACK 1               -
Depart: NO OTR TRACK 2               -
```

Up / Down to a line, **Enter** to act on it.

## What the trains do

Know what your signals mean to a train before working them:

| Station state | What a train does |
|---|---|
| **Unmanned** | Signals dark, no flag. Trains run through at line speed and take no notice of the station |
| Manned, entry signal **red** | The train stops at the signal and waits. When you clear it, the train carries on |
| Manned, entry signal **green** | The train comes in, at no more than **40 km/h** past the flag post |
| **Red flag**, or no flag | The train stops in the station, clear of the switches at both ends |
| **Green flag** | The train passes through without stopping, at no more than 40 km/h |

A train reads the flag **as it passes the post**. Up to then it obeys whatever the flag
shows, so you can still wave a train through while it is coming in. Once it is past the
post, its decision stands, and changing the flag does nothing for that train.

A train that has **stopped** at the station waits there. It leaves when its driver
decides to go, which should be when you show it the departure signal. The auto-driver
stops at the station and hands the train back. Pressing `P` again sends it on.

**Nothing in the simulator enforces train orders.** A train will leave whether or not
an order has been agreed. The orders are the bookkeeping that keeps two trains from
being sent toward each other, and keeping it is your job.

## Manning the station

**Enter** on **Switch station on (manned)**. The station:

- lights its entry signals, **all red**,
- connects to the nearest manned station on each side, and takes over its share of the
  line between them.

The neighbours refuse only if a train order is already booked on a section you would
take over. In that case nothing changes, and the message line says which section. A
train standing on the line is not a reason to refuse.

Clearing a signal at an unmanned station also mans it.

## Letting a train in

1. **Check the road.** The station has no track circuits, so the map can't tell you
   whether a track is free: look. The map shows the train positions.
2. **Set the switches** for the track the train is to take.
   - **Motor switches** (ringed on the map): click them.
   - **Hand switches**: these are thrown from the lineside. Fly the camera to the switch
     and press `T` with the crosshair on it.
3. **Set the flag**: Enter on `Flag: …` cycles **no flag → red → green**. Red if the
   train is to stop here, green if it is to pass through.
4. **Clear the entry signal** at the end the train is coming from: Enter on it. Only
   one signal can be green at a time, so clearing one puts the other back to red.

When the train is in, **put the signal back to red** with **All red**. A signal holds
whatever it was last given. No train passing and no timer puts it back.

If the train came under a train order, go to the panel line **Train arrived: from …**
and press Enter to report it. That releases the section behind it.

## Sending a train on

1. **Ask for the line.** **Request dispatch** → the next station → **Passenger**,
   **Cargo** or **Other**. The far end answers by itself: *line clear* if the section is
   free in the books, or a refusal saying why.
2. **Set the switches** for the departure road.
3. **Show the departure signal.** Enter on the `Depart: …` line for the track the train
   is standing on. The TXP figure appears beside the track. Only one position shows at a
   time, so showing one takes down any other.
4. When the train has left, Enter on **Train on track: -> …** to report it to the far
   end. Until it has left, you may **Withdraw order** instead.
5. Take the departure signal down: Enter on its line again.

The far station reports **Train arrived** when the train gets there. Only then is the
section free for another train.

## Crossing two trains

The station's main job is to let two trains pass each other on a single-track line: one
waits in the loop while the other goes by on the main. At Oteråga, the main is track 1
and the loop is track 2.

Say train A is northbound and arrives first, and train B is southbound.

1. **Take A into the loop.** Set the switches for track 2, set the flag **red**, and
   clear the southern entry signal. A stops in the loop, clear of both switches.
2. **All red** once A is in, and report A's arrival if it came under an order.
3. **Set the road for B along the main**: switches back to straight.
4. **Let B through.** Clear the northern entry signal. Set the flag **green** if B is to
   pass without stopping, or leave it **red** if B stops here too. Remember that a train
   reads the flag as it passes the post.
5. Once B is through (or stopped and sent on), **All red**, and report B's arrival or
   departure as its orders require.
6. **Send A on**: ask for the line north, set the switches from the loop to the main,
   and show the departure signal at **NO OTR TRACK 2**. Report *Train on track* once it
   has gone.

The order of events is what matters. **Don't clear a signal for the second train until
the first is clear of the switches it will cross.** Nothing at a simple station locks
the switches for you.

## Closing the station

Before switching off, put the signals to **All red** and take down any flags and
departure signals. Then Enter on **Switch station off (unmanned)**. The signals go dark,
and the line either side is handed back to the neighbouring stations, which now work it
as one section.

You can't unman a station while it holds a booked section: either an order you have
asked for, or one coming to you. Finish or withdraw it first.

## Setting up a crossing without the keyboard

For testing, these start the simulator in a ready state:

| Variable | |
|---|---|
| `EBANER_TXP_OPEN=Oteråga` | man these stations at startup (comma-separated) |
| `EBANER_GREEN="OTR 1"` | clear one simple entry signal by name, manning its station |
| `EBANER_THROW="x,y;x,y"` | throw the switch nearest each world point to diverging |
| `EBANER_AUTO=1` | hand the train to the auto-driver at startup |
| `EBANER_MAP=Oteråga`, `EBANER_PANEL=1` | open the map and the station panel there |
