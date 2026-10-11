# Working a TXP station without line block

*Stasjon med lokal sikring, uten linjeblokk.*

Some stations have a proper **interlocking**: track circuits, main entry and exit
signals, and routes that throw and lock the switches and clear the signals. What they
lack is a **line block** to the next station. Once a train leaves the station limits,
nothing in the signalling knows where it is or which way it is going. The plain line
between the stations is protected only by the **train orders** the TXPs agree between
them.

That splits the work in two:

- **Inside the station, the interlocking protects you.** You can't set a route over an
  occupied track, a locked switch or a conflicting route.
- **Out on the line, only you and the TXP at the other end protect the train.** The
  interlocking would happily let you clear an exit signal toward a train that is already
  coming the other way.

This guide covers working such a station. For a station with only simple entry signals
and a flag, see [Working a TXP station with simple signals](txp-station.md). The map
itself is covered in [Using the traffic manager](traffic-manager.md).

## Which stations

These are stations with a TXP and their own interlocking, with entry and exit routes,
on lines with no line block either side. At the time of writing that is Bodø, Mo i Rana,
and the stations signed `NO SEG`, `BJE`, `DVT` and `MOS`.

A line block is train detection and interlocking out on the line between two stations,
monitored by the signalling system: the exit, entry and any block signals all depend on
it, and it sets the line for one direction at a time. A line with a line block is worked
by the signalling alone, with no TXP and no train orders: see
[Working a remote-controlled station](remote-controlled-station.md).

## What the signals do

| | |
|---|---|
| **Main signals at rest** | Red. A train stops at an entry or exit signal that offers it no road, and waits |
| **Setting a route** | Throws the switches, locks them and the route's track circuits, and clears the signal. A train standing at it then carries on by itself |
| **C1 / C2** | C1 is a straight road. C2 goes over a turnout to the side, at reduced speed |
| **Releasing** | Each track circuit is released as the train enters it. The route is gone once the train is in its last circuit |
| **Distant signals** | Repeat the next main signal, so the driver knows before reaching it |

Some main signals are tied to a level crossing (`crossing` in the overlay). After you set
the route, the signal stays red for a few seconds until the crossing has closed and shows
white, and only then turns green. The route is set; wait for it.

The **manned** switch on the station panel doesn't light or darken these signals. They
work whether the station is manned or not. Manning is what puts the station on the
**train-order network**, and that is the part this guide is about.

## Manning the station

**`O`** for the map, **`N`** / **`B`** to your station, **`E`** for the panel, and Enter
on **Switch station on (manned)**. The message line names the stations you now share a
line with: `Mo i Rana: manned, signals at danger (line to … and …)`.

You deal only with your neighbour on each side. A station that opens between two manned
stations takes over the middle of their line, and is refused only if a train order is
already booked on it.

## Sending a train

Do these in this order. **The order is the safety.**

1. **Get a train order.** Panel → **Request dispatch** → the station at the other end →
   **Passenger**, **Cargo** or **Other**. The far end answers by itself:
   - *line clear for a … train*: the section is yours. Carry on.
   - *refused - …*: the section is already booked. **Don't clear the exit signal.**
     Wait, and ask again once the other train has arrived.
2. **Set the exit route.** **`R`** → **Out of the station** → the route from the track the
   train is standing on, and **Enter**. The switches go over, and the exit signal clears.
   If it is refused, the message line says why (`occupied`, `conflicts with …`,
   `switch held` and so on). Clear that first. You can also **arm** it with `Q`, so that
   it sets itself once it can, but **only arm an exit after you have the train order**:
   an armed route doesn't ask about train orders either.
3. **Show the departure signal**, if you want the TXP figure out for a passenger train:
   Enter on the `Depart: …` line for that track. It is only what the driver sees; the
   exit signal is what lets the train go.
4. **When the train has left**, report it: Enter on **Train on track: -> …**. Until it
   has left you may **Withdraw order** instead. Cancel the exit route too if the train
   isn't going after all, because a route never gives back road no train used.
5. **Take the departure signal down** again.

The section stays booked until the far station reports the train arrived. Until then,
neither end can get another order for it, and that is what keeps a second train off the
line.

## Receiving a train

When a neighbour has asked for the line toward you, your panel shows
**Expecting … from …**. Once they report the train on the line, it reads
**Train arrived: from …**.

1. **Set the entry route** in good time: **`R`** → **Into the station** → the track you
   want it on. An entry route needs its **whole** road clear, so you can't let a train
   into an occupied track. If the train should stop here, choose a track whose exit
   signal you leave at red, and the train will stop at it.
2. **Watch it come in.** The track circuits turn red on the map as it enters, and the
   occupied sections are named beside the legend.
3. **Report the arrival** only once the **whole train** is in, clear of the line: Enter
   on **Train arrived: from …**. That frees the section, and the far end can send the next
   train. Reporting early can let a second train onto the line behind the tail of the
   first.

## Passing a train through

Set the **entry** route into a track and the **exit** route out of the far end. The two
routes may share road, and the driver gets a green at both signals and runs through.
You need **two train orders**: the line in was agreed by your neighbour behind the
train, and you need your own order for the line ahead **before** you set the exit route.
Report its arrival on the way in and *train on track* on the way out.

## Crossing two trains

Two trains meeting at your station from opposite directions, one to wait for the other:

1. Both neighbours have asked for their lines toward you, and both trains are on their
   way. **Don't give either of them an exit** yet.
2. **Set the entry route for the first** into one track, typically the loop (C2). It
   stops at the red exit signal at the far end.
3. **Set the entry route for the second** into the other track. Both entries may stand
   at once if they don't conflict. If they do, the second is refused with the reason:
   wait for the first to get in, or arm it with `Q`.
4. **Report both arrivals.** Both lines are now free in the books.
5. **Ask for each line onward**, and set each exit route once you have its order. Each
   train leaves on the line the other came in on, which is now free.

## Things the interlocking will not catch

- **A train out on the line.** With no line block, nothing in the signalling detects a
  train between the stations. The only record is the train order.
  Never clear an exit signal without *line clear* from the far end.
- **A following train.** Nothing holds a second train behind the first out on the line.
  One order covers one train; wait for the arrival before asking again.
- **A route left standing.** A route the train didn't use stays set and keeps its
  switches locked until you cancel it, with Enter on it in the `R` list or a click on its
  signal.
- **A train sent without an order.** Trains don't read the orders, so the simulator
  won't stop one that is given a green without one.

## Closing the station

Cancel any routes you set that weren't used, and take the departure signals down. Then
Enter on **Switch station off (unmanned)**. The lines either side merge into one section
between your two neighbours. You can't close while an order on either side is still
open: finish it, or withdraw it if the train hasn't left.

The signals stay as the interlocking leaves them. Unmanning a station with an
interlocking doesn't darken it.

## Startup options

| Variable | |
|---|---|
| `EBANER_TXP_OPEN=A,B,C` | man these stations at startup |
| `EBANER_MAP=<station>`, `EBANER_PANEL=1` | open the map and the station panel there |
| `EBANER_PANEL=dest` / `type` / `route` | open at a dispatch step, or the `R` route picker |
| `EBANER_ROUTE=<n>[,<n>…]` | set the station's routes at startup, in order (`e3`, `x2` index the in / out lists) |
| `EBANER_ROUTES=1` | print every route the interlocking knows at load |
