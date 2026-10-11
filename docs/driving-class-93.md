# Driving the Class 93

The NSB Class 93 (Bombardier Talent) is a two-section diesel railcar with a cab at each
end, and the easiest train in the simulator to start with. These are the steps to get one
moving, stop it, and change ends, plus what to do when it will not move.

## Picking it

On the start screen, choose **NSB Class 93 (Talent)** under *trains*. The **x2** and
**x3** entries are two or three sets coupled in multiple. To skip the menu:

```sh
EBANER_VEHICLE=0 ebaner "" Bodø      # 0 = one set, 1 = two, 2 = three
```

The train always starts on the running line at the station you chose.

## The controls

| Key | |
|---|---|
| `V` | Sit in a cab. Press again for the next cab, and again to leave |
| `I` | Start / stop both engines |
| `F` / `N` / `R` | Reverser: Forward, Neutral, Reverse |
| `,` | Lever one step toward **power** |
| `.` | Lever one step toward **brake** |
| `Space` | Emergency brake |
| `T` | Throw the switch under the crosshair |
| `P` | Hand the train to the auto-driver, or take it back |

The Class 93 has **one combined lever** per cab, and it moves along a single line:

```
EMERG · B4 · B3 · B2 · B1 · N · P1 · P2 · P3 · P4 · P5
◄──────── brake ──────── .   ,  ──────── power ────────►
```

Each key press moves it one notch. It cannot be at power and brake at the same time:
going toward power releases the brake one notch at a time first, and going toward
brake takes the power off first.

The keys work the cab you are sitting in, or the front cab if you are not in one. There
is no locomotive brake (`H`/`J`) and no separate train brake handle (`K`/`L`). Those
belong to the Di 4 and the CD 312.

## What to watch

The cab has a speedometer, a **duplex air gauge** and a rev counter per engine. The HUD
shows the same values, plus the main reservoir:

| | Normal running |
|---|---|
| **BP**, brake pipe (red needle) | 5.0 bar = brakes released. Lower = braking |
| **BC**, brake cylinder | 0 = brakes off. About 3.8 bar at full service |
| **MR**, main reservoir | 6.5–8 bar. The compressors keep it there |
| Engine revs | about 700 rpm idle, up to 1500 under full power |

## Starting from cold

The train starts the way a train left overnight would: **engines off, reservoir full,
brake pipe empty, and so the brakes are hard on.** You can't release the brakes until
the pipe is charged, and the engines are what charge it.

1. **`V`** to sit in the front cab.
2. **`I`** to start the engines. Both crank together and settle at about 700 rpm.
3. **`F`** to put the reverser in Forward. While every cab is in Neutral, the brakes are
   held on and no power is available.
4. **`,`** to bring the lever back to **N**, a notch at a time. Each notch the brake comes
   off, the pipe charges further. Wait at N until **BP reaches 5.0 bar and BC falls to
   0**. Until then some brake is still on, and power will be fighting it.
5. **`,`** again for **P1**. The train pulls away through the torque converter, and the
   revs flare before the drive takes hold. Go up to P5 for more power. The five-speed
   gearbox changes gear by itself on road speed, and you hear the revs drop at each
   upshift.

Reverse works the same way with **`R`**, except that power cuts out above about
**40 km/h**. Reverse is for shunting.

## Slowing and stopping

Use **`.`** to go back through N into the brake notches:

| Notch | Deceleration |
|---|---|
| B1 | 0.35 m/s² |
| B2 | 0.66 m/s² |
| B3 | 0.98 m/s² |
| B4 (full service) | 1.30 m/s² |

The notches ask for a deceleration, not a brake pressure, so a heavy train and a light
one slow at the same rate. Gradients are not corrected for, so brake harder going
downhill.

At speed, the Class 93 **brakes on its engines first.** In a brake notch the fuel is cut
and the train turns the engines over through the gearbox. You can hear the revs follow
the road speed, and the disc brakes only make up the difference. The engine brake
weakens below about 25 km/h, so the discs take over before the train stops, and they
hold it once stopped.

**Avoid repeated brake-and-release on a long descent.** Each application uses air
from the bogies' auxiliary reservoirs, which refill from the brake pipe much more slowly
than they empty. A second application before they have recharged is weaker than the
first. On a long downhill, hold a steady notch instead of pumping the brake.

To stop at a mark, B2–B3 from line speed is comfortable. Keep B4 for when you have
misjudged it.

## Emergency

**`Space`** cuts the power and dumps the brake pipe. Every bogie brakes as hard as it
can, and faster than a service application. The engine brake drops out.

To recover, bring the lever back to N with `,` and let the pipe recharge to 5.0 bar,
as when starting from cold.

The train can also go into emergency without you asking:

- **Low main reservoir.** If MR falls below **4.5 bar**, the safety device applies the
  emergency brake. It clears once the reservoir is back above **5.5 bar**. With the
  engines running, the compressors refill it, so the fix is usually to wait at idle.
  With the engines off, start them.
- **A parted brake pipe.** This happens after a coupling made too hard, or a train
  split. The brake pipe is fail-safe: losing it applies the brakes.
- **No cab in gear.** With every reverser at N, the brakes are held on.

In a train of two or three sets, any set whose safety device trips stops the **whole
train**. The HUD names the set responsible.

## Changing ends

Every cab has its own lever and reverser, and the train takes its commands from **the
cab whose reverser is in gear.** To drive back the way you came:

1. Stop, and put the lever into a brake notch.
2. Press **`V`** until you are in the cab at the other end.
3. Press **`F`**. Putting this cab in gear centres every other cab's reverser, so you
   don't need to go back and set the old one to N.
4. Release the brake and take power as usual.

If the controls move but the train does nothing, you are probably sitting in a cab that
isn't the one driving. The HUD warns you when this happens. Press `F` or `R` where you
are sitting to take control there.

## Two or three sets

A coupled train is driven from either **end** cab. The cabs at the couplers in the
middle are shut down: you can sit in them with `V`, but they won't go into gear. The
lever's commands reach every set, and each set brakes with its own air, so a set whose
reservoir is lower than the others brakes a little differently.

The **Scharfenberg** couplers couple by contact. Drive slowly into another Class 93:

| Closing speed | Result |
|---|---|
| under 5 km/h | Couples cleanly |
| 5–14 km/h | Couples, but the shock parts the brake hoses and the train goes into emergency |
| over 14 km/h | A collision: both trains derail |

Closing speed is the difference between the two trains' speeds, not your own speed.

To **uncouple**, stop, stand within 30 m of the coupler, and press **`U`** twice: once
to select it, and once more within 5 seconds to part it. Both portions then stay braked
until a driver takes them over. In the cab you want to drive, select N and then F or R
to release the hold.

## Letting the computer drive

**`P`** hands the train to the auto-driver. It starts the engines, puts the reverser in
gear if none is, and drives the road the switches and signals give it. Any move of the
lever takes control back. Pressing `P` again also takes it back, and leaves the train
braked at B2.

Note that there is no dead-man device. If you leave a train in gear under power, it
keeps running without you.
