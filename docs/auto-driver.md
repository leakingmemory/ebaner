# The auto-driver

`P` hands the train to the auto-driver; `P` again, or a hand on the controls, takes it
back. `EBANER_AUTO=1` hands the train over at startup and logs what it is doing once a
second.

## What it does

It drives forward along whatever road the switches are set for, up to the line speed,
and stops where it must:

| | |
|---|---|
| Line speed | the track's speed, and **70 km/h** on anything that isn't main line |
| A manned station worked by hand signals | at most **40 km/h** past the flag post. It stops in the station unless the flag is **green** when it passes the post |
| A main signal showing no road | stops about **20 m** short |
| The end of the rails | stops about 20 m short |

Any signal at danger stops it, a dwarf included. A **distant** never stops it, because it
only repeats the signal ahead.

A movement a **dwarf** has let past a main signal at danger is a shunt, and runs at
shunting speed. One a main signal has given a road to is a train, and runs at line speed.
The nearest signal ahead decides which.

**How the mode ends.** A train stopped at a signal starts again by itself when the signal
clears. Every other stop ends the mode: at a station, at the end of the rails, or by the
driver taking the controls, the train is left stopped and handed back.

## Where it stops at a station

A train crossing another must stand **clear of the switches at both ends**, so the stop is
inside the band between the outermost switches on its road through the station. Inside
that band it stops:

1. at the **platform**, for a passenger train, as near the TXP as the platform allows;
2. otherwise at the **TXP position** for that track;
3. otherwise in the middle of the band.

The stopping point belongs to the station and the track, and is worked out once per road
the first time it is needed. Every road through a station with a flag post is stopped,
not only the one the post stands on.

## How it brakes

**The braking curve.** It plans to come down at **0.55 m/s²** far out, easing to **0.25**
over the last **400 m** before the mark. The distance it plans over has the brake's
build-up time taken out first (`d − v · 3 s`), so it brakes earlier with a long, slow-to-
apply train.

**Whether** to brake comes from the curve. **How hard** comes from the deceleration the
road needs, `(v² − v_limit²) / 2d`, matched against what each notch gives
(`Vehicle::notchDecel`).

- Applications go on a notch at a time, with a pause between, unless the stop is in
  question. Full service is used only when nothing lighter will do.
- A rate between notches is held by cycling the handle.
- At low speed the notch is capped by speed: **B2** below 14 km/h, **B3** below 29, full
  service above.
- It holds still inside a band around the target, and waits **1.5 s** after any handle
  movement before moving again, except to brake harder.
- It aims to be at a speed restriction about **60 m** before it.

**Separate handles** (Di 4, CD 312) are used in the order a driver uses them:

1. above **30 km/h**, the **electric brake** first;
2. below that, the **independent brake** first;
3. the **train brake** for whatever is still needed;
4. at a stand, the train brake on firmly and the electric brake off.

**Power on the approach.** Climbing toward a stop, the train would stall short of it, so
below **18 km/h** on the approach it takes power to draw up to the mark. That is the only
time it powers toward a stop. A stop holds back power only within 800 m of it, so a train
isn't kept slow for the whole approach to a terminus.

## Code and tests

The decisions are a pure function of distances and speeds (`planDrive`,
`src/AutoDriver.h`), and the station stopping point is `stationStopDistance`. Both are
tested without a dataset. Only the road scan knows what a signal or a station is; it
reads the road five times a second and costs about 0.1 ms.

Which signals stop a train is `signalStopsTrain` in `src/SignalPaths.h`.
