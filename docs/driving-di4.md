# Driving the Di 4

The NSB Di 4 is a 1981 Henschel diesel-electric locomotive. It is a different machine to
drive from the [Class 93](driving-class-93.md). Power and brake are on separate handles.
The train brake has no electro-pneumatic (EP) control, so an application has to travel
back through the carriages. The locomotive has a rheostatic (electric) brake and a brake
of its own. The engine also takes several seconds to answer the power handle. This guide
covers getting a train moving, stopping it, and handling it on a gradient.

## Picking it

On the start screen, under *trains*:

- **NSB Di 4 + 5 (cafe 2nd)**: the standard day train, five carriages.
- **NSB Di 4 + 5 + 2 sleepers**: the night train, about 435 t.

The locomotive on its own, **NSB Di 4 (Henschel)**, is under *locomotives*. Running
light is the easiest way to learn the controls. To skip the menu, use
`EBANER_VEHICLE=6` for the light engine, `3` for the day train and `4` for the night
train. Those numbers move whenever a vehicle is added to the list; the start screen
shows the current ones.

## The controls

| Key | |
|---|---|
| `V` | Sit in a cab. Press again for the other cab, and again to leave |
| `I` | Start / stop the engine |
| `F` / `N` / `R` | Reverser: Forward, Neutral, Reverse |
| `,` / `.` | **Power controller** up / down |
| `K` / `L` | **Train brake**: release / apply one notch |
| `H` / `J` | **Locomotive brake**: release / apply one notch |
| `Space` | Emergency brake |
| `P` | Hand the train to the auto-driver, or take it back |

There are three handles, and each moves independently of the others:

**Power controller** (`,` / `.`), under the left hand:

```
E5 · E4 · E3 · E2 · E1 · N · P1 · P2 · P3 · P4 · P5
◄──── electric brake ───  .   ,  ────── power ──────►
```

Above N it is power. Below N it is the **electric brake**. Pressing `.` at N goes
straight into E1, so count your presses.

**Train brake** (`K` / `L`), under the right hand. This is the automatic air brake on the
whole train:

```
REL · B1 · B2 · B3 · B4 · EMERG
```

**Locomotive brake** (`H` / `J`): L0 (released) to L5 (full). It puts air straight into
the locomotive's own brake cylinders from its main reservoir. The carriages and the
brake pipe are not touched.

Because the handles are separate, you can hold the train on the brake and wind power on
against it. The Class 93's single lever cannot do that.

The HUD shows all three on one line, for example
`POWER P2   , / .   LOCO L0  H/J   TRAIN REL  K/L / Space`.

## What to watch

| | |
|---|---|
| **BP**, brake pipe | 5.0 bar = train brakes released |
| **BC**, brake cylinder | the locomotive's own cylinders |
| **MR**, main reservoir | 6.5–8 bar while the compressor works |
| `ENG … rpm` | 315 idle, 900 at full power |
| `exc` | the load regulator: how much of the generator's field is on |
| `amps` | traction current, as a share of the limit |
| `load` | how hard the diesel is actually working |

`exc`, `amps` and `load` are what the Di 4 is driven on. The cab's second large dial is
the load meter (traction current).

## Starting from cold

The train starts **engine off, train brake in EMERG, brake pipe empty**, so every
brake is on.

1. **`V`** to sit in the front cab.
2. **`I`** to start the engine. It settles at about 315 rpm.
3. **`F`** for Forward.
4. **`K`** five times, from EMERG to **REL**. The pipe now charges from the main
   reservoir at the front, and that takes time. A light engine is released in about
   3 s, the day train in about **12 s**, and every carriage's pipe and reservoirs fill
   from the locomotive. Wait for **BP 5.0 bar**.
5. If you are on a gradient, apply the locomotive brake first with **`J`** a couple of
   notches. It holds the train while you take power, and you release it with **`H`** as
   the train starts to pull.
6. **`,`** to **P1**, then work up the notches.

**The power comes on slowly.** Each notch raises the engine's target speed. The governor
takes about **seven seconds** to bring the engine from idle to full speed, and the load
regulator applies excitation more slowly still. The revs arrive first, and the pull keeps
building for a couple of seconds after that. Give each notch time before taking the
next. If nothing seems to happen, look at `exc` climbing before deciding something is
wrong.

**At low speed the ammeter is at its limit and the engine barely works.** Pull is limited
by traction current up to about **24 km/h**. Rail power is pull times speed, so at
2 km/h the diesel gives about a tenth of its power, a quarter at 5 km/h and half at
10 km/h. Full revs, a pegged ammeter and a lightly loaded engine is normal at a start.
Past 24 km/h the current falls away, and the engine is fully loaded.

Reverse is the same with **`R`**, but power cuts out above about **40 km/h**.

There is **no speed limiter**. The Di 4 is rated for 140 km/h but has the power for much
more: running light at P5, it settles around 205 km/h. Watching the speed is your job.

## Braking

There are three brakes, and each has its own use.

### The electric brake: holding speed

Below N on the power controller. The traction motors work as generators, and the energy
is burned off in resistor grids on the roof. Its strength is limited to **180 kN** up to
about 28 km/h, and by the grids' 1400 kW above that. That is enough to hold the 435 t
night train on a 1 % gradient at any speed.

- Use it to **hold a speed down a long gradient** without using air.
- The HUD shows `E-BRAKE E3  120 kN` while it works. You can also hear it: the grid
  blowers wind up as the brake current rises and coast down after.
- It **fades out below about 5 km/h** and is gone before the train stops. It cannot hold
  a train at a stand; use air for that.
- While it is working, the locomotive's **own automatic brake is held off**, so the
  carriages brake on air and the locomotive on the grids. An emergency drops the
  electric brake and puts air on everything.

### The train brake: stopping

`L` applies, `K` releases. B1 is a 0.4 bar reduction of the brake pipe, the smallest that
does anything. B4 is full service, 1.5 bar.

The Di 4 and its carriages have **no EP brake**. The driver's valve vents the pipe at the
locomotive, and the application travels back through the train as each carriage's valve
(the distributor) responds. The train still brakes **nearly all together**. The day train
starts to slow after about **1.7 s**, with the last carriage about half a second behind.
What takes the time is filling the brake cylinders, which takes a few seconds on every
vehicle.

**Releasing is the slow part.** Every carriage's pipe has to be refilled from the one main
reservoir at the front, which takes about 12 s on the day train. Plan for it: release
early before taking power, and don't expect the brake to come off the moment you press
`K`.

**Avoid repeated brake-and-release on a long descent.** Each application uses air from the
carriages' auxiliary reservoirs, which refill from the pipe slowly. A second application
made before they have recharged is weaker than the first. Hold speed on the electric brake
and keep the air for stopping.

### The locomotive brake: standing and shunting

`J` applies and `H` releases. It acts on the locomotive only, using its own main
reservoir, so it is the brake for:

- **holding the train at a stand** while you charge the pipe or wait,
- **running light** or **shunting** without a train-brake application,
- a gentle stop with a light engine.

It costs very little air: standing on it for five minutes uses about 0.3 bar. It also
still works while the electric brake is on, if you want both.

## A stop at a platform

1. From line speed, wind the controller down through N into **E2–E4** to take speed off.
2. About a kilometre out, add **B1–B2** on the train brake.
3. As the speed falls below about 10 km/h the electric brake fades, and the air does the
   rest. Come back to **N** on the controller.
4. Stand on **B2** or on the locomotive brake. Before departure, release, wait for BP to
   recover, and take power.

## Emergency

**`Space`** cuts the power and puts the train brake in EMERG. The pipe is dumped, every
carriage's valve opens its own vent, and the whole train brakes as hard as it can. The
electric brake drops out.

To recover, bring the train brake back to REL with `K`, and let the pipe recharge to
5.0 bar. On the day train that takes some seconds.

It can also happen without you asking:

- **Low main reservoir.** Below **4.5 bar** the locomotive's safety device applies the
  emergency brake. It clears above **5.5 bar**. The running compressor refills it. A
  locomotive standing with its engine off loses air and trips after about half an hour.
- **A parted brake pipe**, from coupling too hard or splitting the train.
- **No cab in gear.** With both reversers at N, the brakes are held on.

When the reservoir is low, the locomotive stops feeding the brake pipe below 5.5 bar
and resumes at 6.0. On a long train this means charging may pause and then continue by
itself. That is normal.

## Changing ends

The Di 4 has a cab at each end. The train takes its commands from the **cab whose
reverser is in gear**.

1. Stop and secure the train, with the locomotive brake on or the train brake applied.
2. **`V`** to the other cab.
3. **`F`**. This puts the reverser in gear where you are sitting and centres the other
   cab. Pressing `F` or `R` in a cab always takes control there.
4. Release and drive.

If your handles move but nothing happens, you are in a cab that isn't driving. The HUD
says so.

To **run round** the train, stop, walk to the coupler behind the locomotive, and press
**`U`** twice (within 5 s). Drive the locomotive round, and couple onto the other end at
**under 5 km/h**. Both portions stay braked after parting until a driver takes them
over: in the cab, select N and then F or R.

## Letting the computer drive

**`P`** hands the train to the auto-driver. It starts the engine, puts a cab in gear,
releases the brakes and drives the road ahead. Any move of a handle takes control back.
Pressing `P` again also takes it back, and leaves the train brake at B2.

Note that there is no dead-man device. A train left in gear under power keeps running
without you.
