# Driving the CD 312

The CargoNet CD 312 (Vossloh EURO 4000) is the freight locomotive: 123 t, 3178 kW and
400 kN of starting pull, rated for 120 km/h. It works the same way as the
[Di 4](driving-di4.md): a power controller with an electric brake below N, a train
brake, and a locomotive brake, on the same keys. What is different is the train behind
it: **22 wagons, 599 m and 1797 t**, with a plain air brake from one end to the other.
This guide is mostly about handling that train.

## Picking it

On the start screen:

- **CargoNet freight (599 m)**, under *trains*: the CD 312 with nine pocket wagons and
  thirteen container flats, built to fit a 600 m loop.
- **CargoNet CD 312 (Euro 4000)**, under *locomotives*: the locomotive on its own. This
  is the easiest way to learn the controls before taking the full train.

To skip the menu, `EBANER_VEHICLE=5` gives the freight and `7` the light engine. Those
numbers move when a vehicle is added; the start screen shows the current ones.

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

The three handles move independently:

```
Power controller   E5 · E4 · E3 · E2 · E1 · N · P1 · P2 · P3 · P4 · P5
Train brake        REL · B1 · B2 · B3 · B4 · EMERG
Locomotive brake   L0 … L5
```

Below N, the power controller is the electric brake, so `.` at N goes straight into E1.

The HUD shows the handles on one line, and the engine on another:
`ENG RUNNING  900 rpm   exc 100%  amps 100%  load  40%`. `exc` is the load regulator
(how much of the generator's field is on), `amps` the traction current as a share of
its limit, and `load` how hard the diesel is working.

## Starting from cold

The train starts **engine off, train brake in EMERG, brake pipe empty**. All 112 axles
are braked.

1. **`V`** to sit in the front cab.
2. **`I`** to start the engine. The EMD 710 idles at about **200 rpm**.
3. **`F`** for Forward.
4. **`J`** a couple of notches, to hold the locomotive on its own brake.
5. **`K`** five times, from EMERG to **REL**, to start charging the brake pipe.

**Charging the freight from cold takes about two minutes.** All 600 m of brake pipe and
every wagon's auxiliary reservoir fill from the locomotive's one main reservoir. The
reservoir drops while this happens, to about 5.6 bar. If it falls below **5.5 bar** the
locomotive stops feeding the pipe until the compressor has it back above **6.0**, and
then carries on by itself. The charge then goes in steps. This is normal and needs
nothing from you.

**Don't take power until BP reads 5.0 bar** and has stayed there. Any wagon still
braked at the back is weight you are dragging.

6. **`,`** to **P1**, and release the locomotive brake with **`H`** as the train starts
   to take the load. Then work up the notches.

## Getting a heavy train moving

**The power builds slowly.** The governor takes about **6½ seconds** to bring the engine
from 200 to 900 rpm, and the excitation comes on more slowly still. Each notch keeps
building for several seconds after the revs have settled. Give it time before adding
more.

**At low speed the pull is limited by current, not power.** Below about **24 km/h** the
locomotive gives its full 400 kN, and the ammeter is at its stop while the engine does
little work: at 5 km/h only about a quarter of its power reaches the rail. Above 24 km/h
the pull falls away as speed rises, and the engine is fully worked.

**400 kN is all that 123 t will grip.** The CD 312 is right at its adhesion limit
(398 kN), and 1797 t is a heavy train for it. It will lift the freight on the Saltfjell
gradients, but with little to spare. On a climb:

- Take power in steps, and expect the speed to rise slowly.
- If you stop on a gradient, hold it on the train brake, not the locomotive brake alone.
  The locomotive's brake acts on 1 vehicle out of 23.

Reverse works the same way with **`R`**, but power cuts out above about **40 km/h**.

## Braking

### The electric brake

The power controller below N. It holds the locomotive's own air brake off while it
works, burns the energy in the roof grids, and makes about **220 kN** up to 29 km/h,
with the grids' 1.8 MW limiting it above that. It **fades out below 5 km/h**. Both
figures are estimates: no source publishes them for this locomotive.

On the freight, 220 kN is a minority of what the train needs to stop, but it is the
right brake for **holding speed down a gradient**, because it uses no air. Use it first
and add the train brake for what it cannot hold.

### The train brake

`L` applies, `K` releases. The wagons have **no EP brake**. The application starts at
the locomotive and travels down the train as each wagon's valve responds. On the
freight the last wagon starts braking about **3.6 s** after the first, and filling the
brake cylinders takes a few seconds more on every wagon. A 600 m train brakes later and
more slowly than a railcar. Start early.

**Releasing takes much longer than applying.** Releasing the freight takes about
**two minutes**: the last wagons are mostly off after about 55 s, and the whole train
after 2 minutes. If you release and take power straight away, you will be pulling against
the back half of the train.

**Avoid repeated brake-and-release on a long descent.** Each application uses air from
the wagons' auxiliary reservoirs, and they refill from the pipe slowly. A second
application made before they have recharged is weaker than the first, and on this train
recharging is slow. Hold speed on the electric brake, add a light, steady train brake if
needed, and leave it on.

### The locomotive brake

`J` applies, `H` releases. It acts on the locomotive only, from its own main reservoir.
On the freight it is worth only about **0.09 m/s²**. Use it for:

- holding the locomotive while you charge the pipe or take power,
- running light,
- shunting.

Don't rely on it to stop or hold the train.

## A stop

As a guide, the auto-driver stops the freight like this:

1. Above about **30 km/h**, the **electric brake** goes on first.
2. Below 30 km/h, the **locomotive brake** goes on first instead.
3. The **train brake** takes whatever is left, which on this train is most of it. It is
   applied early and lightly, B1–B2.
4. At a stand, the train brake goes on **firmly** to hold it, and the electric brake is
   released.

It aims to stop about 20 m short of a signal. From full speed, start braking well before
you would with a passenger train.

## Emergency

**`Space`** cuts the power and puts the train brake in EMERG. The pipe is dumped, and
every wagon's valve opens its own vent as the pressure falls past it, so the emergency
runs down the train as a wave. The electric brake drops out.

**Recovering takes time.** Bring the train brake back to REL with `K`. The wagons' vents
close by themselves once the pipe stops falling, and the whole 600 m then has to
recharge. Expect about two minutes before the whole train is off. Wait for BP 5.0 bar before taking power.

It can also happen without you asking:

- **Low main reservoir.** Below **4.5 bar** the locomotive's safety device applies the
  emergency brake. It clears above **5.5 bar**. The feed cut-out at 5.5 bar is there to
  keep a long charge from ever getting this low.
- **A parted brake pipe**, from coupling too hard or splitting the train.
- **No cab in gear.** With both reversers at N, the brakes are held on.

## Changing ends and running round

The CD 312 has a cab at each end, and the train takes its commands from the **cab whose
reverser is in gear**. Pressing `V` to the other cab and then `F` there takes control
there, and centres the other cab.

To run round the train:

1. Stop and apply the train brake.
2. Walk to the coupler behind the locomotive and press **`U`** twice (within 5 s). Both
   portions stay braked until a driver takes them over.
3. Drive the locomotive round on the locomotive brake.
4. Couple onto the other end at **under 5 km/h**. Faster than that, the shock parts the
   brake hoses, and the train goes into emergency.
5. Select N and then F or R in the cab you will drive from, to release the hold. Then
   recharge the whole pipe before moving.

## Letting the computer drive

**`P`** hands the train to the auto-driver. It starts the engine, puts a cab in gear,
releases the brakes and drives the road ahead, braking in the order described above.
Any move of a handle takes control back. Pressing `P` again also takes it back, and
leaves the train brake at B2.

There is no dead-man device. A train left in gear under power keeps running without you.
