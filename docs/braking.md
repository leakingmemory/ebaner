# Braking

How the brakes are modelled. How to drive them is in the driving guides:
[Class 93](driving-class-93.md), [Di 4](driving-di4.md), [CD 312](driving-cd312.md).

## The automatic air brake

The train brake is **failsafe**: it is applied by *losing* air, not by being given it.

- A **brake pipe** runs the length of the train, charged to **5 bar**.
- Every bogie has its own **distributor**, its own **auxiliary reservoir** and its own
  **brake cylinder**.
- The distributor remembers the highest pipe pressure it has seen, and fills its cylinder
  from its auxiliary in proportion to how far the pipe has fallen **below** that.

So the driver's handle commands the pipe, not the brake:

| Notch | Pipe reduction | Cylinder |
|---|---|---|
| B1, the lightest | 0.4 bar | ≈ 1.0 bar |
| B4, full service | 1.5 bar | 3.8 bar |
| Emergency | the pipe is dumped | full, sooner and faster, through a wider port |

A pipe that is cut, by a burst or by parting a coupling, applies every brake on the train
with nobody commanding it.

An auxiliary is sized so that emptying it into its cylinder equalises at full-service
pressure. The auxiliaries refill from the pipe far more slowly than the cylinders empty,
so a second application made before the pipe has recharged is weaker than the first.
Cycling the brake down a long descent wears it out.

Braking force is capped by wheel–rail adhesion, and the brake holds a vehicle at rest.

## How an application travels

**EP (electro-pneumatic).** The Class 93 works its pipe electrically: the handle tells a
valve on every unit to vent or recharge its own length of pipe in step, so the whole
train applies together. Emergency reaches the far end in **0.42 s**, one unit or three.
The Di 4, the CD 312 and the hauled stock have no EP.

A vehicle works its own pipe if the driver's valve is on it, if it has EP, or if its own
device is calling for emergency. Every other vehicle only follows its neighbours through
the hoses.

**The accelerator.** A distributor that sees the pipe falling fast past the emergency
threshold opens its own vent, which pulls its neighbour under the threshold in turn. An
emergency application therefore travels down a hauled train as a **wave**, at about
**280 m/s**. What takes the time is the **cylinders filling**, three to five seconds to
95 % in P, and that is the same for every vehicle. So a hauled train brakes slowly but
nearly together: a Di 4 with five carriages starts to pull up at **1.7 s**, the last
carriage 0.53 s behind.

The accelerator is a differential valve. It reseats when the rapid fall stops, either
because the pipe is empty or because the driver is refilling it, and must recharge before
it can act again. On a release the vents give way one at a time from the locomotive back.

**Release** has no accelerator: every pipe is filled from the one main reservoir at the
front. A Di 4 with five carriages takes about **12 s** to release, against 3.4 s for the
light engine. The 600 m freight charges from cold in about **2 minutes**, its last wagon
bites **3.6 s** after an emergency, and a release has the last wagon mostly off in 55 s
and the whole train in 2 minutes.

## The main reservoir

A locomotive or railcar has a **main reservoir**, kept up by its compressor, which
feeds the brake pipe through the **feed valve**. The pressures, from low to high:

| bar | |
|---|---|
| 4.5 | **low-reservoir safety device** trips: an emergency application, latched until the reservoir is back above **5.5** |
| 5.5 | the feed valve stops filling the pipe |
| 6.0 | the feed valve opens again |
| 6.5 | the compressor cuts in |
| 8.0 | the compressor cuts out |

Filling the pipe is what spends reservoir air, so it is the first thing to stop when the
compressor falls behind, and it resumes by itself. Charging the 600 m freight from cold
takes the reservoir down to about 5.57 bar, short of the trip.

The safety device is there to catch a **failed compressor**. A machine whose engines are
off loses its air through leakage, and in time trips it.

**Carriages and wagons have no main reservoir** and no safety device. Their auxiliaries
are fed from the brake pipe and nothing else.

## Multiple working

In a train of coupled Class 93 sets:

- The **driving cab's** brake notch reaches every set over the digital link, and each set's
  EP valve works its own length of pipe from its own reservoir. Sets in different states
  of charge brake differently.
- The **pipe is one pipe**: air crosses the couplings, so a burst at one end reaches the
  other in finite time.
- **Any set's** safety device puts the whole train into emergency, and the HUD names the
  set that did it.

## The engine brake (Class 93)

The Class 93's mechanical transmission connects the engine solidly to the wheels, so the
train can be braked against it. In the brake notches the fuel is cut and the train turns
the engine over.

Each notch is read as a **deceleration**:

| B1 | B2 | B3 | B4 |
|---|---|---|---|
| 0.35 m/s² | 0.66 m/s² | 0.98 m/s² | 1.30 m/s² |

The engine brake supplies what it can, and the friction brake is asked only for the rest.
Because the target is a deceleration it is corrected for the train's load, but not for
gradient: the driver still brakes harder downhill.

The engine's braking torque is taken as **0.7 of its rated power at governed speed**
(compression release), roughly constant with revs. At the rail that comes to about
11 kN in 5th, 18 kN in 3rd and 30 kN in 1st, so the engine brake is strongest in low
gear. It fades out as the engine falls back to idle, around 7 m/s in 1st, and the
friction brake holds the train at rest.

An emergency, a burst pipe and the low-reservoir trip cancel the engine brake and apply
everything on air. The blend can only hold the cylinders **below** what the pipe commands,
never above.

## The independent brake (Di 4, CD 312)

A second valve straight to the **locomotive's own cylinders**, fed from its main reservoir.
It touches neither the brake pipe nor anything behind the locomotive. It holds the
locomotive at a stand, and is used for shunting and running light. Holding for five
minutes costs about 0.3 bar of main reservoir.

## The electric brake (Di 4, CD 312)

The traction motors work as generators and the energy is burned in roof **grids**: a
rheostatic brake, commanded on the power controller's `E1`–`E5` notches.

The Di 4's brake curve has the shape of its traction curve: **flat at 180 kN** up to
28 km/h, where motor current is the limit, then **P/v** for the **1400 kW** grids above
that. Both figures are estimates; 180 kN holds the 435 t night train on a 1 % grade. The
CD 312's figures are estimates too.

- It **fades out at low speed** and is gone before the train stops.
- While it is working, the locomotive's own cylinders are held off from the **automatic**
  application, so the shoes and the grids don't both brake the locomotive. The carriages
  brake on air as usual, and the independent brake still works.
- An emergency drops the electric brake out and fills every cylinder.

Not modelled: grid thermal limits.

## Coupling shock

Two trains coupling at **5 to 14 km/h** part the brake hoses at the joint, and the new
train goes into emergency. See [Rolling stock](rolling-stock.md#coupling-and-collision).
