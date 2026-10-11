# Rolling stock

The vehicles and trains in the simulator, and how they are modelled. Braking has its own
page, [Braking](braking.md); driving is covered in the guides for the
[Class 93](driving-class-93.md), [Di 4](driving-di4.md) and [CD 312](driving-cd312.md).

## Choosing a train

The start screen, and **place a train** in the Esc menu, list the stock under headings:

1. **Trains**: complete formations.
2. **Locomotives**, single.
3. **Carriages**, single, to make up a train by hand.
4. **Freight wagons**, which the CD 312 hauls.
5. **Debugging**: bare test shapes (a wheelset, a bogie, a carriage underframe, and an
   articulated module on three bogies).

`EBANER_VEHICLE=<n>` skips the start screen; the start screen numbers the list.

## The physics every vehicle shares

Each train moves along the track as one body (`src/Vehicle.cpp`, `src/Consist.cpp`):

- gravity along the grade, and weight on the rails;
- Davis running resistance, so a train coasts to a stop;
- a curve overturning limit;
- tractive effort and braking capped by wheel–rail adhesion.

A train that runs off the end of its track derails and slides to a stop on the ground.

Engine count, power, wheel size, driven axles, starting effort, axles per bogie and the
cab's control layout are each vehicle's own, in its row of `kVehicleSpecs`. That table
is kept in the order the pickers show it, because a heading is printed wherever the
category changes.

## NSB Class 93 (Bombardier Talent)

A two-section diesel railcar with a cab at each end: raked noses, a modelled saloon, and
in each cab a seat, a desk, the combined power/brake lever on the right and the
**R / N / F** reverser on the left. One set is about 41 m and 70 t.

**Transmission: diesel-mechanical.** A **torque converter** for starting, which multiplies
torque while it slips, feeds a **5-speed automatic gearbox** that shifts on road speed.
Tractive effort is the least of the gear/converter limit, the engine's constant-power
curve and adhesion, so the train starts hard and tapers toward its top speed. The HUD
shows speed and revs; the gear isn't shown.

**Engines.** Two Cummins N14E-R, one at each end, 306 kW each, idling at about 700 rpm and
giving full power at 1500. `I` starts and stops them. Their compressors keep the main
reservoir up.

**Engine brake.** Because the gearbox connects the engine solidly to the wheels, the
brake notches brake on the engine first. See [Braking](braking.md#the-engine-brake-class-93).

**Starting state.** As stock left overnight: main reservoir full, brake pipe empty, so
the brakes are hard on, and engines off.

The cab's speedometer, duplex air gauge (brake pipe and brake cylinder) and lever follow
the simulation, and the HUD repeats them with the main reservoir.

**Multiple working.** The start screen also offers two and three sets coupled:

| | Length | Mass | Axles | Power |
|---|---|---|---|---|
| one set | ≈ 41 m | ≈ 70 t | 6 | 612 kW |
| two sets | 83.6 m | 140 t | 12 | 1224 kW |
| three sets | 125.7 m | 210 t | 18 | 1836 kW |

## NSB Di 4 (Henschel)

A **diesel-electric** Co'Co' locomotive, one of five Henschel built in 1981 for this line.
An **EMD 16-645E3B** of 2450 kW at 900 rpm drives an alternator, which feeds
**three-phase asynchronous** traction motors from Brown Boveri and NEBB. 120 t, 20.8 m,
140 km/h, six axles on two three-axle bogies.

**Body.** A full-width welded carbody with the roof at one height end to end, the hatch
and radiator panels recessed between the cantrails, and the flanks tumbling in toward the
roof. Each nose has a **chin**: it reaches furthest forward at a knee a little over half
way up, rakes back above that, and tucks back in below it to a skirt over the buffer beam.
Cream bands on the flanks, a yellow snowplough at each end, and side buffers with a screw
coupling.

**Traction curve.** Three pieces:

- **flat** at the starting effort below a corner speed, where the motor current is the
  limit;
- **P / v** above it, the engine's power spread over the speed;
- never more than six driven axles can hold.

The corner falls where the first two meet, about **24 km/h**. At low speed the motors are
at full current but the engine is lightly loaded: at 2 km/h it gives about a tenth of its
power, at 10 km/h about half.

**Power builds up with a lag.** The **governor** takes the engine from its 315 rpm idle to
900 rpm over about seven seconds, and the **load regulator** builds up the excitation more
slowly still. Available power follows the revs the engine actually has, not the notch.
Excitation drops much faster than it builds. The HUD shows `exc`, `amps` and `load`. Off
power, the revs fall back to idle.

**No speed limit is enforced.** Run light at full power it settles at about 205 km/h,
against a rated 140 km/h. The rating is a gearing and certification limit, not where the
locomotive runs out of pull; keeping to it is the driver's job.

**Brakes.** A rheostatic electric brake on the power controller, an independent brake for
the locomotive, and the train brake. See [Braking](braking.md).

**Controls.** Two handles and a reverser, as in a German cab of the period: the **power
controller** (`E5`…`E1` · N · `P1`…`P5`) under the left hand, the **driver's brake valve**
on a curved quadrant under the right.

**Cabs.** One at each end, from interior photographs: two equal windscreens, the driver
on the right, a steel-blue desk the width of the cab, and in front of the driver a display,
two large dials (speed, and the **load meter**: the traction ammeter) and four small
gauges. They follow the simulation. Each side has a door window and a quarter-light.
`EBANER_CAB=<n>` seats the driver in a cab at startup.

Not modelled: the engine bogging under load, the EMD's discrete governor steps, and the
falling-off of effort at high speed in an AC drive.

## CargoNet CD 312 (Vossloh EURO 4000)

The freight locomotive; the Di 4 keeps the passenger trains. Six were built by Vossloh
España in 2009 and leased to CargoNet, to haul about 25 % more train than the Di 4 over
this line.

| | |
|---|---|
| Mass | 123 t |
| Length | 23.02 m |
| Power | 3178 kW, EMD 16-710G3C-U2 |
| Starting effort | 400 kN |
| Top speed | 120 km/h |
| Wheels | 1.067 m, Co'Co' |
| Idle | 200 rpm |

At 0.33 adhesion its six driven axles hold **398 kN**, so the locomotive is at the limit of
adhesion. It pulls about 30 % more than the Di 4 at 20 m/s.

**Body.** The Di 4's kind of roof, narrower than the body, with recessed hatch and fan
panels. The nose is **vertical** for the lower two fifths, then rakes back **1.10 m** in a
straight line to the roof, measured from a side elevation. Grey flanks with two louvred
radiator panels with vertical slats, an orange band at solebar level and round the nose,
orange grab rails, a dark window band wrapping round the cab, red markers high and
headlights low on the grey nose, and an orange snowplough.

**Cab.** The driver sits **inside a wraparound desk**: a light grey top for the handles,
a black instrument panel raked up to the windscreen, a taupe coaming. Left to right: the
fault panel, the power handle and reverser, the load meter and speedometer, the air gauge
and two small dials, the brake handles, and the EMD screens, keypad and start and emergency
buttons. The desk is set higher than a real one (1.02 m over the cab floor) so the panel
is in view from the seat.

**Estimates.** Width, height, bogie wheelbase, bogie centres and the electric brake
figures. Length, mass, power, pull and wheel diameter are sourced.

## Carriages: NSB Type 5

Built by **Strømmens Værksted**, 1977–81: 25.3 m on two two-axle bogies, 43 t, 160 km/h.
Red-and-grey livery, a red plug door at each end, the roof tumbling in above the
cantrail. Window and door positions and the interiors come from the operator's seat-map
drawings, and the interiors show through the windows.

| Variant | |
|---|---|
| **BC5-3** | Family coach, rebuilt 2010–12 with wheelchair spaces and a playroom. The left door is wider. Nine saloon windows, a 2.2 m blank side across the centre (the table bays), nine rows of four seats, a WC at one end, and the playroom and wheelchair bays at the other |
| **FR5-1** | Café car. Booths at one end, a blank side along the servery, a counter with stools, and a bike and ski bay. Its doors are at −1.4 m and +7.6 m, either side of the servery |
| **B5-3** | 2nd class. The BC5-3's windows and doors, with seventeen rows of four: 68 seats |
| **B5-5** | 2nd class with eight seats for passengers with pets. Drawn with the B5-3's body, since nothing differs outside or through the windows |
| **A5-1** | 1st class. A window across the centre and a short extra one at the far end; twelve rows of four (48 seats) at 1.27 m pitch, and a steward's service point |

The variants share one body and differ only in their `Layout`: where the windows and
doors are, and what is inside.

## Sleeping car: WLAB-2

Strømmen, 1986–87. 27.0 m long and 3.24 m wide, larger than the Type 5s, 50 t. Fifteen
small windows of 1.05 m at a 1.385 m pitch, one per compartment: fourteen sleeping
compartments and one accessible. Doors at ±12.09 m. Inside, compartments with berths at
two heights down one side and a corridor down the other.

## Freight wagons

**Sdggmrss T3000e, pocket wagon.** Articulated: two platforms on **three bogies**, the
middle one shared, so the wagon bends in the middle. 34.20 m over buffers, pivots at
2 × 14.20 m, 35 t tare and 100 t load, 120 km/h, 75 m minimum curve. Each half carries a
**semi-trailer**, its wheels in a well 330 mm above the railhead and its nose on a gooseneck
over the end bogie. Deep plate-girder sides. Drawn loaded, 95 t.

**Sgnss 60', container flat.** 19.60 m over buffers, pivots at 14.06 m, Y25 bogies, 19 t
tare and 71 t load, 120 km/h empty and 100 km/h loaded. Two 20 ft containers stand at
the ends, over the bogies. The 1.175 m deck height is an estimate.

## Formations

| Train | | |
|---|---|---|
| **Di 4 and five carriages** | Di 4, BC5-3, FR5-1, B5-3, B5-5, A5-1, the café second and 1st class on the tail | 335.4 t, 150.3 m |
| **Night train** | the same, with two WLAB-2 sleepers on the back | 435.4 t, 205.5 m |
| **Freight** | CD 312, nine pocket wagons and thirteen container flats, mixed | 1797 t, 598.82 m, 112 axles |

The freight is built to a **length limit** of 600 m, as a real freight train is planned:
it is the longest train the two wagon types make within it.

## Consists and cabs

Every train is a **consist**, a single vehicle being a consist of one. The consist owns
the motion: a coupled train has one speed, and every vehicle's pull and braking add into
it. Each vehicle keeps its own air system, engines, compressor and cabs.

- **One reverser.** Putting a cab into gear centres every other cab, so the train takes
  its commands from one cab. The HUD says when the cab you are in isn't the one driving,
  and `F` or `R` takes the reverser where you sit.
- Only the cabs at the **ends** of a train can drive. Cabs at a coupling are shut down:
  they refuse to go into gear, but can be sat in and their reversers centred. Coupling
  centres the reverser and takes the power off every cab no longer at an end.
- The controls and the HUD follow the **set in charge**: the one holding the reverser,
  or, with nothing in gear, the first one with a cab. So a locomotive that has run round
  its train is still driven from its own cab.
- Every vehicle follows its own path through the switches, so a switch thrown under a
  train splits it, and the train reports it.

## Coupling and collision

Trains couple by being driven together, and what happens depends on the **closing speed**
between them:

| Closing speed | |
|---|---|
| under 5 km/h | they couple. Momentum is conserved |
| 5–14 km/h | they couple, but the shock parts the brake hoses and the new train goes into emergency |
| over 14 km/h | a collision: every vehicle of both trains derails |

The gap is measured **along the rails**, following the switches, so two trains side by
side on a loop and the main aren't in contact.

## More than one train

Every train in the world is simulated, whether anyone is driving it or not. There is no
dead man's device, so a train left in gear under power keeps going by itself. The
interlocking reads track occupancy, so an unattended train holds its circuits and works
the crossings like any other.

From the **Esc** menu:

- **Drive another train** lists every train with its speed and the track circuit it is
  on, and seats you in the cab in gear, or the front one.
- **Place a train** puts one where you are or at any station. It is refused when the line
  there can't hold the train's outer axles, when there is no line near, or when a train
  is already there. Placing a train doesn't move you.

`EBANER_TRAINS="Trofors,Svenningdal"` places trains at startup, and `EBANER_MENU=train`
opens the picker.
