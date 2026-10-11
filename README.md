# ebaner

A train simulator for the Norwegian railways, in Vulkan and C++.

Trains are driven over the line as it actually is. The terrain, the track and the
gradients come from a `terrainmapper` export of the national data, and what the train
does on them is modelled rather than scripted: a continuous air brake that has to be
kept charged and that feels how long the train is, traction worked out from each
machine's own transmission, couplers that can be parted and run round, track circuits,
signals, interlocked routes and line blocks, and a driver the computer can take over
from. Every train in the world is simulated, not only the one being driven.

It also draws the world it runs through — the elevation tiles around a **chosen
station** stitched into a continuous 3D surface, with the railway, roads, buildings
and platforms on top, streaming in as you travel. The camera starts on the running
line at that station: at a terminus like Bodø that is the buffer-stop end of track 1,
resolved from the track geometry rather than named here.

A second program, **`ebaner-trackedit`**, is the authoring side — the track,
signalling and interlocking editor the dataset's overlay is written with.

## What it does

- **Terrain** — heightmap tiles shaded with hillshade and coloured by **AR50 land
  cover** when the export provides it (forest, agriculture, open land, bog,
  glacier, water, built-up …), falling back to an elevation ramp otherwise. The
  overlapping LOD quadtree is de-overlapped (finest tile per area) and
  watertight-stitched so there are no cracks between resolutions
  (`src/TerrainMesh.cpp`). Where a **surface** railway falls below the terrain, the
  ground is **cut into a trench** to fit the track and ballast — a flat floor at the
  ballast base with side walls sloping up at ~20° until they meet the original
  ground — so the rails sit in a cutting instead of being buried. Tunnels and
  bridges are left alone, and where a surface track enters a tunnel in a cutting the
  trench stops at the mouth, leaving a vertical portal wall of terrain. The carve
  edits the height grid as a world-space function (so LOD seams stay watertight) at
  load time (`src/TerrainCarve.cpp`; set `EBANER_NOCARVE=1` to disable).
- **Railway** — a realistic cross-section: a ballast bed, concrete sleepers and
  two rusty rails (`src/TrackMesh.cpp`). The export splits each line into separate
  segments at medium transitions (surface / tunnel / bridge); `buildTrackPaths`
  (`src/TrackPath.cpp`) **joins segments that meet end-to-end** (through degree-2
  nodes of the same track type) into continuous routes, so the line is unbroken
  through tunnels and the train can run straight through instead of derailing at a
  portal. Centrelines are then smoothed through the coarse surveyed points with a
  **centripetal Catmull-Rom** spline, so curves are continuous. Sleepers are real
  3-D boxes near the camera and a repeating texture at distance (distance LOD).
  Track is **superelevated (banked)** on curves from the OSM speed limit + curvature.
- **Only what is on screen** — the ground, the buildings and the ballast-and-rails run
  are each cut into pieces with a bounding sphere, and a piece outside the view frustum
  is not submitted (`Frustum` in `src/VulkanRenderer.cpp`). The world streams for twenty
  kilometres around the camera and a view holds a wedge of that, so most of it was being
  sent to the card every frame to be thrown away behind the eye. The pieces come off
  what the geometry already is — a terrain tile, the run of one map tile's buildings, a
  200 m length of line — and the bounds are measured from the vertices actually emitted
  rather than from the footprints they were derived from, since a roof stands several
  metres above the wall top it was built on. Nothing is reordered to achieve it: which of
  two roofs at the same height is the one drawn depends on the order they are submitted
  in, so grouping them by anything other than the order they already arrive in would
  quietly change the picture. Worth 39-51% of the frame time on the machine it was
  measured on, and it renders pixel for pixel what it did before.
- **Roads** — category-coloured asphalt ribbons (`src/RoadMesh.cpp`): the public
  network (Europavei/Riksvei/Fylkesvei/Kommunal) prominent, private tracks thin
  and faint.
- **Buildings** — OSM footprints extruded into lit prisms coloured by type, with
  **pitched roofs** (flat / gabled / hipped / pyramidal / skillion) from the OSM
  `roof_shape` tag (`src/BuildingMesh.cpp`).
- **Station platforms** — OSM `railway=platform` footprints (area platforms and
  buffered platform edges) extruded into low lit slabs beside the tracks
  (`src/PlatformMesh.cpp`). The slab top is set a standard 0.76 m above the
  nearest **rail head** (not the raw DTM), so long platforms sit flat at rail
  level instead of sinking into rising ground. The surface is styled to the
  Norwegian standard: an asphalt centre with light **concrete edge slabs** and a
  painted **yellow safety line** where the slabs begin, on the track-facing
  long edges.
- **Switch stands** — a classic Norwegian manual switch stand (sporveksel) at
  every turnout, i.e. wherever one track's end joins another track's line
  (`src/SwitchMesh.cpp`, `src/SwitchNetwork.cpp`). Each stand is a weighted lever
  (a bar ending in a low **cylindrical counterweight**) driving a throw rod to the
  movable rail, plus a tall post carrying a rotating **indicator target**. The
  target is one mechanical two-faced plate — an **arrow** on one face, a **filled
  circle** on the other, and edge-on it reads as a **vertical line**. Set straight
  it is edge-on to the track so both ends see the line; thrown to diverging it
  turns 90° so the toe/common approach sees the arrow (a facing train will be
  diverted to the siding) and the branch approach sees the filled circle (a train
  may run from the siding onto the main, but not the reverse). A broken switch
  sits part-turned with the lever centred.
- **Movable switches** — the switches actually route the train. Aim the centre
  crosshair at a stand and press **T** to throw it (straight ⇄ diverging); the
  lever and target animate and a train reaching the turnout is diverted to the
  set track. Enter a switch **facing** (from the common/toe side) and it takes you
  to the track it is set for — into a **broken** (neutral) switch it **derails**.
  Enter it **trailing** (from one of the two branches toward the common track)
  with the switch set against you and the train forces it through: it continues
  onto the common track but the switch is left **broken** in the neutral position
  (throw it again to repair it). Trailing through an already-broken switch just
  continues onto the common track.
- **Rail vehicle** — chosen on an in-window **start screen** (a text menu),
  and by the same list again from the Escape menu when placing another train.
  Both are grouped under headings — complete **trains** first, since that is
  what someone starting the simulator is nearly always after, then single
  **locomotives**, then single **carriages** to build a formation up by hand,
  then **freight wagons** — which are the CD 312's work and not the Di 4's, so
  they are kept apart from the passenger stock — and the bare test shapes last
  under **debugging** where they are out of the way but still reachable. `kVehicleSpecs` is stored in that order and must stay
  in it: the pickers write a heading wherever the category changes, so a row out
  of place would print its heading twice. The choices are a
  single-axle wheelset, a dual-axle bogie (two wheelsets + frame), a full-length
  **carriage underframe** on two dual-axle bogies (one at each end), or a longer
  **articulated module** on three bogies whose two underframe sections hinge over
  the shared middle (Jacobs) bogie so it flexes on curves, or a liveried **NSB
  Class 93** (Bombardier Talent) DMU — a two-section body with raked cab noses, a
  modelled saloon (low-floor gangway, raised vestibules, doors, boxed tech/WC
  areas, wall/ceiling lining and oriented seats) and a basic **driver's cab**
  (seat + desk, a combined power/brake lever on the driver's right and a smaller
  **R/N/F reverser** on the left) at each end
  (`src/VehicleMesh.cpp`). A small 1-DOF physics model (`src/Vehicle.cpp`) gives it
  mass, gravity resolved into along-track acceleration + weight-on-rails, Davis
  running resistance, box inertia and a curve overturning limit. It rolls under
  gravity on grades, can be **hand-pushed** (Up/Down), coasts to a stop via rolling
  resistance, and if it runs off the end of its track it derails and is slowed to a
  stop by ground friction proportional to its weight. A single **combined
  power/brake lever** per cab (`,` toward power N → P1–P5, `.` toward brake N → B1–B4
  → Emergency, Space slams emergency) drives both a real **automatic air brake**
  and the traction. The brake is failsafe, which is to say it is applied by *losing*
  air rather than by being given any: a **brake pipe** runs the length of the train
  charged to 5 bar, and every bogie carries its own **distributor**, its own small
  **auxiliary reservoir** and its own **cylinder**. The distributor watches the pipe
  and remembers the highest pressure it has seen; what it puts in its cylinder is
  proportional to how far the pipe has fallen *below that memory*, filled from its own
  auxiliary. So the handle commands the pipe, not the brake — `B1` is the smallest
  reduction that does anything (0.4 bar, giving ≈1.0 bar of cylinder) and `B4` is full
  service (1.5 bar, 3.8 bar of cylinder) — and **a pipe that is cut applies everything
  with nobody commanding it**. Part a coupling and both halves stop. The emergency
  position dumps the pipe instead of reducing it, which the distributors read as the
  largest reduction there is and answer through a wider port, so it goes on sooner,
  faster and to the stop. The Class 93 works that pipe **electrically**: the handle
  does not make a reduction at the front and wait for it to travel, it tells an EP
  valve on every unit to vent or recharge its own length of pipe in step — which is
  what makes the brake quick, and changes nothing about what the distributors then do.
  An auxiliary is sized so that emptying it into its cylinder equalises at full-
  application pressure, so a second application made before the pipe has recharged is
  weaker than the first: cycling the brake down a long descent wears it out, because
  the auxiliaries refill from the pipe far more slowly than the cylinders empty.
  Braking force is capped by wheel–rail adhesion and also holds the vehicle at rest. On the power side the vehicle **drives** through a modelled
  **diesel-mechanical transmission**: a **torque converter** for
  launch (torque multiplication while it slips, so the revs flare then couple) feeding
  a **5-speed automatic gearbox** that shifts on road speed (the engine steps down at
  each upshift); tractive effort is the least of the geared/converter limit, the
  constant-power hyperbola and the adhesion cap, so it launches hard then tapers toward
  a level top speed. The gearbox is modelled but not shown — only speed and rpm are on
  the HUD. This said **diesel-hydraulic** for a long time, and the word was wrong: the
  Talent family comes as diesel-mechanical (630 kW), diesel-electric (1,100 kW — the DB
  644) and electric, and the Class 93's 612 kW from two [Cummins
  N14E-R](https://en.wikipedia.org/wiki/Norske_Tog_Class_93) through "a five-stage gear
  box" puts it with the [DB 643](https://de-academic.com/dic.nsf/dewiki/292064) in the
  **mechanical** group. It is not pedantry: gears connect the engine to the wheels
  solidly and a hydrodynamic drive does not, so the label was also a statement that the
  machine had no **engine brake** — see the blend below, which is the thing that was
  missing because of it. Each cab has its **own** power/brake lever and reverser, operating
  independently; the keyboard drives the cab you are seated in (driver view `V`, else
  the front cab). The **reverser** (`F` / `N` / `R`) gates both across both cabs: with
  both handles in Neutral, or both out of Neutral, the brakes go to emergency and no
  power is available; only when **exactly one** cab is out of Neutral does that cab's
  lever take control, with power applied in its direction (reverse is capped to a
  shunting speed). Power and the recharge come from the **diesel engines** (one per cab
  end, 2 × 306 kW): start them with `I` (both crank to idle ~700 rpm together, shown as
  a rev-counter bar per engine on the left cab LCD; they are two Cummins N14E-R, 14 L,
  full power at 1500 rpm), they rev up under power, and their compressors refill the
  reservoir at idle — which also lifts it back above the low-air safety trip. It starts as
  stock left overnight does: **reservoir full, brake pipe empty and the brakes therefore
  hard on**, engines off — releasing means charging the pipe, which is what the engines
  are for. The cab's speed and **duplex air gauges** (brake pipe in red against brake
  cylinder, the two a driver actually watches) and the combined lever animate with the
  sim, mirrored on a HUD that carries the reservoir as well.
- **NSB Di 4 (Henschel)** — a second machine, and a quite different one: a
  **diesel-electric** Co'Co' locomotive, five of which Henschel built in 1981 for this very
  line. An **EMD 16-645E3B** of 2450 kW at 900 rev/min turning an alternator into **three-
  phase asynchronous** traction motors from Brown Boveri and NEBB — the first in revenue
  service anywhere — 120 t, 20.8 m, 140 km/h, on **six axles in two three-axle bogies**.
  It is a **full-width welded carbody** with the roof at **one height end to end**, and a
  nose at each end that has a **chin**: the front reaches furthest forward at a knee a
  little over half way up, rakes back above that to the roof edge, and tucks back *in*
  below it to a short skirt over the buffer beam. The hatch and radiator panels are
  **recessed** between the cantrails, which is what reads as a lower roof from the side and
  from a bridge. Head on, the **flanks tumble home**: the roof is about seven tenths of the
  body's width with the shoulders chamfered down to the sides, which is also why an overhead
  photograph shows red either side of the grey. The flanks carry **cream bands**, there is a
  **yellow snowplough** at each end — on this line not an ornament — and it couples the older
  way, on **side buffers and a screw coupling** rather than the railcar's centre Scharfenberg.
  Modelled from photographs and a side elevation, after two attempts from memory got it
  wrong in different ways — first a hood unit, then a wedge pointed in *plan* when the folds
  actually run across the locomotive. Nothing about it is
  the Class 93 in different paint. Where a torque converter and five gears make the railcar's
  pull *step* as it shifts, a generator makes one continuous curve of three straight pieces —
  **flat** below a corner speed, where the motors' current is the limit; **P / v** above it,
  which is all the power there is spread over the speed; and never above what six driven
  axles can hold. The corner is not a number anyone chose: it is simply where the first two
  cross, at 2460 kW × 0.85 ÷ 314 kN ≈ 24 km/h, and moves wherever the rating and the starting
  effort put it. So it pulls away hard and then runs out of pull, which is what a heavy diesel
  loco does and what the railcar does not.
- **CargoNet CD 312 (Vossloh/Stadler EURO 4000)** — the machine that works these freights
  now, and the Di 4's opposite number: the Di 4 is left on the passenger trains, and this
  one hauls the goods. Six were built by Vossloh España in 2009, leased from Beacon Rail
  and lettered CargoNet — "CargoNet Diesel" — and bought to take **25 % more train** than
  the Di 4 did over this line. **123 t, 23.02 m, 3178 kW, 400 kN starting, 120 km/h** on
  **1.067 m** wheels, Co'Co' again, and an EMD again: the **16-710G3C-U2** is the direct
  descendant of the Di 4's 16-645. Where the Di 4 is adhesion-rich, this is right at the
  limit — 123 t on six driven axles holds **398 kN** at 0.33, so its 400 kN rating is
  precisely all the weight can put down, which is what a freight locomotive is built to be.
  It out-pulls the Di 4 by about **30 %** at 20 m/s, which is the operator's claim arrived
  at from the figures rather than taken on trust.
  It shares the Di 4's **roof** — narrower than the body, shoulders chamfered down to the
  flanks, hatch and fan panels darker and **recessed between the cantrails** — and almost
  nothing else. Where the Di 4's nose has a **chin**, reaching furthest forward at a knee
  half way up and tucking back *in* below it, this one is in two parts: **vertical** for the
  lower two fifths, and then the whole rake above it — the windscreen and the roof — going
  back **1.10 m** in a straight line. Traced off the side elevation at 50 px/m rather than
  judged: 0.00 m of setback at the solebar and still nothing at 0.43 of the body height,
  then +0.30, +0.70 and +1.10 at 0.63, 0.80 and the roof. Grey flanks with two short **louvred radiator panels**, one behind each
  cab, whose slats run **vertically**; an **orange band at solebar level** the whole length
  and round the nose; orange grab rails at the cab doors; a dark **window band** the glazing
  sits in, wrapping from the windscreen round both cab sides; red markers high on the nose
  with headlights low on the **grey** — there is no black panel across the front; and an
  orange **snowplough** reaching down to 15 cm over the railhead. Built from a three-quarter
  view first, which got four of those wrong at once — a black lower front that was only the
  nose in shadow, a near-vertical face, horizontal louvres, and a flat wide roof — and
  corrected against a **side elevation** of 0312 002 at Grorud, which settles all four. The
  first pass at the rake then guessed 2.10 m from a knee a third of the way up, which is
  nearly twice the setback and starts it too low; measuring it off that same photograph
  gave the two-part profile above, and the difference between guessing and measuring here
  is the difference between a wedge and a locomotive.
  Its **cab** is a different room from the Di 4's, and the difference is the thing to get
  right: the driver sits **inside a wraparound desk** rather than behind a bench with the
  controls off to one side. Three surfaces — a light grey desk top the handles stand on, a
  black instrument panel raked steeply up from it to the windscreen sill, and a taupe
  coaming capping it. Left to right as the photograph has them: the fault panel of switches
  and coloured buttons, the power handle on its quadrant with the big black ball knob and
  the reverser beside it, the **load meter and speedometer** with a red indicator between
  them, the twin-needle **air gauge** and two small dials, the brake handles, and on the
  right wing the **EMD screens**, a keypad, and the green start and red emergency buttons.
  A blue seat inside the curve, cream lining, checker plate underfoot.
  Four things had to be *measured* rather than reasoned about, and each produced the same
  symptom — a cab that looked like geometry nobody had written. The panel's outward normal
  is worked out by asking which way the driver is, not from the sign of a cross product:
  this basis is `(right, tangent, up)` with `right = cross(up, tangent)`, so it is **left
  handed** and the right-hand rule gives the wrong answer — signed by hand it went wrong
  twice, in both directions, each time putting every dial a centimetre *inside* the panel.
  The panel itself has to be **steep and close**; taken up to the nose skin it rakes at 22°
  over 1.2 m and is seen almost edge-on. The gauge faces have to be **pale**, because a dark
  face on a dark panel is invisible at the distance a driver sits from it. And the desk sits
  high, 1.02 m over the cab floor, which is a compensation and not a measurement: the body
  floor here is 1.55 m over the railhead — the height stack counts the wheel radius twice —
  so a realistic 0.85 m desk is 60° below straight ahead and outside the view entirely.
  The colours are sampled from the photographs rather than guessed.
  **Width, height, bogie wheelbase and bogie centres are estimates** — no source gives any
  of the four, and a three-quarter photograph cannot be scaled — as are the two dynamic
  brake figures, which are flagged in the spec the way the Di 4's are. The length, mass,
  power, pull and wheel diameter are sourced. It goes on the line **light**: its train is a
  freight train and there is not a freight wagon in the model yet.
  Nor does the power arrive when the handle moves. **Two lags in series**, and they are
  different machinery: the **governor** walks a 45-litre two-stroke from its 315 rpm idle to
  900 over about seven seconds, and behind it the **load regulator** winds excitation on
  more slowly still, so the engine arrives at its notch speed first and the load goes on
  building for a couple of seconds after — which is what a diesel-electric sounds like from
  the seat. Available power is taken from the **revs the engine actually has**, not from the
  notch; taking it from the notch handed the driver everything the instant he touched the
  handle with the engine still idling underneath. Excitation comes off far faster than it
  goes on, because dropping it is only switching it off. The flat part of the curve is a
  **current** limit — tractive effort is what amperes buy — and the consequence is the part
  that surprises: rail power is effort × speed, so at **2 km/h with the ammeter against its
  stop the diesel is giving a tenth of its power**, a quarter at 5 km/h and half at 10, and
  only past the corner is it fully loaded while the current falls away. Full revs, pegged
  ammeter, loafing engine is not a fault; it is the machine. The HUD carries all three —
  `exc`, `amps`, `load` — because the lag is otherwise indistinguishable from a broken
  locomotive. Not modelled: the engine **bogging** under load and the regulator backing off
  to catch it, the notch-by-notch **discrete governor steps** of an EMD, and the
  breakdown-torque region above a second corner where an AC drive's effort falls as 1/v².
  And **nothing limits maximum speed** — light engine at full notch it settles around
  205 km/h against a rating of 140. That is deliberate, and the arithmetic is the reason.
  Placing the second corner so that 140 km/h came out as the balancing speed would put it
  at about 52 km/h, which would mean a locomotive that makes full power only between 21
  and 52 km/h and is two thirds loaded at 80 — a shunter, not the machine that worked the
  Nordlandsbanen. The absurdity is the finding: **140 km/h is a gearing and certification
  limit** — traction motor speed, brakes, bogie stability — and not the speed this thing
  runs out of pull at. A light engine really would go well past it, which is why there is
  no wall here and no overspeed cutoff. Exceeding a rating is the driver's business, as it
  is everywhere else in this simulator. It hauls nothing yet — a train of *unlike* vehicles
  is its own job — so it runs light engine. Its **governor answers the notch and nothing
  else** — there is no geared speed dragging the revs, which is why a diesel-electric winds
  up standing still — but only while it is *pulling*: off power the revs belong to the same
  cranking code every machine uses, so `I` starts it to its own 315 rpm idle and stops it
  dead. Setting them from the traction code unconditionally, as this first did, left a
  stopped locomotive sitting at idle speed calling itself *running*, and made it impossible
  to shut down at all — the revs went back up the same step they came down.
  It also has a **motstandsbremse** — a rheostatic brake, the traction motors driven as
  generators and the energy burned in roof grids rather than put back anywhere. That it has
  one is not in doubt: the trials of Di 4 651 record *"Effekt av elektrisk motstandsbrems
  ble testet"*. It is commanded on the **power controller**, which runs `E5..E1 · N ·
  P1..P5` either side of neutral — standard BBC practice of the period, and the cab
  photographs show two levers and a reverser with no third handle. Its curve has the shape
  of the traction curve because it shares the hardware and the same two limits bound it:
  **flat at 180 kN** to a corner at 28 km/h where the motor current is the limit, then
  **P/v** above it where the **1400 kW** grids are. Both those figures are **estimates** —
  no source I could reach publishes either for this locomotive — chosen as typical of a
  2450 kW machine and sanity-checked against what the brake is for: 180 kN holds the 435 t
  night train on a 1% grade at any speed it runs at.
  Two things about it matter more than the numbers. It **fades out below about 5 km/h and is
  gone before the train stops**, because a machine turning slowly generates nothing — and
  without that it would have been a *parking brake*, since the consist clamps retardation to
  the speed the train has rather than letting it push the train backwards. And while it is
  working the locomotive's **own friction brake is held off**, since applying both
  duplicates the effort and cooks the shoes; the carriages brake on air as usual, and an
  emergency drops the electric brake out and fills every cylinder there is.
- **The railcar brakes on its engine before it brakes on its shoes.** A mechanical
  transmission connects the engine to the wheels solidly enough to be braked against, so
  moving the Class 93's lever into the retardation zone cuts the fuel and lets the train
  turn the engine over — and the notch is read as a **deceleration** rather than as a
  brake pressure. The engine brake supplies what it can of that and the **shoes are asked
  only for the shortfall**, which is how a modern unit brakes and is why its pads last.
  The targets are not new numbers: each notch already implied one, through the reduction
  it commands and `kFullServiceDecel`, so B1–B4 come out at **0.35, 0.66, 0.98 and 1.30
  m/s²** — and the 1.30 at full service sits against the **1.2 m/s²** quoted for the DB
  643. Expressed as a deceleration the demand is **load-corrected for free** and
  deliberately *not* corrected for gradient: the driver still brakes harder down a bank.
  Measured, B1 at 17 m/s went from 1.01 bar of cylinder to **0.26** for the same 0.41
  m/s², and the engine is *heard* doing it — being driven by the train, the revs follow
  the road through the gearbox instead of sitting at idle.
  The one figure that is chosen rather than derived is how hard an engine brakes. Plain
  motoring drag is about a tenth of rated torque, which on 70 t is 0.05 m/s² and would be
  invisible; what makes it a brake is **compression release**, and the N14 is a classic
  Jacobs application, so the model takes **0.7 of rated power at governed speed** as a
  braking *torque* — near enough constant with revs, because the work is done once per
  cycle. So what reaches the rail goes with the **gear**: 11 kN in 5th, 18 in 3rd, 30 in
  1st, and
  the brake gets stronger as the box comes down, which is the right shape for something
  meant to hold a speed down a bank rather than to stop a train. It **fades out** as the
  geared engine speed falls back to idle — around 7 m/s in 1st — so the shoes have taken
  over well before the train comes to rest, and they are what holds it there.
  Emergency, a **burst pipe** and the low-reservoir trip all cancel the engine brake and
  apply everything pneumatically, as before. The blend can only ever hold the cylinders
  *below* what the pipe says and never above it, which is what makes that safe.
- **The independent brake** — the *Zusatzbremse*, a second valve straight on to the
  locomotive's own cylinders from its own main reservoir, touching neither the train pipe
  nor anything behind the drawbar. Every mainline locomotive has one, because without it
  there is no way to hold the engine at a stand, shunt, or run light without dragging a
  train brake application around; and the cab photographs show a second handle beside the
  train brake on its quadrant, which is where German practice puts one. `H` releases and
  `J` applies, immediately left of the train brake's `K`/`L`, so the two read along the row.
  It runs off the **main reservoir** rather than the auxiliaries, which is why a locomotive
  can stand on its own brake as long as it likes: five minutes holding costs 0.3 bar.
  It also puts the **dynamic brake interlock** on a proper footing. That interlock used to
  suppress the locomotive's brake *force* after the fact, leaving pressure in cylinders that
  made none — a gauge that lied. It now holds the **cylinders** off instead, so what the
  gauge reads is what the shoes do; and it holds off only the **automatic** application,
  because the driver's own valve is his to use. Under the grids a train application leaves
  the loco's cylinders empty while the carriages brake on air, and the independent still
  fills them if he asks.
  And you can **hear it**, which matters more than it sounds: the electric brake is the one
  control on this locomotive with no other feedback — no gauge moves, and the diesel stays
  at idle throughout — so a silent one is a control you drive by the HUD. The **grid
  blower** is fed from the braking current, so the fans speed up the harder the brake works:
  wind the controller back and the roar builds over a couple of seconds, notch off and it
  **coasts down** rather than stopping. Rushing air through the ducts with the blade-passing
  tone under it, and both the **level and the pitch** rise with the load — measured across
  the notches at 214 → 384 → 538 → 618 Hz. It is mixed as a **hum** rather than as a rush:
  four harmonics of the blade-passing tone, and the shaft throbbing under the air once a
  revolution because a fan that size is never balanced to nothing. Against the first cut at
  it, that is **+1.7 dB at 60–120 Hz and +4 to +6 dB from 120 to 500 Hz** at full brake,
  with the hiss above 500 Hz **down** 2 dB — a blower heard from the cab is loud enough to
  talk over, and what makes it loud is the bottom half. That the blower is current-fed is an
  **assumption**: reasonable for a BBC machine of this period and much the more interesting
  behaviour, but not something I could source for this locomotive. `EBANER_AUDIO_DUMP_GRID`
  renders the application to a WAV. Still not modelled: grid **thermal limits**, which there
  is no temperature anywhere in this model to carry — a real one fades as the grids heat and
  runs its fans on after the brake releases.
  It is **driven differently**, too. A railcar of the 1990s gives its driver one lever that
  walks power and brake along a single axis; a locomotive of 1981 does not. The interior
  photographs show a short ball-topped **power controller** on a gated base under the left
  hand, and under the right — set directly below the three air gauges, where a driver's eye
  goes when he is braking — a long lever swinging fore and aft in a **curved quadrant**,
  which is the signature of a driver's brake valve of the period. That matches German
  practice of these years, *Fahrschalter* left and *Führerbremsventil* right, which is what
  a Henschel cab of 1981 would be: the Di 4 descends directly from the three Henschel-BBC
  **DE 2500** three-phase prototypes (DB class 202, 1971). So the vehicle table carries a
  `ControlLayout`, the Di 4's cab draws both handles and a reverser stub between them, and
  `,` / `.` work the power controller while **`K` releases and `L` applies** the train
  brake. The difference is not decoration: with two handles you can hold a train on the
  brake and wind power on against it, which one lever cannot express at all.
  It now has a **cab at each end to sit in**, drawn from three interior photographs of a
  class of five: two windscreens of equal size either side of a narrow centre pillar with
  roller blinds above them, the **driver on the right** with a second seat and a plain flat
  table on the left, and one **steel-blue desk** the full width of the cab. In front of the
  driver a raised housing rakes back at him carrying, left to right, a rectangular display,
  two large dials and a row of four small gauges — and on a diesel-electric the second large
  one is a **load meter**, the traction ammeter the machine is actually driven on and the one
  gauge a Class 93 driver has no use for. Speed, load, main reservoir, brake pipe, brake
  cylinder and engine revs all track the sim, and the combined lever with its yellow collar
  leans with the handle. Each side carries **two lights** as well, set between the
  windscreen and the cab bulkhead and level with the screen, as the side elevation has
  them: the rectangular **door window**, and forward of it past a narrow pillar a
  **quarter-light** whose leading edge rakes back as it rises, so it stands on the sill as
  a wedge. Both are cut through the skin and the lining together with a reveal joining
  them — the flank used to carry a single pane laid on an *uncut* side, which is a dark
  patch and no daylight. `EBANER_CAB=<n>` seats the driver at start-up, because otherwise a
  cab is reachable only by keypress and cannot be screenshotted or checked headlessly.
  Fitting the cab is what found the **windscreen a metre too high**: the sill had been scaled
  off the side elevation between the roof and the *buffer beam* rather than the railhead, and
  the short ruler drove the whole nose up until the glass sat above a seated driver's eye. It
  is measured from the railhead now, and the sill lands at 2.46 m where the photographs put
  it. The lining follows the nose's own rake and folds where the nose folds, since a flat one
  either stands out through the front of the locomotive or leaves bare red paint beside the
  driver's shoulder.
  Making room for it meant the simulator stopping treating one vehicle as the only possible
  one. Engine count, power, wheel size, driven fraction, starting effort and axles-per-bogie
  were file-scope constants shared by every vehicle there would ever be; they are now the
  vehicle's own, in its row of the table, and `body` went back to meaning only which mesh to
  draw. The reverser interlock likewise asks whether a thing has engines rather than whether
  it is a Class 93.
- **Coupled sets (multiple working)** — a Class 93 runs in multiple, and the start
  screen offers **two and three sets coupled** as well as one: 83.6 m over the couplers,
  140 t, 12 axles, four car bodies, four cabs and four diesels, or 125.7 m, 210 t, 18
  axles, six car bodies, six cabs and six diesels for 1836 kW (`src/Consist.h`). Every train
  is a **consist**, a single set being a consist of one, so there is no separate code
  path for the common case. A consist owns the *motion* — a coupled train has one
  speed, and every set's pulling and braking adds into it — and nothing else: each set
  keeps its **own air system, its own compressor, its own low-reservoir safety device,
  its own engines and its own two cabs**. Two mechanisms run over that, and they are
  deliberately not the same one. The **digital link** carries the driving cab's brake
  notch and power demand to every set, whose EP valve works its own length of the brake
  pipe from its own reservoir and whose bogies then each fill their own cylinder from
  their own auxiliary, so two sets in different states of charge brake differently. The
  pipe itself is one pipe: air crosses the couplings, so a set whose own valve is doing
  nothing still loses its pipe when the set in front dumps, and a burst at one end
  reaches the other in finite time. The **emergency line** does
  not run over the link at all: any set whose own safety device trips puts the *whole
  train* into emergency, on its own account, whether or not anything was commanding it
  — and the HUD names the set that did it. Only the cabs at the two **ends** of the
  train drive; the cabs at the couplers are shut down and refuse to be put *into* gear,
  though they can still be sat in with `V` and their reversers can always be **centred** —
  a cab is shut down by being coupled to, which can happen while somebody is sitting in it
  in gear, so a rule that refused every change would strand that reverser and leave the
  train being driven from a coupler. Coupling therefore centres the reverser and takes the
  power off every cab that is no longer at an end, which is what shutting a cab down means.
  The reverser interlock is read across the whole
  train, so a cab in gear in each set is the interlock case and holds the brakes on.
  Each set carries its own place on the network and crosses its own turnouts, so a
  switch thrown under the train does to the trailing set what it would do to a train on
  its own; the coupler is measured every step and a train the points have split says
  so. Sets being separate objects with separate state is also what makes coupling and
  uncoupling in the world a later addition rather than a rewrite.
- **Coupling, and collision** — two trains now know each other exists, which until
  recently they did not: each consist was stepped alone and two driven together passed
  through each other in silence. A Class 93's **Scharfenberg** couples by being driven
  into, so there is nothing to arm and nothing to aim at — and the collision is not a
  separate mechanism but the same contact judged by a bigger number. What decides is the
  **closing rate**, not either train's speed: one catching another at 12 km/h while that
  one runs at 11 is closing at 1 and couples gently. Under **5 km/h** the couplers engage
  and the two become one train, momentum conserved rather than speed — a light set shunted
  by a heavy one leaves faster than the heavy one arrived. Between 5 and **14 km/h** they
  still engage, but the shock parts the brake hoses at the joint, and every distributor on
  the new train reads that as the pipe being lost and applies from its own auxiliary, with
  nothing commanding it. Above that it is not a coupling: every set of both trains goes on
  the ground, the same free-body slide that running off the end of the track already gave
  them. The gap is measured **along the rails** and not through the air, so two trains
  standing four metres apart on the two roads of a passing loop are as far apart as they
  ought to be; where a turnout lies between two ends, the distance is a walk that follows
  the points. Coupling is the one thing that takes a train *out* of the world, so it also
  puts right every index and pointer that named it. You hear all of it: one **synthesized
  voice** covers the whole range, with the closing speed moving the thud's pitch down, the
  noise darker and the decay longer together — two coupler heads clacking at walking pace
  is a fifth of a second and bright, a wreck is over a second and mostly low end. It is
  heard from where the trains met rather than through the near train's envelope, so a
  collision thirty kilometres away is thirty kilometres away.
  `EBANER_AUDIO_DUMP_IMPACT=out.wav` renders the range offline, as the other voices have.
- **Sdggmrss T3000e (pocket wagon)** — the articulated double pocket wagon combined
  transport runs on, and the first piece of freight stock here. Two platforms on **three
  bogies with the middle one shared**, so it is one wagon that bends in the middle rather
  than two wagons coupled: **34.20 m** over buffers, bogie pivots at **2 × 14.20 m**, 35 t
  tare and 100 t of load on six axles, 120 km/h, and it will go round a **75 m** curve.
  Each half carries one **semi-trailer**, and the shape of the wagon is entirely explained
  by that: a trailer standing on a flat deck is half a metre over the loading gauge, so its
  wheels drop into a **well** whose floor is 330 mm over the railhead, and its nose sits up
  on a **gooseneck** over the end bogie with the kingpin at 980 mm, the container deck
  either side at 1155 mm. Drawn **loaded** — 35 t of wagon and two laden trailers at 30 t
  is 95 t, well inside the 135 t it is rated for; an empty one would be its own row.
  The sides are **deep solid plate girders**, which is the whole character of the wagon
  from beside the track: a continuous web from the deck down past the trailer's wheels,
  with a flange along the top, stiffeners up it and amber lashing stanchions along the
  edge. Built first as two thin bars at deck level with daylight between them, which is
  not a wagon — you could see straight through into the well and out the far side.
  Checked by **casting rays across it** rather than by hunting for a camera angle: from
  0.5 m over the railhead up to the deck at 1.155 m, **every** ray is stopped. Below
  0.42 m they go through, which is right — that is under the solebar, where a real wagon
  has daylight and you see the track through it — and above the deck they pass only in the
  gaps between the two trailers.
  The articulation needed **no new code**: `bodySectionFrames()` already gives a
  three-bogie vehicle two sections, each oriented by its own pair of bogies, so the wagon
  bends at the shared bogie and each half follows the rail it is actually on — which is
  exactly what the "Articulated (3 bogies)" test shape at the bottom of the vehicle table
  was built to prove, long before there was anything real to use it on.
- **Sgnss 60' (container flat)** — the plain four-axle flat that does most of the work in
  intermodal traffic, and the companion to the pocket wagon: **19.60 m** over buffers,
  bogie pivots at **14.06 m**, the same Y25 bogies on a 1.80 m wheelbase, 19 t tare and
  71 t of load, 120 km/h empty and 100 loaded at 22.5 t an axle, round a 75 m curve.
  Loaded with **two 20 ft boxes** — and they go at the **ends, over the bogies**, not
  together in the middle, because that is where the wagon wants the weight and how a pair
  of TEU actually rides. The gap amidships is correct, not a modelling shortcut. Corner
  castings under each box, since that is what a container really stands on. Same steel as
  the pocket wagon, because it is the same steel: these run coupled together in the same
  trains. The **deck height of 1.175 m** over the railhead is an estimate — the makers'
  sheets give the length, the pivots, the bogie and the weights, but not that.
- **NSB Type 5 carriages, and a locomotive that hauls them** — the
  [BC5-3](https://www.norsketog.no/tog/personvogner/type-5): 25.3 m on two two-axle bogies,
  43 t, 160 km/t, built at **Strømmens Værksted** 1977–81 and rebuilt 2010–12 as
  *"rullestolplass og lekerom"* — wheelchair spaces and a playroom. Those are interior
  features and a carriage is a thing you cannot get into, so what is drawn is the outside:
  red-and-grey livery as three bands — red skirt, silver flanks, dark window band — with a
  red plug door at each end, **the left one wider** for the wheelchair rebuild, and the roof
  tumbling in above the cantrail.
  The window and door spacing and the **interior** are measured, not supposed. The
  interactive seat maps on that page are backed by **SVG drawings** — `/uploads/trains/
  seatmaps|filled/coach/Type5/Type5-BC5-3.svg` — and an SVG is geometry, so rendering the
  two and reading the pixel runs gives the real arrangement: two 0.66 m door windows at
  ±11.36 m, nine saloon windows of 1.22 m (0.83 m for the first), and **2.23 m of blank
  side across the centre** where the two table bays are. Inside, nine rows of four across a
  centre aisle at the measured spacings, the middle two facing over tables, a
  wheelchair-accessible **WC** behind the partition at one end — which is why there is no
  window there — and the **playroom** and wheelchair bays at the other. None of it can be
  entered, but all of it is behind glass and shows through the windows, which is the point
  of drawing it: the first attempt ran the interior lining the full height, six centimetres
  behind the glass, so the carriage read as glazed and completely empty.
  The **FR5-1** café car comes off the same drawings and is a quite different vehicle from
  outside: five windows over the booth seating at one end, a long **blank flank** where the
  servery is, three narrow ones over the far counter, another blank over the bike and ski
  bay — and its two doors are **not at the ends** but at −1.4 m and +7.6 m, either side of
  the servery, which is where the plan puts its exits. Inside: booths with tables, a counter
  with stools down one side, and racks in the stowage bay. Both carriages share one body and
  differ only in a `Layout` — where the windows and doors are, and what furniture goes in —
  because a Type 5 is a Type 5 and the seat plan is what makes one a family carriage and
  another a café.
  The **B5-3** is the plain 2nd class coach the BC5-3 was rebuilt out of, and it makes the
  point about where these carriages differ: its windows and doors are **identical** — same
  spacing, same central blank, same door positions — and inside it has **seventeen rows of
  four, 68 seats**, against the family carriage's nine rows and 36. What the BC5-3 spends
  those eight rows on is the **playroom**, which takes the last four metres of its saloon.
  Comparing the two through the glass is what caught a real error: the playroom had been put
  at +10.9 m, out in the vestibule, leaving two windows looking into an empty carriage. The
  drawing puts it at **+5.8 to +10.2**, with the wheelchair bays beside it, and that is only
  visible by looking at the carriage whose seating *does* run that far back.
  The **B5-5** is the same coach with eight seats set aside for passengers with pets, and it
  **shares the B5-3's body** — not for want of looking. Both side elevations and both seat
  plans were compared pixel by pixel: seat rows and windows agree to the centimetre, the
  only strong differences anywhere are at the two doors where one drawing carries an orange
  highlight and the other does not, and the eight pet seats are marked by being *green on
  the plan*, which is a seat-map convention for a designated area and not the colour of the
  moquette. Nothing to see, so nothing to draw — a near-duplicate mesh would be a lie about
  how much is known.
  The **A5-1** is the 1st class comfort coach, and the one variant whose *shell* is genuinely
  different rather than just its furniture: there is a **window across the centre** where
  every 2nd class variant has 2.2 m of blank side, plus a short extra one at the far end.
  Inside, **12 rows of four — 48 seats** — spaced **1.27 m** apart against 2nd class's 0.92,
  which is what the extra legroom looks like on a drawing, with a steward's service point
  amidships.
  So the variants share one body and one `Layout`, differing in where the windows and doors
  go and what furniture is inside — and the standard rake is **a list and not a count**:
  `"BC5-3,FR5-1,B5-3,B5-5,A5-1"`, café **second**, 2nd class **third and fourth**, 1st class
  on the **tail**, as it is marshalled. 335.4 t, 150.3 m.
- **The freight** — a CD 312 and 22 wagons, built to a **length limit** rather than to a
  wagon count, which is how a freight train is actually planned. 600 m is what has to fit,
  so this is the longest train the stock will make that does not exceed it: nine pocket
  wagons and thirteen container flats, interleaved because a real intermodal train is
  loaded with whatever is going that night. **598.82 m** over the couplers — 1.18 m short,
  and no other mix of the two types gets closer. `Consist::length()` sums the bodies and
  adds a coupler gap between each pair, so the 22 couplers are inside that figure and the
  arithmetic has to allow for them. **112 axles, 1797 t**: a heavy train, and deliberately
  so — it is inside what 398 kN of starting effort will lift on the Saltfjell grades, but
  not by much, which is the whole argument for having replaced a 2450 kW locomotive with a
  3178 kW one.
- **The night train** — the same five with two
  [WLAB-2](https://www.norsketog.no/tog/personvogner/wlab2) **sleepers** on the back.
  Strømmen again, but 1986–87 and not a Type 5 at all: **27.0 m on a 3.24 m body**, longer
  and wider and taller than the coaches it runs with, 50 t tare. Its side says exactly what
  it is — **fifteen small windows of 1.05 m at an even 1.385 m pitch**, one for each
  compartment, which is the *14 sovekupeer* and the *1 HC sovekupé* the builder lists, with
  a door at each end falling at ±12.09 m where the drawing's door outline is to the
  centimetre. Inside, compartments down one side with a **berth at two heights** in each and
  a **corridor** down the other, so from a platform one side shows berths behind the glass
  and the other the corridor wall. Getting that wrong is invisible in a screenshot and
  obvious to a ray cast through a window: the first attempt centred the berths on the
  carriage rather than on the compartment, giving berth → wall → berth, a sleeper with beds
  in its corridor.
  It is also the real test of hauling: **three different vehicle lengths in one train** —
  20.8, 25.3 and 27.0 m — which a pitch taken from the front of the train cannot space.
  **435.4 t, 205.5 m**, and still two cabs.
  Hauling meant a train could stop being **all one vehicle**. A `Consist` took one spec and
  made copies of it, and its spacing came from `lead().length()` alone — so a 20.8 m
  locomotive pulling 25.3 m carriages would have laid every one of them 4.5 m inside the
  next. Pitch, length, inertia and resistance are per-vehicle now, and a cab index counts
  only **real driving positions**, so this train has two cabs and not twelve.
- **A brake that feels how long the train is** — and it did not, at all. Slamming a Class 93
  to emergency reached full brake at the far end in **0.42 s whether the train was one unit
  or three**, 41 m or 124 m, because the application was not travelling: emergency was a
  train-wide line and every vehicle dumped its own pipe in the same tick. That is not a bug,
  it is **EP** — electro-pneumatic, the driver's valve echoed electrically at every vehicle —
  and a railcar of the 1990s has it. A locomotive of 1981 and carriages of 1977 do not. One
  rule now decides: *a vehicle works its own pipe if the driver's valve is on it, or it has
  EP, or its own device is calling for emergency; everything else only follows its
  neighbours.* The Class 93 keeps EP and keeps its 0.42 s.
  The first version of this then got the *mechanism* wrong, and it is worth writing down
  which way. It made the application seep along by diffusion and scaled it by pipe volume,
  which gave 75 m/s down the train where [UIC requires
  250](https://railwaynews.net/uic-544-1-brakes-braking-performance.html). The missing part
  is the **accelerator**: a distributor that sees the pipe falling past the emergency
  threshold opens its own vent rather than waiting for its air to be drawn away down a
  hundred metres of hose, and each one doing that drags its neighbour under the threshold
  in turn. The application therefore travels as a **wave**, and what actually takes the
  time is the **brake cylinders filling** — three to five seconds to 95% in P — which is
  local to each vehicle and the same however long the train is. So a hauled train brakes
  **slowly but nearly together**, not front-first: a Di 4 with five carriages starts to
  pull up at **1.7 s**, much where the light locomotive does, with the last carriage
  **0.53 s** behind it — 282 m/s down 150 m of train.
  Volume does dominate, but on the **release**, where there is no accelerator to help
  because nothing local can make air: every pipe has to be filled from the one main
  reservoir at the front, and the train takes **12 s** against the light engine's 3.4 s.
- **A train that drives itself.** Every train was driven by hand, which is fine for one
  and impossible for a scenario: the level crossings, the block signals and the train-order
  network are all about trains *meeting* each other, and there was no way to have a second
  one moving. `P` hands the train over. It runs forward along whatever road the switches
  are set for, up to the line speed, and stops where a stop is called for — **70 km/h** on
  anything that is not main line, **40** through a manned station worked by hand signals,
  a stand at a **main signal that is not offering a road**, and a stand at a manned
  station unless its flag is showing green. A signal it has stopped for starts it again by
  itself when it clears — that is the one exception, because a signal clears of its own
  accord and the train should then go. **Every other way the mode ends, it ends the
  same**: stopped, and handed back. A station, because the order there is given by hand
  and a driver has to be sent on; the end of the rails, because there is nowhere further
  to go and a train left armed against a buffer stop is waiting for something that will
  not happen. Making them alike is the point — something that starts the mode can rely
  on what it finds when it is given back. A hand on the controls takes it back too.
  The judgement is a **pure function** of a list of distances and speeds
  (`planDrive`, `src/AutoDriver.h`), which is why most of it is tested to the metre with
  no dataset at all; only the *scan* knows what a signal or a station is.
  **Whether** to brake comes from the target, which already carries the braking curve for
  everything ahead; **how hard** from the deceleration the road is asking for,
  `(v² − v_limit²) / 2d`, matched against what each notch is worth (`Vehicle::notchDecel`,
  the same `serviceDecel` the blend works to). Deciding *both* from the need was the
  mistake that made it hunt: the need is over half a metre per second squared the moment a
  70 restriction comes within braking distance at 130, so the train braked while sitting
  exactly **on** the curve it was meant to be following, released, drifted up, braked
  again, and arrived at the restriction doing 52 instead of 70.
  The curve is not one rate over one distance. It is planned at **0.55 m/s²** far out,
  easing to **0.25** at the mark over the last **400 m** — squared rather than linear, so
  it is most of the way into light braking by the middle of the transition. And the
  distance it plans over has the **build-up run** taken out of it first, `d − v·3 s`,
  because the seconds between the handle moving and the shoes being on stop nothing and
  the train covers them at the speed it already had. Counted out of the distance rather
  than hidden in a fudge factor it scales with speed and with the train — a long one is
  slow to apply and slow to let go, and this is what makes it brake earlier.
  The end rate is below what the lightest notch gives (B1 is worth 0.35), which is
  deliberate: a rate between notches is held by **cycling** the handle, and the average is
  what the train feels — a light set answers quickly enough for that to be smooth, and a
  long heavy one averages it along its own length. Trying instead to hold a steady
  application to the stand over-braked it: a steady B1 is more than the shallow end asks
  for, so the train came to rest fifty metres short and had to be driven up again. What
  *is* held rather than thrown away is anything above the lightest notch, worked down a
  step at a time. Applications go **on** in steps too, a notch at a time with a beat
  between, unless the stop is genuinely in question. Full service is what is left when
  nothing lighter covers the need — an auto-driver that reaches for everything it has at
  each restriction has nothing in hand for the one it misjudged.
  **A crawl is not an emergency.** Being a hundred per cent over the limit sounds
  alarming and is not, when the limit is 5 km/h: it is five km/h of overspeed, and the
  energy in 5–10 km/h is a quarter of what is in 10–15. The brake was chosen from the
  deceleration the arithmetic asked for, and at low speed the arithmetic asks for a great
  deal — at 10 km/h with the mark 8 m off, the build-up run `v·3 s` took the whole
  distance, the floor handed back **one metre**, and the sum came out above 3 m/s². Full
  service, to lose five km/h: the train stopped dead short of the mark, had to be driven
  up again, and did it over and over. The usable distance is now floored at a third of
  what is actually there and never under 8 m, and the notch is **capped by the speed** —
  B2 under 14 km/h, B3 under 29, full service only above. Capped by the speed, note, not
  by how far over the limit the train is, which is the distinction the old sum missed.
  Approaching a stop from a stand 150 m off it now uses **nothing harder than B2** and
  stands 24 m short, where before it slammed on and bounced.
  **A machine with separate handles is braked in the order a driver uses them**, which is
  not "apply the train brake harder". Above **30 km/h** the grids go on first — they cost
  nothing, they are quick, and they wear nothing out. Below it the grids have faded, so
  the **locomotive's own brake** goes on first. The **train brake takes whatever is left
  over**, and on a long train that is most of it: the grids and the loco's shoes work on
  one vehicle out of twenty, which the independent's worth to the whole train makes
  explicit — 0.47 m/s² behind a Di 4 and five carriages, 0.09 behind the 600 m freight.
  What is left over is measured rather than guessed, since the grids report the force
  they are actually making. At a stand the train brake goes on **firmly** and the grids
  are let go, which do nothing there anyway. Braking a Di 4 and five carriages to a stop
  2.5 km off: E5 with B2 at 88 km/h, E5 with B1 at 44, the grids off and **L4** at 10,
  then L1 with B3 to hold — standing 22.6 m short.
  It also turned up the reason the brake kept going to **emergency**, which three
  separate rounds of this had been chasing. `kMaxBrakeNotch` is the ELECTRIC brake's
  range on the power controller, E1–E5, and is *the same number* as `kEmergencyNotch` —
  so `notchFor`, whose fallback was `kMaxBrakeNotch`, dumped the pipe every time a
  restriction wanted more than full service, and the comment saying it never commanded
  an emergency was simply false. Full service is one notch below, and now has a name:
  `kFullServiceNotch`.
  Stopping at a signal it aims **20 m** short and simply holds the brake inside 25 m.
  From 700 m out at 64 km/h it now tracks the curve to within a km/h or two the whole way
  down — 36.6 against 37.5, 26.8 against 26.7, 15.4 against 15.8 — and stands 24 m short,
  on B2 at worst. The 600 m freight stands 22 m short.
  **Braking reverses to power** where it has to. Climbing to a signal the train loses
  speed to gravity and would stall short of it, so below **18 km/h** on the approach the
  power comes back on and draws it up to the mark — on a 2 % climb it holds about 18 and
  stands 5.9 m short, where on the level and running downhill it uses no power at all.
  That is the *only* time it powers toward a stop, and the restriction matters as much as
  the allowance: the braking curve toward a signal at danger permits 87 km/h at 600 m, so
  a controller that simply tracks it accelerates hard **at** a red signal and then brakes
  hard at it. Safe by construction, since the curve is the guarantee, and not what anybody
  does. Approaching a stop the driver holds what he has and lets it fall.
  What stops it hunting is what a driver does rather than what a controller does: a band
  around the target inside which **nothing moves**, and a **1.5 s dwell** after any handle
  moves before it may move again — waived only for *more* brake, since a brake that has to
  wait is not a brake. It holds an open 70 between 68 and 72 on one notch of power, and is
  at the limit **before** the board: the curve aims 60 m early, because the controller
  allows itself a little over the target and aiming at the board puts the train through it
  a few km/h fast.
  Eight faults, and every one came from driving it rather than from reading it. It held the
  brake whenever a stop existed *anywhere* ahead, so a train standing with a red signal a
  kilometre off never moved. Fixed, it crept the last few metres at 0.9 km/h for ever —
  too slow for the curve to ask for anything, too slow for rolling resistance to finish.
  Fixed again, it found the band between the slowest speed it would drive toward and a
  stand, and sat six metres short of a signal with the handles off and the brakes
  *released*. It stopped at a
  **distant**, which carries no authority at all: it repeats, from braking distance, what
  the first main signal ahead is showing, so a train that stops at one stops a kilometre
  short of the signal that actually meant it. Which signals mean stop is now
  `signalStopsTrain` in `src/SignalPaths.h`, beside `signalGivesAuthority` — one statement
  in the file that owns what a signal means, rather than a condition buried in a road scan
  where no test could reach it. Nothing passes a signal at danger, a **dwarf included**;
  what a dwarf can do is the other way round, authorising a movement past a *main* signal
  at danger, which is what shunting past a red exit is. A movement a dwarf is letting go
  is a shunt and runs at **shunting speed**; one a main signal has offered a road to is a
  train and runs at the line speed, and keeps it past the signal that gave it. The nearest
  signal ahead decides which it is — not "is there a main signal at all", which would have
  made a train on open line with no signals for miles into a shunt.
  And the braking in a row leaving Bodø, which looked like phantom signals, was real ones
  read a moment too late: **a dwarf falls to danger as the train moves over it**, and the
  road is re-read five times a second, so there is one tick where the signal being passed
  is a metre *ahead* of the nose and newly at danger. `needDecel` is `v²/2d`, which at a
  metre is enormous, so full service went on and came off again a tick later — once per
  signal passed. Anything closer than one tick of travel has been passed, and is ignored.
  And approaching Bodø it ran at **3 km/h** for the last two kilometres. Bodø is the end
  of the line, so the lookahead found the buffers — and the answer to a road that runs out
  was to clamp the whole target to a crawl, everywhere, rather than to plan a stop at the
  place the rails actually stop. The end of the road is a **stop like any other** now,
  20 m short of the rails, planned for with the same braking curve as a signal: on a dead
  end 780 m off the train runs up to it properly and stands **25 m short**. The same
  change stopped a related nonsense — withholding power from the moment a stop becomes
  the binding constraint, which for a stand two kilometres off still allows 160 km/h, so
  a train approaching a terminus was held down for the whole approach. That only applies
  within 800 m now. Underneath both was a latent one: with no switch network attached,
  the road walk handed back a stretch running off the end of the rails and reported
  success, so nothing was ever told the road had run out.
  And every distance was
  half a train too long, because `walkRoad` starts at the leading set's **centre**: it
  stood 20 m past each signal it stopped at. It now stands 4 m short of a 400 m mark and
  496 m from a 500 m one.
  The bench lied too, which is worth its own line: given a positive initial speed and
  driven from **cab 0**, the power pushes against the train's own momentum, because the
  two cabs of a set face opposite ways. The trace showed a train shedding 90 km/h at full
  power with no brake on — a fault in the test, read for an hour as a fault in the driver.
  **Where** it stands at a station is worked out, not authored. The flag post is a point
  on **one track** — Oteråga's stands on the main — and the driver only looked for marks
  on the road it was taking, so a train routed into the loop never met it: crossing two
  trains there, the red flag was out, the first ran straight through and the two nearly
  met head on. The flag belongs to the **station**, and every road through it is stopped.
  The post is a poor mark anyway: it says where the TXP stands, not where a train belongs.
  What a train crossing another has to do is stand **clear of the switches at both ends**,
  so the band between the outermost switches on its own road is the rule — the two that
  its loop leaves and rejoins the main by, or the outermost pair on the main, and a yard
  as complicated as Fauske falls out of that rather than needing its own case. Inside the
  band: a **platform** for a train carrying people, positioned as near the TXP as the
  platform allows; else the **TXP's own spot**, which is authored per track (`NO OTR TRACK
  1` on the main, `NO OTR TRACK 2` on the loop) and is the best answer there is; else the
  middle. A platform outside the band is ignored rather than clamped to its edge, because
  the edge is the switch. Two switches make a band and one does not — a single switch
  ahead means the train is already inside with the other end behind it, and calling that a
  band would put the stop exactly on the switch. The mark is **booked** when the station
  stop is first raised and aimed at thereafter, since half the switches fall behind as the
  train runs in and a mark recomputed from what is left jumps forward to the exit switch
  just as the train nears the middle.
  Two more came out of driving it at Oteråga, and both were about **booking too early**.
  The mark was booked from whatever the first scan saw, and the first scan sees the
  station from two kilometres away — beyond both its switches, so with no band at all and
  nothing to go on but the fallback. Worse, "is the station on this road" allowed a
  closest approach of **500 m**, which is exactly how far off the last sample of a 2 km
  walk is when the station is still beyond it: the station read as being on the road, at
  the far end of the walk, and the mark was frozen there — about 500 m short of the
  station, which at Oteråga is outside its entry signal. That is where the train stood.
  So the band is worked out **fresh whenever it can be** and the booking is used only
  when it cannot — which is the one case it was for, the entry switch falling behind as
  the train runs in — and only a real band is ever booked. The threshold is 250 m, since
  a station node stands beside its own tracks.
  That is twice this scan was fixed by reading it and twice it was still wrong, so the
  deciding came out of the road loop into `stationStopDistance` where a test can reach
  it — and the test drives Oteråga's own distances off the ground: entry signal at 31 m,
  switch at 278, flag post at 498, far switch at 1104, and the train standing at **691**.
  And then the loop failed anyway, with the target flicking between 40 and 0 and the
  brake slamming on for tenths of a second at a time while the train ran out the far end.
  `Turnout::world` is **already world coordinates** — the header says so — and the
  attachment added the scene origin to it, moving every turnout to the same nonsense
  place. They all attached to whichever flagged station was nearest *that*, which is one
  station for the whole network and **a different one depending on where the session
  started**: it came out right from one start and wrong from another, which is why the
  same move behaved differently for me than it did in the cab. With no switches belonging
  to the station there is no band, so the fallback was used — and the fallback is the
  road's closest approach to the station node, which flickers in and out of range as the
  node goes by. Hence a stop that appeared at nothing-metres, vanished, and reappeared.
  It failed once more after that, and the right answer was to stop asking the question
  every tick. **Where a train stands at a station is a property of the station and the
  track**, not of where the train happens to be — so it is worked out once per road, the
  first time that station is wanted, and then left alone. Everything bolted on to stop
  the answer moving under the train — booking the first one seen, falling back to the
  station node, refusing a band of one switch — was patching something that should never
  have been recomputed. It is lazy rather than done at load because a platform can only
  be found in a tile that is in memory, and at load only the ground around the start is.
  Three more faults fell out of making it deterministic, each visible in the line it now
  prints per road. **A turnout at the end of a loop lies on two of the road's stretches
  at once** — the main by its arc length there, the loop by its own — so it was counted
  twice, and two entries made a "band" out of one switch whose two ends are the same
  place: the stop landed exactly on the switch. **Nearest-station is not enough on its
  own**: there are seven flagged stations on the line and turnouts everywhere between
  them, so a siding out in the country attached to whichever was least far away — Oteråga
  came out with three roads through it, two of them somebody else's, each with a stopping
  point a train could be sent to. A turnout now has to be within 1,500 m of the node.
  And **a platform lies beside the rails, not over them**: the footprint covers the
  surface people stand on, so a point taken on the track centre is never inside one.
  Sampling the centreline found no platform anywhere on the line, which read as the
  dataset having none — the tile holding Oteråga has one. It looks 4 m to either side
  now, and Oteråga's platform is found on both roads.
  Finding it froze the simulator for **fifteen seconds** as the train pulled away, which
  the profiler put squarely on the auto-driver's first call. `platformTopAt` answers "is
  there a platform here, **and how high is its top**" — and the top means finding the
  rail the slab is built against, which searches **every path in the network**, twice,
  plus a dense ground sample along every footprint edge. Reasonable once per platform
  when the slab is built; ruinous as a containment test asked twice every ten metres
  along a station road. `platformCovers` answers only the half that was wanted:
  **15,641 ms → 5.8 ms**, same answer.
  Attaching switches to stations by "nearest station" was wrong once, too: the station
  list holds every stopping place on the line, and an unstaffed halt a few hundred metres
  off is nearer to a crossing station's switches than the station itself — which left
  Oteråga's turnouts attached elsewhere and the band empty at the one station that
  mattered. It is nearest among the stations that **have a flag**.
  It reads the road **five times a second**, not every frame, and costs about 0.1 ms.
- **A 600 m freight train found two more of these, and both were invisible on a short
  one.** The first: the accelerator could not fire on hauled stock at all. `bpRate_` was
  measured from inside the vehicle's own step, *after* `Consist` had already moved air
  across the couplings — and the couplings are the only thing that moves a wagon's pipe,
  since it has no valve, no EP and no device of its own. So a wagon's rate was always
  exactly zero, the accelerator never tripped, and the wave above was really diffusion
  through the hoses. On 150 m of carriages diffusion is nearly as quick, which is why the
  figures looked right; on 600 m it is not. The pipe is now marked before anything in the
  step touches it, so the rate is the rate of *that pipe*, hoses included.
  The second is the one that was actually reported — an **emergency that sticks on and
  cannot be released**. It had two halves. The vent closed when the pipe came back *up*,
  but the open vent is precisely what holds the pipe down, so the only way out was for all
  600 m to reach zero and have nothing left to vent: on a long train, for ever. A real
  accelerator is a differential valve and reseats when the **rapid fall stops**, which
  covers both of the ways that happens — the pipe is empty, or the driver is refilling
  against it and the two have met. It then has to recharge before it can act again, which
  is what stops a wagon dipping for one step and eating the refill. All of it is per
  vehicle, so on a release the vents give way one at a time from the locomotive backwards
  and the fill travels with them. The other half was the **low-reservoir safety trip at
  6.0 bar against a compressor cut-in of 6.5**: charging a pipe costs main-reservoir air,
  23 vehicles of it cost about 5.75 bar, and there were only 2.0 between full and the trip.
  The train could not be charged *at all* without tripping a device meant to catch a failed
  compressor — an emergency nobody commanded, every time, which the accelerator then
  refused to let go of. The trip belongs well below the compressor's working range, at
  **4.5 bar**, and there is now a third pressure between the two: the **feed valve's
  cut-out at 5.5 bar**, opening again at 6.0. Filling a pipe is what spends reservoir air,
  so it is the first thing to give up when the supply is falling behind — the fill stops,
  the compressor is left to get ahead of it alone, and the fill resumes by itself once
  there is air to do it with. It sits below the compressor's cut-in, so the compressor is
  already working by the time it closes, and a clear bar above the emergency trip, so it
  is what keeps the reservoir *off* that trip rather than something that happens after it.
  The reading order is now 4.5 emergency, 5.5 feed closes, 6.0 feed opens, 6.5 compressor
  cuts in, 8.0 cuts out. Charging 600 m from cold bottoms the reservoir out at **5.57
  bar** — so on the worst thing in the timetable the protection is armed and does not
  quite fire, and anything more demanding than that gets a fill in steps instead of an
  emergency. As with the safety device it is the locomotive's own reservoir and its own
  valve: what the wagons at the far end are doing is not something the driver can see, and
  must not be what decides whether the pipe may be filled. The 600 m train now charges from cold in 2 minutes without tripping
  anything, bites at the last wagon in **3.6 s**, and — with the handle thrown back to
  release while wagons are *still* venting — has that last wagon mostly off in **55 s** and
  the whole train off in **2 minutes**, with no vent re-opening. Nothing a wagon does holds
  the driver's handle hostage, which is as it must be: the driver cannot see the last
  wagon.
- **A carriage has no main reservoir**, and no low-reservoir safety device. That device is a
  locomotive's, there to catch a **failed compressor** — and a vehicle that never had one
  cannot have one fail. Getting this wrong stopped a hauled train dead after **half an hour
  of running**, wherever it happened to be, with the brake handle sitting in release: every
  vehicle leaked its reservoir and only a vehicle with a compressor could make it up, so an
  unpowered one fell 8.0 → 6.0 bar in about **34 minutes** and tripped. That latched an
  emergency across the whole train, and nothing on a carriage could ever recharge it above
  the reset pressure, so it never cleared — a locomotive cycling its compressor happily at
  the front and seven carriages quietly running out of air behind it. A carriage carries
  **auxiliary** reservoirs fed from the train pipe and nothing else. The real fault is still
  modelled: a machine that *has* a compressor and is not running it does lose its air, and
  does trip, at 34 minutes.
- **A cab index is not two per set**, and a locomotive running round its train is what
  proves it. `cabCount` counts only real driving positions, so a Di 4 on five carriages has
  two cabs and not twelve — but three places still worked the index out as `set * 2 + end`,
  which is true only of a train that is all cabs. Draw the locomotive off a 600 m freight,
  run it to the other end and couple on, and it becomes set **22** of 23 while still
  holding cabs 0 and 1. `setReverser`'s "centre every other cab" then centred *wagon 0*, so
  selecting the near cab left the far one in gear as well: two cabs in gear, `activeCab`
  reads neither, the reverser interlock calls for emergency and the controls are dead. The
  cab gauges read wagon 0's brake pipe at the same time, and the driver's seat was worked
  out as cab `2 × 22`, which `driveTrain` clamps — so on contact he was thrown to the other
  end of his locomotive. All of it goes through `cabUnit`/`cabEnd` now, with `cabIndex` as
  their inverse for anything that has to hold a seat across a coupling. The last piece was
  the **uncouple hold**: it is released by *sampling* the reversers for Neutral and then
  gear, which assumes a held train has nothing in gear — true of one just parted, false of
  one a driver has just coupled onto in gear. Taking over a held portion now starts at
  Neutral, and his next selection is the cycle that releases it.
  The same run-around then showed the other half of it: what the driver has *in front of
  him* was read off `lead()`, the front of the train. With the locomotive at the back that
  is a wagon, so the CD 312's cab offered one combined handle in place of its power
  controller, train brake and locomotive brake, and the electric brake read 0 kN. The
  reverser interlock was asked of the same wagon, which has no engines — so it had quietly
  stopped applying to hauled trains altogether. `controls`, `hasIndependentBrake` and the
  interlock now ask the set in charge: the one holding the reverser, or the first set with
  a cab when nothing is in gear, which is the locomotive wherever it has ended up.
- **One reverser, one driving cab** — the train takes its commands from the cab holding
  the reverser, so putting a cab into gear **centres every other cab**: a locomotive has one
  reverser handle and the driver carries it to the end he is working from. This is also a
  trap closed. Leave the reverser at one end, walk to the other, and every control there
  moves and reads back while the locomotive does nothing — no error and no clue, because the
  commands are being taken from a cab you are not sitting in. It only bites a machine with
  two cabs you can sit in, which is why it surfaced the day the Di 4 got its second one. The
  HUD now says so outright when the cab you are in is not the one driving, and F or R takes
  the reverser where you sit.
- **More than one train** — every train in the world is simulated, whether or not anyone
  is driving it. There is no dead-man device, so a portion left in gear under power goes
  on running by itself indefinitely, holds its own track circuits, forces its own switches
  and is heard from the lineside as it passes; the interlocking reads occupancy and not
  the driver's train, so everything it does is as true of an unattended one. `Esc` opens
  the menu, and from it: **drive another train**, which lists every train there is with its
  speed and the track circuit it is standing on — `TRO T1`, `SGD-TRO` — and seats you in
  the cab that is in gear, or the front one if none is; and **place a train**, either where
  you are standing or at any of the 720 stations, opened on the nearest to you. Placing is
  refused rather than fudged when the line will not hold the train (what has to fit is the
  outer *axles*: 60 m of line takes a Class 93 with its ends hanging over, 30 m does not),
  when there is no line near what was asked for, and when a train is already standing on
  the spot. `EBANER_TRAINS="Trofors,Svenningdal"` does the same before the first frame, and
  `EBANER_MENU=train` opens the picker, so a two-train scenario for the signalling can be
  set up and screenshotted without touching the keyboard. Placing a train never moves the
  driver: you keep what you were driving and take the new one over when you want it.
  With an audio backend
  (PulseAudio or PortAudio), the brake air is **synthesized** in real time: a hiss
  whose loudness tracks the airflow — a subdued charge on apply and a prominent,
  brighter vent on release — fading as the pressure equalizes and with camera
  distance to the bogies, plus a valve click at each change of the handle or the
  safety. Each diesel gets an **idle drone** whose character is the engine's own, not one
  set of constants: how fast it beats, how loud it is, how much weight it carries below
  the firing rate and how much of the bark gets out all travel with the voice, so a
  locomotive and a railcar in the same train sound like different machines. The firing
  rate falls out of the cylinder count and the cycle, and it goes the opposite way from
  intuition — the Class 93's six cylinders firing every *other* revolution at 700 rpm beat
  35 times a second, while the Di 4's **V16 two-stroke** firing every cylinder every
  revolution at 315 rpm beats 84. The big slow engine has the faster beat. What makes it
  sound big is underneath: a V16 is two banks of eight with a manifold each, so it radiates
  hard at half the firing rate and below, and the Di 4 carries **2.4× the low-end energy**
  and runs **2–2.8× louder** than the railcar, weighted low (low/mid 1.9 against 1.1 at
  idle). Weight is not the whole of it, though, and a sine stack is a smooth thing where a
  big diesel is not: the **exhaust blast** is a pulse over about a third of the firing
  period rather than a wave, **no two firings are alike** (a third of the amplitude either
  way — sixteen cylinders are never quite in step), and the **gear train and injectors**
  are heard as a rattle at half the firing rate, deliberately left *outside* the insulation
  low-pass because a pipe radiates dark and a crankcase cover does not. That is a change of
  spectrum and not of level: the extra harmonics are paid for by trimming the voice back,
  so the Di 4's engine measures the same RMS it did and carries **+2 to +10 dB** from
  500 Hz up. All of it keys off the rumble weight the voice already carries, so the railcar
  — which has none — is untouched, and its dump is **bit-identical** to the reference.
  A third engine found two faults in all this that two could not. The CD 312's EMD 710
  idles at **200 rpm** where the 645 idles at 315 and the railcar's Cummins at 700, and
  the voice used to fade in over a fixed window of revs — 100 to 300 — so an engine that
  idles below 300 never finished arriving and sounded **half a locomotive** for ever. The
  fade is now a fraction of the engine's *own* idle, which leaves both older machines at
  full voice exactly where they were and changes only the **0.2 s** of cranking, where it
  is the better answer anyway: no combustion voice at all while the starter is still
  turning it. The other is the rumble weight. It is generated at a quarter and a half of
  the firing rate, and at 53 Hz firing those land at **13 and 27 Hz** where the Di 4's
  land at 21 and 42 — so with the Di 4's figure **63 % of this engine's output was below
  20 Hz**, heard by nobody and eating the headroom the rest of the train needs. It carries
  half as much, and a slightly brighter low-pass to put the difference where it can be
  heard. Measured, not guessed: `EBANER_AUDIO_DUMP_ENGINE` renders the script — off, crank,
  idle, compressor, full song, shut down — and `EBANER_AUDIO_ENGINE` picks which machine,
  because "louder and heavier" is a claim about two sounds and cannot be checked by
  listening to one. The first attempt was 4× and **clipping**; the dump caught both that
  and a fault of its own, where a single-engine locomotive was sounded through two voices
  and came out twice as loud as it should. The railcar keeps its **muffled idle drone**
  (a firing thrum with a soft combustion knock, heavily low-passed for the
  insulated character, each detuned a little further than the last so several of them
  beat rather than doubling into one), following its own rpm and faded by distance to
  the car body it sits in. The synth holds **six** of them, so a three-set train is
  heard as six separate engines and not as one loud one; a voice no engine is driving
  is exactly inert, so the ceiling costs nothing when the train is shorter. While a compressor charges the
  reservoir it adds a higher, muffled **pump hum** and loads its engine down a touch
  (a small, audible idle droop). Under all of it is the **wheel on the rail** — see
  below (`src/Audio.cpp`).

### Rolling noise

The loudest thing a running train makes, and above about 60 km/h it buries the engines.
It is synthesised from what the sim already knows about the contact patch, and every part
of it is measured rather than tuned by ear (`ctest -R rolling-noise`).

**Speed** sets the level by the law measured on real track — 30·log₁₀(v), an amplitude
proportional to v^1.5, so four times the speed is eight times the level. It also raises the
band: the roughness wavelength that dominates passes faster, so the roar climbs from a low
grumble at a crawl to a full rush at line speed. Below walking pace it is gated to exact
silence.

**Weight** — axle load, so a bare wheelset (1.3 t) and a loaded Class 93 (11.7 t/axle) are
worlds apart — makes it **darker as well as louder**. A heavier axle spreads its contact
patch, and a patch that wide cannot be excited by the short-wavelength roughness a small one
can; heavier wheels also radiate lower. Measured, the centroid drops from ~950 Hz to ~690 Hz
and the energy below 250 Hz nearly triples. That is the difference between a clatter and a
rumble, and it is why weight does not just sound like moving the camera closer.

**Friction load** comes from four places, kept apart because each drives a different sound:
traction and the Davis running resistance put a gritty edge on the roar; the **friction
brake** adds a soft low rumble and nothing above it — the Class 93 brakes on discs and there
is very little to hear; and the **lateral load the cant does not take out** drives a
**curve squeal**, a stick-slip howl in the wheel-mode band around 2.1 kHz, gated to curves
tighter than ~300 m so it can never leak onto straight track.

**Wheels over the points.** Continuous welded rail has no joints to beat against, so the
line is smooth and the switches are where the gaps are: every wheel drops across the gap in
the running rail at the frog, and the sim counts one knock per axle per turnout. A bogie
gives its double-thump and a carriage its four-beat, spaced by the vehicle's own axles, and
a station throat rattles because the turnouts really are that close together. Impact noise
grows with speed more slowly than rolling noise does, so the knocks stand out over the points
at yard speed and are half-buried in the roar at line speed.

Off the rails there is none of it: a derailed vehicle is sliding on ballast.

## Requirements

System packages (all found via CMake / pkg-config):

- Vulkan SDK / loader + headers
- GLFW 3
- GLM
- `glslc` (shader compiler, from shaderc)
- PulseAudio (`libpulse-simple`) or PortAudio (**optional** — either enables the
  synthesized brake sound, PulseAudio preferred; the build is silent without both)
- Lua 5.4 (**optional** — enables the dataset's `overlay.lua`; see *Scripting* below.
  Found as `lua5.4` / `lua-5.4` / `lua54` via pkg-config. LuaJIT is deliberately not
  accepted: it would answer to "some Lua" and quietly give 5.1 semantics)
- CMake ≥ 3.20, a C++20 compiler

## Build

```sh
cmake -S . -B build
cmake --build build -j
```

This builds three executables that share the same rendering/loading engine
(`libebaner_engine`): the **`ebaner`** simulator above, **`ebaner-trackedit`**
below, and **`ebaner-dumptrack`**, which answers questions about the network the
way the sim sees it - after `track-edits.txt` is applied, which is the only form
worth deriving anything from. `--near` lists what is around a point, a track id
prints its vertices, `--switches` shows the turnouts and which overrides reached
them, `--route` asks the editor's own search whether a road exists, and `--gaps`
lists the loose ends: endpoints with nothing inside the 1 m the path builder
joins at, each with the nearest other loose end, which is what a `link` edit
would join. A buffer stop is a loose end too, so the distance is the tell - a
real break is two ends facing each other a few tens of metres apart.

### Measuring a train that is moving

Every profile before this one was of a train **standing still**, which is a profile of a
different program: nothing crosses a section boundary, so occupancy never changes, aspects
are never recomputed, the signal mesh is never rebuilt and no crossing steps a phase. All
four of those reported **zero calls a second** while a 600 m freight sat at Rognan. Moving
at 72 km/h past the same crossings the frame was **61–64 ms**, not the 20 that standing
still had suggested.

`EBANER_ROLL=<m/s>` is what makes that measurable without a keyboard: the train spawns at
that speed with the brake released and coasts. 1797 t against Davis resistance rolls a long
way, which is the long window a profile wants.

What the moving profile then said, at Rognan:

| block | mean |
|---|---|
| **vehicle mesh** | **19–21 ms** |
| crossings | 4.9 ms |
| signal mesh rebuild + upload | 3.9 ms |
| sim step | 3.0 ms |
| **occupancy — the track circuits** | **1.6–2.2 ms** |
| `drawFrame` | 1.5 ms |
| HUD text | 1.0 ms |

Two things follow. The **track circuits are not the cost** — threading them could buy 2 ms
of 62 and would leave the interlocking acting on a frame-old world. And the frame is
**CPU-bound**: `drawFrame` is 1.5 ms, and running the same scene in map mode, which skips
every 3-D mesh, saves only ~6 ms.

### Not building what cannot be seen

The vehicle mesh was the whole top of that list, and measuring *inside* it settled how to
cut it: of the per-frame cost, the network walks are **4 %** and emitting the 83 388
vertices is the other 96 %. So the only way to make it cheaper is to emit fewer of them.

`refresh` now takes the view-projection and builds a unit only if its bounding sphere is in
frustum; the rest have their slice of the buffer filled with **one repeated point**, which
leaves every triangle in it degenerate. Nothing is rasterised, the vertex count does not
move, and the index buffer on the GPU stays exactly as valid as it was. The `Frustum` is
the one the terrain and building draws already use, lifted out of `VulkanRenderer.cpp` into
`Frustum.h` rather than copied.

| at Rognan, 600 m freight, **moving** | before | after |
|---|---|---|
| **cab view** (what you drive from) | 23.4 ms (43 fps) | **18.5 ms (54 fps)** |
| of which vehicle mesh | 8.4 ms | **2.0 ms** — 1 unit of 23 built |
| chase view | 23.4 ms | 23.9 ms — nothing to cull, whole train in shot |

Proved rather than eyeballed, because the failure mode is a wagon quietly missing: over
**192 camera positions** round a curved train, near and far, high and low, every vertex the
cull dropped was checked against the view volume — **5 496 156 dropped, 0 of them on
screen**.

One incidental find worth the note: calling `glfwGetFramebufferSize` a second time in the
frame, to get the aspect for that view-projection, cost **~6 ms a frame** on this machine.
It asks the window system and can take a round trip. The numbers were already in hand from
the top of the frame.

### What a crossing asks an axle

A level crossing has to know which of its roads each wheel is on, and it used to find out
by **searching**: walk the approach in 5 m steps calling `poseAt` at each one, from
`s - outerM - 200` to `s + outerM + 200`. With approaches of 968–1473 m that is up to
**670 samples, per axle, per crossing track, per frame**. A cheap straight-line reject
keeps distant crossings free, which is why it never mattered: a 6-axle railcar costs 4 000
samples at the crossing it is standing on and nothing anywhere else.

The 600 m freight has **112 axles**, and every one of them is inside the reject radius of
the crossing it is standing at: **75 000 `poseAt` calls in one frame**, for one crossing.
That is the whole of "it gets terrible near a level crossing with a long train".

The answer was already known and thrown away. Every axle's position comes from `walkTo`,
which produces the path index and the arc position along it — exactly what those 670
samples reconstruct — and `railFrame` kept only the world point. Axles now also come as
**`AxlePlace { pos, path, s }`**, and a crossing answers by subtraction: same path, and
within reach along it. The search stays as the **fallback**, because a track can belong to
more than one `TrackPath` and a train walking a different one over the same rails still has
to be seen; the fast form says whether its answer is *final*, so the fallback runs only for
that case.

The same block walked **all 3388 turnouts** for every axle standing on a crossing, to find
the points between it and the crossing. Turnouts do not move — only their states do — so
that list is built once per path at load.

Measured at Rognan, which has two crossings, with the 600 m freight:

| | before | after |
|---|---|---|
| whole frame | 43.4 ms (23 fps) | **20.6 ms (49 fps)** |
| of which crossings | dominant | **1.8 ms** |
| one railcar, for comparison | 8.4 ms | 7.7 ms |

Checked rather than assumed, because a crossing that stops seeing a train does not fail
loudly — it just fails to close. Running both forms side by side over the real network at
four stations agreed **1 635 000 times out of 1 635 000**, with no answer more than 5 m
apart.

### What a long train costs, and what it need not

A 600 m freight is 23 units and **83 388 vertices** — the longest train before it was the
8-unit night train — and the per-frame path was rebuilding all of it, indices and all, and
then copying 3 MB twice. On an Intel HD 510 that was **8.4 ms of a 20.5 ms frame**.

Three of those things cannot change between frames and are no longer redone. The **index
buffer** is fixed from the moment it is attached (that is why changing a train's
composition has to go through `attachVehicle`), so the 144 000 index pushes were building
exactly the buffer the GPU already held. The **opaque/glass split** is fixed for the same
reason, so `sortGlass`'s colour sort over 48 000 triangles was finding the same answer
every frame; only the `texLayer` stamp has to be put back, and which vertices carry it is
recorded once. And `updateVehicleVertices` **swaps** rather than copies, since the
deferral past the frame fence is what it is for, not the copy.

| | before | after |
|---|---|---|
| 600 m freight, vehicle mesh | 8.37 ms | **6.30 ms** |
| 600 m freight, upload | 0.62 ms | **0.00 ms** |
| 600 m freight, whole frame | 20.51 ms (49 fps) | **17.04 ms (59 fps)** |
| one railcar, vehicle mesh | 3.01 ms | **2.03 ms** |

`refresh()` is checked against `build()` offline rather than by eye: for every vehicle in
the table it reproduces the buffer **bit for bit**, vertices, glazing tags and indices
alike. The failure it guards against is silent — a train drawn from the wrong vertices
looks like a heap of triangles — so it also warns if a refresh ever emits a different
vertex count, which is what a composition change without an `attachVehicle` would do.

What is left of the 6.3 ms is the geometry itself, and the way to cut that is to stop
making geometry nobody can see: the draw has no **culling** at all, so all 23 units are
submitted whether they are in front of the camera or half a kilometre behind it. The
machinery is already here — `drawFrame` builds a `Frustum` and the terrain and building
draws skip chunks with `frustum.sees(centre, radius)` — and the vehicle is the one that
does not use it.

## Test

```sh
ctest --test-dir build --output-on-failure
```

The tests run against a real terrainmapper export, since what the code has to cope
with is decided by what the data is actually like — there is no synthetic dataset to
check against. They look for one at `../norway-rails`; point them elsewhere with
`cmake -S . -B build -DEBANER_TEST_DATASET=/path/to/export`. Without an export they
report as skipped rather than failing, so a build without one is still clean.

- **`track-edit`** — an edit made in the track editor reaches every store that holds
  track geometry: the windowed tiles the terrain carve reads, and the resident network
  the paths, junction graph and turnouts are built from. Editing only one of them fails
  silently — applied to the tiles alone, raising a rail re-cuts the ground and leaves
  the rail where it was.

## Run

```sh
./build/ebaner ../norway-rails            # pick the station on the start screen
./build/ebaner ../norway-rails Fauske     # or name it, and skip straight past
./build/ebaner "" Fauske                  # dataset from $EBANER_DATA or an install
```

### Where to start

Both binaries take an optional station name after the dataset. The stations come from
the export itself — every tile's `meta.json` lists the ones inside it, 720 in all — so
any of them works, not a list kept here. The name is matched ignoring case and the
Norwegian letters, so `Bodo` finds `Bodø` and `oteraga` finds `Oteråga`; an unknown name
prints the near misses rather than guessing. Omitted, both ask on screen before loading
anything (arrows, PgUp/PgDn, Enter; Esc quits).

Starting far afield is free — the scene is built around whichever station you pick.
Driving a long way from it is a different matter, and is what the floating-origin work
still outstanding is for.

On startup the console prints the resolved start point (UTM 33N), the look direction,
tile/triangle counts, and vehicle physics (mass, inertia, tipping limit) — and, first of
all, two `[Paths]` lines saying which dataset and which shaders it settled on, which is
the first question to ask of a build that will not start.

### Where the dataset is looked for

The export is **installed separately**. The tiles run to tens of gigabytes, so they are
in neither this repository nor any package built from it, and nothing here installs
them; this is the reading side only. Named on the command line it is taken as given —
a wrong path is reported as the wrong path rather than quietly replaced. Otherwise the
first of these that holds a `tiles/` directory wins:

| | where |
|---|---|
| 1 | `$EBANER_DATA` |
| 2 | `$XDG_DATA_HOME/ebaner`, or `~/.local/share/ebaner` *(Unix)* |
| 3 | `<directory of the binary>/data` |
| 4 | `<directory of the binary>/../share/ebaner` |
| 5 | each of `$XDG_DATA_DIRS` + `/ebaner`, by default `/usr/local/share` then `/usr/share` *(Unix)* |
| 6 | the `datadir` this build was configured with, `$prefix/share/ebaner` |

**Rows 2 to 6 are each tried twice**: the directory itself, and `norway-rails` inside
it. So `/usr/share/ebaner/norway-rails` is found, which is where a package installs the
export — `share/ebaner` already holds the program's own `shaders/`, and emptying tens of
thousands of terrain tiles in beside them is no way to lay out a prefix. An export
unpacked so that its `tiles/` sit directly in `share/ebaner` is found too; neither
layout has to know about the other.

Rows 3 and 4 are what a Windows build will have, where there is no XDG and the binary's
own directory is the only thing to go on. Found nothing, the program lists every place
it looked and stops.

**No development path is searched and none is compiled in.** Working on the program
means saying where the data is — on the command line, or in `EBANER_DATA`:

```sh
./build/ebaner ../norway-rails Fauske
EBANER_DATA=../norway-rails ./build/ebaner "" Fauske
```

The compiled shaders need no such help and are found the same way: `$EBANER_SHADERS`,
then `<binary>/shaders`, then `<binary>/../share/ebaner/shaders`, then the configured
`datadir`. The second of those is the Windows install layout and the build tree both —
CMake puts the `.spv` files in `build/shaders` beside `build/ebaner` — so nothing has to
know it is running from a build, and no build path is baked into the binary.

"<binary>" is the directory the running executable is in. It comes from the OS where
there is a way to ask (`/proc/self/exe`, `GetModuleFileName`); failing that, from
`argv[0]`, which is used as given when it is a path and looked up on `$PATH` when it is
a bare name — the same search the shell did, against the same `PATH`, finding the same
file.

## Install

Step-by-step instructions for Linux — distribution packages, fetching the dataset and
its overlay, and an unofficial Gentoo overlay — are in [`INSTALL.md`](INSTALL.md).

```sh
cmake --install build --prefix /usr/local     # or DESTDIR=... for packaging
```

A plain GNU layout, through `GNUInstallDirs`, so `-DCMAKE_INSTALL_LIBDIR=...` and the
rest work as expected:

| | |
|---|---|
| `$prefix/bin` | `ebaner`, `ebaner-trackedit`, `ebaner-dumptrack` |
| `$prefix/share/ebaner/shaders` | the compiled `.spv` files |
| `$prefix/share/doc/ebaner` | `README.md`, `LICENSE` |

Put the export at `$prefix/share/ebaner/norway-rails` (beside `shaders/`, not in among
it), or anywhere else in the list above.

### Windows

`.github/workflows/windows.yml` builds x64 executables with MSVC and vcpkg. It is
**dispatched by hand, and runs on a pushed `v*` tag** — nothing else. A Windows runner
bills at twice the Linux rate, and the project is developed and tested here.

It installs the Vulkan SDK from LunarG (headers, `vulkan-1.lib`, and `glslc`), takes
glfw3, glm, lua and portaudio from `vcpkg.json`, builds against the
**`x64-windows-static`** triplet, runs `ctest`, and packs `cmake --install` output into
`ebaner-windows-x64.zip` — the same `bin/` and `share/ebaner/shaders/` layout as above,
which the path search finds with no help. On a tag the zip is attached to the release.

Static on purpose: the zip needs no DLLs beside it and no MSVC redistributable. The one
thing loaded at run time is `vulkan-1.dll`, which comes with the graphics driver and is
not ours to ship. The dataset is still installed separately — put it in `data\` beside
the executables, or point `EBANER_DATA` at it.

Of the 18 tests, 11 run on the runner and 7 report skipped, having found no export.

`vcpkg.json` is inert for a Linux build: manifest mode engages only when the vcpkg
toolchain file is passed, which it never is here.

**Before spending a run on it**, the whole tree can be compiled for Windows locally with
the mingw-w64 cross compiler. It is not MSVC and will not find everything MSVC objects
to, but it catches anything POSIX that has crept in, and it costs nothing:

```sh
mkdir -p /tmp/wininc && ln -sfn /usr/include/{vulkan,vk_video,GLFW,glm} /tmp/wininc/
for f in src/*.cpp tools/*.cpp tests/*.cpp; do
    x86_64-w64-mingw32-g++ -std=c++20 -fsyntax-only -I src -I /tmp/wininc \
        -DEBANER_DATADIR='"C:/x"' "$f" || echo "FAILED: $f"
done
```

All 70 translation units pass it today. `src/Paths.cpp` goes further than that and has
been run: cross-compiled and executed under wine, it resolves its own directory through
`GetModuleFileNameW` and finds both the installed layout (`bin\..\share\ebaner`) and
the portable one (`shaders\` and `data\` beside the executable).

## Controls

For a walk-through of driving a train from cold, see
[Driving the Class 93](docs/driving-class-93.md),
[Driving the Di 4](docs/driving-di4.md) and
[Driving the CD 312](docs/driving-cd312.md).

The driving keys work the cab you are sitting in (`V`), or the front cab in any other
view. What `,` and `.` move depends on the machine: the Class 93 has one combined
power/brake lever, while the Di 4 and CD 312 have a power controller with a separate
train brake (`K`/`L`) and locomotive brake (`H`/`J`).

| Input        | Action                          |
|--------------|---------------------------------|
| W / A / S / D| Move horizontally               |
| Q / E        | Move down / up                  |
| Mouse        | Look                            |
| Left Shift   | Move faster (×8)                |
| C            | Toggle chase camera (the vehicle) |
| V            | Driver's-seat view; press again to switch cab, again to exit |
| I            | Start / stop the diesel engines (all of the set's engines together) |
| Up / Down    | Hand-push the vehicle fwd / back |
| , / .        | Class 93: combined lever toward power / toward brake (N → P1–P5, N → B1–B4 → EMERG). Di 4, CD 312: power controller up / down (P1–P5 above N, the electric brake E1–E5 below it) |
| K / L        | Train brake: release / apply a notch (Di 4, CD 312) |
| H / J        | Locomotive (independent) brake: release / apply (Di 4, CD 312) |
| Space        | Emergency brake                 |
| F / N / R    | Reverser: Forward / Neutral / Reverse. Putting a cab in gear centres every other cab |
| T            | Throw the switch under the crosshair (straight ⇄ diverging) |
| U            | Uncouple the coupler nearest the camera (within 30 m, train stopped): press once to select it, again within 5 s to part it |
| P            | Hand the train to the **auto-driver**, or take it back |
| O            | Open / close the traffic-manager map |
| M            | Mute / unmute sound             |
| Tab          | Release/grab cursor             |
| Esc          | Open menu (Exit); press again to resume. In a picker, backs out one step |

In the traffic-manager map (`O`), the cab keys are replaced by the map's own. See
[Using the traffic manager](docs/traffic-manager.md) for how to work it, and
[Working a TXP station](docs/txp-station.md) for a station worked by hand, or
[without line block](docs/txp-station-without-line-block.md) for one with its own
interlocking but none on the line, and
[Working a remote-controlled station](docs/remote-controlled-station.md) for a stretch
with line block:

| Input        | Action                          |
|--------------|---------------------------------|
| W / A / S / D| Pan                             |
| Mouse wheel, Z / X | Zoom in / out             |
| Left click on a switch | Throw it (motor switches only, and only when its locking allows) |
| Left click on a signal | Arm it, then click a ringed destination to set the route. Click a signal with a route set to cancel it |
| R            | Pick an exit route at the current station |
| E            | Pick an entry signal at the current station |
| N / B, Right / Left | Next / previous station |
| Up / Down, Enter | Move and choose in a picker |
| Q            | In the `R` route list: arm the highlighted route, to set itself once it can |

## Track editor (`ebaner-trackedit`)

```sh
./build/ebaner-trackedit ../norway-rails          # or: ... ../norway-rails Rognan
```

The editor shows the surveyed track points over the ghosted scene. In it you fix the
geometry, draw sidings, and lay out track circuits, main, block and simple signals,
level crossings, flag posts and TXP positions. Everything is saved to the dataset's
`overlay/` directory, which the simulator applies at load. See
[The track editor](docs/track-editor.md).

## Scripting

Everything in `overlay/` so far is a *fact*: a crossing is a line in a file, a TXP position
is a line in a file. Some of what a railway needs is not a fact but a **rule** — a
timetable, a train that runs itself, when a station is worked — and there was nowhere to
write a rule down. `<dataset>/overlay/overlay.lua` is that place. The simulator runs it once at
startup, after the world, the paths, the switches and every other overlay are loaded, so
that when it is eventually given an API there is already something for it to look at.

```lua
print("God dag frå Nordlandsbanen!")
```

| | |
|---|---|
| no file | nothing said, like every other overlay |
| ran | `[Script] ran overlay.lua` |
| it failed | `[Script] overlay.lua: <the Lua message>`, and the simulator carries on |
| file present, built without Lua | `script: overlay.lua present but built without Lua; not run` |

A bad script does not stop the simulator: it is authored content, and authored content that
does not resolve is reported and stepped over. But a script that is *never run* is told
about — that is the one place this differs from the optional audio, where silence is a fair
thing to leave unsaid.

The interpreter **stays open** after the chunk returns, so what a script defines is still
defined afterwards. That is what makes hooks possible later without a rewrite. The full
standard library is loaded, `io` and `os` included: the script is in the reader's own
dataset and written by them, so a sandbox would keep nobody out and would rule out the file
work a scripted authoring task wants to do.

Nothing of the simulator is exposed yet — no trains, no signals, no stations, no TXP
network, and no `init`/`tick`. Those want designing against a real first script. The editor
does not run scripts; the host is in the engine library so it can be given them later.

## Data format

See `../terrainmapper/doc/game-export-format.md`. In short: 256×256 little-endian
`float32` heightmaps (`terrain.hm32`, row 0 = north), EPSG:25833 (UTM 33N) metres,
tiled across four fully-overlapping LOD levels. World coordinates are rendered
relative to the start point to preserve float precision.

Per tile the simulator also reads, when present:

- `landcover.u8` — 256×256 uint8 AR50 `artype` codes; the terrain is textured by
  land type (procedural per-class surfaces in a Vulkan texture array, sampled in
  world space — `src/Textures.cpp`, `shaders/terrain.frag`). Requires exporting
  terrainmapper **with an AR50 dataset loaded**.
- `tracks.bin` — railway polylines (deduped by `trackId`), including a per-vertex
  OSM **speed limit** used for banking.
- `roads.bin` — road polylines (deduped by geometry), category + number.
- `buildings.bin` — OSM building footprints with kind, roof shape, base
  elevation and height.
- `platforms.bin` — OSM station-platform footprints with base elevation and slab
  height.

The track/road/building geometry appears only when the export was produced with
the corresponding sources (national rail register + NVDB roads + OSM enrichment).

## Debug environment variables

| Variable            | Effect                                                        |
|---------------------|--------------------------------------------------------------|
| `EBANER_DATA`       | The terrainmapper export to use, ahead of every other place searched (a path given on the command line still wins). |
| `EBANER_SHADERS`    | The directory holding the compiled `.spv` files, ahead of every other place searched. |
| `EBANER_SCREENSHOT` | Render ~20 frames, write that frame to the given PPM, exit.   |
| `EBANER_CAM`        | Scripted camera `"x,y,z,yawDeg,pitchDeg"` (scene-relative m). |
| `EBANER_NOSTITCH`   | Skip the seam-stitching pass (to inspect raw tile seams).     |
| `EBANER_NOCARVE`    | Skip carving railway cuttings into the terrain.               |
| `EBANER_NOOVERLAY`  | Ignore the `overlay/` track edits (link fixes).               |
| `EBANER_EDMODE`     | `ebaner-trackedit` only: start in this mode, by its menu name. |
| `EBANER_VEHICLE`    | Skip the start screen and preselect a vehicle, by its index in `kVehicleSpecs` (`0` = a single Class 93, `1`/`2` = two and three coupled, `3` = Di 4 + 5, `4` = the night train, `5` = the 600 m freight, `6` = a light Di 4, `7` = a CD 312, `14` = a pocket wagon, `15` = a container flat). The table is ordered for the pickers, so these move when a vehicle is added - the start screen numbers the list. |
| `EBANER_PROFILE`    | Print per-frame timings once a second: whole frame, the sim step, occupancy, crossings, signal aspects, distant walks, the signal mesh, and the vehicle mesh and its upload. With `EBANER_SCREENSHOT=/dev/null EBANER_SHOTFRAME=900` the simulator exits by itself with the reports flushed, which is how a profile gets taken without a file or a keypress. |
| `EBANER_RADIUS`     | Half-window of world to load, in metres (default 20000). Two kilometres loads in seconds where the full 40 km square takes minutes, which is what makes a profile or a frame-time measurement affordable at all. Inspection only: drive out of it and the world ends. |
| `EBANER_ROLL`       | Spawn the train at this speed in m/s with the brake released, so it coasts. A profile of a standing train is a profile of a different program - nothing changes occupancy, so nothing downstream of it runs. |
| `EBANER_GREEN`      | `"OTR 1"` — clear one simple entry signal by name before the first frame, manning its station. The other half of setting up a crossing without a keyboard: with every signal at danger a train stops at the first one, and nothing that happens inside a station could be watched. |
| `EBANER_THROW`      | `"x,y;x,y"` — throw the turnout nearest each world point to diverging before the first frame. A crossing move is a train taking the loop rather than the main, and until this there was no way to set one up without a keyboard, so every bug in what the driver makes of a loop had to be found by hand. |
| `EBANER_AUTO`       | Hand the train to the auto-driver at startup and log what it is doing once a second. A train that drives a real line with no keyboard, which is what makes the signalling and the crossings testable without a driver sitting in the cab. |
| `EBANER_CHASE`      | `1` rides the vehicle from the start (the chase camera is otherwise only reachable by pressing C, so the one view that shows the machine could not be screenshotted). |
| `EBANER_AUDIO_DUMP` | Render a scripted brake sequence to the given WAV and exit.   |
| `EBANER_AUDIO_DUMP_ENGINE` | Render an engine start/idle/stop to the given WAV, exit. |
| `EBANER_AUDIO_DUMP_CROSSING` | Render a crossing bell activating/falling silent, exit. |
| `EBANER_AUDIO_DUMP_ROLLING` | Render rolling noise sweeping speed, weight, curve and brake, exit. |

## Not yet implemented

A per-engine/per-bogie driveline split, hydrodynamic (retarder) braking through the
converter, wheelslip/slip-control, a detented reverser gate / key interlock (no
reversing above zero speed), brake-pipe/triple-valve propagation and
multi-unit consist braking, per-wheel grip-vs-slip and overturn at speed,
terrain-grounded derailment, hauled carriage stock,
and streamed/dynamic tile loading.

## License

ebaner is licensed under the **GNU General Public License, version 3 (GPLv3)** —
see [`LICENSE`](LICENSE).

Copyright © Jan-Espen Oversand &lt;sigsegv@radiotube.org&gt;.

## Contributing

Contributions are welcome under the terms in [`CONTRIBUTING.md`](CONTRIBUTING.md).
Note that, in addition to the GPLv3, contributors grant the maintainer
(Jan-Espen Oversand) the right to relicense the project and to offer it under
additional licenses.
