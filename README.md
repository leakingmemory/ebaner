# ebaner

A small Vulkan/C++ viewer for the `terrainmapper` game-export data
(`../norway-rails`). It stitches the elevation tiles around a **chosen station** into
a continuous 3D surface and draws the railway, roads and buildings on top, with a
first physics-driven rail vehicle. The ground streams in as you travel. The camera
starts on the running line at that station — at a terminus like Bodø that is the
buffer-stop end of track 1, resolved from the track geometry rather than named here.

## What it renders

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
  and the bare test shapes last under **debugging** where they are out of the
  way but still reachable. `kVehicleSpecs` is stored in that order and must stay
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
  **diesel-hydraulic transmission** (per the real Di 93): a **torque converter** for
  launch (torque multiplication while it slips, so the revs flare then couple) feeding
  a **5-speed automatic gearbox** that shifts on road speed (the engine steps down at
  each upshift); tractive effort is the least of the geared/converter limit, the
  constant-power hyperbola and the adhesion cap, so it launches hard then tapers toward
  a level top speed. The gearbox is modelled but not shown — only speed and rpm are on
  the HUD. Each cab has its **own** power/brake lever and reverser, operating
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
  It looks nothing like the Di 4 and the difference is one line: **this nose has no chin**.
  The front is close to vertical with a single slight rake all the way up. Grey flanks with
  a big **louvred radiator panel** whose slats run *across*, a **golden-yellow band at
  solebar level** running the whole length and carrying on round the nose, black underframe,
  fuel tank and lower front, red markers high at the nose corners over headlights low on the
  black, and a **yellow snowplough** reaching down to 15 cm over the railhead. Taken off a
  photograph of **0312 002-7**; the colours are sampled from it rather than guessed.
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
  **0.65 s** behind it — 231 m/s down 150 m of train.
  Volume does dominate, but on the **release**, where there is no accelerator to help
  because nothing local can make air: every pipe has to be filled from the one main
  reservoir at the front, and the train takes **12 s** against the light engine's 3.4 s.
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
  — which has none — is untouched, and its dump is **bit-identical** to the reference. Measured, not guessed: `EBANER_AUDIO_DUMP_ENGINE` renders the script — off, crank,
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
(`libebaner_engine`): the **`ebaner`** viewer above, **`ebaner-trackedit`**
below, and **`ebaner-dumptrack`**, which answers questions about the network the
way the sim sees it - after `track-edits.txt` is applied, which is the only form
worth deriving anything from. `--near` lists what is around a point, a track id
prints its vertices, `--switches` shows the turnouts and which overrides reached
them, `--route` asks the editor's own search whether a road exists, and `--gaps`
lists the loose ends: endpoints with nothing inside the 1 m the path builder
joins at, each with the nearest other loose end, which is what a `link` edit
would join. A buffer stop is a loose end too, so the distance is the tell - a
real break is two ends facing each other a few tens of metres apart.

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

The dataset path defaults to `../norway-rails` if omitted. On startup the console
prints the resolved start point (UTM 33N), the look direction, tile/triangle
counts, and vehicle physics (mass, inertia, tipping limit).

## Controls

| Input        | Action                          |
|--------------|---------------------------------|
| W / A / S / D| Move horizontally               |
| Q / E        | Move down / up                  |
| Mouse        | Look                            |
| Left Shift   | Move faster (×8)                |
| C            | Toggle chase camera (the vehicle) |
| V            | Driver's-seat view; press again to switch cab, again to exit |
| I            | Start / stop the diesel engines (both together) |
| Up / Down    | Hand-push the vehicle fwd / back |
| , / .        | Combined lever: step toward power / toward brake (the viewed cab) |
| Space        | Emergency brake                 |
| F / N / R    | Reverser: Forward / Neutral / Reverse (the viewed cab) |
| T            | Throw the switch under the crosshair (straight ⇄ diverging) |
| M            | Mute / unmute sound             |
| Tab          | Release/grab cursor             |
| Esc          | Open menu (Exit); press again to resume |

## Track editor (`ebaner-trackedit`)

```sh
./build/ebaner-trackedit ../norway-rails          # or: ... ../norway-rails Rognan
```

A **WYSIWYG track-network editor** that reuses the same engine: it renders the
identical scene (terrain, roads, buildings, rails) and, on top, overlays the **raw
rail geo-points** as round markers and the **links between consecutive points** of
each track as lines — the surveyed geometry behind the smoothed rails. Markers/links
are coloured by track type (**amber** main line, **cyan** siding, **magenta** yard),
and **dead ends** (loose ends of a broken link) are marked in **red**. The terrain,
rails and buildings are drawn **ghosted** (half-transparent, editor only) and the
geo-point network is drawn **on top** (x-ray, sized so near points read clearly), so
no point is buried in the terrain or rails. Walk around with the same free-fly
controls (W/A/S/D, Q/E, mouse, Shift, Tab, Esc); a HUD shows the track / geo-point /
dead-end counts and the camera position.

**Editing model.** Edits **preview immediately** in the editor (applied to the
in-memory geometry the same way the viewer applies them at load) and are written to a
**drop-in overlay**, `<dataset>/overlay/track-edits.txt`, only when you press
**Ctrl+S** — a separate directory the generator never touches, so regenerated base
tiles can be dropped in without losing manual edits. On the next load, `TerrainData`
applies the overlay over the generated tiles for **both** the viewer and the editor.
(`EBANER_NOOVERLAY=1` ignores the overlay; unsaved edits are discarded on quit — the
HUD shows the unsaved count.)

**Selecting points.** Press **Tab** to free the cursor, then **left-click** a
geo-point to select it (it turns **white**); **Ctrl+click** toggles points for a
multi-selection; clicking empty space clears it. Picking is screen-space from the
cursor. The **elevation** of the point under the cursor is labelled beside it, and the
HUD shows the selected point's elevation (or the `min..max` range for a group).

**Selecting a stretch (`Left` / `Right`).** Clicking every point of a kilometre of
line is not on, so the arrows run the selection along it: **Right** extends it forward,
**Left** backward, one point at a time, **Shift** ten, **Ctrl** fifty (auto-repeating
while held). It follows the route rather than one track — the main line through a
station is several tracks end to end — and at a junction it takes the straightest
continuation, the same rule the path builder chains by, so it does not wander off down
a siding. The HUD shows how many points and how many metres are selected.

**Raising / lowering.** With points selected, **Up** / **Down** nudge their elevation
by 0.1 m; **Shift** makes it 1 m and **Ctrl** 10 m, for dropping a long stretch onto
the terrain rather than fixing a stray point. All auto-repeat while held, the coarser
steps more slowly. Every selected point moves by the *same* amount, so the grade
through the selection is preserved — it is a shift, not a flattening (for that, see
`G` below). Each is an `elev` override, previewed live and saved with Ctrl+S.

**Straightening a grade (`G`).** Where a track profile has a vertical bump that
shouldn't be there, select points on that track (at least the two ends of the run)
and press **G**: every point from the first to the last selected is snapped onto a
straight, **endpoint-anchored** grade between them (a `elev x y z` overlay line per
point — only the elevation changes). The bump flattens for the rails, the smoothed
path the train rides, and the terrain carve.

**Connecting a siding to the track it crosses.** Sidings often overshoot the track
they should join — the end pokes past it with a red dead-end on the far side. Select
that dead-end and press **J**: its end snaps onto the nearest track its trajectory
crosses (a `move` overlay edit), trimming the overshoot to a clean turnout point; once
the end lies on the track it's no longer flagged red. (Only the geometry is moved — the
crossed track isn't split, so the vehicle's through-routes are unaffected.)

**Adding a connecting rail (building a switch).** The export sometimes omits the
short connecting rails at a crossover or slip, so two tracks that should be joined
by switches only cross. Select two geo-points on **different** tracks (click one,
Ctrl+click the other) and press **R**: a straight connecting rail is added between
them (a `rail …` overlay edit). Its ends sit on the two tracks, so the switch
detection makes a **switch at each end** — a train can then divert across it (main ⇄
siding). Keep the meeting angle shallow, as a real turnout does; too steep and it is
treated as a crossing rather than a switch. Live preview; Ctrl+S saves; the switch
stands appear on reload / in the viewer.

**Scissors crossover (`C`).** For two roughly-parallel tracks (e.g. the main and a
loop), select one geo-point on **each** track, opposite each other, and press **C**: it
lays a **double crossover** — two short diagonal rails that cross in the middle (the
diamond), a switch on each track at each end — so trains can cross between the two lines
either way as well as run straight, like a station-throat scissors. Pick a spot where
both tracks are at grade (it uses the overlay-applied elevations). Live preview; Ctrl+S
saves; switch stands appear on reload / in the viewer.

**Auto-slip at a diamond (`K`).** For an actual diamond crossing (two tracks crossing at
a clear angle), select a single geo-point in the **middle of the crossing** and press
**K**: it finds the two crossing tracks and adds the two diagonal connecting rails (a
slip), a switch on each track at each end. Uses the overlay-applied elevations; skipped
if the crossing is nearly parallel (use `C`) or too steep to be a switch.

**Fixing broken links.** Some exports leave a line disconnected across a gap (e.g. a
tunnel approach), which derails the train at the loose end. Aim the centre crosshair
at a red dead-end and **Enter** to pick end **A**, aim at the other loose end and
**Enter** again for **B**, then **L** to link them (a `link …` overlay edit; **X**
clears the pick). The two segments join into one continuous route and the train runs
across the former gap.

**Drawing a new siding.** The export carries the roads that were surveyed, and a station
can want one that was not — Mo i Rana's industrial trackage is not in it at all. Pick **New
sidings** from the Esc menu and **click a spot on an existing track**: that fixes where the
road starts, and its rail-head height becomes the height of the whole thing. Then **click on**
to lay it out point by point. A click that lands on another track **finishes** the road
there, taking that track's height, so a switch forms at that end too; **Enter** finishes it
where it stands, leaving a buffer stop. **X** takes back the last point (and on the first,
gives up on the road); **Y** makes it a **yard** track rather than a siding; **right-click**
selects one already drawn, for **Y** or **X**. Ctrl+S writes them as `track …` lines in the
same `overlay/track-edits.txt` as every other track edit.

Only x and y come from a click, because a click is a **ray**: it meets one horizontal plane
in exactly one point, and nothing else about it says how far away the ground is. So the road
is drawn flat, at the height it started from, and the elevation is geometry mode's job
afterwards — select it and use Up/Down or **G**, exactly as for any other track. (The HUD
shows the ground height under the cursor while drawing, so how far the road is running off
the hillside is visible as it is laid. A camera looking level has no answer to give and the
mode says so rather than placing a point on the horizon.)

A drawn road is an **ordinary track** everywhere: it takes a `trackId` of its own, written
into the record so the circuits, signal paths and crossings anchored to it by `<trackHex>:
<frac>` stay put whatever else is added to the file. Its turnout has to earn itself the way
any other does — between **8° and 35°** of divergence — so the HUD shows the angle the road
leaves its parent track at, and says when it is outside that window and no switch will form.

**Moving track points.** Drawing puts the points where you clicked; fixing the alignment
afterwards is a separate mode, **Move track points**. Click a point to pick it up and drag
it, or nudge it with the **arrow keys** — 0.1 m a tap, **Shift** 1 m, **Ctrl** 10 m, and
they move it the way you are looking rather than along world north and east, so a nudge goes
where it looked like it would. The height never changes: the point rides its own horizontal
plane, and elevation stays geometry mode's job.

It moves **drawn roads and exported track alike**, because a bump in the ride is usually one
surveyed vertex a little out of line with the two beside it, and there is no way to take that
out by changing heights. The two are carried differently. A drawn road *is* its `track`
record, so moving one of its points is an edit to that record. An exported track has no
record — it comes off the tiles — so the correction travels as a **`move`** line naming the
vertex by where it stands and saying where it should be. The export is never touched.

```
move <oldX> <oldY> <oldZ> <newX> <newY> <newZ> [<trackId>]
```

The old position is how the vertex is found, so it carries a height as well as a place: two
vertices of one track can sit at one spot, and only the height tells them apart — the same
problem `elev` needs its `fromz` for. The optional track id says *which* track's vertex is
meant where several meet, in decimal, as an `elev` names its track.

`elev` and `move` are applied **in the order they are written**, not in two passes by kind.
Each line names its vertex by where everything above it left the thing, which is also what
the editor sees on screen when it writes one — so a regrade after a move finds the moved
point, and a move after a regrade finds the regraded height. Two passes can only ever satisfy
one of those.

For a surveyed point the HUD reports the thing being judged: how far the two neighbouring
vertices are, and **how far off the straight line between them** this one sits. That offset
is the bump, and it is not readable from the ride at any speed. The neighbours are looked for
across a track boundary as well as along the track, because a boundary is exactly where the
bumps are.

Where two tracks meet end to end the export holds the meeting point **twice**, once on each
track, at identical coordinates. It is one point of the railway, so the whole group moves
together — one `move` line per track. Moving a single copy would pull the two ends apart and
break the chain, quietly, with nothing on screen to show for it; the HUD says how many tracks
share the point before you move it.

Let go of an **end** within 2 m of an existing track and it snaps onto it, taking that
track's height so the two sit flush and the switch is certain rather than nearly. Two metres
because that is exactly the touch tolerance the switch detector uses — measured, an end 2 m
off the line still makes its switch and one 2.2 m off does not.

The HUD reports what **both** ends are standing on and at what angle, whichever point is
being moved, because moving the *second* point of a leg swings the angle at the first one
just as surely as moving the end does — and a switch quietly lost that way has nothing else
on screen to show for it.

**A turnout that is not really there (`noswitch`).** Turnout detection is geometry: a track
end standing on another track's line is a switch. Where several ends meet at *one* point that
reads every end as a branch off the others, so a single set of points can come out as two
turnouts — two stands drawn on top of each other, two independent states standing for one pair
of blades, and a branch id that may name the very road the through route runs on. When it does,
the interlocking asks that switch to be *diverging* for a route that in fact runs straight over
it, and throws it the wrong way.

No reading of the geometry settles which of the two was meant, so the site says. In Switches
mode, select the wrong stand and press **X**: it writes a `noswitch` line to
`overlay/switch-types.txt` and the turnout is gone from the next build.

```
noswitch <x> <y> <radius> [<trackIdHex>] [all]
```

With a track id it drops only the turnout whose branch is that road — the surgical form, and
what a doubled set of points wants. Without one it clears every turnout in the radius, which is
the "inhibit this area and lay my own track through it" case; there roads drawn in the editor
keep their switches unless the line ends in `all`, since otherwise the replacement track's own
turnouts would go with the ones being replaced.

It lives in `switch-types.txt` because it is switch authoring like the motor/manual overrides
— and because that file is rewritten whole on save, so anywhere else in it a `noswitch` line
would be dropped the next time a switch type was saved. The writer carries them back out.

**Simple entry signals.** The entry signals proper are routes between track-circuit
borders, with C1/C2 authorities and full interlocking — which needs circuits drawn
through the station, so they exist at Bodø and nowhere else. These are the plain
alternative: a short head with two steady lamps, red over green, and no circuits or
routes behind them. Pick **Simple entry signals** from the
Esc menu, then **click any point on any track** to place one (as with a distant signal —
no border needed), **right-click** to select, **F** to turn it round, **F2** to name,
**X** to delete, **Ctrl+S** to save to `overlay/simple-entry-signals.txt`.

Each attaches to the **nearest station** in the export, and the station is what they are
interlocked by. A station is either **off** — unmanned, its signals dark, trains running
through without reference to them — or **on**, when they show red and **one of them may
be cleared at a time**. A typical station has two, one at each end, and clearing one puts
the other back to red. Stations start off, so placing signals on a line does not stop the
traffic already running over it; switching a station off again clears any green with it,
so it comes back on in a known state.

The editor HUD names the station a selected signal attached to and how far off it is,
flagging anything beyond 4 km as suspect — that is almost certainly not the station it
serves. An explicit station name can be written into the overlay line to overrule the
nearest-station rule where two stations sit close together.

In the viewer they are worked from the traffic manager (`O`), where **E** opens the
station being worked: the first line switches the station on or off, then **All red**,
then its signals. A signal holds whatever it was last given — nothing else resets it, no
train passing and no timer — which is why the state of each is on the line beside it.

**Which station** the traffic manager is working is named in its title, and everything it
does — the entry-signal panel, the flag and TXP lines under it, and the exit routes `R`
offers — acts on that one station. Opening the map lands on the station nearest the train,
which is the one meant nine times out of ten; **N** and **B** then walk to the next and
previous station along the line, bringing the view with them. The list is the stations of
the line the run started on, in the order they come in, whether or not anything is
authored at them yet — that is what makes it possible to go and author something at the
next one. `EBANER_MAP=1` opens the traffic manager straight away, and `EBANER_MAP=<name>`
opens it at a named station, which is how it is screenshotted headlessly.

**Which station works a route** is normally worked out from where its in-station end lies:
routes whose ends sit within 800 m of one another are one place. That is right until a route
runs somewhere far from the platforms — a branch out to an industrial siding a couple of
kilometres beyond the station. Those cluster as a place of their own, and since the picker
offers *the cluster nearest the station being worked*, a cluster with no station near it can
never be offered at all: the route is built, has a mast, and is simply unreachable from the
panel. Mo i Rana's two mining routes were exactly that — one of them sharing a mast with a
route that **was** offered.

So a route may name its station outright, in any of the four route files:

```
entry 16 "NO MO INFN MINING" 63e:0.826377 d0000000:0.097737 type C2 station "Mo i Rana" …
```

It overrules the geometry: the route joins whichever cluster the panel would offer when that
station is worked, so naming a station and walking to it come to the same thing. A movement
has two halves and either may carry it — the approach or the signal, the exit route or the
exit signal — and if both name one they must agree (the approach wins, and it says so). A
name the export does not have falls back to the geometry with a warning.

**Setting a route** (`R`) asks first **which way**: into the station or out of it, with the
count of each, and then offers that list. A worked station has both — Mo i Rana has 7 in and
6 out — and one flat list of them was both long to work and easy to misread, since "NO MO NB
T2" leaves the station and "NO MO INFN T2" comes into it and they sat next to each other.
The kind stays in the title of the list, so what is being chosen is on screen while it is
chosen. **Esc** backs out one step, then closes.

The panel behind all of this can no longer overflow. Every length in it scales with the
framebuffer, so the number of rows that fit is a constant — **15** — and not something a
bigger screen buys more of; past that it simply grew off the top and bottom of the screen,
taking the first and last items with it and saying nothing. A longer list is now windowed
around the selection with a count of what is out of sight at each end. That is why the
start-up station picker no longer needs the window it used to keep by hand for its 687
stations, and why no picker in the traffic manager can run off the screen again.

Setting a route asks only that the road **beyond the signal** is clear. The
run-up to it — the platform road the departure starts from — is where the train being
cleared is standing, so a train there is the ordinary case and not a reason to refuse. It
is the same division the release makes at the other end: only a circuit beyond the signal
puts it back to danger, because running up to a signal must not cancel the authority the
driver is about to act on. An entry signal's authority begins at its own mast, so all of
its circuits are beyond it and the whole route must be clear — a train may not be let in to
an occupied platform.

**An approach in front of an entry signal.** An exit signal is authored in two parts: the
signal and the road beyond it in `overlay/exit-signals.txt`, and the roads leading **up to**
it in `overlay/exit-routes.txt`, many-to-one, one line in the picker each. An entry signal
needs the same where several roads converge on one mast, and `overlay/entry-approaches.txt`
is where those are written. In the editor: right-click an entry route, press **B**, click
the start border out on the approach — the gesture the exit routes already use.

An approach names no signal. An exit route can, because an exit signal is a single record;
an entry mast is several records sharing a start border, so there is no id to name — the
approach ends **on** the mast's border, facing the way the mast faces, and that is what says
which mast it belongs to. It carries no C1/C2 either: what the driver is authorised to do is
settled by where they are being let in to, so the entry record decides and the approach
only says which road they came in on. The picker then offers one line per pair, named
`approach > destination`.

An entry signal with **no** approach is unchanged, which is nearly all of them: its
authority begins at the mast, its record is the whole movement, and nothing about it moves.
Where there is one, the approach's circuits are locked with the route and its points are
held — and, as at an exit signal, a train standing on it is the train being let in rather
than one in the way.

Two routes may **share road**, which is what lets a train be passed **through** a station
rather than stopped in it: set the entry route into a platform and the departure out of it
and the driver gets a green at the entry signal and another at the exit. What is refused is
a circuit lying beyond **both** signals — each would have authorised a movement onto the
same rails — or a route running the **other way** over rails we share, which is two
movements facing each other. Neither test cares which of the two was set first.

**A distant on a main signal's own mast.** A distant signal (*forsignal*) normally stands
out on the line on its own post, repeating what the first main signal ahead is showing. At
a short station it hangs instead on the entry signal's mast, on an adapter that carries it
**0.20 m in front of the main head and 0.40 m below it** — one pole, two separate signals.
An entry signal built that way repeats the exit signal for whichever road the points are
set for, so a driver reads both the authority to enter and what waits at the far end.

Select a main signal in **Entry signals** or **Exit signals** mode and press **F**. It is a
fact about the *mast*, not about one road into the station: several route records sharing a
start border are one signal, so the flag goes on all of them at once, written as a `distant`
keyword on each line of `overlay/entry-signals.txt` or `overlay/exit-signals.txt`.

**A two-lamp head for the siding roads.** The main signal's head carries three lamps — top
green, middle red, bottom green — and says all three things it can: stop, proceed over a
deviation (C2, the upper green alone), proceed with no restriction (C1, both greens). A
siding with its own signal does not get one. Its mast carries **two lamps, red over
green**: stop, or go.

Press **2** on a selected signal, in either mode; it is written as a `twolamp` keyword and
set across the mast exactly as `distant` is. Everything else about the signal is unchanged —
same mast, same routes, same interlocking, and it may still share a pole with a dwarf or
carry a distant.

It does not touch what a route authorises. A C1 route over such a mast stays C1 wherever it
is named; the head simply lights its one green for **any** clearance, C1 and C2 alike,
because it has no second lamp to tell them apart. You would not normally put one where a C1
route runs, but nothing stops you.

What it shows is the ordinary distant rule — walk forward along the road the points are
actually set for and repeat the first main signal facing the same way — with one addition:
**it is switched off entirely while the main under it is at danger.** There is nothing to
warn about ahead of a signal you have to stop at, and a lit repeat under a red would be
read as permission the main is not giving. It never reads the signal it hangs on, and where
there is no main signal ahead within reach it warns of a stop, which is what a station with
no exit signal means for a train being let into it.

**Block signals (*blokksignal*).** A main signal standing out on the plain line between two
stations, worked by nobody. Majavatn to Svenningdal is 31 km — the longest gap on this line
— and one train at a time over that is a waste of it, so a track-circuit border cuts the
line in two and a block signal stands at that border facing each way. A train may then
follow another onto the line, held at the block signal until the one in front is clear of
the section beyond it.

It carries **no route of its own**. The road it governs is the signal path already authored
from its border facing its way, and that same road is what it opens before it clears — one
description of the road rather than two that can drift apart. Author it with **Block
signals** from the Esc menu and **click a border** (not a free point: away from one there is
nothing for it to divide); **right-click** selects, **F** turns it round — which picks the
*other* of the border's two roads, so it is not cosmetic — **B** walks the post across the
line, **F2** names, **X** deletes, **Ctrl+S** saves to `overlay/block-signals.txt`.

```
block <id> "<name>" <trackHex>:<frac> <+|-> [left]
```

Its head is an exit signal's — three lamps, solid greens — but **the red flashes**, as an
entry signal's does. The two masts at one border are normally put back to back on the same
side of the line, one cable run serving both, and each stands **0.5 m back down its own
approach** so the pair is two poles a metre apart with the joint between them rather than
one pole drawn twice in the same place. Back rather than forward, because a head looks down
its own approach: stepped forward the two would stand nose to nose, each in the other's
sightline with its back to the train it governs.

A signal that resolves to no road draws orange and says so in the HUD: it is not an error
that stops anything, it simply stands at danger for ever, which is the safe way to fail.
The line block itself is not authored at all — it is what the roads already say. The
section ahead of the northbound signal plus the section ahead of the southbound one is the
whole of the plain line, and the far ends of those two roads are the borders where the
stations' own signals stand.

**The line takes a direction.** Two trains cleared toward each other from opposite ends do
not collide — the block signals stop them — but they come to a stand nose to nose with no
way out, and normal working should not be able to reach that state at all. So the first
departure onto the line **claims** it, and a departure the other way is refused for as long
as the claim stands. A second train the *same* way is not refused: that is the following
move the block signal exists to protect.

The claim outlives the route that made it, and it has to. A route is given up as its last
circuit is entered, and by then the train is out on the line with nothing else on record
saying which way it went — so the claim is released only when the line is genuinely clear:
no route into it, and no circuit on it occupied. A train that reached the line without a
route at all has no direction on record, and nothing is let out against it.

Nothing about a block signal appears in the traffic manager. There is no hold, no override
and no way to set its road by hand — the map refuses it, and the dwarf that would otherwise
stand on the same border is taken away, so there is nothing there to click. A distant reads
one exactly as it reads a station's signals; that costs no code at all, because the distant
walk counts everything that is not a dwarf or another distant.

**Level crossings.** Secured by lights, and optionally by half-barriers too. Pick **Level
crossings** from the Esc menu and **click a point on any track**; **right-click** selects,
**T** then a click adds another track the same crossing spans, **B** toggles the variant
between lights alone and lights with barriers, **F2** names, **X** deletes, **Ctrl+S** saves
to `overlay/level-crossings.txt`. There is no flip: a crossing faces both ways.

**A crossing inside a station spans both roads**, and that is one crossing rather than two:
one road shut, one bell, one pair of booms. What it is *not* is one set of circuits. Each
track it spans has its own three, read independently, so a train on one road runs the
sequence while the other road is left alone — and two trains, one on each, are each seen
for themselves. Each track gets its own pair of train heads, so "which road is this warning
about" has an answer.

**A train is on one road**, and out on the approach it is on neither. Which road is decided
in two steps, because geometry can only answer the first.

*Where it is*: the nearest road takes it, and no other. Asking each road "is this train
near enough to be mine" gets this wrong, because the two roads of a station converge at
their turnouts and run about a metre apart there — both say yes, and a departure on the
main line arms the loop's circuits as it passes the points.

*Where it is going*: out beyond the turnouts the roads have not divided, so a train there is
on the rails that lead to both, and which road it is **for** is a question about the points.
A facing turnout between it and the crossing, set to the other road, is what settles it.
Arm the road it will not take and the sequence runs on the wrong side — that road's heads
clear to white while the train passes the other's at red. Between the turnouts nothing is in
the way and the geometry stands.

The pulse belongs to the crossing and the lamps to the track. Either track arming puts
**every** head on the crossing onto the fast flash, but only the track the train is on goes
white: a track standing idle beside one that is closing shows **red on the fast pulse**. The
approach and repeat distances are the crossing's too, derived once from the **fastest** track
it spans — the warning time a crossing needs is set by the fastest train that can reach it,
and two roads arming one crossing at two distances would be two crossings sharing a road.

Four heads, each **red over white** and flashing: two facing the train, one each way along
the track, and two facing the road across it. Three detection circuits come with the
crossing rather than being drawn — an **inner** one over the crossing and out past the
train signals either side, and an **approach** circuit each way. The approach distance is
derived from the line speed there (braking distance plus the time the sequence needs), so
a 130 km/h main line arms about 1.5 km out and a 40 km/h branch a couple of hundred
metres; the editor HUD shows what it worked out, and a distance written into the overlay
line overrules it.

The sequence, with the road's lights first and the train's second:

| | road | train |
|---|---|---|
| **idle** | white, slow | red, slow |
| **closing** (5 s) | red, fast | red, fast |
| **secured** | red, fast | white, fast |
| **opening** (5 s) | red, fast | red, fast |

An approach circuit arms it on the **edge** — clear to occupied — and only from idle,
which is what stops a departing train re-arming the crossing from the far circuit. The
inner circuit arms it whenever it is occupied, no edge and no gate, because it is the
fallback for an approach circuit that has failed. The release runs on the inner circuit
clearing and on nothing else, and mirrors the delay: the train's signal drops to red at
once, the road opens 5 s later. A crossing armed by a movement that then turned back
releases itself after a minute of everything being clear.

**A signal at danger breaks the approach circuit.** Nothing beyond one can reach the
crossing without first passing it, so the circuit is cut there and sees nothing further out.
Inside a station that is the difference between a crossing that shuts for the traffic
actually coming and one that shuts for a train standing at a red signal for as long as it
stands there. When the signal clears the circuit is whole again, and the train arms the
crossing as it comes on — the clear-to-occupied edge the gate is looking for.

Only the **circuit-driven** signals do this: the dwarfs and the two mains. A simple station
signal has no circuits behind it and goes dark when its station is unmanned, when trains run
past it without reference to it at all — reading one as a barrier would break a crossing's
detection every time a station was switched off. A signal counts if it stands inside the
approach and **faces** the crossing, whatever its own routes lead to: an entry signal's
authority commonly ends at the platform, short of the crossing, and it protects the approach
to it none the less. Open means a proceed aspect — C1 or C2 on a main, clear on a dwarf; a
dwarf saying a train stands in the road ahead is not an authority to move, and where a dwarf
shares a main's pole either head clearing opens it.

At Skonseng the two entry signals stand 782 m and 528 m from the crossing inside its 821 m
approach, so with both at danger those circuits reach only that far, and with a route set
they reach the whole way. The crossings out on the line have no signals in their approaches
and are untouched.

**Distant signals** repeat the crossing's own train indication far enough back to stop
from, one each side. The aspects are identical to the crossing's — same red-over-white,
same flash, same period and phase, so the two agree — except that **the red is violet**,
which is what tells a driver they are reading the warning and not the crossing itself.

They stand at **four fifths of the braking distance** at the line speed there: a distant is
a warning to start braking, not the last moment to, and the remaining fifth is the margin.
That figure is capped so the signal always sits **well inside the approach circuit** — the
train must already have armed the crossing before it can read the repeat, or it would be
shown an idle crossing by the very train that is about to close it — and floored so it
never lands inside the inner circuit at slow line speeds. At 130 km/h that is 745 m
against a 1473 m approach; at 40 km/h it comes down to 75 m. Nothing to author: they are
derived from the crossing, and the editor HUD shows where they came out.

Where a crossing spans two roads, **a repeat stands for the road the points lead to**. Out
on the single track beyond a station a driver does not yet know which road they will be
taken to, and the mast does not move when the points are thrown — so what it is warning
about is resolved by walking the road ahead, taking each turnout as it is currently set.
Where the points cannot say — a broken switch, or a fan the interlocking does not know — it
**warns**, rather than picking a road it cannot know. Two repeats that would stand at the
same spot are one mast, and a repeat that cannot be placed at its braking distance at all
(a loop is short, and past its turnout is another road) simply is not there: the one on the
road the points select covers both.

Each crossing runs its **own** blink, phased off its id, so two within sight of one
another are visibly out of step — as real ones are, each having its own oscillator.

**Barriers**, where a crossing has them, are half-barriers: one boom each side, hanging on
the same post as that side's road head and covering the lane its traffic arrives on. They
carry a red lamp halfway along, on the boom's upper face so it swings round with it,
flashing on the crossing's own period.

They keep a clock of their own, which is deliberately not the phase clock:

- they start down **7 s after the crossing activates** — the road gets its flashing first,
  and the boom falls into a gap that has already begun to clear — and take **8 s**, so
  they are fully down at 15 s. That is exactly the running time the approach distance
  already reserves, so barriers cost no extra warning distance and the circuits do not
  move;
- they start up **the moment the train is off the inner circuit**, which is the same
  moment the road's 5 s red delay starts running.

That second one is why the position is state of its own rather than something read off the
phase: opening runs 5 s and the lift takes 8, so **the booms are still rising for about
3 s after the crossing has gone back to idle**. They are moved at a rate toward where they
ought to be rather than placed from a timestamp, so a second train arriving while they are
still coming up finds them where they were, and nothing jumps.

Written into the overlay as a trailing `barriers` keyword. Both trailing fields — the
approach override and this — are optional and may come in either order, so a crossing
written before barriers existed reads back unchanged.

**Flag posts.** The hand signal a station's TXP hangs out: **red for stop, green for
pass through, and an empty fixture** when the station is unmanned or neither applies. Pick
**Flag posts** from the Esc menu and **click a point on any track**; **right-click**
selects, **F** swaps which side of the track the post stands on, **F2** names, **X**
deletes, **Ctrl+S** saves to `overlay/flag-posts.txt`.

The flag is not carried. It is on a stick that slots horizontally into a fixture near the
top of the post, so it hangs down toward the track and stays there. The post and its
fixture stand whether or not anything is in them — an empty fixture is itself an
indication. Because the flag hangs it reads from either direction, so there is no facing
to author; `F` is placement, not aim.

Each post attaches to the **nearest station**, which is only how the panel groups them:
`O` for the traffic manager, then **E**, gives one line per post under the worked
station's signals, and **Enter cycles it** — no flag, red, green, no flag. Every post is
independent. Setting one says nothing about any other, several may show different things
at once, and none of them is tied to the manned switch: a manned station has no flag out
most of the time.

Nothing constrains the flag against the fixed signalling either — a green flag against a
red entry signal is allowed, because part of why a hand signal exists is to say what the
signalling cannot.

**Avalanche warning signals.** Three lamps in a column on a post — **red over white over
red** — protecting a stretch where the mountain can come down on the line. At rest the
**white flashes**: it is not saying "proceed", the road ahead is nobody's business here, it
is saying that the watch is being kept and has nothing to report. A driver who sees a dark
head has learned something too, which is why the resting aspect is a flash. On a warning
**both reds flash together and the white goes out**.

Pick **Avalanche signals** from the Esc menu and **click a point on any track**;
**right-click** selects, **F** turns the head round, **B** walks the post across the track,
**Y** previews the warning aspect, **F2** names, **X** deletes, **Ctrl+S** saves to
`overlay/avalanche-signals.txt`. `F` and `B` are separate on purpose — turning the head
round should not move the post.

Unlike every other signal here it **governs nothing**: no route runs through one, no
interlocking reads one, no station owns one. What it shows is a property of the
mountainside. **Nothing can raise a warning yet** — there is no detector, and the aspect
exists so that the day there is one, it writes into a vector the rebuild already draws
from. `Y` in the editor is the only way to see it, and it is a preview: it is never saved.

The flashing costs nothing per frame. A lit lens carries its own period and phase in its
vertices and the shader reads them against a clock in the push constant, so a blinking head
never rebuilds a mesh — the same mechanism a flashing danger and a level crossing already
use, and why several lamps on one head can blink independently. The two reds are given the
same phase, so they flash together; giving one of them half a period would make them
alternate like a road crossing, and that is the whole of the change.

**TXP positions (permission to leave).** The one hand signal that hangs on nothing: the
TXP walks out to a spot beside the track where the driver can see them and holds up a
round sign — **a green ring around a white centre**, a European prohibitory sign with the
red swapped for green — giving a stopped train permission to leave. Pick **TXP
positions** from the Esc menu and **click a point on any track**; **right-click** selects,
**F** turns the signal round, **B** swaps which side of the track they stand, **F2** names,
**X** deletes, **Ctrl+S** saves to `overlay/txp-positions.txt`.

The direction is where the train departs to — the driver is looking that way, so the TXP
faces back against it. The side is absolute, so turning the signal round does not walk
them across the track.

A station is meant to have **as many positions as it needs**, and most need several: a TXP
standing at one end of Fauske cannot be seen from the other, so anywhere a train might be
stopped wants its own spot. **The editor draws a figure at every position**, which is the
only way to see what a station actually covers while authoring it.

**Which stations can talk to each other** is worked out from these positions rather than
authored anywhere. A station is a TXP station because a position was placed at it — there
is no separate way to be one — and its neighbours are the stations either side of it along
the running line. So the graph follows the positions as they are placed, and there is
nothing to keep in step with them. Today that gives Bodø — Oteråga — Fauske — Rognan.

**Manning a station puts it on the network**, and that is not a local act. The chain that
actually works the line is the *manned* stations and the sections between them, so opening
one sends a connect to the nearest manned station either side, and they have to agree.

- The first station on a line opens with nothing sent — there is nobody to ask.
- The second sends one connect, and the two then work the line between them.
- One opening **between** two manned stations asks **both**, and takes over the section
  they held: the link that spanned it goes, and two shorter ones replace it.
- They may only agree if that section is **clear**. A station cannot appear in the middle
  of a section with a train in it, because it would then be holding a road it was never
  told about — so the opening is refused, and nothing moves. It says which line was
  occupied.
- Unmanning hands the whole of what it held back to the two either side, joining them
  again. That needs nobody's agreement: a longer section under fewer stations takes
  nothing away from anyone.

Occupancy is measured from a little way inside each station rather than from the station
itself — the section is the line *between* them, and a train standing at a platform is not
on it. The exchange is printed as it happens (`[TXP] CONNECT`, `ACCEPT`, `REJECT`).

**Dispatching a train** is the exchange that uses those sections. From the station panel:
**Request dispatch** → where to → what kind of train. The far end answers automatically.

| | |
|---|---|
| **request** | granted only if the section is clear **in the books** — refused because it is already booked, not because something is standing in it |
| **line clear** | both ends hold the section: there is one record, not one per station |
| **train on track** | sent by the end the line was given to, once the train has left |
| **train arrived** | sent by the **far** end — a station cannot report the arrival of a train it dispatched, which would clear a line the train is still on |
| **withdraw** | the end that asked may cancel, but only until the train has left; after that only its arrival releases the line |

While a section is held, nothing else may have it: another request is refused from either
end, a station trying to **open** into it is refused, and the two stations holding it
cannot **unman** — the sections either side would merge into one line carrying two sets of
books, with nobody left to report the train arrived.

`EBANER_TXP_OPEN=A,B,C` mans stations at startup through the network, and
`EBANER_ROUTES=1` prints every movement the interlocking knows at load — the road it
covers, what it holds beyond the signal and which mast it lights, which is otherwise only
visible a line at a time through the picker.

`EBANER_PANEL=1|dest|type|route` opens the station panel, optionally at a dispatch step, or
the **R** route picker (`route` at its first step, `entry`/`exit` straight into a list), and
`EBANER_ROUTE=<n>[,<n>…]` sets the worked station's nth routes at startup, in order, so a
second one meets the state the first left. A bare number is the nth of everything the station
has, in the order `EBANER_ROUTES` prints; `e3`/`x2` index within one of the picker's two
lists — the dispatcher's UI is otherwise reachable only by keypress and could not be checked
at all.

It is a chain, which is all a line worked by train orders needs: a station deals with the
one either side and with nobody else. Two things it cannot express, both wanting an
authored override if they ever turn up — a **junction**, where a station would deal with
three; and a **break in the line**, where the stations either side land on different
routes and are chained separately. The second is arguably right, since a severed line
cannot pass a train order, but it shows as an edge quietly missing rather than as a fault.

Worked from the station panel (`O`, then **E**), one line per position which Enter shows
or takes down. **Only one shows at a time per station**: showing one stands the TXP down
wherever they were. That is the one place the "one person" argument really holds — unlike
the flags, which sit in fixtures and can all be out at once. In the sim nothing is drawn
at a position that is not showing, and nothing depends on the manned switch.

## Scripting

Everything in `overlay/` so far is a *fact*: a crossing is a line in a file, a TXP position
is a line in a file. Some of what a railway needs is not a fact but a **rule** — a
timetable, a train that runs itself, when a station is worked — and there was nowhere to
write a rule down. `<dataset>/overlay/overlay.lua` is that place. The viewer runs it once at
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

Per tile the viewer also reads, when present:

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
| `EBANER_SCREENSHOT` | Render ~20 frames, write that frame to the given PPM, exit.   |
| `EBANER_CAM`        | Scripted camera `"x,y,z,yawDeg,pitchDeg"` (scene-relative m). |
| `EBANER_NOSTITCH`   | Skip the seam-stitching pass (to inspect raw tile seams).     |
| `EBANER_NOCARVE`    | Skip carving railway cuttings into the terrain.               |
| `EBANER_NOOVERLAY`  | Ignore the `overlay/` track edits (link fixes).               |
| `EBANER_EDMODE`     | `ebaner-trackedit` only: start in this mode, by its menu name. |
| `EBANER_VEHICLE`    | Skip the start screen and preselect a vehicle, by its index in `kVehicleSpecs` (`0` = a single Class 93, `1`/`2` = two and three coupled, `3` = Di 4 + 5, `4` = the night train, `5` = a light Di 4, `6` = a CD 312). The table is ordered for the pickers, so these move when a vehicle is added - the start screen numbers the list. |
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
