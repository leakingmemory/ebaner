// ebaner - a Vulkan viewer for terrainmapper rail/terrain exports.
// Copyright (C) 2026 Jan-Espen Oversand <sigsegv@radiotube.org>
//
// This file is part of ebaner. ebaner is free software: you can redistribute it
// and/or modify it under the terms of version 3 of the GNU General Public License
// as published by the Free Software Foundation.
//
// ebaner is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public License for more details. You
// should have received a copy of the license along with ebaner; if not, see
// <https://www.gnu.org/licenses/>.

#pragma once

#include "Occupancy.h" // PathSpan
#include "TrackPath.h"  // TrackPose

#include <glm/glm.hpp>

#include <iterator>
#include <vector>

class TrackPath;
class SwitchNetwork;

// How the body is drawn on top of the running gear. Only the drawing: what a vehicle *is*
// - how many engines, what turns the wheels, how many axles a bogie carries - is in the
// spec below and not read off this. It used to be, and that is why there could only ever
// be one real vehicle.
enum VehicleBodyStyle {
    BodyUnderframe = 0, // bare floor plate per section (a base to build on)
    BodyClass93 = 1,    // NSB Class 93 (Bombardier Talent) exterior
    BodyDi4 = 2,        // NSB Di 4 (Henschel) diesel-electric locomotive
    BodyType5 = 3,      // NSB Type 5 (Strommens Vaerksted) passenger carriage
    BodyType5Fr = 4,    // the same body as a cafe car: different windows, different doors
    BodyType5B = 5,     // and as a plain seating coach: the same shell, seating throughout
    BodyType5A = 6,     // and as 1st class: a window across the centre, and more legroom
    BodyWlab2 = 7,      // NSB WLAB-2 sleeping car: compartments, one small window each
    BodyCD312 = 8,      // CargoNet CD 312 (Vossloh/Stadler EURO 4000) freight locomotive
    BodyPocket = 9,     // Sdggmrss T3000e articulated double pocket wagon, loaded
    BodyFlat = 10,      // Sgnss 60' container flat, loaded with two 20 ft boxes
};

// What turns the wheels. Two machines that could hardly be less alike: one puts its engine
// through a torque converter and a gearbox, so its pull steps as it changes gear; the other
// turns a generator and its motors pull flat to a corner speed and then fall away as 1/v,
// holding the rated power. Neither is a special case of the other.
// How the driving position is laid out. A railcar of the 1990s gives the driver one
// combined lever - push for power, pull for brake - and a locomotive of 1981 does not: it
// has a power controller under the left hand and a driver's brake valve on a curved
// quadrant under the right, worked independently and often at the same time. Which it is
// changes what the keyboard does and what the cab draws, so it belongs to the vehicle.
enum ControlLayout {
    ControlCombined = 0, // one lever, power and brake on one axis (Class 93)
    ControlSeparate = 1, // power controller and train brake, a hand each (Di 4)
};

enum DriveKind {
    DriveNone = 0,      // unpowered: a wagon, a carriage, a bare wheelset
    DriveHydraulic = 1, // torque converter + 5-speed box (Class 93)
    DriveElectric = 2,  // prime mover -> alternator -> traction motors (Di 4)
};

// A selectable rail vehicle type. The running gear is described by a bogie count
// (0 = single bare axle, 1 = one bogie, 2 = end bogies, 3 = end + middle),
// bogieSpacing (end-bogie-to-end-bogie distance) and wheelbase (axle spacing
// within a bogie). Two or more bogies carry a body of `length` drawn per `body`.
// What part of the list a vehicle belongs in. The table is ordered by this and the
// pickers put a heading in wherever it changes, so the complete trains a driver actually
// wants are at the top and the bare test shapes are out of the way at the bottom.
enum VehicleCategory {
    CatTrain = 0,    // complete formations, ready to drive
    CatLoco = 1,     // a locomotive on its own
    CatCarriage = 2, // a single carriage, for building something up
    CatWagon = 3,    // freight stock, which is the CD 312's work and not the Di 4's
    CatDebug = 4,    // bare wheelsets and underframes: shapes to test the physics on
};

inline constexpr int kVehicleCats = 5; // how many headings the pickers will draw

constexpr const char* categoryName(int c) {
    return c == CatTrain      ? "TRAINS"
           : c == CatLoco     ? "LOCOMOTIVES"
           : c == CatCarriage ? "CARRIAGES"
           : c == CatWagon    ? "FREIGHT WAGONS"
                              : "DEBUGGING";
}

struct VehicleSpec {
    const char* name;
    float mass;         // kg
    float length;       // m, bounding box along travel
    float width;        // m, across the track
    float height;       // m, vertical
    float wheelbase;    // m axle spacing within a bogie (0 = single axle)
    float bogieSpacing; // m end-bogie-to-end-bogie distance (0 = <2 bogies)
    int   bogieCount;   // 0 bare axle, 1 bogie, 2 end bogies, 3 end + middle
    int   body;         // VehicleBodyStyle
    int   units;        // sets coupled into one train (1 = a single set)

    // What the vehicle *is*, as against how it is drawn. These were file-scope constants
    // in Vehicle.cpp shared by every vehicle there would ever be, which is workable
    // exactly as long as there is one. Defaulted, so the rows above stay as they were and
    // only a machine has to say anything.
    int   axlesPerBogie = 2;         // 3 on a Co'Co'
    int   engines = 0;               // prime movers; 0 is unpowered
    int   drive = DriveNone;         // DriveKind
    float powerW = 0.0f;             // rated at the engine, W
    float wheelRadius = 0.42f;       // m; sets the axle centre height as well as gearing
    float drivenFrac = 1.0f;         // of the weight, on driven axles
    float startTE = 0.0f;            // N, the flat low-speed limit (electric drive)
    // The engine's own speeds. A governed speed belongs to an engine and not to the
    // program: an EMD 645 turns 900 at full power where the railcar's Cummins turns 1500,
    // and borrowing one for the other puts the revs and the sound half as fast again as
    // they should be. Defaulted to the Class 93's, so its rows do not move.
    float idleRpm = 700.0f;
    float governedRpm = 1500.0f; // Vehicle::kMaxRpm, which is declared below this
    // How long the governor takes to walk the engine from idle to governed speed. On the
    // machine for the same reason its idle is: a 45-litre two-stroke turning a generator
    // is not a high-speed railcar diesel under a converter, and one number for both makes
    // whichever it was not written for feel wrong. Defaulted to the railcar's.
    float spoolTime = 0.9f;      // s
    // What the engine sounds like, which is as much the machine's own as its rev range.
    // The firing rate falls straight out of the first two: a four-stroke fires each
    // cylinder once every two revolutions and a two-stroke once every one, so a 6-cyl
    // four-stroke beats 3 to the rev and a 16-cyl two-stroke beats 16. The rest is bulk.
    // A 170-litre V16 EMD 645 is not a Cummins behind a modern railcar's sound-deadening.
    // It is far louder, and what it radiates is weighted much further down: a V16 is two
    // banks of eight with a manifold each, so there is a great deal of energy at half the
    // firing rate and below, and that is what is heard as size. Note that the firing rate
    // itself goes the other way - sixteen cylinders firing every revolution at 315 rpm is
    // 84 Hz where six firing every other one at 700 rpm is 35, so the big slow engine has
    // the faster beat. Weight comes from what is under the firing rate, not from it.
    int   cylinders = 6;         // firing strokes per cycle
    bool  twoStroke = false;     // fires every revolution rather than every other
    float engineVolume = 1.0f;   // how loud, against the railcar's
    float engineRumble = 0.0f;   // half- and quarter-order weight: the size of the block
    float engineBright = 0.11f;  // radiated-spectrum low-pass; lower is darker and bigger
    int   controls = ControlCombined; // ControlLayout
    // Driving positions. Defaulted to two, so every row written before carriages existed
    // keeps exactly what it had; a carriage says nothing and is driven from nowhere.
    int   cabs = 2;
    // Electro-pneumatic brake control: the driver's valve is echoed electrically at every
    // vehicle, so they all vent their pipe in step and the application does not have to
    // travel. A railcar of the 1990s has it. A locomotive of 1981 and carriages of 1977
    // do not - they have the plain automatic air brake, where the valve vents at one point
    // and the application walks the length of the train. That difference is why a hauled
    // train's brake is slow and a railcar's is not.
    bool  epBrake = true;
    // What this vehicle brings with it when it is put on the road: a comma-separated list
    // of name fragments, in order from the machine backwards. A list and not a count-of-one
    // -kind, because a real formation is not all one carriage - the standard rake here has
    // the cafe second. A formation, not a property of the machine, which is why it is the
    // only field that says anything about other vehicles.
    const char* hauls = nullptr;
    // What the whole train is called, where that differs from what the machine is called.
    // The locomotive stays a Di 4 when it has carriages behind it; only the formation gets
    // the longer name, and only the menu and the train list show it.
    const char* formation = nullptr;
    // The rheostatic brake - motstandsbremse - where the machine has one: the traction
    // motors driven as generators and the energy burned in roof grids. Two limits, the
    // same two the traction curve has and for the same reasons: a current limit that makes
    // the effort flat at low speed, and what the grids will take, which makes it P/v above
    // the corner. Zero means no dynamic brake at all, which is every other row here.
    //
    // Both figures for the Di 4 are ESTIMATES. No source I could reach gives either for
    // this locomotive; they are typical of a 2450 kW machine and chosen so the brake can do
    // its job - 180 kN holds the 435 t night train on a 1% grade at any speed it runs at.
    //
    // At the END of the struct on purpose. The rows below are positional, so a field
    // inserted higher up slides every value after it into the wrong slot without a word.
    float dynBrakeN = 0.0f; // N, the flat maximum
    float dynBrakeW = 0.0f; // W, what the resistor grids will dissipate
    // Whether this vehicle has an independent brake of its own - the Zusatzbremse, a
    // second valve on to its own cylinders. Every mainline locomotive has one; a railcar
    // has nothing to be independent OF, so this is a locomotive's field.
    bool independentBrake = false;
    // Which part of the list this belongs in. kVehicleSpecs MUST stay sorted by it - the
    // pickers put a heading in wherever it changes and would otherwise repeat one.
    int category = CatDebug;
};

// A formation: the same machine, with something coupled behind it. Written as a copy so
// the locomotive's twenty-odd numbers live in one row and cannot drift between the light
// engine and the train.
constexpr VehicleSpec hauling(VehicleSpec base, const char* name, const char* what) {
    base.formation = name; // the train's name; base.name stays the machine's
    base.hauls = what;
    base.category = CatTrain; // a formation is a train, wherever its locomotive belongs
    return base;
}

// Put a row in a part of the list. Needed because `category` is at the end of a struct
// with a dozen defaulted fields, so a row written positionally cannot reach it without
// spelling out everything in between.
constexpr VehicleSpec inCat(VehicleSpec v, int c) {
    v.category = c;
    return v;
}

// What to call a spec in a list: the formation if it is one, the machine otherwise.
constexpr const char* specTitle(const VehicleSpec& v) {
    return v.formation != nullptr ? v.formation : v.name;
}

// Find a vehicle by a fragment of its name. Used to resolve what a formation hauls, and
// by the tests, which were each carrying their own copy of this.
const VehicleSpec* specNamed(const char* fragment);

// The Di 4, named so the light engine and the train it hauls are one set of numbers.
// The CD 312's numbers, named for the same reason the Di 4's are: the light engine and the
// freight train it heads are one machine, and one set of figures.
inline constexpr VehicleSpec kCD312Spec = {
    "CargoNet CD 312 (Euro 4000)", 123000.0f, 23.02f, 2.98f, 4.26f, 3.90f, 12.30f, 2,
    BodyCD312, 1,
    3, 1, DriveElectric, 3178000.0f, 0.5335f, 1.00f, 400000.0f, 200.0f, 900.0f, 6.5f,
    16, true, 0.95f, 1.3f, 0.100f, ControlSeparate, 2, false, nullptr, nullptr,
    220000.0f, 1800000.0f, true};

inline constexpr VehicleSpec kDi4Spec = {
    "NSB Di 4 (Henschel)", 120000.0f, 20.80f, 3.176f, 4.35f, 3.85f, 11.75f, 2, BodyDi4, 1,
    3, 1, DriveElectric, 2450000.0f, 0.55f, 1.00f, 360000.0f, 315.0f, 900.0f, 7.0f,
    16, true, 0.92f, 2.7f, 0.075f, ControlSeparate, 2, false, nullptr, nullptr,
    180000.0f, 1400000.0f, true};

// The vehicles offered on the start screen.
inline constexpr VehicleSpec kVehicleSpecs[] = {
    // Ordered by category, and it MUST stay that way: the pickers put a heading in
    // wherever the category changes and would repeat one if a row were out of place.

    // --- Complete trains, which is what anyone starting the simulator wants ---
    // 2 x 306 kW through a torque converter and five gears, 0.84 m wheels, four of its
    // six axles driven. These were the file-scope constants every vehicle shared.
    inCat({"NSB Class 93 (Talent)", 70000.0f, 41.5f, 2.75f, 3.80f, 2.50f, 30.00f, 3, BodyClass93, 1,
     2, 2, DriveHydraulic, 612000.0f, 0.42f, 0.67f, 0.0f}, CatTrain),
    // Two sets coupled: the figures stay per set and `units` says how many. A Class 93
    // runs in multiple in service, and the two sets keep their own air, engines and
    // safety systems - see Consist.
    inCat({"NSB Class 93 x2 (Talent)", 70000.0f, 41.5f, 2.75f, 3.80f, 2.50f, 30.00f, 3, BodyClass93, 2,
     2, 2, DriveHydraulic, 612000.0f, 0.42f, 0.67f, 0.0f}, CatTrain),
    inCat({"NSB Class 93 x3 (Talent)", 70000.0f, 41.5f, 2.75f, 3.80f, 2.50f, 30.00f, 3, BodyClass93, 3,
     2, 2, DriveHydraulic, 612000.0f, 0.42f, 0.67f, 0.0f}, CatTrain),
    // The standard rake, as it is marshalled: cafe second, 2nd class third and fourth,
    // and 1st class on the tail.
    hauling(kDi4Spec, "NSB Di 4 + 5 (cafe 2nd)", "BC5-3,FR5-1,B5-3,B5-5,A5-1"),
    // And the night train: the same five with two sleepers on the back.
    hauling(kDi4Spec, "NSB Di 4 + 5 + 2 sleepers",
            "BC5-3,FR5-1,B5-3,B5-5,A5-1,WLAB-2,WLAB-2"),
    // And the freight, built to a length limit rather than to a wagon count: 600 m is what
    // has to fit, so this is the longest train of the stock there is that does not exceed
    // it. Nine pocket wagons and thirteen container flats behind the locomotive comes to
    // 598.82 m over the couplers - 1.18 m short - and no other mix of the two gets closer.
    // (Consist::length() sums the bodies and adds kCouplerGap between each pair, so the
    // couplers are in that figure and the arithmetic has to allow for 22 of them.)
    //
    // 1797 t, which is a heavy train and meant to be: it is inside what 398 kN of starting
    // effort will lift on the Saltfjell grades, but not by much, and that is the point of
    // having bought a 3178 kW locomotive to replace a 2450 kW one.
    //
    // The two types are interleaved rather than blocked, because a real intermodal train
    // is loaded with whatever is going that night.
    hauling(kCD312Spec, "CargoNet freight (599 m)",
            "Sgnss,Sgnss,T3000e,Sgnss,T3000e,Sgnss,Sgnss,T3000e,Sgnss,T3000e,Sgnss,Sgnss,"
            "T3000e,Sgnss,T3000e,Sgnss,Sgnss,T3000e,Sgnss,T3000e,Sgnss,T3000e"),

    // --- A locomotive on its own ---
    // NSB Di 4: Henschel, 1981, five built for Nordlandsbanen - this line. A Co'Co', so
    // six axles in two three-axle bogies and every one of them driven, which is what lets
    // a 120 t locomotive put down 360 kN without slipping. One EMD 16-645E3B of 2450 kW at
    // 900 rev/min turning an alternator, into three-phase asynchronous traction motors
    // from Brown Boveri and NEBB - the first in revenue service anywhere. 140 km/h on
    // 1.10 m wheels.
    //
    // The wheelbase here is the bogie's outer axle to outer axle, and the real bogie is
    // asymmetric: 1.85 m then 2.00 m. Spread evenly over the 3.85 m instead, which puts
    // the middle axle 75 mm from where it belongs and is not a thing anyone can see.
    // Bogie centres follow from 15.60 m between the outer axles.
    inCat(kDi4Spec, CatLoco),

    // CargoNet CD 312: Vossloh Espana, 2009, six leased from Beacon Rail - the machine that
    // actually works the Nordlandsbanen freights now, bought because it takes 25% more
    // train than the Di 4 it replaced on them. A Co'Co' again, and an EMD again: the
    // 16-710G3C-U2 is the direct descendant of the Di 4's 16-645, 28 years and 65 cubic
    // inches a cylinder later, and it turns the same 900 rev/min for 3178 kW against 2450.
    // 400 kN starting, 123 t, 120 km/h on 1.067 m wheels. It idles at 200, which is the
    // 710's own figure and a good deal slower than the Di 4's - so this V16 beats 53 times
    // a second where that one beats 84, and sounds the bigger machine for it.
    //
    // Less half- and quarter-order weight than the Di 4 carries, and that is not because
    // it is a smaller engine - it is the same size. The weight is generated at a quarter
    // and a half of the firing rate, and at 53 Hz firing those land at 13 and 27 where the
    // Di 4's land at 21 and 42. Thirteen hertz is below anything that will reproduce it:
    // measured with the Di 4's figure, 63% of this engine's output was under 20 Hz, heard
    // by nobody and eating the headroom the rest of the train needs. What is audible here
    // is the firing rate and its harmonics, so it gets a slightly brighter low-pass too.
    //
    // WIDTH, HEIGHT, WHEELBASE AND BOGIE CENTRES ARE ESTIMATES. No source I could reach
    // gives any of the four for this class, and the only photograph is a three-quarter
    // view that cannot be scaled. They are what a 23.02 m Co'Co' inside European loading
    // gauge has to be, near enough, and the length, mass, power and pull that matter are
    // all sourced.
    //
    // No `hauls`. That is a statement and not an omission: this is the freight locomotive,
    // its train is a freight train, and there is not one freight wagon in the model yet.
    // The passenger carriages belong to the Di 4 and are not to be hung on this instead.
    inCat(kCD312Spec, CatLoco),

    // --- Single carriages, for building a formation up by hand ---
    // NSB Type 5, Strommens Vaerksted 1977-81, 92 built. BC5-3 is a 2010-12 rebuild of a
    // B5-2 - "rullestolplass og lekerom", wheelchair spaces and a playroom - so it has an
    // interior this simulator has no way to show, a carriage being a thing you cannot get
    // into. What it does have from outside is 25.3 m on two two-axle bogies, and no engine,
    // no cab and no EP: it brakes on the train pipe and nothing else.
    //
    // Written by field rather than by position like the rows above it, because a carriage's
    // honest answer to most of them is "nothing". Fourteen zeroes for an engine it does not
    // have would say less than leaving them out.
    {.name = "NSB BC5-3 (Type 5)",
     .mass = 43000.0f,
     .length = 25.30f,
     .width = 3.10f,
     .height = 4.115f,
     .wheelbase = 2.50f,
     .bogieSpacing = 18.00f,
     .bogieCount = 2,
     .body = BodyType5,
     .units = 1,
     .wheelRadius = 0.46f, // 920 mm wheels
     .cabs = 0,
     .epBrake = false,
     .category = CatCarriage},
    // The cafe car, from the same drawings. 44 t, and the same body as the BC5-3 with a
    // quite different arrangement in and on it.
    {.name = "NSB FR5-1 (Type 5)",
     .mass = 44000.0f,
     .length = 25.30f,
     .width = 3.10f,
     .height = 4.115f,
     .wheelbase = 2.50f,
     .bogieSpacing = 18.00f,
     .bogieCount = 2,
     .body = BodyType5Fr,
     .units = 1,
     .wheelRadius = 0.46f,
     .cabs = 0,
     .epBrake = false,
     .category = CatCarriage},
    // The plain 2nd class seating coach the BC5-3 was rebuilt out of. Same shell and the
    // same windows; seventeen rows of four instead of nine, so 68 seats against 36.
    {.name = "NSB B5-3 (Type 5)",
     .mass = 43200.0f,
     .length = 25.30f,
     .width = 3.10f,
     .height = 4.115f,
     .wheelbase = 2.50f,
     .bogieSpacing = 18.00f,
     .bogieCount = 2,
     .body = BodyType5B,
     .units = 1,
     .wheelRadius = 0.46f,
     .cabs = 0,
     .epBrake = false,
     .category = CatCarriage},
    // B5-5: the same coach with eight of its seats set aside for passengers with pets.
    // It shares the B5-3's body deliberately, not for want of looking. Its side elevation
    // and its seat plan were compared against the B5-3's pixel by pixel: the seat rows and
    // the windows are the same to the centimetre, the only strong differences anywhere are
    // at the two doors where one drawing carries the orange highlight and the other does
    // not, and the eight pet seats are marked by being GREEN on the plan - a seat-map
    // convention for a designated area, not the colour of the moquette. There is nothing
    // to see, so there is nothing to draw, and a near-duplicate mesh would be a lie about
    // how much is known.
    {.name = "NSB B5-5 (Type 5)",
     .mass = 43200.0f,
     .length = 25.30f,
     .width = 3.10f,
     .height = 4.115f,
     .wheelbase = 2.50f,
     .bogieSpacing = 18.00f,
     .bogieCount = 2,
     .body = BodyType5B,
     .units = 1,
     .wheelRadius = 0.46f,
     .cabs = 0,
     .epBrake = false,
     .category = CatCarriage},
    // A5-1, the 1st class comfort coach. 48 seats against the 2nd class 68, and the one
    // variant whose shell is genuinely different: a window across the centre where every
    // other one has 2.2 m of blank side, and an extra short one at the far end.
    {.name = "NSB A5-1 (Type 5)",
     .mass = 42000.0f,
     .length = 25.30f,
     .width = 3.10f,
     .height = 4.115f,
     .wheelbase = 2.50f,
     .bogieSpacing = 18.00f,
     .bogieCount = 2,
     .body = BodyType5A,
     .units = 1,
     .wheelRadius = 0.46f,
     .cabs = 0,
     .epBrake = false,
     .category = CatCarriage},
    // WLAB-2, the sleeping car. Strommen again but 1986-87 and not a Type 5: 27.0 m on a
    // 3.24 m body, longer and wider and taller than the coaches it runs with, 50 t tare,
    // and 15 compartments - 14 sovekupeer and one HC - for 30 berths.
    {.name = "NSB WLAB-2 (sovevogn)",
     .mass = 50000.0f,
     .length = 27.00f,
     .width = 3.24f,
     .height = 4.22f,
     .wheelbase = 2.50f,
     .bogieSpacing = 19.00f,
     .bogieCount = 2,
     .body = BodyWlab2,
     .units = 1,
     .wheelRadius = 0.46f,
     .cabs = 0,
     .epBrake = false,
     .category = CatCarriage},

    // --- Freight stock: the CD 312's work ---
    // Sdggmrss T3000e, the articulated double pocket wagon combined transport runs on:
    // two platforms on THREE bogies, the middle one shared, so it is one wagon that bends
    // in the middle rather than two wagons coupled. 34.20 m over buffers, bogie pivots at
    // 2 x 14.20 m, 35 t tare and 100 t of load on six axles, 120 km/h, and it will go round
    // a 75 m curve. Each half carries one semi-trailer, wheels dropped into the pocket and
    // the kingpin up on the gooseneck - which is the whole point of the design, because a
    // trailer sitting on a flat deck is half a metre too tall for the loading gauge.
    //
    // Modelled LOADED, with a trailer in each pocket: 35 t of wagon and two laden trailers
    // at 30 t is 95 t, well inside the 135 t gross the wagon is rated for. An empty one is
    // a different drawing and a different mass, and would be its own row.
    //
    // The articulation needs no new code. bodySectionFrames() already gives a three-bogie
    // vehicle two sections, each oriented by its own pair of bogies, so the wagon bends at
    // the shared bogie and each half follows the rail it is actually on - which is what
    // the "Articulated (3 bogies)" test shape at the bottom of this table was built to
    // prove.
    {.name = "Sdggmrss T3000e (pocket wagon)",
     .mass = 95000.0f,
     .length = 34.20f,
     .width = 2.60f,
     .height = 4.20f,
     .wheelbase = 1.80f,     // Y25 bogie
     .bogieSpacing = 28.40f, // 2 x 14.20 m between pivots, the middle one shared
     .bogieCount = 3,
     .body = BodyPocket,
     .units = 1,
     .wheelRadius = 0.46f, // 920 mm wheels
     .cabs = 0,
     .epBrake = false,
     .category = CatWagon},

    // Sgnss 60', the plain four-axle container flat that does most of the work in
    // intermodal traffic. 19.60 m over buffers, bogie pivots at 14.06 m, Y25 bogies on a
    // 1.80 m wheelbase, 19 t tare and 71 t of load, 120 km/h empty and 100 loaded at
    // 22.5 t an axle, and round a 75 m curve like the pocket wagon.
    //
    // Loaded with two 20 ft boxes, which is the "two standard containers" this was asked
    // for and also how these run: a pair of TEU goes at the ENDS, over the bogies, not
    // together in the middle, because that is where the wagon wants the weight. A 60 ft
    // deck with 2 x 6.06 m on it leaves a gap amidships, and that gap is correct.
    //
    // 19 t of wagon and two boxes at 22 t is 63 t, well inside the 90 t gross. The deck
    // height of 1.175 m over the railhead is an ESTIMATE - the makers' sheets give the
    // length, the pivots, the bogie and the weights, but not that - and it is what a
    // container flat of this class has to be for a 2.59 m box to clear the gauge.
    {.name = "Sgnss 60' (container flat)",
     .mass = 63000.0f,
     .length = 19.60f,
     .width = 2.60f,
     .height = 3.80f,
     .wheelbase = 1.80f,     // Y25 bogie, as the pocket wagon has
     .bogieSpacing = 14.06f,
     .bogieCount = 2,
     .body = BodyFlat,
     .units = 1,
     .wheelRadius = 0.46f, // 920 mm wheels
     .cabs = 0,
     .epBrake = false,
     .category = CatWagon},

    // --- Bare shapes to test the physics on. Last: they are not trains ---
    {"Single-axle wheelset", 1300.0f, 0.20f, 2.20f, 0.92f, 0.00f, 0.00f, 0, BodyUnderframe, 1},
    {"Dual-axle bogie", 4000.0f, 2.60f, 2.50f, 1.05f, 1.80f, 0.00f, 1, BodyUnderframe, 1},
    {"Carriage (two bogies)", 34000.0f, 25.0f, 3.00f, 1.30f, 2.50f, 18.00f, 2, BodyUnderframe, 1},
    {"Articulated (3 bogies)", 45000.0f, 30.0f, 2.70f, 1.30f, 2.50f, 22.00f, 3, BodyUnderframe, 1},
};
// Counted off the table rather than written down beside it. A hand-kept number that falls
// behind the array makes the last entry unreachable everywhere at once - the start screen,
// the EBANER_VEHICLE clamp and the spawn clamp all read this one - and nothing warns. The
// cast keeps it an `int`: std::clamp is a single-type template and every caller passes
// ints alongside it.
inline constexpr int kNumVehicleSpecs = static_cast<int>(std::size(kVehicleSpecs));

// Gravity on the vehicle resolved at its current pose. The along-track part is
// "free" (it drives acceleration up/down grades); the remainder is reacted by the
// rails (the weight-on-rails / normal force, which future friction and adhesion
// forces will build on).
struct GravityResolution {
    glm::vec3 gravityForce;    // N, world m*g (downward)
    glm::vec3 alongTrackForce; // N, component along the tangent
    glm::vec3 alongTrackAccel; // m/s^2, alongTrackForce / mass
    glm::vec3 weightOnRails;   // N, component reacted by the rails
    float gradeRad;            // track slope (+ = climbing in travel direction)
};

// Curve overturning limit: the horizontal lateral acceleration (and the speed at
// a given radius) at which the vehicle tips over the outer rail, accounting for
// track cant. Bare, low wheelsets are very hard to tip; taller bodies tip sooner.
struct TippingLimit {
    float latAccelLimit; // m/s^2, horizontal lateral accel at overturning
    float critSpeed;     // m/s at the queried radius (inf on straight track)
};

// A rigid frame the wheelset mesh (and chase cam) can be built from, in any
// vehicle state: origin plus the cross-track/forward/up axes.
struct VehicleFrame {
    glm::vec3 pos;
    glm::vec3 right;
    glm::vec3 tangent;
    glm::vec3 up;
};

enum class VehicleState { OnRail, Derailed, Stopped };

// Per-engine state, derived from its rpm (cranking up / at idle / spinning down).
enum class EngineState { Off, Starting, Running, Stopping };

// What the digital link carries from the driving cab to every set of the train, and
// what a set hands back for the train's motion to be worked out from.
//
// `emergency` is deliberately not part of the commanded notch: it is the train-wide
// emergency line, which runs independently of the link so that a set can brake the whole
// train on its own account even when nothing is commanding it. See Consist.
// One brake unit: a bogie's distributor, its auxiliary reservoir and its cylinder.
//
// The distributor is the whole of the automatic brake in one object. It watches the brake
// pipe and remembers the highest pressure it has seen (`ctrl`); what it puts in the
// cylinder is proportional to how far the pipe has fallen *below that memory*, not to the
// pipe's absolute pressure. That is what makes the brake answer a reduction rather than a
// number, and what makes a severed pipe - which reads as the largest reduction there is -
// apply everything with nobody commanding it.
//
// The air it uses is its own. Filling the cylinder empties the auxiliary, which refills
// only from the pipe, so a driver who cycles the brake without giving the pipe time to
// recharge finds less in it each time. One per bogie so that a later fault can be one
// bogie's fault.
struct BrakeUnit {
    float aux = 0.0f;    // auxiliary reservoir (bar)
    float bc = 0.0f;     // brake cylinder (bar)
    float ctrl = 0.0f;   // control reservoir: the highest pipe pressure seen (bar)
    float bcRate = 0.0f; // d(bc)/dt (bar/s), for the sound
};

struct LinkCommand {
    int brakeNotch = 5;       // commanded by the active cab (Vehicle::kEmergencyNotch)
    float demand = 0.0f;      // signed traction demand [-1,1], + = train forward
    bool reverse = false;     // the reverser is in R (holds it to a shunting speed)
    bool powering = false;    // the cab is asking for power at all
    bool emergency = true;    // the train-wide emergency line is up
    // Whether the driver's brake valve is on THIS vehicle. On stock with EP the valve is
    // echoed electrically everywhere and this does not matter; on plain automatic air it
    // is the one place the pipe is vented and charged, and every other vehicle's pipe only
    // follows its neighbours'. That is what makes a long train slow to brake.
    bool valveHere = true;
    // What the independent brake valve is asking of the vehicle that has it, 0..1.
    float independent = 0.0f;
    // The electric brake, 0..1, from the controller's range below neutral. Separate from
    // `demand` because it is not negative traction: it is a retarding force that comes off
    // the same machines, and it answers to different limits.
    float dynamic = 0.0f;
};

// One set's contribution to the whole train's motion this step, in the physical frame
// (+ = the direction the train faces), so sets running on paths that disagree about
// which way is +s still add up.
struct UnitStep {
    float tractiveEffort = 0.0f; // N, signed
    float brakeForce = 0.0f;     // N, >= 0
};

// A rail vehicle (1 or 2 axles) riding a TrackPath at arc-length s (body centre),
// with a 1-DOF along-track physics model.
//
// A vehicle is one *set*: its own air system, its own compressor and low-reservoir
// safety device, its own engines and its own two cabs. It does not integrate its own
// motion - a Consist owns the speed of the whole train and steps every set with it,
// which is what lets several sets be coupled and still each brake on their own account.
class Vehicle {
public:
    Vehicle(const TrackPath* path, const VehicleSpec& spec, float s,
            float initialSpeed = 0.0f);

    // Attach the switch network so the vehicle can divert at turnouts. `paths` is
    // the same vector the constructor's TrackPath points into (turnouts index it).
    void attachNetwork(const std::vector<TrackPath>* paths, SwitchNetwork* net);
    // True once (then cleared) after the vehicle forced/broke a switch, so the caller
    // can rebuild the switch-stand mesh.
    bool consumeSwitchChanged();

    // Step this set's own subsystems - engines, compressor, air brake, low-reservoir
    // safety, transmission - at the train's current speed, and report what it is
    // contributing to the train's motion. Does not move the set: see advance().
    UnitStep stepSubsystems(float dt, const LinkCommand& cmd, float physicalSpeed);
    // Move this set along its own path by a physical distance (+ = train forward),
    // crossing its own turnouts on the way. True if it derailed.
    bool advance(float ds);
    // Off the rails: ground friction brings the free body to rest.
    void stepDerailed(float dt);
    // This set is calling for emergency on its own account (its low-reservoir safety
    // has tripped). The train-wide emergency line is the OR of this over every set.
    bool safetyBrakeDemand() const { return safetyBrake_; }
    // The one cab of *this set* that is in gear, or -1 if neither or both are. The
    // train-wide rule (exactly one cab in gear over the whole train) is the consist's.
    int localActiveCab() const;
    // The physical speed the consist last stepped this set with (m/s, unsigned).
    void setPhysicalSpeed(float v) { physV_ = v; }
    // Where a set sits on the network: which path, where along it, and which way round.
    struct TrackAnchor { int pathIdx = -1; float s = 0.0f; int orient = 1; };
    // Follow the track `bodyOffset` metres from this set's centre (+ = toward its nose)
    // and report where that lands, crossing turnouts by rail geometry on the way. How a
    // consist places the set coupled behind this one. False if it runs off the end.
    bool anchorAtOffset(float bodyOffset, TrackAnchor& out) const;
    // Put this set down at an anchor (used when the consist is first laid out).
    void placeAt(const std::vector<TrackPath>& paths, const TrackAnchor& a);

    // Rigid body frame for the camera / vehicle frame, in the current state.
    VehicleFrame frame() const { return bodyFrame(); }
    VehicleFrame bodyFrame() const;
    // On-rail pose of each axle (1 single axle, 2 bogie, 4 carriage), for the mesh.
    std::vector<VehicleFrame> axleFrames() const;
    // Pivot frame of each bogie (0 for a bare axle, up to 3), each chording its
    // own axles.
    std::vector<VehicleFrame> bogieFrames() const;
    // Arc-length offsets of each axle from the body centre. A description of where
    // the wheels are and nothing more - axleFrames() already publishes the same thing
    // in world space - and it is what says when each axle reaches a turnout.
    std::vector<float> axleOffsets() const;
    // The stretches of road this set's wheels stand on, from its rear-most axle to its
    // front-most, split wherever the body crosses a turnout onto another path. This is what
    // a track circuit is asked about: a section holds a train when the two overlap.
    //
    // Between the outermost axles rather than at each of them. A counter holds a section
    // from the first axle in to the last out, and per-axle points would let a section
    // shorter than the gap between two bogies read clear with a train standing on it.
    //
    // False if the body ran off the end of the track before it was all placed - a dead end
    // under part of the train - in which case `out` holds as much of it as there was rail
    // for. Nothing about the answer depends on which way the train faces or how fast it is
    // going; a span has two ends and no direction.
    bool occupiedSpans(std::vector<PathSpan>& out) const;
    // The road between two offsets along the body, which need not lie within this set: the
    // walk follows the rails, so a consist uses it to cover its whole length in one go.
    // The two must straddle the body centre, which is where the walk starts from.
    bool spansBetween(float offsetA, float offsetB, std::vector<PathSpan>& out) const;
    // Sort by path and arc length, then coalesce what touches. Published because a consist
    // gathers its sets' spans and has to do the same to them.
    static void coalesceSpans(std::vector<PathSpan>& spans);
    // Frame of each underframe body section (0 when <2 bogies, 1 for a carriage,
    // 2 for a 3-bogie module), each oriented by its bogie pair so an articulated
    // module flexes at the middle bogie.
    std::vector<VehicleFrame> bodySectionFrames() const;
    VehicleState state() const { return state_; }
    // Put this set on the ground where it stands, carrying the speed it had into the
    // slide. What running off the end of the track already does to itself, exposed so
    // that a collision can do it too - there is nothing different about being wrecked
    // by another train than by a buffer stop.
    void derail() { derailFreeze(); }
    float speed() const; // m/s

    TrackPose pose() const;
    GravityResolution gravity() const;

    // Principal mass moments of inertia (kg*m^2) treating the vehicle as a uniform
    // box, in the pose frame: x = roll (about travel axis), y = pitch (about
    // cross-track axis), z = yaw (about vertical). Yaw resists heading change in
    // turns; the others matter for pitch/roll dynamics.
    glm::vec3 inertia() const;

    // Centre-of-mass height above the rail running surface (box model: height/2).
    float comHeight() const { return 0.5f * height_; }

    // Overturning limit at a given track curvature (1/m) and cant (rad).
    TippingLimit tippingLimit(float curvature, float cant) const;

    // Davis-equation running resistance force (N) opposing motion at the given
    // speed (m/s): A + B*|v| + C*v^2 — rolling/bearing (A, proportional to weight),
    // flange/track (B*v, proportional to weight), aerodynamic drag (C*v^2, from
    // the frontal area). Used as the on-rail rolling resistance.
    float rollingResistance(float speed) const;
    // Just the aerodynamic term of the above (C*v^2). A train of several sets pushes
    // the air once, not once per set, so a consist has to be able to take this out.
    float dragResistance(float speed) const;

    // Air brake (notched direct brake). The handle sits at notch 0 (release),
    // 1..4 (graduated service) or kEmergencyNotch (emergency); each commands a
    // brake-cylinder pressure the local air system laps onto from the main
    // reservoir. Pressures are in bar.
    // One gravity for the whole program. It lives here because this is where the
    // forces are worked out, and a second copy of it somewhere else is a slow drift
    // waiting to happen rather than a rounding difference.
    static constexpr float kGravity = 9.81f; // m/s^2
    static constexpr int kEmergencyNotch = 5;
    static constexpr int kMaxPowerNotch = 5; // combined lever: N .. P1..P5 power side
    // And, on a machine whose controller doubles as its electric brake, the range below
    // neutral: E1..E5. Only reachable where dynBrakeN says there is a brake to command.
    static constexpr int kMaxBrakeNotch = 5;
    static constexpr int kMaxIndNotch = 5; // the independent brake, REL .. full
    // Per-cab brake handle (0 release .. kEmergencyNotch emergency), power notch
    // (0 N .. kMaxPowerNotch) and reverser (-1 R, 0 N, +1 F); cab 0 and cab 1 are
    // the two ends. Brake and power are the two sides of one combined lever and are
    // mutually exclusive (see moveHandle) — at most one is non-zero per cab.
    void setBrakeNotch(int cab, int notch);
    int brakeNotch(int cab) const;
    const char* brakeNotchName(int cab) const;
    void setPowerNotch(int cab, int notch);
    int powerNotch(int cab) const;
    // Move the combined lever one step: dir < 0 toward power (brake bleeds to 0
    // first, then power rises), dir > 0 toward brake (power bleeds to 0 first, then
    // brake rises to emergency).
    // The combined lever, and - where the machine has two handles instead - the power
    // controller and the driver's brake valve, worked independently. Both notches exist
    // either way; what differs is whether one lever walks them in sequence or two levers
    // move them apart, so a machine with separate controls can power against the brake.
    void moveHandle(int cab, int dir);
    void movePower(int cab, int dir);
    void moveBrake(int cab, int dir);
    // The independent brake - the Zusatzbremse. A second valve that puts air straight
    // into THIS vehicle's cylinders from its main reservoir, touching neither the train
    // pipe nor anything behind the drawbar. It is what a locomotive is held on at a
    // stand, what it is shunted on, and what lets it run light without dragging a train
    // brake application around with it.
    void moveIndependent(int cab, int dir);
    int independentNotch(int cab) const;
    bool hasIndependentBrake() const { return independent_; }
    int controls() const { return controls_; }
    int cabCount() const { return cabs_; }  // 0 on a carriage: driven from nowhere
    bool hasDynamicBrake() const { return dynBrakeN_ > 0.0f; }
    float dynamicBrakeForce() const { return dynBrake_; } // N this step, >= 0
    // How hard the grids are working, 0..1. What the blower is fed from.
    float dynamicBrakeFrac() const {
        if (dynBrakeN_ <= 0.0f) return 0.0f;
        const float f = dynBrake_ / dynBrakeN_;
        return f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
    }
    bool epBrake() const { return epBrake_; }
    // What this engine sounds like. Firings per revolution falls out of the cylinder
    // count and the cycle: every cylinder every revolution on a two-stroke, every other
    // one on a four-stroke.
    float firingsPerRev() const {
        return static_cast<float>(cylinders_) * (twoStroke_ ? 1.0f : 0.5f);
    }
    float engineVolume() const { return engineVolume_; }
    float engineRumble() const { return engineRumble_; }
    float engineBright() const { return engineBright_; }
    // The engine's own idle. The sound needs it to know when the engine has caught, which
    // is a thing only this machine can say: 200 on an EMD 710, 315 on a 645, 700 on the
    // railcar's Cummins.
    float idleRpm() const { return idleRpm_; }
    // Combined-lever position for the mesh/HUD: +brake (1..5), 0 neutral, -power
    // (-1..-kMaxPowerNotch); handleName gives "P3" / "N" / "B2" / "EMERG".
    int handlePosition(int cab) const;
    const char* handleName(int cab) const;
    void setReverser(int cab, int dir);
    int reverser(int cab) const;
    const char* reverserName(int cab) const;
    // Brake actually applied by *this set*: the notch the link commanded, unless this
    // set's own safety device has tripped or the train-wide emergency line is up, in
    // which case emergency regardless of what was commanded.
    int effectiveNotch() const;
    const char* effectiveBrakeName() const;
    // Something other than the commanded notch is holding this set in emergency.
    bool forcedEmergency() const { return safetyBrake_ || trainEmerg_; }
    // The single cab currently in gear (reverser out of Neutral), or -1 if none
    // or both are — the cab whose power/brake lever is live (see effectiveNotch).
    int activeCab() const;
    // Power actually demanded, after the reverser interlock and low-air safety: 0
    // unless exactly one cab is in gear (its power notch then rules).
    int effectivePowerNotch() const;
    // Signed traction demand in [-1,1]: direction * effectivePowerNotch/max.
    float tractionDemand() const;
    float tractiveEffort() const { return tractiveEffort_; } // N, + in +s (for HUD)
    // A diesel-electric is driven on these two and they say different things. The ammeter
    // is what the driver watches away from a stand, because effort is what current buys;
    // the engine's own load is effort times speed, and at a crawl that is a fraction of
    // what the diesel could give however hard the ammeter is pegged.
    float tractionAmpsFrac() const;  // motor current, of the limit the drive will pass
    float enginePowerFrac() const;   // rail power, of rated - how hard the diesel works
    float loadRegulator() const { return load_; } // excitation, 0..1, as it winds on
    // Friction-brake force actually applied last step (N, always >= 0), after the
    // brake-cylinder pressure is turned into shoe force and capped by wheel/rail
    // adhesion. The pressure alone does not say this - the cap bites in emergency -
    // and the rolling-noise synth needs the force at the friction surfaces, not the
    // pressure behind them.
    float brakeForce() const { return brakeForce_; }
    // Wheels crossing turnouts, counted since the vehicle was created. Modern track is
    // welded and silent between them, so the points are where the gaps are: every wheel
    // drops across the gap in the running rail at the frog, and a bogie's pair of them
    // is the double knock heard passing over a switch. One count per axle per turnout.
    unsigned railImpacts() const { return railImpacts_; }
    float mrPressure() const { return mrPres_; } // main reservoir (bar)
    float bpPressure() const { return bp_; }     // brake pipe, this set's length (bar)
    // Brake cylinder (bar). One number for a set that has one per bogie: the mean, since
    // that is what its share of the braking is proportional to and what a gauge shows.
    float bcPressure() const;
    // Rate of brake-cylinder pressure change (bar/s): + charging (apply), −
    // venting (release), 0 when equalized/released. The local air, at the bogies.
    float bcRate() const;
    // Rate of brake-pipe pressure change (bar/s), same sign convention. This is the
    // train line, and it is the loud one: a service reduction or an emergency dump is
    // heard the length of the train, where a cylinder filling is heard at the bogie.
    float bpRate() const { return bpRate_; }
    // The brake units, one per bogie, for the gauges and for later fault modelling.
    int brakeUnitCount() const { return static_cast<int>(brakes_.size()); }
    const BrakeUnit& brakeUnit(int i) const { return brakes_[static_cast<std::size_t>(i)]; }
    // Cut this set's brake pipe to atmosphere, as parting a coupling does. Nothing
    // commands the brakes on afterwards - the distributors see the pipe gone and apply
    // from their own air, which is the property the whole arrangement exists to have.
    void burstBrakePipe();
    // Close the cock again, as a shunter does by hand after parting a train. Until this
    // is done the pipe cannot hold air and the brakes cannot be released.
    void closeBrakePipeCock();
    bool brakePipeCut() const { return pipeCut_; }
    // Move air into or out of this set's length of pipe, as the hose to the next set
    // does. Only the consist calls this - it is the one thing about the brake that is
    // not a set's own business, because the pipe is one pipe.
    void nudgeBrakePipe(float dBar);
    // Let air out of this set's main reservoir, down to the given pressure. A set can
    // lose its air on its own account - a leak, a compressor that has stopped - and
    // that is the fault its low-reservoir safety device exists to catch, so it has to
    // be something that can happen to one set and not the other.
    void ventReservoir(float toBar);
    // True when the low-reservoir safety has forced an automatic emergency
    // application (overriding the handle) because the reservoir fell too low.
    bool safetyBrakeActive() const { return safetyBrake_; }

    // Diesel engines (one per cab end; synchronous start/stop). Once running they
    // drive the air compressor at idle and the hydraulic transmission under power.
    // Two Cummins N14E-R (14 L), full power at 1500 rev/min (the tachometer top).
    static constexpr float kMaxRpm = 1500.0f;
    void toggleEngines();                   // start if off, else stop (both together)
    int engineCount() const { return engineCount_; }
    float engineRpm(int i) const;           // current speed (rev/min)
    EngineState engineState(int i) const;
    bool enginesRunning() const;            // all engines at idle
    bool compressorRunning() const { return compActive_; } // charging the reservoir

    float s() const { return s_; }
    int pathIdx() const { return pathIdx_; }
    // Which way this set faces along its path's +s (+1 or -1).
    int orientation() const { return orient_; }
    // Slide along the current path without walking turnouts: the small correction a
    // consist makes to take up the slack a turnout crossing leaves in a coupler.
    void nudge(float ds) { s_ += ds; }
    bool enginesOn() const { return engineOn_; }
    float mass() const { return mass_; }
    float length() const { return length_; }
    float width() const { return width_; }
    float height() const { return height_; }
    float wheelbase() const { return wheelbase_; }
    float bogieSpacing() const { return bogieSpacing_; }
    int bogieCount() const { return bogieCount_; }
    int axlesPerBogie() const { return axlesPerBogie_; }
    int drive() const { return drive_; }
    // Rolling radius (m). Sets the axle centre height as well as the gearing, so the
    // body of a locomotive on 1.10 m wheels stands higher than a railcar on 0.84 m ones.
    float wheelRadius() const { return wheelRadius_; }
    int bodyStyle() const { return bodyStyle_; }
    const char* name() const { return name_; }
    const TrackPath* path() const { return path_; }

private:
    // Advance the transmission (auto gearbox + torque converter) and work out the
    // resulting tractive effort, left in tractiveEffort_ for the caller to apply to
    // the whole train. `demandSigned` is the demand in [-1,1] signed by track
    // direction (this set's orientation folded in), `demand` its magnitude,
    // `powering` whether traction is live, `reverse` whether the reverser is in R.
    void updateTraction(float demandSigned, float demand, bool powering, bool reverse,
                        float dt);
    // The diesel-electric drive, for a vehicle whose spec says so. Kept apart from the
    // hydraulic one above rather than folded into it: they share almost nothing.
    void updateElectricDrive(float demandSigned, float demand, bool powering,
                             bool reverse, float dt);
    void updateDynamicBrake(float demand, bool inGear, float dt);
    // Arc-length offsets of each bogie centre from the body centre (s_).
    std::vector<float> bogieCentres() const;
    // Half the span between the body's two support points (the two end bogies for
    // a carriage/module, the two axles for a lone bogie, 0 for a single axle).
    float supportHalf() const;

    // On-rail rigid frame for a body part at body-offset `o` (measured toward the
    // nominal +s / cab-1 end) with chord half-length `h` (0 = a single point). The
    // body is rigidly rotated 180 deg about vertical when orient_ = -1 (a diverted
    // train that runs against the new path's +s), so the drawn body keeps its
    // physical heading and the same cab leads.
    VehicleFrame railFrame(float o, float h) const;

    // World pose at a signed body offset (`+` = toward the nose) reached by walking
    // the connected track network from the centre (path_, s_, orient_), crossing
    // turnouts by rail geometry (diverging switch: toe->frog follows the siding; a
    // siding always joins the main at its junction end). The tangent/right point
    // nose-ward. Lets each axle sit on the rail it is physically on, so a diverting
    // train bends through the turnout. Falls back to a straight path sample if there
    // is no switch network.
    TrackPose walkPose(float bodyOffset) const;
    // The walk itself: leaves the path, arc-length and nose direction it ends on.
    // False when there is no switch network to walk (the caller gets a straight sample).
    bool walkTo(float bodyOffset, int& cp, float& cs, int& nose) const;
    // The same walk, recording every stretch it crosses rather than only where it lands.
    // False if it ran out of track before covering `bodyOffset`.
    bool walkSpans(float bodyOffset, std::vector<PathSpan>& out) const;

    // Resolve turnout crossings between sBefore and s_ on the current path (divert /
    // merge / trailing-break / facing-broken derail). Returns true if it derailed.
    bool crossTurnouts(float sBefore);
    // Count each *axle* passing a turnout between sBefore and s_, for the sound. A
    // separate question from crossTurnouts, which asks where the body centre is routed;
    // this one asks which wheels have just crossed a frog, and every axle does.
    void countRailImpacts(float sBefore);
    // Move onto another path, preserving the physical velocity vector (and flipping
    // orient_ so the reverser/push keep driving the same physical way).
    void swapPath(int newIdx, float newS);
    void derailFreeze(); // freeze the body pose and go to Derailed

    const TrackPath* path_;
    float s_;
    float mass_;
    float length_, width_, height_, wheelbase_, bogieSpacing_;
    int bogieCount_;
    // What the machine is, taken from its spec rather than from a shared constant.
    float idleRpm_ = 700.0f, governedRpm_ = kMaxRpm, spoolTime_ = 0.9f;
    int cylinders_ = 6;
    bool twoStroke_ = false;
    float engineVolume_ = 1.0f, engineRumble_ = 0.0f, engineBright_ = 0.11f;
    int controls_ = ControlCombined;
    int cabs_ = 2;
    bool epBrake_ = true;
    bool independent_ = false;
    int indNotch_[2] = {0, 0}; // per-cab independent brake handle
    float indDemand_ = 0.0f;   // what the link is asking of this vehicle's own cylinders
    float dynBrakeN_ = 0.0f, dynBrakeW_ = 0.0f;
    float dynBrake_ = 0.0f; // what the electric brake is making this step, N
    bool valveHere_ = true;  // the driver's brake valve is on this vehicle
    bool emergVent_ = false; // the accelerator: this distributor is dumping its own pipe
    float load_ = 0.0f;      // load regulator: excitation as a fraction of full
    float railPower_ = 0.0f; // W at the rail this step, for the engine-load reading
    int axlesPerBogie_ = 2, drive_ = DriveNone;
    float powerW_ = 0.0f, wheelRadius_ = 0.42f, drivenFrac_ = 1.0f, startTE_ = 0.0f;
    int bodyStyle_;
    const char* name_;

    VehicleState state_ = VehicleState::OnRail;
    // The train's speed, written by the consist each step. A set does not integrate
    // its own motion - the whole train has one speed - but the transmission, the
    // running resistance and the derail exit velocity all need to know it.
    float physV_ = 0.0f;                // m/s, + = the direction the train faces
    int cmdNotch_ = kEmergencyNotch;    // what the digital link last commanded
    bool trainEmerg_ = true;            // the train-wide emergency line, last step
    // Air-brake state (bar). Starts held: reservoir full, pipe empty, brakes applied.
    float mrPres_;                      // main reservoir pressure
    float bp_ = 0.0f;                   // brake pipe, this set's length of it
    float bpRate_ = 0.0f;               // d(bp_)/dt (bar/s), for the sound
    std::vector<BrakeUnit> brakes_;     // one per bogie
    bool pipeCut_ = false;              // the pipe is open to atmosphere (a parted hose)
    int brakeNotch_[2] = {kEmergencyNotch, kEmergencyNotch}; // per-cab handle (brake side)
    int powerNotch_[2] = {0, 0};        // per-cab handle (power side, 0 = neutral)
    int reverser_[2] = {0, 0};          // per-cab R/N/F (-1/0/+1)
    int gear_ = 1;                      // current automatic gear (1..5)
    float shiftTimer_ = 0.0f;           // s remaining in an upshift dwell (TE cut)
    float tractiveEffort_ = 0.0f;       // N applied this step (+ in +s), for the HUD
    float brakeForce_ = 0.0f;           // N of friction brake this step, for the sound
    unsigned railImpacts_ = 0;          // axles that have crossed a turnout, for the sound
    bool compOn_ = false;               // compressor state (cut-in/cut-out governor)
    bool safetyBrake_ = false;          // low-reservoir automatic emergency (latched)
    int engineCount_ = 0;               // diesel engines (2 for a Class 93, else 0)
    bool engineOn_ = false;             // commanded on/off (both engines together)
    float engineRpm_[2] = {0.0f, 0.0f}; // per-engine speed
    bool compActive_ = false;           // a compressor is pumping (loads the engine)
    glm::vec3 pos_{0.0f};               // derailed free-body position
    glm::vec3 vel_{0.0f};               // derailed velocity
    glm::vec3 fRight_{0.0f}, fTangent_{1.0f, 0.0f, 0.0f}, fUp_{0.0f, 0.0f, 1.0f};

    // Switch-network routing.
    const std::vector<TrackPath>* paths_ = nullptr; // path pool (path_ points inside)
    SwitchNetwork* net_ = nullptr;
    int pathIdx_ = -1;             // index of path_ within *paths_
    int orient_ = 1;              // physical-forward sign vs +s (flips on a path swap)
    bool switchChanged_ = false;   // set when the vehicle broke a switch
    int skipTurnout_ = -1;        // turnout to skip for one frame after swapping through it
};
