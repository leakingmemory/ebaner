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
};

// A formation: the same machine, with something coupled behind it. Written as a copy so
// the locomotive's twenty-odd numbers live in one row and cannot drift between the light
// engine and the train.
constexpr VehicleSpec hauling(VehicleSpec base, const char* name, const char* what) {
    base.formation = name; // the train's name; base.name stays the machine's
    base.hauls = what;
    return base;
}

// What to call a spec in a list: the formation if it is one, the machine otherwise.
constexpr const char* specTitle(const VehicleSpec& v) {
    return v.formation != nullptr ? v.formation : v.name;
}

// Find a vehicle by a fragment of its name. Used to resolve what a formation hauls, and
// by the tests, which were each carrying their own copy of this.
const VehicleSpec* specNamed(const char* fragment);

// The Di 4, named so the light engine and the train it hauls are one set of numbers.
inline constexpr VehicleSpec kDi4Spec = {
    "NSB Di 4 (Henschel)", 120000.0f, 20.80f, 3.176f, 4.35f, 3.85f, 11.75f, 2, BodyDi4, 1,
    3, 1, DriveElectric, 2450000.0f, 0.55f, 1.00f, 360000.0f, 315.0f, 900.0f, 7.0f,
    16, true, 0.92f, 2.7f, 0.075f, ControlSeparate, 2, false};

// The vehicles offered on the start screen.
inline constexpr VehicleSpec kVehicleSpecs[] = {
    {"Single-axle wheelset", 1300.0f, 0.20f, 2.20f, 0.92f, 0.00f, 0.00f, 0, BodyUnderframe, 1},
    {"Dual-axle bogie", 4000.0f, 2.60f, 2.50f, 1.05f, 1.80f, 0.00f, 1, BodyUnderframe, 1},
    {"Carriage (two bogies)", 34000.0f, 25.0f, 3.00f, 1.30f, 2.50f, 18.00f, 2, BodyUnderframe, 1},
    {"Articulated (3 bogies)", 45000.0f, 30.0f, 2.70f, 1.30f, 2.50f, 22.00f, 3, BodyUnderframe, 1},
    // 2 x 306 kW through a torque converter and five gears, 0.84 m wheels, four of its
    // six axles driven. These were the file-scope constants every vehicle shared.
    {"NSB Class 93 (Talent)", 70000.0f, 41.5f, 2.75f, 3.80f, 2.50f, 30.00f, 3, BodyClass93, 1,
     2, 2, DriveHydraulic, 612000.0f, 0.42f, 0.67f, 0.0f},
    // Two sets coupled: the figures stay per set and `units` says how many. A Class 93
    // runs in multiple in service, and the two sets keep their own air, engines and
    // safety systems - see Consist.
    {"NSB Class 93 x2 (Talent)", 70000.0f, 41.5f, 2.75f, 3.80f, 2.50f, 30.00f, 3, BodyClass93, 2,
     2, 2, DriveHydraulic, 612000.0f, 0.42f, 0.67f, 0.0f},
    {"NSB Class 93 x3 (Talent)", 70000.0f, 41.5f, 2.75f, 3.80f, 2.50f, 30.00f, 3, BodyClass93, 3,
     2, 2, DriveHydraulic, 612000.0f, 0.42f, 0.67f, 0.0f},
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
    kDi4Spec,
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
     .epBrake = false},
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
     .epBrake = false},
    // The standard rake: five carriages with the cafe second, as it is marshalled.
    hauling(kDi4Spec, "NSB Di 4 + 5 (cafe 2nd)", "BC5-3,FR5-1,BC5-3,BC5-3,BC5-3"),
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
    int controls() const { return controls_; }
    int cabCount() const { return cabs_; }  // 0 on a carriage: driven from nowhere
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
