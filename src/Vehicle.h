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

#include "TrackPath.h" // TrackPose

#include <glm/glm.hpp>

#include <iterator>
#include <vector>

class TrackPath;
class SwitchNetwork;

// How the body is drawn on top of the running gear.
enum VehicleBodyStyle {
    BodyUnderframe = 0, // bare floor plate per section (a base to build on)
    BodyClass93 = 1,    // NSB Class 93 (Bombardier Talent) exterior
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
};

// The vehicles offered on the start screen.
inline constexpr VehicleSpec kVehicleSpecs[] = {
    {"Single-axle wheelset", 1300.0f, 0.20f, 2.20f, 0.92f, 0.00f, 0.00f, 0, BodyUnderframe, 1},
    {"Dual-axle bogie", 4000.0f, 2.60f, 2.50f, 1.05f, 1.80f, 0.00f, 1, BodyUnderframe, 1},
    {"Carriage (two bogies)", 34000.0f, 25.0f, 3.00f, 1.30f, 2.50f, 18.00f, 2, BodyUnderframe, 1},
    {"Articulated (3 bogies)", 45000.0f, 30.0f, 2.70f, 1.30f, 2.50f, 22.00f, 3, BodyUnderframe, 1},
    {"NSB Class 93 (Talent)", 70000.0f, 41.5f, 2.75f, 3.80f, 2.50f, 30.00f, 3, BodyClass93, 1},
    // Two sets coupled: the figures stay per set and `units` says how many. A Class 93
    // runs in multiple in service, and the two sets keep their own air, engines and
    // safety systems - see Consist.
    {"NSB Class 93 x2 (Talent)", 70000.0f, 41.5f, 2.75f, 3.80f, 2.50f, 30.00f, 3, BodyClass93, 2},
    {"NSB Class 93 x3 (Talent)", 70000.0f, 41.5f, 2.75f, 3.80f, 2.50f, 30.00f, 3, BodyClass93, 3},
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
struct LinkCommand {
    int brakeNotch = 5;       // commanded by the active cab (Vehicle::kEmergencyNotch)
    float demand = 0.0f;      // signed traction demand [-1,1], + = train forward
    bool reverse = false;     // the reverser is in R (holds it to a shunting speed)
    bool powering = false;    // the cab is asking for power at all
    bool emergency = true;    // the train-wide emergency line is up
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
    // Frame of each underframe body section (0 when <2 bogies, 1 for a carriage,
    // 2 for a 3-bogie module), each oriented by its bogie pair so an articulated
    // module flexes at the middle bogie.
    std::vector<VehicleFrame> bodySectionFrames() const;
    VehicleState state() const { return state_; }
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
    void moveHandle(int cab, int dir);
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
    float bcPressure() const { return bcPres_; } // brake cylinder (bar)
    // Rate of brake-cylinder pressure change (bar/s): + charging (apply), −
    // venting (release), 0 when equalized/released. Drives the air-brake sound.
    float bcRate() const { return bcRate_; }
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
    int bodyStyle_;
    const char* name_;

    VehicleState state_ = VehicleState::OnRail;
    // The train's speed, written by the consist each step. A set does not integrate
    // its own motion - the whole train has one speed - but the transmission, the
    // running resistance and the derail exit velocity all need to know it.
    float physV_ = 0.0f;                // m/s, + = the direction the train faces
    int cmdNotch_ = kEmergencyNotch;    // what the digital link last commanded
    bool trainEmerg_ = true;            // the train-wide emergency line, last step
    // Air-brake state (bar). Starts held: reservoir full, emergency applied.
    float mrPres_;                      // main reservoir pressure
    float bcPres_;                      // brake cylinder pressure
    float bcRate_ = 0.0f;               // d(bcPres_)/dt (bar/s), for the sound
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
