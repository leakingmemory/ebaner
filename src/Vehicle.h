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

#include <vector>

class TrackPath;

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
};

// The vehicles offered on the start screen.
inline constexpr VehicleSpec kVehicleSpecs[] = {
    {"Single-axle wheelset", 1300.0f, 0.20f, 2.20f, 0.92f, 0.00f, 0.00f, 0, BodyUnderframe},
    {"Dual-axle bogie", 4000.0f, 2.60f, 2.50f, 1.05f, 1.80f, 0.00f, 1, BodyUnderframe},
    {"Carriage (two bogies)", 34000.0f, 25.0f, 3.00f, 1.30f, 2.50f, 18.00f, 2, BodyUnderframe},
    {"Articulated (3 bogies)", 45000.0f, 30.0f, 2.70f, 1.30f, 2.50f, 22.00f, 3, BodyUnderframe},
    {"NSB Class 93 (Talent)", 70000.0f, 41.5f, 2.75f, 3.80f, 2.50f, 30.00f, 3, BodyClass93},
};
inline constexpr int kNumVehicleSpecs = 5;

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

// A rail vehicle (1 or 2 axles) riding a TrackPath at arc-length s (body centre),
// with a 1-DOF along-track physics model.
class Vehicle {
public:
    Vehicle(const TrackPath* path, const VehicleSpec& spec, float s,
            float initialSpeed = 0.0f);

    // Advance the simulation. `pushInput` in [-1, +1] is a hand push along the
    // track (+1 = toward increasing s), applied only while on the rails. Gravity
    // also accelerates it; light rolling resistance coasts it to a stop; running
    // off either end derails it (then ground friction stops it).
    void update(float dt, float pushInput = 0.0f);

    // Rigid body frame for the camera / vehicle frame, in the current state.
    VehicleFrame frame() const { return bodyFrame(); }
    VehicleFrame bodyFrame() const;
    // On-rail pose of each axle (1 single axle, 2 bogie, 4 carriage), for the mesh.
    std::vector<VehicleFrame> axleFrames() const;
    // Pivot frame of each bogie (0 for a bare axle, up to 3), each chording its
    // own axles.
    std::vector<VehicleFrame> bogieFrames() const;
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

    // Air brake (notched direct brake). The handle sits at notch 0 (release),
    // 1..4 (graduated service) or kEmergencyNotch (emergency); each commands a
    // brake-cylinder pressure the local air system laps onto from the main
    // reservoir. Pressures are in bar.
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
    // Brake actually applied, after the reverser interlock and low-air safety:
    // emergency unless exactly one cab is out of Neutral (its handle then rules).
    // Non-cab vehicles just use handle 0.
    int effectiveNotch() const;
    const char* effectiveBrakeName() const;
    bool interlockEmergency() const; // the reverser interlock is forcing emergency
    // The single cab currently in gear (reverser out of Neutral), or -1 if none
    // or both are — the cab whose power/brake lever is live (see effectiveNotch).
    int activeCab() const;
    // Power actually demanded, after the reverser interlock and low-air safety: 0
    // unless exactly one cab is in gear (its power notch then rules).
    int effectivePowerNotch() const;
    // Signed traction demand in [-1,1]: direction * effectivePowerNotch/max.
    float tractionDemand() const;
    float tractiveEffort() const { return tractiveEffort_; } // N, + in +s (for HUD)
    float mrPressure() const { return mrPres_; } // main reservoir (bar)
    float bcPressure() const { return bcPres_; } // brake cylinder (bar)
    // Rate of brake-cylinder pressure change (bar/s): + charging (apply), −
    // venting (release), 0 when equalized/released. Drives the air-brake sound.
    float bcRate() const { return bcRate_; }
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
    // Advance the transmission (auto gearbox + torque converter) and apply the
    // resulting tractive effort to v_. `demandSigned` is the demand in [-1,1] signed
    // by track direction (cab orientation folded in), `demand` its magnitude,
    // `powering` whether traction is live, `reverse` whether the reverser is in R.
    void updateTraction(float demandSigned, float demand, bool powering, bool reverse,
                        float dt);
    // Arc-length offsets of each bogie centre from the body centre (s_).
    std::vector<float> bogieCentres() const;
    // Arc-length offsets of each axle from the body centre (s_).
    std::vector<float> axleOffsets() const;
    // Half the span between the body's two support points (the two end bogies for
    // a carriage/module, the two axles for a lone bogie, 0 for a single axle).
    float supportHalf() const;

    const TrackPath* path_;
    float s_;
    float mass_;
    float length_, width_, height_, wheelbase_, bogieSpacing_;
    int bogieCount_;
    int bodyStyle_;
    const char* name_;

    VehicleState state_ = VehicleState::OnRail;
    float v_ = 0.0f;                    // on-rail scalar speed (+ = increasing s)
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
    bool compOn_ = false;               // compressor state (cut-in/cut-out governor)
    bool safetyBrake_ = false;          // low-reservoir automatic emergency (latched)
    int engineCount_ = 0;               // diesel engines (2 for a Class 93, else 0)
    bool engineOn_ = false;             // commanded on/off (both engines together)
    float engineRpm_[2] = {0.0f, 0.0f}; // per-engine speed
    bool compActive_ = false;           // a compressor is pumping (loads the engine)
    glm::vec3 pos_{0.0f};               // derailed free-body position
    glm::vec3 vel_{0.0f};               // derailed velocity
    glm::vec3 fRight_{0.0f}, fTangent_{1.0f, 0.0f, 0.0f}, fUp_{0.0f, 0.0f, 1.0f};
};
