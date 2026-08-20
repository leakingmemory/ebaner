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

#include "Vehicle.h"

#include <vector>

// A train: one or more sets coupled together and driven as one.
//
// Every train is a consist, a single set being a consist of one, so there is one code
// path and no special case for the common thing. What a consist owns is the *motion* -
// a coupled train has one speed, one position along the line, and the pulling and
// braking of every set adds into it. What it does not own is anything a set does for
// itself: each Vehicle keeps its own air system, its own compressor, its own
// low-reservoir safety device, its own engines and its own two cabs.
//
// That division is the whole design. It is what makes the two things the prototype does
// true here rather than approximated:
//
//   * the *digital link* carries the driving cab's brake notch and power demand to every
//     set, which then works out its own brake-cylinder pressure from its own reservoir
//     and its own tractive effort from its own engines - so two sets in different states
//     of charge brake differently, as they would;
//
//   * the *emergency line* is a separate mechanism that does not run over the link at
//     all. Any set whose own safety device trips puts the whole train into emergency,
//     independently, and it does so whether or not anything was commanding it.
//
// It is also what lets sets be coupled and uncoupled later: the sets are already
// separate objects with separate state, and an uncoupled set is a consist of one.
class Consist {
public:
    // Lay out `spec.units` sets from the given place, the first at `s` and the rest
    // coupled behind it, each following the track by the walk (so a train laid across a
    // turnout is placed on the rails it is actually on).
    Consist(const std::vector<TrackPath>* paths, const TrackPath* path,
            const VehicleSpec& spec, float s, float initialSpeed = 0.0f);

    void attachNetwork(const std::vector<TrackPath>* paths, SwitchNetwork* net);
    // Advance the whole train one step. `pushInput` in [-1,+1] is a hand push.
    void update(float dt, float pushInput = 0.0f);
    bool consumeSwitchChanged();

    // --- the sets ---------------------------------------------------------------
    int unitCount() const { return static_cast<int>(units_.size()); }
    const Vehicle& unit(int i) const { return units_[i]; }
    Vehicle& unit(int i) { return units_[i]; }
    // The set at the head of the train as laid out (cab 0's set). "Lead" is about the
    // order the sets sit in, not which end is driving - either end can drive.
    const Vehicle& lead() const { return units_.front(); }
    Vehicle& lead() { return units_.front(); }

    // --- cabs, addressed across the whole train ---------------------------------
    // Cab c lives on set c/2 at that set's local cab c%2, so cab 0 is at one end of the
    // train and cab cabCount()-1 at the other.
    int cabCount() const { return 2 * unitCount(); }
    // Whether a cab may take the reverser. When sets are coupled the cabs at the
    // coupler are shut down, as they are in practice, and only the two at the ends of
    // the train can drive. A shut-down cab can still be sat in and looked out of.
    bool cabDrivable(int cab) const;
    void setBrakeNotch(int cab, int notch);
    int brakeNotch(int cab) const;
    void setPowerNotch(int cab, int notch);
    int powerNotch(int cab) const;
    void moveHandle(int cab, int dir);
    int handlePosition(int cab) const;
    const char* handleName(int cab) const;
    void setReverser(int cab, int dir);
    int reverser(int cab) const;
    const char* reverserName(int cab) const;
    // The one cab in gear across the whole train, or -1 if none or more than one is.
    // More than one is the interlock case, and it now catches a cab in each set.
    int activeCab() const;
    bool interlockEmergency() const;
    // The train-wide emergency line: any set's own safety device, or the interlock.
    bool emergencyLine() const;
    // Which set is calling for emergency on its own account (-1 = none), so the cab can
    // be told *which* one rather than just that something tripped.
    int trippedUnit() const;
    bool safetyBrakeActive() const { return trippedUnit() >= 0; }
    int effectiveNotch() const;             // what the brakes are actually doing
    const char* effectiveBrakeName() const;

    // --- the whole train --------------------------------------------------------
    VehicleState state() const;
    float speed() const { return std::abs(v_); }  // m/s
    float mass() const;                           // kg, every set
    float length() const;                         // m, over the couplers
    float width() const { return lead().width(); }
    float height() const { return lead().height(); }
    float wheelbase() const { return lead().wheelbase(); }
    const char* name() const { return name_; }
    // Running resistance of the whole train: the weight-borne terms scale with the
    // train, the aerodynamic one does not - only the leading set meets the air.
    float rollingResistance(float speed) const;
    // Where the train is, taken at the leading set: the curve and cant the wheels are
    // being pushed across, which is what the rolling noise asks for.
    TrackPose pose() const { return lead().pose(); }
    GravityResolution gravity() const;      // at the leading set
    glm::vec3 inertia() const;
    float comHeight() const { return lead().comHeight(); }
    TippingLimit tippingLimit(float curvature, float cant) const;
    // Air, reported for the set the given cab is in (the driver reads his own set's
    // gauges); the HUD lists every set's separately.
    float mrPressure(int cab = 0) const;
    float bcPressure(int cab = 0) const;
    float bcRate() const;                   // loudest airflow over the train, for sound
    float tractiveEffort() const { return tractiveEffort_; }
    float brakeForce() const { return brakeForce_; }
    unsigned railImpacts() const;           // every wheel of every set
    int axleCount() const;
    bool compressorRunning() const;         // any set's compressor pumping

    // --- engines, over the whole train ------------------------------------------
    void toggleEngines();                   // the link starts and stops them together
    int engineCount() const;
    float engineRpm(int i) const;
    EngineState engineState(int i) const;
    bool enginesRunning() const;

    // --- geometry, over the whole train -----------------------------------------
    VehicleFrame frame() const { return lead().frame(); }
    std::vector<VehicleFrame> axleFrames() const;
    std::vector<VehicleFrame> bogieFrames() const;
    std::vector<VehicleFrame> bodySectionFrames() const;
    // Arc-length offsets of every axle from the *train's* centre.
    std::vector<float> axleOffsets() const;

    // Nose-to-nose over the Scharfenberg heads of two coupled sets. A modelling choice:
    // the real gap is small and the couplers are drawn closed, so this is what keeps the
    // two noses from sharing the same metre of track rather than a measured figure.
    static constexpr float kCouplerGap = 0.6f;

private:
    // Distance between the centres of two adjacent sets.
    float unitPitch() const { return lead().length() + kCouplerGap; }
    // Put the trailing sets where the coupler says they should be, following the track.
    void layOut();

    std::vector<Vehicle> units_;
    const std::vector<TrackPath>* paths_ = nullptr;
    const char* name_ = "";
    float v_ = 0.0f;                 // m/s, + = the direction the train faces
    float tractiveEffort_ = 0.0f;    // N, summed over the sets (for the HUD)
    float brakeForce_ = 0.0f;        // N, summed over the sets
};
