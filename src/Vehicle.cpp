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

#include "Vehicle.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr float kG = 9.81f;         // m/s^2
constexpr float kGauge = 1.435f;    // standard track gauge (tipping pivot width)
constexpr float kFrictionMu = 0.6f; // derailed ground friction ("digging in")
constexpr float kStopSpeed = 0.1f;  // m/s below which a derailed vehicle stops
constexpr float kPushForce = 500.0f; // N, a person's sustained hand shove
// Davis running-resistance coefficients (SI). A and B scale with weight; C is
// aerodynamic. Values are light, appropriate for steel wheel on rail.
constexpr float kDavisA = 0.002f;    // rolling + bearing, per unit weight
constexpr float kDavisB = 0.0001f;   // flange/track, per unit weight, s/m
constexpr float kDragCd = 1.0f;      // aerodynamic drag coefficient (bluff box)
constexpr float kAirDensity = 1.225f; // kg/m^3

// Air brake (all pressures in bar). A notched direct brake: each handle notch
// commands a brake-cylinder target the local system laps onto from the main
// reservoir; a governed compressor keeps the reservoir charged.
constexpr float kMRCapacity = 8.0f;      // main reservoir full / compressor cut-out
constexpr float kMRCutIn = 6.5f;         // compressor cut-in
constexpr float kBCFullService = 3.4f;   // brake cylinder at full service (B4)
constexpr float kBCEmergency = 3.8f;     // brake cylinder in emergency
constexpr float kBCApplyRate = 1.5f;     // service apply rate (bar/s)
constexpr float kBCEmergRate = 6.0f;     // emergency apply rate (bar/s)
constexpr float kBCReleaseRate = 1.2f;   // release rate (bar/s)
constexpr float kCompRate = 0.20f;       // compressor recharge (bar/s)
constexpr float kMRPerBC = 0.04f;        // MR bar spent per bar of BC charged
constexpr float kMRLeak = 0.001f;        // reservoir leak (bar/s)
// The compressor is driven by the engine, which isn't modelled yet, so it stays
// off: the reservoir only depletes (slowly, over many applications) for now.
constexpr bool kCompressorEnabled = false;
constexpr float kFullServiceDecel = 1.3f; // deceleration at full service (m/s^2)
constexpr float kAdhesionMu = 0.20f;     // wheel/rail grip cap on brake force
constexpr float kMRSafetyTrip = 6.0f;    // low reservoir -> automatic emergency
constexpr float kMRSafetyReset = 6.5f;   // safety clears once recharged above this

// Brake-cylinder target (bar) for a handle notch: 0 release, 1..4 graduated
// service up to full service, emergency a touch higher.
float targetBC(int notch) {
    if (notch <= 0) return 0.0f;
    if (notch >= Vehicle::kEmergencyNotch) return kBCEmergency;
    return kBCFullService * static_cast<float>(notch) / 4.0f;
}
} // namespace

Vehicle::Vehicle(const TrackPath* path, const VehicleSpec& spec, float s,
                 float initialSpeed)
    : path_(path),
      s_(s),
      mass_(spec.mass),
      length_(spec.length),
      width_(spec.width),
      height_(spec.height),
      wheelbase_(spec.wheelbase),
      bogieSpacing_(spec.bogieSpacing),
      bogieCount_(spec.bogieCount),
      bodyStyle_(spec.body),
      name_(spec.name),
      v_(initialSpeed),
      mrPres_(kMRCapacity),   // reservoir starts at capacity
      bcPres_(kBCEmergency) {} // brakes start in emergency (held)

void Vehicle::setBrakeNotch(int notch) {
    brakeNotch_ = std::clamp(notch, 0, kEmergencyNotch);
}

const char* Vehicle::brakeNotchName() const {
    static const char* kNames[] = {"REL", "B1", "B2", "B3", "B4", "EMERG"};
    return kNames[std::clamp(brakeNotch_, 0, kEmergencyNotch)];
}

namespace {
VehicleFrame frameOf(const TrackPose& p) {
    return {p.pos, p.right, p.tangent, p.up};
}

// Rigid frame spanning two on-rail poses (chords the curve between them).
VehicleFrame chordFrame(const TrackPose& pr, const TrackPose& pf,
                        const glm::vec3& fallbackTangent) {
    VehicleFrame f;
    f.pos = (pr.pos + pf.pos) * 0.5f;
    const glm::vec3 chord = pf.pos - pr.pos;
    const float cl = glm::length(chord);
    f.tangent = (cl > 1e-6f) ? chord / cl : fallbackTangent;
    const glm::vec3 up = glm::normalize(pr.up + pf.up);
    f.right = glm::normalize(glm::cross(up, f.tangent));
    f.up = glm::normalize(glm::cross(f.tangent, f.right));
    return f;
}
} // namespace

TrackPose Vehicle::pose() const { return path_->poseAt(s_); }

float Vehicle::rollingResistance(float speed) const {
    const float w = mass_ * kG;                     // weight (N)
    const float A = kDavisA * w;                    // rolling + bearing (N)
    const float B = kDavisB * w;                    // flange/track (N per m/s)
    const float C = 0.5f * kAirDensity * kDragCd * (width_ * height_); // aero
    const float v = std::abs(speed);
    return A + B * v + C * v * v;
}

float Vehicle::speed() const {
    return (state_ == VehicleState::OnRail) ? std::abs(v_) : glm::length(vel_);
}

float Vehicle::supportHalf() const {
    if (bogieCount_ >= 2) return 0.5f * bogieSpacing_; // chord the two end bogies
    if (bogieCount_ == 1) return 0.5f * wheelbase_;    // chord the bogie's axles
    return 0.0f;                                       // single axle
}

std::vector<float> Vehicle::bogieCentres() const {
    const float bc = 0.5f * bogieSpacing_;
    switch (bogieCount_) {
        case 1: return {0.0f};
        case 2: return {-bc, bc};
        case 3: return {-bc, 0.0f, bc};
        default: return {}; // 0: no bogie (single bare axle)
    }
}

std::vector<float> Vehicle::axleOffsets() const {
    if (bogieCount_ == 0) return {0.0f}; // single bare axle
    const float wb = 0.5f * wheelbase_;
    std::vector<float> out;
    for (float c : bogieCentres()) { out.push_back(c - wb); out.push_back(c + wb); }
    return out;
}

VehicleFrame Vehicle::bodyFrame() const {
    if (state_ != VehicleState::OnRail)
        return {pos_, fRight_, fTangent_, fUp_};
    const float half = supportHalf();
    if (half < 1e-3f) return frameOf(path_->poseAt(s_)); // single axle
    // Chord the two support points (end bogies for a carriage/module, the two
    // axles for a lone bogie).
    return chordFrame(path_->poseAt(s_ - half), path_->poseAt(s_ + half),
                      path_->poseAt(s_).tangent);
}

std::vector<VehicleFrame> Vehicle::bogieFrames() const {
    const float wb = 0.5f * wheelbase_;
    std::vector<VehicleFrame> out;
    for (float c : bogieCentres()) {
        if (state_ == VehicleState::OnRail)
            out.push_back(chordFrame(path_->poseAt(s_ + c - wb),
                                     path_->poseAt(s_ + c + wb),
                                     path_->poseAt(s_ + c).tangent));
        else // derailed: bogie pivots offset along the frozen body tangent
            out.push_back({pos_ + fTangent_ * c, fRight_, fTangent_, fUp_});
    }
    return out;
}

std::vector<VehicleFrame> Vehicle::bodySectionFrames() const {
    const std::vector<float> centres = bogieCentres();
    std::vector<VehicleFrame> out;
    if (centres.size() < 2) return out; // <2 bogies carry no underframe body
    const int n = static_cast<int>(centres.size()) - 1; // sections between bogies
    for (int i = 0; i < n; ++i) {
        // Body-section centre, tiled evenly along the body length.
        const float mid = -0.5f * length_ + (i + 0.5f) * length_ / n;
        if (state_ == VehicleState::OnRail) {
            // Orientation from the section's bogie pair (so the body flexes at the
            // shared middle bogie); position on the track at the section centre.
            VehicleFrame f = chordFrame(path_->poseAt(s_ + centres[i]),
                                        path_->poseAt(s_ + centres[i + 1]),
                                        path_->poseAt(s_ + mid).tangent);
            f.pos = path_->poseAt(s_ + mid).pos;
            out.push_back(f);
        } else { // derailed: frozen body axes, section offset along the tangent
            out.push_back({pos_ + fTangent_ * mid, fRight_, fTangent_, fUp_});
        }
    }
    return out;
}

std::vector<VehicleFrame> Vehicle::axleFrames() const {
    std::vector<VehicleFrame> out;
    for (float off : axleOffsets()) {
        if (state_ == VehicleState::OnRail)
            out.push_back(frameOf(path_->poseAt(s_ + off)));
        else // derailed: axles offset from the frozen body along its tangent
            out.push_back({pos_ + fTangent_ * off, fRight_, fTangent_, fUp_});
    }
    return out;
}

void Vehicle::update(float dt, float pushInput) {
    if (state_ == VehicleState::OnRail) {
        const VehicleFrame bf = bodyFrame();
        // Driving acceleration: gravity along the track (downhill in +s is
        // positive) plus the hand push (a force, so a = F/m).
        const float aGrav = -kG * bf.tangent.z;
        const float aPush = pushInput * kPushForce / mass_;
        v_ += (aGrav + aPush) * dt;

        // Low main-reservoir safety: if the reservoir falls below the trip
        // pressure the brakes go to full emergency regardless of the handle,
        // latched until the reservoir recovers above the reset pressure.
        if (mrPres_ < kMRSafetyTrip) safetyBrake_ = true;
        else if (mrPres_ >= kMRSafetyReset) safetyBrake_ = false;
        const int effNotch = safetyBrake_ ? kEmergencyNotch : brakeNotch_;

        // Air brake: lap the brake-cylinder pressure toward the notch target,
        // charging from (and spending) the main reservoir on apply, venting on
        // release; a governed compressor recharges the reservoir.
        const float bcBefore = bcPres_;
        const float tgt = targetBC(effNotch);
        if (tgt > bcPres_) {
            const float rate = (effNotch >= kEmergencyNotch) ? kBCEmergRate : kBCApplyRate;
            const float reach = std::min(tgt, mrPres_); // capped by reservoir pressure
            const float before = bcPres_;
            bcPres_ = std::min(reach, bcPres_ + rate * dt);
            mrPres_ -= kMRPerBC * std::max(0.0f, bcPres_ - before);
        } else if (tgt < bcPres_) {
            bcPres_ = std::max(tgt, bcPres_ - kBCReleaseRate * dt);
        }
        mrPres_ -= kMRLeak * dt;
        // Governed compressor (cut-in below kMRCutIn, runs until full). Charging
        // the cylinders costs kMRPerBC per bar of BC, so without the compressor the
        // reservoir only draws down — enough air for many applications, then the
        // cylinders can no longer fully charge and the brakes fade. The compressor
        // is engine-driven; until engines exist it stays off (kCompressorEnabled).
        if (kCompressorEnabled) {
            if (mrPres_ >= kMRCapacity) compOn_ = false;
            else if (mrPres_ < kMRCutIn) compOn_ = true;
            if (compOn_) mrPres_ += kCompRate * dt;
        }
        mrPres_ = std::clamp(mrPres_, 0.0f, kMRCapacity);
        bcPres_ = std::max(0.0f, bcPres_);
        bcRate_ = (dt > 1e-6f) ? (bcPres_ - bcBefore) / dt : 0.0f; // airflow, for sound

        // Brake force (capped by wheel/rail adhesion) and Davis running resistance
        // both oppose motion, capped so they can't reverse v_ (this also holds the
        // vehicle at rest, up to the grade the brakes can hold).
        const float brake = std::min((bcPres_ / kBCFullService) * mass_ * kFullServiceDecel,
                                     kAdhesionMu * mass_ * kG);
        const float resistDecel = brake / mass_ + rollingResistance(v_) / mass_;
        const float resist = std::min(resistDecel * dt, std::abs(v_));
        v_ -= std::copysign(resist, v_);

        s_ += v_ * dt;

        // Derail when the leading or trailing axle passes an end of the track.
        const float L = path_->length();
        float outerHalf = 0.0f;
        for (float o : axleOffsets()) outerHalf = std::max(outerHalf, std::abs(o));
        if (s_ - outerHalf < 0.0f || s_ + outerHalf > L) {
            const VehicleFrame e = bodyFrame(); // frozen at exit
            pos_ = e.pos;
            fRight_ = e.right;
            fTangent_ = e.tangent;
            fUp_ = e.up;
            vel_ = v_ * e.tangent; // carry the exit velocity
            state_ = VehicleState::Derailed;
        }
    } else if (state_ == VehicleState::Derailed) {
        // Friction opposes velocity, magnitude proportional to weight
        // (deceleration = mu * g); brings the vehicle to rest.
        const float speed = glm::length(vel_);
        if (speed > 1e-5f) {
            const float drop = std::min(kFrictionMu * kG * dt, speed);
            vel_ -= (vel_ / speed) * drop;
            pos_ += vel_ * dt;
        }
        if (glm::length(vel_) < kStopSpeed) {
            vel_ = glm::vec3(0.0f);
            state_ = VehicleState::Stopped;
        }
    }
}

GravityResolution Vehicle::gravity() const {
    GravityResolution r{};
    const TrackPose p = path_->poseAt(s_);
    const glm::vec3 T = p.tangent; // unit; T.z is the sine of the grade

    const glm::vec3 gAccel(0.0f, 0.0f, -kG);
    r.gravityForce = mass_ * gAccel;

    // Split gravity into the free along-track part and the rail-reacted remainder.
    const float alongMag = glm::dot(r.gravityForce, T); // N, signed along +T
    r.alongTrackForce = alongMag * T;
    r.alongTrackAccel = r.alongTrackForce / mass_; // == dot(gAccel, T) * T
    r.weightOnRails = r.gravityForce - r.alongTrackForce;

    r.gradeRad = std::asin(std::clamp(T.z, -1.0f, 1.0f));
    return r;
}

glm::vec3 Vehicle::inertia() const {
    // Uniform box principal moments about the centre of mass.
    const float L2 = length_ * length_, W2 = width_ * width_, H2 = height_ * height_;
    const float c = mass_ / 12.0f;
    return glm::vec3(c * (W2 + H2),  // roll  (about travel/x axis)
                     c * (L2 + H2),  // pitch (about cross-track/y axis)
                     c * (L2 + W2)); // yaw   (about vertical/z axis)
}

TippingLimit Vehicle::tippingLimit(float curvature, float cant) const {
    // Overturn about the outer rail: the horizontal lateral acceleration a where
    // its moment about the pivot (through the CoM height h) overcomes the weight's
    // restoring moment (through the half-gauge b), with the track canted by theta:
    //   (a cosθ − g sinθ)·h = (a sinθ + g cosθ)·b
    //   a = g (b cosθ + h sinθ) / (h cosθ − b sinθ)
    const float b = 0.5f * kGauge;
    const float h = comHeight();
    const float ct = std::cos(cant), st = std::sin(cant);
    const float denom = h * ct - b * st;
    const float inf = std::numeric_limits<float>::infinity();
    const float aLim = (denom > 1e-4f) ? kG * (b * ct + h * st) / denom : inf;

    const float k = std::abs(curvature);
    const float v = (k > 1e-6f && std::isfinite(aLim)) ? std::sqrt(aLim / k) : inf;
    return {aLim, v};
}
