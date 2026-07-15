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
      name_(spec.name),
      v_(initialSpeed) {}

namespace {
VehicleFrame frameOf(const TrackPose& p) {
    return {p.pos, p.right, p.tangent, p.up};
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

VehicleFrame Vehicle::bodyFrame() const {
    if (state_ != VehicleState::OnRail)
        return {pos_, fRight_, fTangent_, fUp_};
    const float half = 0.5f * wheelbase_;
    if (half < 1e-3f) return frameOf(path_->poseAt(s_)); // single axle
    // Bogie: rigid frame spanning the two axle contact points (chords the curve).
    const TrackPose pr = path_->poseAt(s_ - half);
    const TrackPose pf = path_->poseAt(s_ + half);
    VehicleFrame f;
    f.pos = (pr.pos + pf.pos) * 0.5f;
    const glm::vec3 chord = pf.pos - pr.pos;
    const float cl = glm::length(chord);
    f.tangent = (cl > 1e-6f) ? chord / cl : path_->poseAt(s_).tangent;
    glm::vec3 up = glm::normalize(pr.up + pf.up);
    f.right = glm::normalize(glm::cross(up, f.tangent));
    f.up = glm::normalize(glm::cross(f.tangent, f.right));
    return f;
}

std::vector<VehicleFrame> Vehicle::axleFrames() const {
    const float half = 0.5f * wheelbase_;
    if (half < 1e-3f) return {frame()}; // single axle
    if (state_ == VehicleState::OnRail)
        return {frameOf(path_->poseAt(s_ - half)),
                frameOf(path_->poseAt(s_ + half))};
    // Derailed: axles offset from the frozen body along its tangent.
    const VehicleFrame b{pos_, fRight_, fTangent_, fUp_};
    return {{pos_ - fTangent_ * half, b.right, b.tangent, b.up},
            {pos_ + fTangent_ * half, b.right, b.tangent, b.up}};
}

void Vehicle::update(float dt, float pushInput) {
    if (state_ == VehicleState::OnRail) {
        const VehicleFrame bf = bodyFrame();
        // Driving acceleration: gravity along the track (downhill in +s is
        // positive) plus the hand push (a force, so a = F/m).
        const float aGrav = -kG * bf.tangent.z;
        const float aPush = pushInput * kPushForce / mass_;
        v_ += (aGrav + aPush) * dt;

        // Davis running resistance opposes motion, capped so it can't reverse v_
        // (this also holds the axle on grades gentler than the resistance).
        const float rollDecel = rollingResistance(v_) / mass_;
        const float roll = std::min(rollDecel * dt, std::abs(v_));
        v_ -= std::copysign(roll, v_);

        s_ += v_ * dt;

        // Derail when the leading or trailing axle passes an end of the track.
        const float L = path_->length();
        const float half = 0.5f * wheelbase_;
        if (s_ - half < 0.0f || s_ + half > L) {
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
