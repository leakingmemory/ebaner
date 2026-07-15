#include "Vehicle.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr float kG = 9.81f;         // m/s^2
constexpr float kGauge = 1.435f;    // standard track gauge (tipping pivot width)
constexpr float kFrictionMu = 0.6f; // derailed ground friction ("digging in")
constexpr float kStopSpeed = 0.1f;  // m/s below which a derailed vehicle stops
} // namespace

Vehicle::Vehicle(const TrackPath* path, float s, float massKg, float length,
                 float width, float height, float initialSpeed)
    : path_(path),
      s_(s),
      mass_(massKg),
      length_(length),
      width_(width),
      height_(height),
      v_(initialSpeed) {}

TrackPose Vehicle::pose() const { return path_->poseAt(s_); }

float Vehicle::speed() const {
    return (state_ == VehicleState::OnRail) ? std::abs(v_) : glm::length(vel_);
}

VehicleFrame Vehicle::frame() const {
    if (state_ == VehicleState::OnRail) {
        const TrackPose p = path_->poseAt(s_);
        return {p.pos, p.right, p.tangent, p.up};
    }
    return {pos_, fRight_, fTangent_, fUp_};
}

void Vehicle::update(float dt) {
    if (state_ == VehicleState::OnRail) {
        const TrackPose p = path_->poseAt(s_);
        // Along-track gravity acceleration (downhill in +s direction is positive).
        const float a = -kG * p.tangent.z;
        v_ += a * dt;
        s_ += v_ * dt;

        const float L = path_->length();
        if (s_ < 0.0f || s_ > L) { // ran off an end -> derail
            const TrackPose e = path_->poseAt(s_ < 0.0f ? 0.0f : L);
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
