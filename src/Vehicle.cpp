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

#include "SwitchNetwork.h"
#include "TrackPath.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
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
constexpr float kFullServiceDecel = 1.3f; // deceleration at full service (m/s^2)
constexpr float kAdhesionMu = 0.20f;     // wheel/rail grip cap on brake force
constexpr float kMRSafetyTrip = 6.0f;    // low reservoir -> automatic emergency
constexpr float kMRSafetyReset = 6.5f;   // safety clears once recharged above this

// Diesel engines (Cummins N14E-R): crank up to a fixed idle, rev up under power.
constexpr float kIdleRpm = 700.0f;       // low idle
constexpr float kStartRate = kIdleRpm / 4.0f; // rpm/s while cranking (~4 s to idle)
constexpr float kStopRate = kIdleRpm / 3.0f;  // rpm/s while spinning down (~3 s)
constexpr float kCompLoadDrop = 30.0f;   // idle rpm droop while the compressor pumps

// Transmission: diesel-hydraulic — a torque converter for launch feeding a 5-speed
// automatic gearbox (per the real Di 93). Ratios are overall (gearbox * final drive)
// engine-rev to wheel-rev, tuned so the engine sits ~950-1500 rpm under load and top
// gear pulls to ~1500 rpm near 140 km/h on the level.
constexpr float kRatedPowerW = 612000.0f;   // 2 x 306 kW combined
constexpr float kWheelRadius = 0.42f;       // m (0.84 m driving wheels)
constexpr float kGovernedRpm = Vehicle::kMaxRpm; // full-power / governed speed
constexpr float kGearRatio[5] = {4.57f, 3.57f, 2.79f, 2.18f, 1.70f};
constexpr float kTCStall = 2.0f;         // torque-converter stall torque ratio
constexpr float kTCCouple = 0.85f;       // speed ratio at which it couples (TR -> 1)
constexpr float kUpshiftRpm = 1450.0f;   // auto upshift threshold (on geared speed)
constexpr float kDownshiftRpm = 950.0f;  // auto downshift threshold (on geared speed)
constexpr float kConvFloorRpm = 1150.0f; // converter stall speed at full throttle
constexpr float kShiftDwell = 0.4f;      // s of cut traction across an upshift
constexpr float kTractionMu = 0.33f;     // wheel/rail adhesion under power
constexpr float kDrivenFrac = 0.67f;     // fraction of weight on driven axles (~4/6)
constexpr float kEta = 0.9f;             // driveline efficiency
constexpr float kRevSpeedCap = 11.0f;    // m/s (~40 km/h) reverse power cut
constexpr float kRpmSlew = 900.0f;       // rpm/s engine speed rate limit under power
constexpr float kRpmToRad = 2.0f * 3.14159265358979f / 60.0f; // rev/min -> rad/s

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
      bcPres_(kBCEmergency),  // brakes start in emergency (held)
      engineCount_(spec.body == BodyClass93 ? 2 : 0) {} // one diesel per cab end

void Vehicle::attachNetwork(const std::vector<TrackPath>* paths, SwitchNetwork* net) {
    paths_ = paths;
    net_ = net;
    pathIdx_ = (paths_ && path_) ? static_cast<int>(path_ - paths_->data()) : -1;
}

bool Vehicle::consumeSwitchChanged() {
    const bool c = switchChanged_;
    switchChanged_ = false;
    return c;
}

void Vehicle::derailFreeze() {
    const VehicleFrame e = bodyFrame();
    pos_ = e.pos;
    fRight_ = e.right;
    fTangent_ = e.tangent;
    fUp_ = e.up;
    vel_ = v_ * e.tangent;
    state_ = VehicleState::Derailed;
}

void Vehicle::swapPath(int newIdx, float newS) {
    // Preserve the physical velocity vector across the join: if the new path's +s
    // tangent opposes the old one, flip v_ and orient_ together so speed/direction and
    // the driver's controls stay physically continuous.
    const glm::vec3 tOld = path_->poseAt(s_).tangent;
    const glm::vec3 tNew = (*paths_)[newIdx].poseAt(newS).tangent;
    const float g = glm::dot(tOld, tNew) >= 0.0f ? 1.0f : -1.0f;
    v_ *= g;
    orient_ *= g;
    path_ = &(*paths_)[newIdx];
    pathIdx_ = newIdx;
    s_ = newS;
}

bool Vehicle::crossTurnouts(float sBefore) {
    const std::vector<Turnout>& tos = net_->turnouts();
    if (sBefore == s_) return false;
    const int dirS = (s_ > sBefore) ? 1 : -1; // travel direction along +s this step
    auto crossed = [&](float x) { return (x - sBefore) * (x - s_) <= 0.0f; };
    // A swap lands the train exactly on the turnout's junction s; skip re-evaluating
    // that same turnout for the one following frame, else the boundary re-triggers as a
    // spurious trailing move (which would wrongly break the switch we just merged over).
    const int skip = skipTurnout_;
    skipTurnout_ = -1;

    for (int i = 0; i < static_cast<int>(tos.size()); ++i) {
        if (i == skip) continue;
        const Turnout& to = tos[i];
        if (to.mainPath < 0 || to.sidingPath < 0) continue;
        const SwitchState st = net_->state(i);

        if (pathIdx_ == to.mainPath && crossed(to.sMain)) {
            const bool facing = dirS == to.facingS;
            if (facing) {
                if (st == SwitchState::Diverging) { skipTurnout_ = i; swapPath(to.sidingPath, to.sSiding); return false; }
                if (st == SwitchState::Broken) { derailFreeze(); return true; }
                // Straight: run straight through on the main.
            } else { // trailing from the straight branch
                if (st == SwitchState::Diverging) { // set against us: force + break it
                    net_->setState(i, SwitchState::Broken);
                    switchChanged_ = true;
                }
                // Straight / Broken: continue onto the common track.
            }
            return false;
        }
        if (pathIdx_ == to.sidingPath && crossed(to.sSiding)) {
            // Trailing merge from the diverging branch onto the main — only when
            // actually moving toward the branch's end at the turnout.
            const bool towardEnd = (to.sSiding < 1.0f) ? (dirS < 0) : (dirS > 0);
            if (towardEnd) {
                if (st == SwitchState::Straight) { // set against us: force + break it
                    net_->setState(i, SwitchState::Broken);
                    switchChanged_ = true;
                }
                skipTurnout_ = i;
                swapPath(to.mainPath, to.sMain);
                return false;
            }
        }
    }
    return false;
}

void Vehicle::setBrakeNotch(int cab, int notch) {
    if (cab == 0 || cab == 1) brakeNotch_[cab] = std::clamp(notch, 0, kEmergencyNotch);
}

int Vehicle::brakeNotch(int cab) const {
    return (cab == 0 || cab == 1) ? brakeNotch_[cab] : 0;
}

void Vehicle::setPowerNotch(int cab, int notch) {
    if (cab == 0 || cab == 1) powerNotch_[cab] = std::clamp(notch, 0, kMaxPowerNotch);
}

int Vehicle::powerNotch(int cab) const {
    return (cab == 0 || cab == 1) ? powerNotch_[cab] : 0;
}

void Vehicle::moveHandle(int cab, int dir) {
    if (cab != 0 && cab != 1) return;
    if (dir < 0) { // toward power: bleed the brake off first, then raise power
        if (brakeNotch_[cab] > 0) --brakeNotch_[cab];
        else powerNotch_[cab] = std::min(kMaxPowerNotch, powerNotch_[cab] + 1);
    } else if (dir > 0) { // toward brake: bleed power off first, then raise brake
        if (powerNotch_[cab] > 0) --powerNotch_[cab];
        else brakeNotch_[cab] = std::min(kEmergencyNotch, brakeNotch_[cab] + 1);
    }
}

int Vehicle::handlePosition(int cab) const {
    if (powerNotch(cab) > 0) return -powerNotch(cab); // power side is negative
    return brakeNotch(cab);                           // brake side positive, 0 = N
}

const char* Vehicle::handleName(int cab) const {
    static char buf[8];
    const int p = handlePosition(cab);
    if (p == 0) return "N";
    if (p < 0) { std::snprintf(buf, sizeof(buf), "P%d", -p); return buf; }
    if (p >= kEmergencyNotch) return "EMERG";
    std::snprintf(buf, sizeof(buf), "B%d", p);
    return buf;
}

void Vehicle::setReverser(int cab, int dir) {
    if (cab == 0 || cab == 1) reverser_[cab] = std::clamp(dir, -1, 1);
}

int Vehicle::reverser(int cab) const {
    return (cab == 0 || cab == 1) ? reverser_[cab] : 0;
}

const char* Vehicle::reverserName(int cab) const {
    const int r = reverser(cab);
    return r > 0 ? "F" : (r < 0 ? "R" : "N");
}

int Vehicle::activeCab() const {
    if (bodyStyle_ != BodyClass93) return -1;
    int active = -1, count = 0;
    for (int c = 0; c < 2; ++c)
        if (reverser_[c] != 0) { active = c; ++count; }
    return count == 1 ? active : -1; // -1 when both Neutral or both in gear
}

int Vehicle::effectiveNotch() const {
    if (safetyBrake_) return kEmergencyNotch; // low reservoir overrides everything
    if (bodyStyle_ == BodyClass93) {
        const int a = activeCab();
        return a >= 0 ? brakeNotch_[a] : kEmergencyNotch; // interlock: else emergency
    }
    return brakeNotch_[0]; // non-cab vehicles: a single handle, no interlock
}

bool Vehicle::interlockEmergency() const {
    return bodyStyle_ == BodyClass93 && !safetyBrake_ && activeCab() < 0;
}

int Vehicle::effectivePowerNotch() const {
    if (safetyBrake_ || bodyStyle_ != BodyClass93) return 0;
    const int a = activeCab();
    return a >= 0 ? powerNotch_[a] : 0; // no power unless exactly one cab is in gear
}

float Vehicle::tractionDemand() const {
    const int a = activeCab();
    if (a < 0) return 0.0f;
    // The two cabs face opposite ways (cab 0 toward -s, cab 1 toward +s, matching
    // `so` in the cab mesh / drivercam), so a given reverser direction drives the
    // train opposite ways from each end. Fold that into the sign here.
    const float cabSign = (a == 0) ? -1.0f : 1.0f;
    // orient_ keeps a cab's physical forward mapped to the right track direction after
    // a path swap at a switch (see swapPath).
    return orient_ * cabSign *
           static_cast<float>(reverser_[a] * effectivePowerNotch()) /
           static_cast<float>(kMaxPowerNotch);
}

void Vehicle::toggleEngines() {
    if (engineCount_ > 0) engineOn_ = !engineOn_; // both start / stop together
}

float Vehicle::engineRpm(int i) const {
    return (i >= 0 && i < engineCount_) ? engineRpm_[i] : 0.0f;
}

EngineState Vehicle::engineState(int i) const {
    if (i < 0 || i >= engineCount_ || engineRpm_[i] <= 0.0f) return EngineState::Off;
    if (engineRpm_[i] >= kIdleRpm * 0.99f) return EngineState::Running;
    return engineOn_ ? EngineState::Starting : EngineState::Stopping;
}

bool Vehicle::enginesRunning() const {
    if (engineCount_ == 0) return false;
    for (int i = 0; i < engineCount_; ++i)
        if (engineRpm_[i] < kIdleRpm * 0.99f) return false;
    return true;
}

const char* Vehicle::brakeNotchName(int cab) const {
    static const char* kNames[] = {"REL", "B1", "B2", "B3", "B4", "EMERG"};
    return kNames[std::clamp(brakeNotch(cab), 0, kEmergencyNotch)];
}

const char* Vehicle::effectiveBrakeName() const {
    static const char* kNames[] = {"REL", "B1", "B2", "B3", "B4", "EMERG"};
    return kNames[std::clamp(effectiveNotch(), 0, kEmergencyNotch)];
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

TrackPose Vehicle::walkPose(float bodyOffset) const {
    const float d = static_cast<float>(orient_);
    if (!net_ || !paths_ || pathIdx_ < 0) { // no network: straight sample on path_
        TrackPose p = path_->poseAt(s_ + d * bodyOffset);
        p.tangent = d * p.tangent;
        p.right = d * p.right;
        return p;
    }

    constexpr float kTol = 0.05f;              // "at the junction" slack (m)
    const std::vector<Turnout>& tos = net_->turnouts();
    int cp = pathIdx_;                          // current path index
    float cs = s_;                              // current arc-length on it
    int nose = orient_;                         // path-s direction toward the nose
    const int walkSign = bodyOffset >= 0.0f ? 1 : -1; // +1 toward nose, -1 toward tail
    float remaining = std::abs(bodyOffset);
    int prevCross = -1;                         // don't immediately re-cross a turnout

    for (int guard = 0; guard < 64 && remaining > 1e-4f; ++guard) {
        const TrackPath& P = (*paths_)[cp];
        const int arcDir = walkSign * nose;               // path-s direction we move in
        const float distToEnd = arcDir > 0 ? (P.length() - cs) : cs;

        // Nearest turnout junction reached in the travel direction, with its target.
        float bestDist = std::min(remaining, distToEnd);
        int toPath = -1, toTurn = -1;
        float toS = 0.0f;
        for (int i = 0; i < static_cast<int>(tos.size()); ++i) {
            if (i == prevCross) continue;
            const Turnout& to = tos[i];
            if (to.mainPath < 0 || to.sidingPath < 0) continue;
            const SwitchState st = net_->state(i);
            // main -> siding: walking toe->frog on a diverging switch.
            if (cp == to.mainPath && st == SwitchState::Diverging && arcDir == to.facingS) {
                const float dj = (to.sMain - cs) * static_cast<float>(arcDir);
                const float adv = std::max(dj, 0.0f);
                if (dj > -kTol && adv <= bestDist) {
                    bestDist = adv; toPath = to.sidingPath; toS = to.sSiding; toTurn = i;
                }
            }
            // siding -> main: reaching the junction end while heading toward it.
            if (cp == to.sidingPath) {
                const bool towardEnd =
                    static_cast<float>(arcDir) * (to.sSiding < 1.0f ? -1.0f : 1.0f) > 0.0f;
                const float dj = (to.sSiding - cs) * static_cast<float>(arcDir);
                const float adv = std::max(dj, 0.0f);
                if (towardEnd && dj > -kTol && adv <= bestDist) {
                    bestDist = adv; toPath = to.mainPath; toS = to.sMain; toTurn = i;
                }
            }
        }

        if (toTurn < 0) { // no junction ahead: advance (clamped at a dead-end)
            cs += arcDir * std::min(remaining, distToEnd);
            break;
        }
        // Advance to the junction and cross onto the connected path, carrying the
        // world travel direction so the nose direction stays continuous.
        cs += arcDir * bestDist;
        remaining -= bestDist;
        const glm::vec3 fwdWorld = static_cast<float>(arcDir) * P.poseAt(cs).tangent;
        const int contArcDir =
            glm::dot(fwdWorld, (*paths_)[toPath].poseAt(toS).tangent) >= 0.0f ? 1 : -1;
        cp = toPath;
        cs = toS;
        nose = walkSign * contArcDir;
        prevCross = toTurn;
    }

    TrackPose p = (*paths_)[cp].poseAt(cs);
    const float nf = static_cast<float>(nose); // orient tangent/right toward the nose
    p.tangent = nf * p.tangent;
    p.right = nf * p.right;
    return p;
}

VehicleFrame Vehicle::railFrame(float o, float h) const {
    const TrackPose po = walkPose(o);
    if (h < 1e-3f) return {po.pos, po.right, po.tangent, po.up}; // single point
    // Chord the part's rear (o-h) and front (o+h) body ends; each is placed on the
    // rail it is physically on, so the part bends where the network branches.
    VehicleFrame f = chordFrame(walkPose(o - h), walkPose(o + h), po.tangent);
    f.pos = po.pos; // exact centre (sections rely on this)
    return f;
}

VehicleFrame Vehicle::bodyFrame() const {
    if (state_ != VehicleState::OnRail)
        return {pos_, fRight_, fTangent_, fUp_};
    return railFrame(0.0f, supportHalf()); // single axle: supportHalf()==0 -> a point
}

std::vector<VehicleFrame> Vehicle::bogieFrames() const {
    const float wb = 0.5f * wheelbase_;
    std::vector<VehicleFrame> out;
    for (float c : bogieCentres()) {
        if (state_ == VehicleState::OnRail)
            out.push_back(railFrame(c, wb));
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
            // shared middle bogie); each bogie sits on the rail it is on, so a
            // section spanning a turnout bends at its articulation.
            const TrackPose pm = walkPose(mid);
            VehicleFrame f =
                chordFrame(walkPose(centres[i]), walkPose(centres[i + 1]), pm.tangent);
            f.pos = pm.pos;
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
            out.push_back(railFrame(off, 0.0f));
        else // derailed: axles offset from the frozen body along its tangent
            out.push_back({pos_ + fTangent_ * off, fRight_, fTangent_, fUp_});
    }
    return out;
}

void Vehicle::updateTraction(float demandSigned, float demand, bool powering,
                             bool reverse, float dt) {
    tractiveEffort_ = 0.0f;
    if (shiftTimer_ > 0.0f) shiftTimer_ = std::max(0.0f, shiftTimer_ - dt);

    const float sp = std::abs(v_);
    // Engine rev/min if the converter were locked in the current gear (the geared,
    // turbine-side speed). The automatic shifts on this, not the engine speed.
    const float rpmLock = sp / kWheelRadius * kGearRatio[gear_ - 1] / kRpmToRad;

    // Automatic gear selection (runs whether or not we are powering, so the gear
    // always matches road speed). An upshift briefly cuts traction so the revs dip.
    if (shiftTimer_ <= 0.0f) {
        if (gear_ < 5 && rpmLock >= kUpshiftRpm) { ++gear_; shiftTimer_ = kShiftDwell; }
        else if (gear_ > 1 && rpmLock <= kDownshiftRpm) { --gear_; }
    }

    if (!powering) return; // coasting: engines idle (handled in update), no traction

    // Engine speed: the converter keeps the revs up off a throttle-set floor while it
    // slips at low road speed (the launch flare), then tracks the geared speed as it
    // couples; each upshift drops the geared speed, so the revs step down.
    const float floorRpm = kIdleRpm + demand * (kConvFloorRpm - kIdleRpm);
    const float rpmWant = std::clamp(std::max(rpmLock, floorRpm), kIdleRpm, kGovernedRpm);
    float rpm = engineRpm_[0];
    rpm += std::clamp(rpmWant - rpm, -kRpmSlew * dt, kRpmSlew * dt);
    rpm = std::clamp(rpm, kIdleRpm, kGovernedRpm);
    for (int i = 0; i < engineCount_; ++i) engineRpm_[i] = rpm;

    // Torque converter: speed ratio -> torque ratio (stall multiplication at low
    // speed easing to 1:1 lock-up once coupled).
    const float sr = std::clamp(rpmLock / std::max(rpm, 1.0f), 0.0f, 1.0f);
    const float tr =
        sr >= kTCCouple ? 1.0f : kTCStall + (1.0f - kTCStall) * (sr / kTCCouple);

    // Tractive effort: the lesser of the geared/converter torque limit, the
    // constant-power hyperbola, and the wheel/rail adhesion cap. Cut across a shift.
    const float we = rpm * kRpmToRad;                  // engine rad/s
    const float pAvail = demand * kRatedPowerW * kEta; // available power (W)
    const float te = pAvail / std::max(we, 1.0f);      // engine torque (N*m)
    const float teGeared = te * tr * kGearRatio[gear_ - 1] / kWheelRadius;
    const float tePower = pAvail / std::max(sp, 1.0f); // hyperbola (limits at speed)
    const float teAdh = kTractionMu * kDrivenFrac * mass_ * kG;
    float TE = std::min(std::min(teGeared, tePower), teAdh);
    if (shiftTimer_ > 0.0f) TE = 0.0f;                 // unloaded across an upshift
    const float dir = demandSigned >= 0.0f ? 1.0f : -1.0f; // track direction of travel
    if (reverse && sp > kRevSpeedCap) TE = 0.0f;       // reverse stays a shunting speed

    tractiveEffort_ = dir * TE;
    v_ += tractiveEffort_ / mass_ * dt;
}

void Vehicle::update(float dt, float pushInput) {
    // Traction demand (signed by the reverser). Under power the engine speed is set
    // by the transmission (below); otherwise the engines crank to / hold idle.
    const float demandSigned = tractionDemand();
    const float demand = std::abs(demandSigned);
    const int activeC = activeCab();
    const bool reverse = activeC >= 0 && reverser_[activeC] < 0; // reverser in R
    const bool powering =
        engineOn_ && demand > 0.0f && enginesRunning() && state_ == VehicleState::OnRail;
    // Off the rails there are no friction surfaces doing anything; only the on-rail
    // branch below sets this, so clear it or a derailed vehicle reports the last
    // brake force it had for ever.
    brakeForce_ = 0.0f;

    // Engines crank up to / spin down from idle, independent of motion; a running
    // compressor loads its engine down a little (compActive_ is last frame's value).
    if (!powering) {
        const float rpmTarget =
            engineOn_ ? kIdleRpm - (compActive_ ? kCompLoadDrop : 0.0f) : 0.0f;
        for (int i = 0; i < engineCount_; ++i) {
            if (engineRpm_[i] < rpmTarget)
                engineRpm_[i] = std::min(rpmTarget, engineRpm_[i] + kStartRate * dt);
            else if (engineRpm_[i] > rpmTarget)
                engineRpm_[i] = std::max(rpmTarget, engineRpm_[i] - kStopRate * dt);
        }
    }

    if (state_ == VehicleState::OnRail) {
        // Driving acceleration: gravity along the track plus the hand push (a force,
        // so a = F/m). Gravity acts on v_, which is velocity in the path's +s frame, so
        // it uses the raw path tangent: bodyFrame()'s tangent carries orient_, which after
        // a divert flips the vehicle (orient_ = -1) would invert gravity — the vehicle
        // speeding up uphill and slowing downhill.
        const float aGrav = -kG * path_->poseAt(s_).tangent.z;
        const float aPush = orient_ * pushInput * kPushForce / mass_;
        v_ += (aGrav + aPush) * dt;

        // Traction: a torque converter feeding a 5-speed automatic gearbox. Only the
        // in-gear cab powers, in its reverser direction, engines running (`powering`).
        updateTraction(demandSigned, demand, powering, reverse, dt);

        // Low main-reservoir safety: if the reservoir falls below the trip
        // pressure the brakes go to full emergency regardless of the handle,
        // latched until the reservoir recovers above the reset pressure.
        if (mrPres_ < kMRSafetyTrip) safetyBrake_ = true;
        else if (mrPres_ >= kMRSafetyReset) safetyBrake_ = false;
        const int effNotch = effectiveNotch();

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
        // Engine-driven compressors recharge only while the engines idle, scaled by
        // how many are running. With the engines off the reservoir just draws down.
        // "Running" uses a threshold below the compressor load droop so the droop
        // can't make it cut out and hunt.
        int running = 0;
        for (int i = 0; i < engineCount_; ++i)
            if (engineRpm_[i] >= kIdleRpm * 0.9f) ++running;
        if (running > 0 && compOn_)
            mrPres_ += kCompRate * (static_cast<float>(running) / engineCount_) * dt;
        mrPres_ = std::clamp(mrPres_, 0.0f, kMRCapacity);
        // Governor decision for the next step, from the resulting pressure: cut out
        // at capacity, cut in below kMRCutIn. (Deciding after the recharge+clamp is
        // what latches it off at 8 bar; deciding before let the tiny leak keep the
        // pressure hovering just under 8 so it never cut out.)
        if (running > 0) {
            if (mrPres_ >= kMRCapacity) compOn_ = false;
            else if (mrPres_ < kMRCutIn) compOn_ = true;
        }
        compActive_ = (running > 0 && compOn_); // drives the sound + engine load
        bcPres_ = std::max(0.0f, bcPres_);
        bcRate_ = (dt > 1e-6f) ? (bcPres_ - bcBefore) / dt : 0.0f; // airflow, for sound

        // Brake force (capped by wheel/rail adhesion) and Davis running resistance
        // both oppose motion, capped so they can't reverse v_ (this also holds the
        // vehicle at rest, up to the grade the brakes can hold).
        const float brake = std::min((bcPres_ / kBCFullService) * mass_ * kFullServiceDecel,
                                     kAdhesionMu * mass_ * kG);
        brakeForce_ = brake; // what the friction surfaces are actually doing (the sound)
        const float resistDecel = brake / mass_ + rollingResistance(v_) / mass_;
        const float resist = std::min(resistDecel * dt, std::abs(v_));
        v_ -= std::copysign(resist, v_);

        const float sBefore = s_;
        s_ += v_ * dt;

        // Turnouts first: a divert/merge swaps the path (so the end-of-path check
        // below sees the new one); a facing move into a broken switch derails here.
        if (net_ && crossTurnouts(sBefore)) return;

        // Derail when the leading or trailing axle passes an end of the track — but
        // only at a genuine dead-end. A path end that a turnout connects to (a siding
        // end at the switch) is a junction: the body legitimately straddles it while
        // diverting/merging, and a reversing train is handed back to the main by
        // crossTurnouts at the body-centre crossing, so it never truly runs off there.
        const float L = path_->length();
        float outerHalf = 0.0f;
        for (float o : axleOffsets()) outerHalf = std::max(outerHalf, std::abs(o));
        bool connFront = false, connBack = false;
        if (net_)
            for (const Turnout& to : net_->turnouts())
                if (to.sidingPath == pathIdx_)
                    (to.sSiding < 1.0f ? connFront : connBack) = true;
        const bool offFront = s_ - outerHalf < 0.0f;
        const bool offBack = s_ + outerHalf > L;
        if ((offFront && !connFront) || (offBack && !connBack)) {
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
    // theta is the cant into the curve. TrackPose::cant carries the sign of the curvature
    // instead, so that a left-hand and a right-hand curve of the same radius describe the
    // same track; take the magnitude, or half the curves would come out adversely canted.
    const float theta = std::abs(cant);
    const float ct = std::cos(theta), st = std::sin(theta);
    const float denom = h * ct - b * st;
    const float inf = std::numeric_limits<float>::infinity();
    const float aLim = (denom > 1e-4f) ? kG * (b * ct + h * st) / denom : inf;

    const float k = std::abs(curvature);
    const float v = (k > 1e-6f && std::isfinite(aLim)) ? std::sqrt(aLim / k) : inf;
    return {aLim, v};
}
