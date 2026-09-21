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
constexpr float kG = Vehicle::kGravity;
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

// Air brake (all pressures in bar).
//
// An automatic air brake, which is to say a failsafe one: the brake pipe runs the length
// of the train charged to kBPRelease, and it is *losing* pipe pressure that applies the
// brakes. Each brake unit - one per bogie - keeps its own auxiliary reservoir and its own
// distributor, and when the pipe falls the distributor puts its own stored air into its
// own cylinder. Nothing has to be commanded and no wire has to be intact, which is why a
// train whose pipe bursts stops instead of running away.
//
// The Class 93 works that pipe electrically. The handle does not make a reduction at the
// front and wait for it to travel; it tells an EP valve on every unit to vent or recharge
// its own length of pipe, all in step. That is what makes the brake quick, and it changes
// nothing about what the distributors then do.
constexpr float kMRCapacity = 8.0f;      // main reservoir full / compressor cut-out
constexpr float kMRCutIn = 6.5f;         // compressor cut-in
constexpr float kBPRelease = 5.0f;       // brake pipe fully charged, brakes off
constexpr float kBPMinReduction = 0.4f;  // the smallest reduction that does anything
constexpr float kBPFullService = 3.5f;   // full service: a 1.5 bar reduction
constexpr float kBCMax = 3.8f;           // cylinder at full service and at emergency
// Bar of cylinder per bar of pipe drop, so that a full-service reduction gives kBCMax.
// Emergency needs no separate case: the distributor simply sees a 5 bar drop instead of
// 1.5, so it applies sooner, faster and to the stop, which is what emergency is.
constexpr float kBCPerBPDrop = kBCMax / (kBPRelease - kBPFullService);
constexpr float kAuxCapacity = 5.0f;     // auxiliary reservoir, charged from the pipe
constexpr float kBPVentRate = 2.5f;      // EP valve venting the pipe (bar/s)
constexpr float kBPEmergVentRate = 8.0f; // emergency: the pipe dumped wide open (bar/s)
constexpr float kBPChargeRate = 1.6f;    // EP valve recharging from the main reservoir
// The length of pipe the vent and charge rates above are quoted for. A vehicle's pipe
// volume goes with its length, and the rate at which a fixed orifice can empty or fill it
// goes the other way, so everything scales by this over the vehicle's own length. Set to
// the Class 93's, which is what those rates were measured on, so nothing about the railcar
// moves. It is also why the brake on a hauled train is slow twice over: more vehicles to
// travel through, and each one of them bigger than the locomotive at the front.
constexpr float kPipeRefLength = 41.5f;  // m, a Class 93 set
constexpr float kMRPerBP = 0.05f;        // MR bar spent per bar of pipe charged
// And per bar the independent brake puts into a cylinder. Small: it is one vehicle's
// cylinders off the whole reservoir, which is exactly why a locomotive can stand on its
// own brake for as long as it likes where the automatic would have run its auxiliaries
// down long before.
constexpr float kMRPerBC = 0.012f;
// The auxiliary refills slowly - far more slowly than the cylinder empties. That
// asymmetry is the whole of brake fade: a driver who makes and releases applications
// faster than this gets less air each time, which is what running away down a long
// descent actually is.
constexpr float kAuxChargeRate = 0.35f;  // auxiliary reservoir refilling from the pipe
constexpr float kBCApplyRate = 2.5f;     // cylinder filling from the auxiliary (bar/s)
// A distributor has an emergency portion: below a full-service reduction it opens a
// wider port and fills the cylinder faster. Without it emergency and full service reach
// the same pressure at the same speed and differ only in the pipe, which is not what
// pulling the handle to the stop feels like or does.
constexpr float kBCEmergApplyRate = 6.0f;
constexpr float kBCReleaseRate = 1.6f;   // cylinder exhausting to atmosphere (bar/s)
// What a distributor without EP behind it manages, as a fraction of the rates above.
constexpr float kPlainAirFill = 0.22f;
// Where a distributor decides this is an emergency and opens its own vent: falling, and
// already below a full service reduction - so a service application, which stops at 3.5,
// never trips it however deep the driver goes.
constexpr float kBPEmergSense = 2.9f;
// What counts as an emergency-rate fall, what counts as that fall having stopped, and how
// far the pipe must come back before the accelerator can act again.
constexpr float kBPEmergTripRate = 1.2f;  // bar/s
constexpr float kBPVentCloseRate = 0.5f;  // bar/s
constexpr float kBPVentRearm = 0.6f;      // bar above the sense level
// Auxiliary bar spent per bar of cylinder, which is the ratio of the two volumes. A real
// auxiliary is sized so that emptying it into the cylinder *equalises* at the full-
// application pressure - that is what sets kBCMax in the first place, and it is why a
// second application made before the pipe has recharged is weaker than the first. Derived
// rather than picked, because picking it silently caps how hard the brake can ever go on:
// at 0.55 an emergency reached 3.2 bar and looked like a tuning choice.
constexpr float kAuxPerBC = kAuxCapacity / kBCMax - 1.0f; // 0.316
constexpr float kCompRate = 0.20f;       // compressor recharge (bar/s)
constexpr float kMRLeak = 0.001f;        // reservoir leak (bar/s)
constexpr float kFullServiceDecel = 1.3f; // deceleration at full service (m/s^2)
constexpr float kAdhesionMu = 0.20f;     // wheel/rail grip cap on brake force
// The low-reservoir safety is there to catch a FAILED COMPRESSOR, and it has to sit far
// enough below the compressor's working range that ordinary work never reaches it.
// At 6.0, against a cut-in of 6.5, it did not: charging a brake pipe costs kMRPerBP a bar,
// so a 600 m train's 23 lengths of pipe want about 5.75 bar of reservoir and there were
// only 2.0 between full and the trip. The train could not be charged at all without
// tripping an emergency nobody had asked for - which is where "the emergency gets stuck
// on" began, before the accelerator then refused to let go of it.
constexpr float kMRSafetyTrip = 4.5f;    // low reservoir -> automatic emergency
constexpr float kMRSafetyReset = 5.5f;   // safety clears once recharged above this
// The feed valve's own cut-out, which sits BETWEEN those two and the compressor's cut-in:
// low enough that ordinary running never reaches it, high enough to close the feed long
// before the reservoir gets near the emergency trip. A bar of margin either side.
constexpr float kMRFeedCut = 5.5f;       // feed to the brake pipe closes below this
constexpr float kMRFeedResume = 6.0f;    // and opens again once recharged above this

// Diesel engines (Cummins N14E-R): crank up to a fixed idle, rev up under power.
constexpr float kIdleRpm = 700.0f;       // low idle
// Cranking and spinning down, as times rather than rates for the same reason as the
// governor above: a 315 rpm idle reached at a 700 rpm engine's rate arrives in half the
// time it should, and the start sounds hurried on exactly the machine with the big slow
// engine.
constexpr float kCrankTime = 4.0f;       // s, stopped -> idle
constexpr float kSpinDownTime = 3.0f;    // s, idle -> stopped
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
// Alternator, rectifier and six traction motors between the crankshaft and the rail.
// Lower than a mechanical driveline's, which is the price of having no gearbox.
constexpr float kElectricEta = 0.85f;
constexpr float kRevSpeedCap = 11.0f;    // m/s (~40 km/h) reverse power cut
// Where the electric brake fades away. Below the lower figure there is nothing at all, and
// it comes fully in by the upper one - about 5 km/h, by which speed the machines are
// turning fast enough to generate. A brake that did not fade would hold the train at rest.
constexpr float kDynFadeOut = 0.4f;      // m/s
constexpr float kDynFadeIn = 1.4f;       // m/s
// The governor's spool time is VehicleSpec::spoolTime, held as a time rather than an
// rpm/s so it means the same lag on any engine instead of a different one per rev range.
// It used to be one shared 900 rpm/s, which over the Di 4's 315-to-900 span put a 16-645
// at full speed in two thirds of a second.
// The load regulator. Excitation is wound on gradually, both so the engine is never
// asked for power it has not made yet and so the rail does not get a step of torque -
// which on a machine with this much starting effort is a wheelslip. It comes off much
// faster than it goes on, because dropping excitation is only switching it off.
// Slower than the governor can raise the revs, deliberately: the engine arrives at its
// notch speed first and the load goes on building for a couple of seconds after, which is
// what a diesel-electric sounds and feels like from the seat. Wind it on faster than the
// engine and the regulator stops being visible at all - the revs become the only lag.
constexpr float kLoadOnRate = 0.11f;     // fraction of full excitation per second
constexpr float kLoadOffRate = 0.80f;
constexpr float kRpmToRad = 2.0f * 3.14159265358979f / 60.0f; // rev/min -> rad/s

// What the handle asks the pipe to stand at (bar). Release leaves it fully charged;
// B1 is the smallest reduction that does anything at all and B4 is full service;
// emergency asks for nothing left. The handle commands the *pipe* - it does not command
// a cylinder pressure, and it cannot: what each cylinder does with a given reduction is
// its own distributor's business, and on a set whose auxiliaries are low it will be less.
float targetBP(int notch) {
    if (notch <= 0) return kBPRelease;
    if (notch >= Vehicle::kEmergencyNotch) return 0.0f;
    const float span = (kBPRelease - kBPFullService) - kBPMinReduction;
    return kBPRelease - (kBPMinReduction + span * static_cast<float>(notch - 1) / 3.0f);
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
      axlesPerBogie_(std::max(1, spec.axlesPerBogie)),
      drive_(spec.drive),
      powerW_(spec.powerW),
      wheelRadius_(spec.wheelRadius),
      drivenFrac_(spec.drivenFrac),
      startTE_(spec.startTE),
      idleRpm_(spec.idleRpm),
      governedRpm_(spec.governedRpm),
      spoolTime_(spec.spoolTime),
      cylinders_(spec.cylinders),
      twoStroke_(spec.twoStroke),
      engineVolume_(spec.engineVolume),
      engineRumble_(spec.engineRumble),
      engineBright_(spec.engineBright),
      controls_(spec.controls),
      cabs_(spec.cabs),
      epBrake_(spec.epBrake),
      independent_(spec.independentBrake),
      dynBrakeN_(spec.dynBrakeN),
      dynBrakeW_(spec.dynBrakeW),
      bodyStyle_(spec.body),
      name_(spec.name),
      physV_(initialSpeed),
      mrPres_(kMRCapacity), // reservoir starts at capacity
      engineCount_(std::min(spec.engines, 2)) { // the rpm array holds two
    // One brake unit per bogie - three on a Class 93. A vehicle with no bogies at all
    // still gets one, because something has to brake it and a bare wheelset has a shoe.
    brakes_.resize(static_cast<std::size_t>(std::max(1, bogieCount_)));
    // It starts as a vehicle stabled overnight does: reservoir full, pipe empty, and
    // therefore the brakes hard on, held by the auxiliaries the distributors were left
    // charged with. Releasing means charging the pipe, which is what the engines are for.
    for (BrakeUnit& b : brakes_) {
        b.aux = kAuxCapacity;
        b.ctrl = kBPRelease;
        b.bc = kBCMax;
    }
}

float Vehicle::bcPressure() const {
    if (brakes_.empty()) return 0.0f;
    float sum = 0.0f;
    for (const BrakeUnit& b : brakes_) sum += b.bc;
    return sum / static_cast<float>(brakes_.size());
}

float Vehicle::bcRate() const {
    // The loudest of them, not the mean: the sound of air moving is the sound of the
    // one that is moving most, and averaging would quieten a single cylinder venting.
    float r = 0.0f;
    for (const BrakeUnit& b : brakes_)
        if (std::abs(b.bcRate) > std::abs(r)) r = b.bcRate;
    return r;
}

void Vehicle::burstBrakePipe() {
    pipeCut_ = true;
    bp_ = 0.0f;
}

void Vehicle::closeBrakePipeCock() { pipeCut_ = false; }

void Vehicle::nudgeBrakePipe(float dBar) { bp_ = std::max(0.0f, bp_ + dBar); }

void Vehicle::beginPipeStep() {
    bpStepStart_ = bp_;
    pipeMarked_ = true;
}

void Vehicle::attachNetwork(const std::vector<TrackPath>* paths, SwitchNetwork* net) {
    paths_ = paths;
    net_ = net;
    pathIdx_ = (paths_ && path_) ? static_cast<int>(path_ - paths_->data()) : -1;
}

void Vehicle::ventReservoir(float toBar) {
    mrPres_ = std::clamp(std::min(mrPres_, toBar), 0.0f, kMRCapacity);
}

void Vehicle::placeAt(const std::vector<TrackPath>& paths, const TrackAnchor& a) {
    if (a.pathIdx < 0 || a.pathIdx >= static_cast<int>(paths.size())) return;
    path_ = &paths[a.pathIdx];
    pathIdx_ = a.pathIdx;
    s_ = a.s;
    orient_ = a.orient;
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
    vel_ = physV_ * e.tangent;
    state_ = VehicleState::Derailed;
}

void Vehicle::swapPath(int newIdx, float newS) {
    // Preserve the physical velocity vector across the join: if the new path's +s
    // tangent opposes the old one, flip orient_ so the driver's controls stay
    // physically continuous. The speed itself is held in the physical frame and so
    // does not flip - which way the new path counts its arc-length is the orientation's
    // business alone.
    const glm::vec3 tOld = path_->poseAt(s_).tangent;
    const glm::vec3 tNew = (*paths_)[newIdx].poseAt(newS).tangent;
    const float g = glm::dot(tOld, tNew) >= 0.0f ? 1.0f : -1.0f;
    orient_ *= static_cast<int>(g);
    path_ = &(*paths_)[newIdx];
    pathIdx_ = newIdx;
    s_ = newS;
}

void Vehicle::countRailImpacts(float sBefore) {
    if (!net_ || state_ != VehicleState::OnRail) return;
    if (s_ == sBefore) return;
    // Called before crossTurnouts, so sBefore and s_ are always on the same path: a
    // divert or a merge resets s_ onto another one, and comparing across that would be
    // meaningless. Nothing is lost by going first - a diverting train's leading axles
    // cross the point on the path it is leaving, and its trailing axles cross the same
    // point on the path it has joined, each on the frames where they actually do.
    const std::vector<float> offs = axleOffsets(); // hoisted: this is a per-frame path
    for (const Turnout& to : net_->turnouts()) {
        float sT = 0.0f;
        if (pathIdx_ == to.mainPath) sT = to.sMain;
        else if (pathIdx_ == to.sidingPath) sT = to.sSiding;
        else continue;
        for (const float o : offs) {
            // Half-open, so an axle sitting exactly on a turnout at the end of one frame
            // and the start of the next is one crossing rather than two.
            const float a0 = sBefore + o, a1 = s_ + o;
            if ((a0 < sT && sT <= a1) || (a1 < sT && sT <= a0)) ++railImpacts_;
        }
    }
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
    if (cab == 0 || cab == 1)
        powerNotch_[cab] = std::clamp(notch, hasDynamicBrake() ? -kMaxBrakeNotch : 0,
                                      kMaxPowerNotch);
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

const VehicleSpec* specNamed(const char* fragment) {
    if (fragment == nullptr) return nullptr;
    // A formation is addressed by its TRAIN name and a machine by its machine name, and
    // the two passes keep them apart. A formation keeps its locomotive's name - the Di 4
    // with carriages behind it is still a Di 4 - so a single pass in table order would
    // hand back whichever came first, and asking for "Di 4 (Hen" would give you a train
    // of eight the moment the list was reordered to put formations at the top.
    for (const VehicleSpec& v : kVehicleSpecs)
        if (v.formation != nullptr &&
            std::string(v.formation).find(fragment) != std::string::npos)
            return &v;
    for (const VehicleSpec& v : kVehicleSpecs)
        if (v.formation == nullptr && v.name != nullptr &&
            std::string(v.name).find(fragment) != std::string::npos)
            return &v;
    return nullptr;
}

void Vehicle::movePower(int cab, int dir) {
    if (cab != 0 && cab != 1) return;
    // Below neutral only where there is an electric brake to command. Nothing without
    // one can be wound into a range it does not have.
    powerNotch_[cab] = std::clamp(powerNotch_[cab] + dir,
                                  hasDynamicBrake() ? -kMaxBrakeNotch : 0, kMaxPowerNotch);
}

void Vehicle::moveIndependent(int cab, int dir) {
    if (cab != 0 && cab != 1 || !independent_) return;
    indNotch_[cab] = std::clamp(indNotch_[cab] + dir, 0, kMaxIndNotch);
}

int Vehicle::independentNotch(int cab) const {
    return (cab == 0 || cab == 1) ? indNotch_[cab] : 0;
}

void Vehicle::moveBrake(int cab, int dir) {
    if (cab != 0 && cab != 1) return;
    brakeNotch_[cab] = std::clamp(brakeNotch_[cab] + dir, 0, kEmergencyNotch);
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

int Vehicle::localActiveCab() const {
    if (engineCount_ == 0) return -1; // nothing to drive from
    int active = -1, count = 0;
    for (int c = 0; c < 2; ++c)
        if (reverser_[c] != 0) { active = c; ++count; }
    return count == 1 ? active : -1; // -1 when both Neutral or both in gear
}

int Vehicle::effectiveNotch() const {
    // This set's own safety device and the train-wide emergency line both override
    // whatever the link commanded. Either one alone is enough, which is the whole
    // point: a set can brake the train without the link having asked it to.
    if (safetyBrake_ || trainEmerg_) return kEmergencyNotch;
    return cmdNotch_;
}

void Vehicle::toggleEngines() {
    if (engineCount_ > 0) engineOn_ = !engineOn_; // both start / stop together
}

float Vehicle::engineRpm(int i) const {
    return (i >= 0 && i < engineCount_) ? engineRpm_[i] : 0.0f;
}

EngineState Vehicle::engineState(int i) const {
    if (i < 0 || i >= engineCount_ || engineRpm_[i] <= 0.0f) return EngineState::Off;
    if (engineRpm_[i] >= idleRpm_ * 0.99f) return EngineState::Running;
    return engineOn_ ? EngineState::Starting : EngineState::Stopping;
}

bool Vehicle::enginesRunning() const {
    if (engineCount_ == 0) return false;
    for (int i = 0; i < engineCount_; ++i)
        if (engineRpm_[i] < idleRpm_ * 0.99f) return false;
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

float Vehicle::dragResistance(float speed) const {
    const float C = 0.5f * kAirDensity * kDragCd * (width_ * height_);
    return C * speed * speed;
}

float Vehicle::speed() const {
    return (state_ == VehicleState::OnRail) ? std::abs(physV_) : glm::length(vel_);
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
    // Spread evenly across the bogie, whose wheelbase is measured outer axle to outer
    // axle: two axles are its two ends, and three put one on the centre. A Co'Co' has
    // three, and six axles have to come out of here and not four - the brake divides the
    // weight by them, the circuits measure the train by them, and the sound counts them
    // over the frogs.
    const int n = std::max(1, axlesPerBogie_);
    const float wb = 0.5f * wheelbase_;
    std::vector<float> out;
    out.reserve(static_cast<std::size_t>(n) * bogieCentres().size());
    for (const float c : bogieCentres())
        for (int i = 0; i < n; ++i) {
            const float t = n == 1 ? 0.0f
                                   : -1.0f + 2.0f * static_cast<float>(i) /
                                                 static_cast<float>(n - 1);
            out.push_back(c + t * wb);
        }
    return out;
}

bool Vehicle::walkTo(float bodyOffset, int& cp, float& cs, int& nose) const {
    if (!net_ || !paths_ || pathIdx_ < 0) { // no network: straight sample on path_
        cp = pathIdx_;
        cs = s_ + static_cast<float>(orient_) * bodyOffset;
        nose = orient_;
        return false;
    }
    cp = pathIdx_;
    cs = s_;
    nose = orient_;

    constexpr float kTol = 0.05f;              // "at the junction" slack (m)
    const std::vector<Turnout>& tos = net_->turnouts();
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

    return true;
}

// The same walk as walkTo, but keeping the road rather than only the destination.
//
// walkTo answers "where does this offset land"; a track circuit needs "what did the body
// pass over on the way", because a section is held by any part of a train standing in it.
// The loop is walkTo's - if one is ever changed the other has to follow - and the only
// difference is that each stretch is written out before the walk crosses onto the next
// path.
bool Vehicle::walkSpans(float bodyOffset, std::vector<PathSpan>& out) const {
    if (!paths_ || pathIdx_ < 0) return false;
    if (!net_) { // no network: one straight stretch on this path
        const float a = s_;
        const float b = s_ + static_cast<float>(orient_) * bodyOffset;
        out.push_back({pathIdx_, std::min(a, b), std::max(a, b)});
        return true;
    }
    int cp = pathIdx_;
    float cs = s_;
    int nose = orient_;

    constexpr float kTol = 0.05f;              // "at the junction" slack (m)
    const std::vector<Turnout>& tos = net_->turnouts();
    const int walkSign = bodyOffset >= 0.0f ? 1 : -1; // +1 toward nose, -1 toward tail
    float remaining = std::abs(bodyOffset);
    int prevCross = -1;                         // don't immediately re-cross a turnout

    auto keep = [&](int path, float from, float to) {
        if (from == to) return;
        out.push_back({path, std::min(from, to), std::max(from, to)});
    };

    for (int guard = 0; guard < 64 && remaining > 1e-4f; ++guard) {
        const TrackPath& P = (*paths_)[cp];
        const int arcDir = walkSign * nose;               // path-s direction we move in
        const float distToEnd = arcDir > 0 ? (P.length() - cs) : cs;

        float bestDist = std::min(remaining, distToEnd);
        int toPath = -1, toTurn = -1;
        float toS = 0.0f;
        for (int i = 0; i < static_cast<int>(tos.size()); ++i) {
            if (i == prevCross) continue;
            const Turnout& to = tos[i];
            if (to.mainPath < 0 || to.sidingPath < 0) continue;
            const SwitchState st = net_->state(i);
            if (cp == to.mainPath && st == SwitchState::Diverging && arcDir == to.facingS) {
                const float dj = (to.sMain - cs) * static_cast<float>(arcDir);
                const float adv = std::max(dj, 0.0f);
                if (dj > -kTol && adv <= bestDist) {
                    bestDist = adv; toPath = to.sidingPath; toS = to.sSiding; toTurn = i;
                }
            }
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

        if (toTurn < 0) { // no junction ahead: advance, and stop at a dead end
            const float adv = std::min(remaining, distToEnd);
            keep(cp, cs, cs + arcDir * adv);
            // Short of the offset asked for means the body reaches past the end of the
            // rails. walkTo swallows that; here it is the caller's business, since a train
            // whose end is not on any track cannot be sensed by any circuit.
            return adv >= remaining - 1e-4f;
        }
        keep(cp, cs, cs + arcDir * bestDist);
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
    return remaining <= 1e-4f;
}

void Vehicle::coalesceSpans(std::vector<PathSpan>& spans) {
    std::sort(spans.begin(), spans.end(), [](const PathSpan& x, const PathSpan& y) {
        return x.pathIdx != y.pathIdx ? x.pathIdx < y.pathIdx : x.s0 < y.s0;
    });
    std::size_t w = 0;
    for (std::size_t r = 0; r < spans.size(); ++r) {
        if (w > 0 && spans[w - 1].pathIdx == spans[r].pathIdx &&
            spans[r].s0 <= spans[w - 1].s1) {
            spans[w - 1].s1 = std::max(spans[w - 1].s1, spans[r].s1);
            continue;
        }
        spans[w++] = spans[r];
    }
    spans.resize(w);
}

bool Vehicle::spansBetween(float offsetA, float offsetB, std::vector<PathSpan>& out) const {
    out.clear();
    if (!paths_ || pathIdx_ < 0) return false;
    // Out to each end from the body centre, which is where the walk starts. The two offsets
    // straddling it is what makes the union of the two walks the stretch between them and
    // not something longer.
    const bool a = walkSpans(std::max(offsetA, offsetB), out);
    const bool b = walkSpans(std::min(offsetA, offsetB), out);
    // The walks meet at the centre and may cross the same turnouts, so what comes back is
    // in pieces that touch.
    coalesceSpans(out);
    return a && b;
}

bool Vehicle::occupiedSpans(std::vector<PathSpan>& out) const {
    out.clear();
    const std::vector<float> offs = axleOffsets();
    if (offs.empty()) return false;
    float lo = offs.front(), hi = offs.front();
    for (const float o : offs) {
        lo = std::min(lo, o);
        hi = std::max(hi, o);
    }
    // A derailed set keeps the path and arc length it left the rails at, and so goes on
    // holding its circuits - the safe direction to be wrong in, and what the geometric
    // version did with its frozen axle positions.
    return spansBetween(lo, hi, out);
}

// Where a point `bodyOffset` along the train from this set's centre lands: which path,
// where on it, and which way round it faces there. This is how a consist puts the next
// set down behind this one - the same walk that puts each axle on the rail it is
// actually on, asked over a longer distance.
bool Vehicle::anchorAtOffset(float bodyOffset, TrackAnchor& out) const {
    int cp = -1, nose = 1;
    float cs = 0.0f;
    if (!walkTo(bodyOffset, cp, cs, nose) || cp < 0 || !paths_) return false;
    const TrackPath& P = (*paths_)[cp];
    if (cs < 0.0f || cs > P.length()) return false; // walked off the end of the track
    out.pathIdx = cp;
    out.s = cs;
    out.orient = nose;
    return true;
}

TrackPose Vehicle::walkPose(float bodyOffset) const {
    int cp = -1, nose = 1;
    float cs = 0.0f;
    walkTo(bodyOffset, cp, cs, nose);
    TrackPose p = (cp >= 0 && paths_) ? (*paths_)[cp].poseAt(cs) : path_->poseAt(cs);
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

std::vector<AxlePlace> Vehicle::axlePlaces() const {
    std::vector<AxlePlace> out;
    const std::vector<float> offs = axleOffsets();
    out.reserve(offs.size());
    for (const float off : offs) {
        AxlePlace a;
        if (state_ == VehicleState::OnRail) {
            int cp = -1, nose = 1;
            float cs = 0.0f;
            // walkTo says which path the axle ended up on and where; when there is no
            // network attached it answers on the one path the vehicle was placed on.
            walkTo(off, cp, cs, nose);
            a.path = (cp >= 0 && paths_) ? cp : -1;
            a.s = cs;
            a.pos = (a.path >= 0) ? (*paths_)[static_cast<std::size_t>(a.path)].poseAt(cs).pos
                                  : path_->poseAt(cs).pos;
        } else { // derailed: off the rails, so on no road at all
            a.pos = pos_ + fTangent_ * off;
        }
        out.push_back(a);
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

// A diesel-electric. The engine does not drive the wheels at all: it turns an alternator
// at whatever speed the governor is asked for, and the traction motors turn the current
// into pull. There is nothing to change gear and nothing to slip, so where the hydraulic
// drive above steps, this is one continuous curve of three straight pieces:
//
//   flat below the corner    the motors' current and commutation limit
//   P / v above it           all the power there is, spread over the speed
//   never above adhesion     what six axles on 115 tonnes can hold
//
// The corner is not authored. It is simply where the first two cross, so changing the
// rating or the starting effort moves it to where those numbers put it.
void Vehicle::updateElectricDrive(float demandSigned, float demand, bool powering,
                                  bool reverse, float dt) {
    tractiveEffort_ = 0.0f;
    // Not under power, the revs are not this function's business: the cranking block in
    // update() owns them, and it is the one that knows the engine can be off and that a
    // running compressor loads it down.
    //
    // This used to set the revs here on every step whether powering or not, and then
    // clamp them to at least idle. Two things followed, and both looked like the engine
    // rather than the code. A stopped locomotive sat at idle speed and reported itself
    // Running, so the Di 4 read 315 rpm with its engine off; and it could not be shut
    // down at all, because the clamp put the revs back the same step the cranking block
    // wound them toward zero. The hydraulic path never had either fault - it returns
    // here and lets the shared block do it, which is what this now does too.
    if (!powering) {
        // Excitation falls away with the notch. The revs are the cranking block's.
        load_ = std::max(0.0f, load_ - kLoadOffRate * dt);
        railPower_ = 0.0f;
        return;
    }

    // The governor answers the notch and nothing else - there is no geared speed for the
    // engine to be dragged to, which is why a diesel-electric revs up standing still.
    const float span = std::max(1.0f, governedRpm_ - idleRpm_);
    const float slew = span / std::max(0.05f, spoolTime_);
    const float rpmWant = idleRpm_ + demand * span;
    float rpm = engineRpm_[0];
    rpm += std::clamp(rpmWant - rpm, -slew * dt, slew * dt);
    rpm = std::clamp(rpm, idleRpm_, governedRpm_);
    for (int i = 0; i < engineCount_; ++i) engineRpm_[i] = rpm;

    // What the engine is *making*, which is not what the notch asked for until it has got
    // there. Taking this from the notch instead - as this did - hands the driver full
    // power the instant he touches the handle, with the engine still at idle underneath
    // it, and is the whole of why the locomotive answered too quickly. Zero at idle: an
    // idling engine's output goes on its own auxiliaries, not down the alternator.
    const float made = std::clamp((rpm - idleRpm_) / span, 0.0f, 1.0f);
    const float rate = made > load_ ? kLoadOnRate : kLoadOffRate;
    load_ = std::clamp(load_ + std::clamp(made - load_, -rate * dt, rate * dt), 0.0f, 1.0f);

    const float sp = std::abs(physV_);
    const float pRail = load_ * powerW_ * kElectricEta;
    // Three limits, and which one binds is the whole character of the machine.
    //
    // The flat one is a CURRENT limit. Tractive effort is what amperes buy, and there is
    // a maximum the inverters and the motors will pass, so below the corner speed the
    // effort is level and the driver is holding the ammeter against its stop. Excitation
    // scales it because excitation is what sets the current in the first place - the
    // amps come up with the regulator rather than stepping to the limit.
    //
    // The consequence is worth stating because it is not obvious: at a crawl the engine
    // is barely worked. Power at the rail is effort times speed, and 360 kN at 5 km/h is
    // 500 kW of the 2450 there are. The locomotive is at full revs, the ammeter is
    // pegged, and the diesel is loafing - which is why railPower_ is carried out
    // separately rather than inferred from the notch.
    const float teFlat =
        startTE_ > 0.0f ? load_ * startTE_ : std::numeric_limits<float>::max();
    const float tePower = pRail / std::max(sp, 0.05f);
    const float teAdh = kTractionMu * drivenFrac_ * mass_ * kG;
    float TE = std::min(std::min(teFlat, tePower), teAdh);
    const float dir = demandSigned >= 0.0f ? 1.0f : -1.0f;
    if (reverse && sp > kRevSpeedCap) TE = 0.0f;
    tractiveEffort_ = dir * TE;
    railPower_ = TE * sp;
}

// The rheostatic brake: the same machines turned round. They are driven as generators and
// what they make is burned in the roof grids, so this is a retarding force that never
// becomes tractive effort and never goes back into anything.
//
// Its curve has the shape of the traction curve because it shares the hardware and the same
// two limits bound it - current below the corner, and above it what the grids will take.
// The difference that matters is the bottom end: a machine turning slowly generates nothing,
// so the brake fades out and is gone before the train stops. That is not a nicety. Without
// it the "brake" would hold the train at a stand, because the consist clamps retardation to
// the speed it has rather than letting it push the train backwards - so a force that never
// faded would look exactly like a parking brake, and a locomotive would sit on a grade held
// by its generators.
void Vehicle::updateDynamicBrake(float demand, bool inGear, float dt) {
    (void)dt;
    dynBrake_ = 0.0f;
    if (dynBrakeN_ <= 0.0f || demand <= 0.0f) return;
    // Part of the traction system: no engine, no auxiliaries, nothing to excite the
    // machines with - and nothing at all from a cab that is not in gear.
    if (!engineOn_ || !enginesRunning() || !inGear) return;
    if (state_ != VehicleState::OnRail) return;

    const float sp = std::abs(physV_);
    const float fade = std::clamp((sp - kDynFadeOut) / (kDynFadeIn - kDynFadeOut), 0.0f, 1.0f);
    if (fade <= 0.0f) return;

    const float beFlat = demand * dynBrakeN_;
    const float bePower = dynBrakeW_ > 0.0f ? demand * dynBrakeW_ / std::max(sp, 0.05f)
                                            : std::numeric_limits<float>::max();
    // Through the driven wheels, so the same adhesion cap the pull answers to.
    const float beAdh = kTractionMu * drivenFrac_ * mass_ * kG;
    dynBrake_ = fade * std::min(std::min(beFlat, bePower), beAdh);
}

// Motor current as a fraction of what the drive will pass. Effort is what current buys,
// so below the corner speed these are the same number - which is what makes an ammeter
// the gauge a diesel-electric is driven on. It reads the same either way: braking current
// is current, and the needle a driver watches going up a bank is the one he watches coming
// down it.
float Vehicle::tractionAmpsFrac() const {
    if (startTE_ <= 0.0f) return 0.0f;
    const float amps = std::max(std::abs(tractiveEffort_), dynBrake_);
    return std::clamp(amps / startTE_, 0.0f, 1.0f);
}

// And how hard the diesel is actually being worked, which at low speed is not much.
float Vehicle::enginePowerFrac() const {
    if (powerW_ <= 0.0f) return 0.0f;
    return std::clamp(railPower_ / (powerW_ * kElectricEta), 0.0f, 1.0f);
}

void Vehicle::updateTraction(float demandSigned, float demand, bool powering,
                             bool reverse, float dt) {
    // Which machine this is. The two share the notch, the adhesion cap and the reverse
    // speed cap, and nothing else - a torque converter and a generator are not variants
    // of one another.
    if (drive_ == DriveElectric) {
        updateElectricDrive(demandSigned, demand, powering, reverse, dt);
        return;
    }
    tractiveEffort_ = 0.0f;
    if (shiftTimer_ > 0.0f) shiftTimer_ = std::max(0.0f, shiftTimer_ - dt);

    const float sp = std::abs(physV_);
    // Engine rev/min if the converter were locked in the current gear (the geared,
    // turbine-side speed). The automatic shifts on this, not the engine speed.
    const float rpmLock = sp / wheelRadius_ * kGearRatio[gear_ - 1] / kRpmToRad;

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
    const float floorRpm = idleRpm_ + demand * (kConvFloorRpm - idleRpm_);
    const float rpmWant = std::clamp(std::max(rpmLock, floorRpm), idleRpm_, governedRpm_);
    float rpm = engineRpm_[0];
    const float hSlew = (governedRpm_ - idleRpm_) / std::max(0.05f, spoolTime_);
    rpm += std::clamp(rpmWant - rpm, -hSlew * dt, hSlew * dt);
    rpm = std::clamp(rpm, idleRpm_, governedRpm_);
    for (int i = 0; i < engineCount_; ++i) engineRpm_[i] = rpm;

    // Torque converter: speed ratio -> torque ratio (stall multiplication at low
    // speed easing to 1:1 lock-up once coupled).
    const float sr = std::clamp(rpmLock / std::max(rpm, 1.0f), 0.0f, 1.0f);
    const float tr =
        sr >= kTCCouple ? 1.0f : kTCStall + (1.0f - kTCStall) * (sr / kTCCouple);

    // Tractive effort: the lesser of the geared/converter torque limit, the
    // constant-power hyperbola, and the wheel/rail adhesion cap. Cut across a shift.
    const float we = rpm * kRpmToRad;                  // engine rad/s
    const float pAvail = demand * powerW_ * kEta;      // available power (W)
    const float te = pAvail / std::max(we, 1.0f);      // engine torque (N*m)
    const float teGeared = te * tr * kGearRatio[gear_ - 1] / wheelRadius_;
    const float tePower = pAvail / std::max(sp, 1.0f); // hyperbola (limits at speed)
    const float teAdh = kTractionMu * drivenFrac_ * mass_ * kG;
    float TE = std::min(std::min(teGeared, tePower), teAdh);
    if (shiftTimer_ > 0.0f) TE = 0.0f;                 // unloaded across an upshift
    const float dir = demandSigned >= 0.0f ? 1.0f : -1.0f; // track direction of travel
    if (reverse && sp > kRevSpeedCap) TE = 0.0f;       // reverse stays a shunting speed

    tractiveEffort_ = dir * TE; // left for the consist to apply to the whole train
}

UnitStep Vehicle::stepSubsystems(float dt, const LinkCommand& cmd,
                                 float physicalSpeed) {
    physV_ = physicalSpeed;
    cmdNotch_ = std::clamp(cmd.brakeNotch, 0, kEmergencyNotch);
    trainEmerg_ = cmd.emergency;
    valveHere_ = cmd.valveHere;
    indDemand_ = independent_ ? std::clamp(cmd.independent, 0.0f, 1.0f) : 0.0f;

    // The link's demand arrives in the physical frame (+ = the way the train faces);
    // this set turns it into its own path's frame, which is what orient_ is for. Two
    // sets whose paths disagree about which way is +s then pull the same way.
    const float demandSigned = static_cast<float>(orient_) * cmd.demand;
    const float demand = std::abs(demandSigned);
    const bool powering = engineOn_ && cmd.powering && demand > 0.0f &&
                          enginesRunning() && state_ == VehicleState::OnRail;
    tractiveEffort_ = 0.0f;
    // Off the rails there are no friction surfaces doing anything; only the on-rail
    // branch below sets this, so clear it or a derailed vehicle reports the last
    // brake force it had for ever.
    brakeForce_ = 0.0f;

    // Engines crank up to / spin down from idle, independent of motion; a running
    // compressor loads its engine down a little (compActive_ is last frame's value).
    if (!powering) {
        const float rpmTarget =
            engineOn_ ? idleRpm_ - (compActive_ ? kCompLoadDrop : 0.0f) : 0.0f;
        const float up = idleRpm_ / kCrankTime, down = idleRpm_ / kSpinDownTime;
        for (int i = 0; i < engineCount_; ++i) {
            if (engineRpm_[i] < rpmTarget)
                engineRpm_[i] = std::min(rpmTarget, engineRpm_[i] + up * dt);
            else if (engineRpm_[i] > rpmTarget)
                engineRpm_[i] = std::max(rpmTarget, engineRpm_[i] - down * dt);
        }
    }
    // The electric brake, which answers the controller's range below neutral. Worked out
    // whether or not the vehicle is powering - the two are opposite ends of one handle and
    // cannot both be commanded - but dropped entirely by an emergency, which wants the
    // friction brake and all of it.
    updateDynamicBrake(cmd.emergency ? 0.0f : cmd.dynamic, cmd.demand != 0.0f || cmd.dynamic > 0.0f,
                       dt);

    if (state_ != VehicleState::OnRail) return {};

    // Traction: a torque converter feeding a 5-speed automatic gearbox. The effort is
    // left in tractiveEffort_ rather than applied - the train moves as one.
    updateTraction(demandSigned, demand, powering, cmd.reverse, dt);

    // Low main-reservoir safety: if the reservoir falls below the trip pressure this
    // set calls for emergency regardless of the handle, latched until the reservoir
    // recovers above the reset pressure. It is this set's own reservoir and this set's
    // own device: nothing about the other sets can set it or clear it.
    //
    // Only on a vehicle that HAS a compressor. The device is a locomotive's and it is
    // there to catch a compressor that has failed; a vehicle that never had one cannot
    // have one fail, and arming it on a carriage made a guaranteed time bomb. The leak
    // below took an unpowered vehicle's reservoir under the trip pressure in about 34
    // minutes, the emergency latched, and nothing on that vehicle could ever recharge it
    // above the reset pressure - so any hauled train stopped irrecoverably after half an
    // hour, wherever it happened to be, with the brake handle sitting in release.
    if (engineCount_ > 0) {
        if (mrPres_ < kMRSafetyTrip) safetyBrake_ = true;
        else if (mrPres_ >= kMRSafetyReset) safetyBrake_ = false;
        // And above that, the feed valve's cut-out. Filling a brake pipe is what spends
        // main reservoir air, so it is the first thing to give up when the supply is
        // falling behind: the feed closes, the compressor is left to get ahead of it
        // alone, and the fill resumes once there is air to do it with. Temporary, and
        // latched between the two pressures so it does not chatter.
        //
        // It is what keeps the reservoir off the safety trip below rather than what
        // rescues it afterwards - charging 600 m of pipe costs about 5.75 bar and will
        // walk the reservoir straight down through both thresholds if nothing stops it.
        // Stopping the fill costs a long train some time; letting it run costs an
        // emergency application nobody asked for.
        //
        // This vehicle's own reservoir and this vehicle's own valve, as the safety device
        // above is: what the wagons at the far end are doing is not something the
        // locomotive can see, and must not be something that decides whether it may fill
        // its pipe.
        if (mrPres_ < kMRFeedCut) feedCut_ = true;
        else if (mrPres_ >= kMRFeedResume) feedCut_ = false;
    }
    const int effNotch = effectiveNotch();

    // The EP valve on this set: it vents its own length of pipe to atmosphere, or
    // recharges it from its own main reservoir, toward what the handle asks for. Every
    // set does this at the same moment from the same command, which is the whole point
    // of an electro-pneumatic brake - the reduction happens along the whole train at
    // once instead of travelling from the front.
    //
    // Venting is faster than charging, and emergency far faster than either. That
    // asymmetry is not a detail: it is why a brake goes on smartly and comes off slowly,
    // and why an emergency application cannot be taken back.
    // Where the pipe stood at the top of the step. Consist marks it before it moves air
    // across the couplings; a vehicle stepped on its own marks it here instead.
    if (!pipeMarked_) bpStepStart_ = bp_;
    pipeMarked_ = false;
    // A pipe that has been cut is open to atmosphere and stays open until somebody
    // closes the cock. The EP valve cannot charge against it - trying only feeds the
    // leak - so the pipe simply empties and the distributors do the rest. This is what
    // separates a burst hose from a momentary bleed, and it is the fault the whole
    // arrangement is built around.
    // Who is allowed to move this vehicle's pipe at all.
    //
    // A vehicle drives its own pipe if the driver's valve is on it, or it has EP - which
    // echoes the valve electrically at every vehicle, so they all vent in step - or its own
    // device is calling for emergency, which is a cock opening here and needs nobody's
    // permission. Anything else only follows its neighbours by the diffusion across the
    // couplings, and its distributors answer the pressure it actually has.
    //
    // That single rule is the difference between a railcar and a hauled train. With EP the
    // application is everywhere at once and the length of the train does not enter into it.
    // On plain automatic air it has to travel, and five carriages of pipe take their time.
    //
    // The accelerator is the part that makes a long train work at all. A distributor that
    // sees the pipe fall past the emergency threshold does not wait for the driver's valve
    // to draw its air away down a hundred metres of hose - it opens its own vent and dumps
    // locally. Each vehicle doing that pulls its neighbour under the threshold in turn, so
    // the application travels as a wave at a few hundred metres a second rather than
    // seeping along by diffusion. Without it the far end of this train bit at 3.8 s, which
    // works out at 75 m/s against the 250 m/s a UIC brake is required to manage.
    // What trips it is the pipe FALLING past the threshold, not merely being below it -
    // bpRate_ still holds last step's signed rate. A vehicle standing with an empty pipe
    // is not in emergency, it simply has no air in it yet, and reading the level alone
    // latched every vehicle at start-up and left the whole fleet unable to release.
    // Trip on a fall at an EMERGENCY rate, not merely a falling pipe: now that the hoses
    // are in the rate, a wagon losing air to a neighbour during an ordinary reduction
    // shows a rate too, and that is a service application and not this.
    if (ventArmed_ && bp_ < kBPEmergSense && -bpRate_ > kBPEmergTripRate) emergVent_ = true;
    // And it reseats when the fast fall STOPS - which is what a differential valve does,
    // and covers both ways that happens: the pipe is down to nothing so there is no more
    // to vent, or the driver is refilling against it and the two have met. It used to wait
    // for the pipe to come back UP, which an open vent prevents, so the only way out was
    // for every last vehicle to reach zero. On a 600 m train that is the emergency that
    // will not let go.
    if (emergVent_ && -bpRate_ < kBPVentCloseRate) {
        emergVent_ = false;
        ventArmed_ = false; // and it cannot act again until the pipe has recharged
    }
    if (bp_ > kBPEmergSense + kBPVentRearm) ventArmed_ = true;
    const bool localDump = pipeCut_ || safetyBrake_ || emergVent_;
    const bool drives = epBrake_ || valveHere_ || localDump;
    // How fast this vehicle's own length of pipe can be emptied or filled. Air has to be
    // moved out of a volume, and the volume goes with the length, so a 25 m carriage is
    // slower than a 21 m locomotive and a train of them slower again.
    const float vol = kPipeRefLength / std::max(4.0f, length_);
    const float bpWant = pipeCut_ ? 0.0f : targetBP(effNotch);
    if (pipeCut_ || emergVent_) {
        bp_ = std::max(0.0f, bp_ - kBPEmergVentRate * vol * dt);
    } else if (!drives) {
        // Nothing of its own: the couplings are the only thing that moves this pipe.
    } else if (bpWant < bp_) {
        const float rate = (effNotch >= kEmergencyNotch) ? kBPEmergVentRate : kBPVentRate;
        bp_ = std::max(bpWant, bp_ - rate * vol * dt);
    } else if (bpWant > bp_ && !feedCut_) {
        // Charged from the main reservoir and limited by it: a set whose reservoir has
        // run down cannot fill its pipe, so it cannot release. Below the feed valve's
        // cut-out it does not even try, and this branch is skipped until the compressor
        // has put the reservoir back.
        const float reach = std::min(bpWant, mrPres_);
        const float before = bp_;
        bp_ = std::min(reach, bp_ + kBPChargeRate * vol * dt);
        mrPres_ -= kMRPerBP * std::max(0.0f, bp_ - before);
    }
    bp_ = std::max(0.0f, bp_);

    // The distributors, one per bogie. Each is on its own: it compares the pipe against
    // its own memory of it and moves its own cylinder with its own stored air.
    for (BrakeUnit& b : brakes_) {
        const float bcBefore = b.bc;
        // The control reservoir follows the pipe up and holds on the way down. Holding
        // is what makes the brake answer the *reduction*; following up is what recharges
        // the memory so the next reduction is measured from a full pipe again.
        b.ctrl = std::max(b.ctrl, std::min(bp_, kBPRelease));
        float autoWant = std::clamp((b.ctrl - bp_) * kBCPerBPDrop, 0.0f, kBCMax);
        // The dynamic brake interlock. While the grids are working, this vehicle's
        // AUTOMATIC application is held off - applying shoes and grids together
        // duplicates the effort and cooks the shoes, and a locomotive with dynamic
        // braking is arranged so it does not happen. The cylinders are genuinely
        // released, not merely discounted afterwards, so the gauge tells the truth.
        // An emergency, a safety trip or a burst pipe all override it.
        if (dynBrake_ > 0.0f && !trainEmerg_ && !safetyBrake_ && !pipeCut_) autoWant = 0.0f;
        // And the independent brake on top, by a double check valve: the two feed the
        // same cylinders and whichever asks for more gets it. This one is the driver's
        // own hand and is NOT held off by the interlock - if he wants shoes as well as
        // grids he can have them.
        const float indWant = indDemand_ * kBCMax;
        const float want = std::max(autoWant, indWant);
        if (want > b.bc) {
            // Filling from the auxiliary, and limited by it. An auxiliary drawn down by
            // repeated applications cannot fill its cylinder, and the brake fades.
            // The cylinders fill at the rate the distributor can pass air, and EP stock
            // fills fast on purpose - that is most of what EP buys. Plain automatic air
            // has no such help: the distributor is working on its own, off a pipe that is
            // itself still falling, and a passenger brake of the period takes a couple of
            // seconds to come up rather than a fraction of one.
            float rate = (bp_ < kBPFullService - 0.1f) ? kBCEmergApplyRate : kBCApplyRate;
            if (!epBrake_) rate *= kPlainAirFill;
            // The automatic brake fills from the auxiliary and is limited by it; the
            // independent fills from the main reservoir, which is why a locomotive can be
            // held on its own brake long after its auxiliaries would have given out.
            const float reach = std::min(want, std::max(b.aux, indWant));
            const float before = b.bc;
            b.bc = std::min(reach, b.bc + rate * dt);
            const float drawn = std::max(0.0f, b.bc - before);
            if (indWant >= autoWant) mrPres_ = std::max(0.0f, mrPres_ - kMRPerBC * drawn);
            else b.aux = std::max(0.0f, b.aux - kAuxPerBC * drawn);
        } else if (want < b.bc) {
            b.bc = std::max(want, b.bc - kBCReleaseRate * dt); // exhausts to atmosphere
        }
        b.bc = std::max(0.0f, b.bc);
        // The auxiliary refills from the pipe, never from the cylinder, and only while
        // the pipe stands above it. This is what makes releasing and recharging one act.
        if (bp_ > b.aux) {
            const float room = std::min(kAuxCapacity, bp_) - b.aux;
            b.aux += std::min(room, kAuxChargeRate * dt);
        }
        b.bcRate = (dt > 1e-6f) ? (b.bc - bcBefore) / dt : 0.0f;
    }
    // The leak, and only where there is a reservoir to leak from. A carriage carries
    // auxiliary reservoirs fed from the train pipe and no main reservoir of its own, so
    // there is nothing here to run down - and nothing that could fill it if there were.
    // Leaking it anyway is what armed the trap described above.
    if (engineCount_ > 0) mrPres_ -= kMRLeak * dt;
    // Engine-driven compressors recharge only while the engines idle, scaled by how
    // many are running. With the engines off the reservoir just draws down - which is
    // a real fault on a machine that has a compressor, and stays modelled. "Running"
    // uses a threshold below the compressor load droop so the droop can't make it cut
    // out and hunt.
    int running = 0;
    for (int i = 0; i < engineCount_; ++i)
        if (engineRpm_[i] >= idleRpm_ * 0.9f) ++running;
    if (running > 0 && compOn_)
        mrPres_ += kCompRate * (static_cast<float>(running) / engineCount_) * dt;
    mrPres_ = std::clamp(mrPres_, 0.0f, kMRCapacity);
    // Governor decision for the next step, from the resulting pressure: cut out at
    // capacity, cut in below kMRCutIn. (Deciding after the recharge+clamp is what
    // latches it off at 8 bar; deciding before let the tiny leak keep the pressure
    // hovering just under 8 so it never cut out.)
    if (running > 0) {
        if (mrPres_ >= kMRCapacity) compOn_ = false;
        else if (mrPres_ < kMRCutIn) compOn_ = true;
    }
    compActive_ = (running > 0 && compOn_); // drives the sound + engine load
    // The rate of this pipe over the whole step, hoses included. It is what the
    // accelerator reads, and what the airflow sound is made from.
    bpRate_ = (dt > 1e-6f) ? (bp_ - bpStepStart_) / dt : 0.0f;

    // Friction-brake force: each bogie brakes its own share of the set's weight with its
    // own cylinder, and the sum is capped by wheel/rail adhesion on the whole of it. One
    // bogie with no air therefore costs exactly its share and no more, which is the
    // reason for keeping them apart.
    const float share = mass_ / static_cast<float>(std::max<std::size_t>(1, brakes_.size()));
    float bf = 0.0f;
    for (const BrakeUnit& b : brakes_)
        bf += (b.bc / kBCMax) * share * kFullServiceDecel;
    // Blending. While the electric brake is working, this vehicle's own friction brake is
    // held off: applying both duplicates the effort and cooks the shoes, and a locomotive
    // with dynamic braking is arranged so that it does not happen. The PIPE is untouched -
    // the carriages behind must still get their application, and these cylinders still
    // fill - what is suppressed is counting their force. An emergency, a safety trip or a
    // burst pipe all override it, because then what is wanted is every shoe there is.
    // The interlock is applied at the cylinders now rather than here, so whatever
    // pressure they have is real and makes the force it looks like it should.
    brakeForce_ = std::min(bf + dynBrake_, kAdhesionMu * mass_ * kG);

    UnitStep out;
    // Back into the physical frame for the train to add up.
    out.tractiveEffort = static_cast<float>(orient_) * tractiveEffort_;
    out.brakeForce = brakeForce_;
    return out;
}

bool Vehicle::advance(float ds) {
    if (state_ != VehicleState::OnRail) return false;
    const float sBefore = s_;
    s_ += static_cast<float>(orient_) * ds; // the physical move, in this path's frame

    // The wheels crossing the points, for the sound. Before crossTurnouts, which may
    // swap the path out from under the comparison.
    countRailImpacts(sBefore);

    // Turnouts first: a divert/merge swaps the path (so the end-of-path check below
    // sees the new one); a facing move into a broken switch derails here.
    if (net_ && crossTurnouts(sBefore)) return true;

    // Derail when the leading or trailing axle passes an end of the track — but only
    // at a genuine dead-end. A path end that a turnout connects to (a siding end at
    // the switch) is a junction: the body legitimately straddles it while
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
        vel_ = physV_ * e.tangent; // carry the exit velocity
        state_ = VehicleState::Derailed;
        return true;
    }
    return false;
}

void Vehicle::stepDerailed(float dt) {
    if (state_ != VehicleState::Derailed) return;
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
