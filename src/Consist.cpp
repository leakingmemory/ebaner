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

#include "Consist.h"

#include "TrackPath.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>

namespace {
constexpr float kG = Vehicle::kGravity;
constexpr float kPushForce = 500.0f;   // N, one person leaning on it
constexpr float kAdhesionMu = 0.20f;   // wheel/rail adhesion under braking
// How far the coupler is allowed to be out before the train is called broken. A set
// crossing a turnout lands on the junction's arc-length exactly, which moves it by up
// to a few centimetres; a set taken away by the points moves by metres.
constexpr float kCouplerSlack = 1.5f;  // m
// How fast air crosses a coupling, as a fraction of the difference per second. Generous
// on purpose: a Class 93 set is 41 m and the whole train a few of them, so the real
// delay end to end is a fraction of a second and the point of modelling it is that it is
// not zero, not that it is long.
constexpr float kPipeCoupling = 3.0f;
} // namespace

Consist::Consist(const std::vector<TrackPath>* paths, const TrackPath* path,
                 const VehicleSpec& spec, float s, float initialSpeed)
    : paths_(paths), name_(spec.name), v_(initialSpeed) {
    const int n = std::max(1, spec.units);
    units_.reserve(n);
    for (int i = 0; i < n; ++i) units_.emplace_back(path, spec, s, initialSpeed);
    // The sets are laid out along the train with the first at one end. Until the
    // network is attached the walk has nothing to follow, so this is a straight
    // placement; layOut() does it properly once attachNetwork has been called.
    const float pitch = unitPitch();
    const int idx = (paths_ && path) ? static_cast<int>(path - paths_->data()) : -1;
    for (int i = 0; i < n && idx >= 0 && n > 1; ++i) {
        Vehicle::TrackAnchor a;
        a.pathIdx = idx;
        a.s = s + (static_cast<float>(i) - 0.5f * static_cast<float>(n - 1)) * pitch;
        a.orient = 1;
        units_[i].placeAt(*paths_, a);
    }
}

Consist::Consist(std::vector<Vehicle>&& units, const std::vector<TrackPath>* paths,
                 const char* name, float v)
    : units_(std::move(units)), paths_(paths), name_(name), v_(v) {
    // Deliberately no layOut(): these sets are already standing where they stand, on
    // the roads they are on. Laying them out again would re-place them at the pitch
    // they are already at - a nudge for nothing, and on a set halfway over a turnout,
    // a nudge that fights the coupler loop for it.
    //
    // tractiveEffort_ and brakeForce_ are left at zero: they are last step's sums and
    // are written afresh on the next one.
}

void Consist::attachNetwork(const std::vector<TrackPath>* paths, SwitchNetwork* net) {
    paths_ = paths;
    for (Vehicle& u : units_) u.attachNetwork(paths, net);
    layOut();
}

void Consist::layOut() {
    // Each set behind the first is put where the coupler says, followed along the
    // track rather than measured in a straight line - a train standing across a
    // turnout is on the rails it is actually on.
    if (!paths_) return;
    for (std::size_t i = 1; i < units_.size(); ++i) {
        Vehicle::TrackAnchor a;
        if (units_[i - 1].anchorAtOffset(unitPitch(), a)) units_[i].placeAt(*paths_, a);
    }
}

bool Consist::consumeSwitchChanged() {
    bool c = false;
    for (Vehicle& u : units_) c = u.consumeSwitchChanged() || c;
    return c;
}

// --- cabs -----------------------------------------------------------------------

bool Consist::cabDrivable(int cab) const {
    if (cab < 0 || cab >= cabCount()) return false;
    // Coupled, only the cabs at the two ends of the train drive; the ones at the
    // coupler are shut down. A single set has both of its cabs live, which is the
    // same rule read on a train of one.
    return cab == 0 || cab == cabCount() - 1;
}

void Consist::setBrakeNotch(int cab, int notch) {
    if (cab >= 0 && cab < cabCount()) units_[cab / 2].setBrakeNotch(cab % 2, notch);
}
int Consist::brakeNotch(int cab) const {
    return (cab >= 0 && cab < cabCount()) ? units_[cab / 2].brakeNotch(cab % 2) : 0;
}
void Consist::setPowerNotch(int cab, int notch) {
    if (cab >= 0 && cab < cabCount()) units_[cab / 2].setPowerNotch(cab % 2, notch);
}
int Consist::powerNotch(int cab) const {
    return (cab >= 0 && cab < cabCount()) ? units_[cab / 2].powerNotch(cab % 2) : 0;
}
void Consist::moveHandle(int cab, int dir) {
    if (cab >= 0 && cab < cabCount()) units_[cab / 2].moveHandle(cab % 2, dir);
}
int Consist::handlePosition(int cab) const {
    return (cab >= 0 && cab < cabCount()) ? units_[cab / 2].handlePosition(cab % 2) : 0;
}
const char* Consist::handleName(int cab) const {
    return (cab >= 0 && cab < cabCount()) ? units_[cab / 2].handleName(cab % 2) : "N";
}
void Consist::setReverser(int cab, int dir) {
    // A shut-down cab refuses the reverser: it is coupled to another set nose to nose
    // and is not a driving position.
    if (cabDrivable(cab)) units_[cab / 2].setReverser(cab % 2, dir);
}
int Consist::reverser(int cab) const {
    return (cab >= 0 && cab < cabCount()) ? units_[cab / 2].reverser(cab % 2) : 0;
}
const char* Consist::reverserName(int cab) const {
    const int r = reverser(cab);
    return r > 0 ? "F" : (r < 0 ? "R" : "N");
}

int Consist::activeCab() const {
    int active = -1, count = 0;
    for (int c = 0; c < cabCount(); ++c)
        if (reverser(c) != 0) { active = c; ++count; }
    return count == 1 ? active : -1; // none, or more than one - including one per set
}

bool Consist::interlockEmergency() const {
    return lead().bodyStyle() == BodyClass93 && activeCab() < 0;
}

int Consist::trippedUnit() const {
    for (std::size_t i = 0; i < units_.size(); ++i)
        if (units_[i].safetyBrakeDemand()) return static_cast<int>(i);
    return -1;
}

bool Consist::emergencyLine() const {
    // The train-wide emergency line. It is asked of every set in turn and is up if any
    // one of them is calling for it, which is what "either set can brake the whole
    // train, independently" means. The reverser interlock rides the same line.
    //
    // So does the hold a parted train comes away with, and it is first because it is
    // the one thing here that no handle and no reverser position can talk its way past
    // in the same breath: it is cleared by its own sequence, in update(), or not at all.
    if (hold_ != UncoupleHold::None) return true;
    if (interlockEmergency()) return true;
    for (const Vehicle& u : units_)
        if (u.safetyBrakeDemand()) return true;
    return false;
}

int Consist::effectiveNotch() const {
    if (emergencyLine()) return Vehicle::kEmergencyNotch;
    const int a = activeCab();
    return a >= 0 ? brakeNotch(a) : Vehicle::kEmergencyNotch;
}

const char* Consist::effectiveBrakeName() const {
    static const char* kNames[] = {"REL", "B1", "B2", "B3", "B4", "EMERG"};
    return kNames[std::clamp(effectiveNotch(), 0, Vehicle::kEmergencyNotch)];
}

// --- the step -------------------------------------------------------------------

bool Consist::mayUncouple(int k, const char*& why) const {
    if (k < 0 || k >= couplerCount()) {
        why = unitCount() > 1 ? "NO SUCH COUPLER" : "NOTHING TO UNCOUPLE";
        return false;
    }
    if (speed() > kUncoupleMaxSpeed) {
        why = "THE TRAIN IS MOVING";
        return false;
    }
    why = "";
    return true;
}

Consist::End Consist::end(bool tail) const {
    End e;
    if (units_.empty()) return e;
    // The set at that end of the train, and which way its own nose points along its
    // path. Set 0 is the front as the train is laid out, so leaving by the front means
    // travelling against that set's facing.
    // The sets are laid out nose to tail, each one ahead of the last in its own facing
    // (layOut walks forward by the pitch), so units_.back() is always the front of the
    // train however the train faces, and units_.front() the rear.
    const Vehicle& u = tail ? units_.back() : units_.front();
    const int outward = tail ? u.orientation() : -u.orientation();
    e.pathIdx = u.pathIdx();
    // The coupler face, not the end of the body: the head stands half the coupler gap
    // proud of it, so two sets standing at the coupled pitch have their faces touching
    // and a gap measured between two such faces is zero when they are together.
    e.s = u.s() + static_cast<float>(outward) * (0.5f * u.length() + 0.5f * kCouplerGap);
    e.outward = outward;
    return e;
}

void Consist::absorb(Consist&& other, bool tail, bool reverseOther) {
    if (other.units_.empty()) return;
    // Momentum, in a frame both trains agree on. Each carries its speed in its own
    // facing, and after this there is one facing - this train's - so the other's has to
    // be read into it first. Two trains met at a closing speed have between them less
    // momentum than either had alone, and a light set shunted by a heavy one leaves
    // faster than the heavy one arrived; taking the mean of the speeds would get both
    // of those wrong.
    const float mA = mass(), mB = other.mass();
    const float vB = reverseOther ? -other.v_ : other.v_;
    if (mA + mB > 0.0f) v_ = (mA * v_ + mB * vB) / (mA + mB);

    std::vector<Vehicle> add = std::move(other.units_);
    if (reverseOther) std::reverse(add.begin(), add.end());
    if (tail) {
        units_.insert(units_.end(), std::make_move_iterator(add.begin()),
                      std::make_move_iterator(add.end()));
    } else {
        // Onto the front: the incoming sets go ahead of set 0, so this train's own
        // numbering shifts along behind them. Nothing about where anything stands
        // changes - only which index it answers to.
        add.insert(add.end(), std::make_move_iterator(units_.begin()),
                   std::make_move_iterator(units_.end()));
        units_ = std::move(add);
    }
    // The hoses are coupled up again at the joint, so the two lengths of pipe become
    // one; update()'s diffusion pass carries air across it from the next step. A rough
    // coupling bursts them again afterwards, which is the caller's business.
    for (Vehicle& u : units_) u.closeBrakePipeCock();
    // Whichever half was held stays held: coupling to a portion standing in emergency
    // does not release it, and the reverser cycle that clears the hold is now one
    // sequence for one train.
    if (other.hold_ != UncoupleHold::None && hold_ == UncoupleHold::None)
        hold_ = other.hold_;
    tractiveEffort_ = brakeForce_ = 0.0f;
}

void Consist::derail() {
    for (Vehicle& u : units_) u.derail();
    v_ = 0.0f;
}

std::optional<Consist> Consist::uncoupleAfter(int k) {
    const char* why = nullptr;
    if (!mayUncouple(k, why)) return std::nullopt;

    // Move the tail out and drop the source range in the same breath, so there is no
    // window in which a moved-from Vehicle is still part of a train. A Vehicle owns
    // nothing - its path, its network and its paths list all point at storage outside
    // it - so moving one is a copy of pointers and PODs and is exactly safe; moving
    // rather than copying is only to avoid the churn.
    std::vector<Vehicle> tail(std::make_move_iterator(units_.begin() + k + 1),
                              std::make_move_iterator(units_.end()));
    units_.erase(units_.begin() + k + 1, units_.end());
    Consist rear(std::move(tail), paths_, name_, v_);

    // The two cabs that were at this coupler are shut-down cabs that have just become
    // the ends of two trains. A shut-down cab refuses the reverser but not the brake
    // handle, so whatever was left wound into it is still there - and if that is power,
    // the portion runs away the moment its hold clears. Leave them as an uncoupling
    // shunter leaves a cab: brake fully applied, power off.
    setBrakeNotch(cabCount() - 1, Vehicle::kEmergencyNotch);
    setPowerNotch(cabCount() - 1, 0);
    rear.setBrakeNotch(0, Vehicle::kEmergencyNotch);
    rear.setPowerNotch(0, 0);

    // And the brake hoses come apart, which is not a command but an event: both open
    // ends dump to atmosphere, every distributor on both portions sees the pipe gone and
    // puts its auxiliary into its cylinder. Nothing had to tell them to. This is the
    // failsafe doing exactly what it is for, and it is why parting a train is safe.
    units_.back().burstBrakePipe();
    rear.units_.front().burstBrakePipe();

    // Both portions come away held, whatever their reversers happen to say. Asking the
    // reversers here would be the bug: a portion already at Neutral would be released
    // on its very first step, which is the one case the requirement is about.
    hold_ = UncoupleHold::InGear;
    rear.hold_ = UncoupleHold::InGear;
    tractiveEffort_ = brakeForce_ = 0.0f;

    std::printf("[Uncouple] coupler %d parted: %d set(s) stay, %d away - both trains "
                "in emergency until a reverser is cycled through N\n",
                k + 1, unitCount(), rear.unitCount());
    std::fflush(stdout);
    return rear;
}

void Consist::update(float dt, float pushInput) {
    // The uncoupling hold, before anything reads the emergency line below. Sampled
    // rather than remembered: three states and the current reversers are a complete
    // edge detector for "has been through Neutral", which is the same recompute-every-
    // frame shape the rest of the brake path has - and unlike watching setReverser it
    // cannot be walked around by reaching into unit(i) and setting the reverser there.
    if (hold_ != UncoupleHold::None) {
        bool anyInGear = false;
        for (int c = 0; c < cabCount(); ++c)
            if (reverser(c) != 0) anyInGear = true;
        if (!anyInGear) {
            hold_ = UncoupleHold::AtNeutral;
        } else if (hold_ == UncoupleHold::AtNeutral) {
            hold_ = UncoupleHold::None;
            // The cocks go back on at the same moment. Parting the train opened the pipe
            // at the break and it has been open ever since - that is why the portion has
            // been standing here with its brakes on. Closing it is a shunter's job on the
            // ground, and cycling the reverser is this simulator's stand-in for having
            // gone and done it; what follows is a pipe that has to be charged from empty,
            // which is why releasing after a split is not instant.
            for (Vehicle& u : units_) u.closeBrakePipeCock();
            std::printf("[Uncouple] reverser cycled through N - cocks closed, pipe "
                        "recharging\n");
            std::fflush(stdout);
        }
    }

    // What the driving cab is asking for, worked out once and sent to every set. The
    // two cabs of a set face opposite ways (even cab toward -s, odd toward +s), so a
    // given reverser direction drives the train opposite ways from each end; that is
    // folded in here, in the physical frame, and each set turns it into its own.
    const int a = activeCab();
    LinkCommand cmd;
    cmd.emergency = emergencyLine();
    if (a >= 0) {
        const float cabSign = (a % 2 == 0) ? -1.0f : 1.0f;
        const int rev = reverser(a);
        cmd.brakeNotch = brakeNotch(a);
        cmd.demand = cabSign * static_cast<float>(rev * powerNotch(a)) /
                     static_cast<float>(Vehicle::kMaxPowerNotch);
        cmd.reverse = rev < 0;
        cmd.powering = !cmd.emergency && powerNotch(a) > 0;
    } else {
        cmd.brakeNotch = Vehicle::kEmergencyNotch;
    }
    if (cmd.emergency) { cmd.demand = 0.0f; cmd.powering = false; }

    // Gravity where each set actually stands, and the hand push. A train of two sets
    // is 84 m long and its halves are genuinely on different parts of the profile; the
    // pose is to hand either way, so it costs a loop to be right about it. Applied
    // before the sets are stepped, because the transmission picks its gear off the
    // speed and should see this step's, not the last one's.
    float totalMass = 0.0f, gravForce = 0.0f;
    for (const Vehicle& u : units_) {
        totalMass += u.mass();
        if (u.state() == VehicleState::OnRail)
            gravForce += u.mass() * (-kG * u.pose().tangent.z *
                                     static_cast<float>(u.orientation()));
    }
    if (totalMass <= 0.0f) return;
    if (state() == VehicleState::OnRail)
        v_ += (gravForce + pushInput * kPushForce) / totalMass * dt;

    // The brake pipe is one pipe. Each set works its own length of it, but the hoses
    // between them are open, so air moves from the fuller length to the emptier. That is
    // what makes the pipe continuous over the whole train and what gives a disturbance at
    // one end a finite time to reach the other - a coupled set whose own valve is doing
    // nothing still loses its pipe when the set in front dumps.
    //
    // The ends of the train are closed cocks and hold whatever is in them. It is only
    // when a coupling is parted that an end is opened to atmosphere, and uncoupleAfter
    // does that explicitly.
    for (int pass = 0; pass < 2; ++pass) // both directions, so it is not order-dependent
        for (std::size_t i = 1; i < units_.size(); ++i) {
            Vehicle& a = units_[pass ? units_.size() - i : i - 1];
            Vehicle& b = units_[pass ? units_.size() - i - 1 : i];
            const float flow = kPipeCoupling * (b.bpPressure() - a.bpPressure()) * dt;
            a.nudgeBrakePipe(flow);
            b.nudgeBrakePipe(-flow);
        }

    // Step every set's own subsystems and collect what each is contributing. This is
    // where the sets stay independent: each works its own length of pipe from its own
    // reservoir, each bogie's distributor fills its own cylinder from its own auxiliary,
    // and each set revs its own engines.
    float totalTE = 0.0f, totalBrake = 0.0f;
    for (Vehicle& u : units_) {
        const UnitStep st = u.stepSubsystems(dt, cmd, v_);
        totalTE += st.tractiveEffort;
        totalBrake += st.brakeForce;
    }
    tractiveEffort_ = totalTE;
    brakeForce_ = std::min(totalBrake, kAdhesionMu * totalMass * kG);

    if (state() != VehicleState::OnRail) {
        for (Vehicle& u : units_) u.stepDerailed(dt);
        v_ = 0.0f;
        return;
    }

    // Every set's pull adds into the one speed, then the brakes and the running
    // resistance come off it, capped so they cannot drag the train backwards (which is
    // also what holds it at rest, up to the grade the brakes can hold).
    v_ += totalTE / totalMass * dt;
    const float resistDecel = (brakeForce_ + rollingResistance(v_)) / totalMass;
    const float resist = std::min(resistDecel * dt, std::abs(v_));
    v_ -= std::copysign(resist, v_);

    // Move every set the same physical distance, each along its own path and over its
    // own turnouts. A set diverted by the points therefore takes the divergence itself,
    // and a switch thrown under the train does to the trailing set exactly what it
    // would do to a train on its own.
    const float ds = v_ * dt;
    bool derailed = false;
    for (Vehicle& u : units_) derailed = u.advance(ds) || derailed;
    if (derailed) {
        // Coupled sets do not part company: one set off the road takes the train with
        // it. What ought to happen to a train cut in half is the uncoupling step's
        // question, not this one's.
        for (Vehicle& u : units_) u.stepDerailed(dt);
        return;
    }

    // The coupler: measure where each set has ended up against where the set in front
    // says it should be, and put the difference right along its own path. Both sets
    // moved the same distance, so this only ever takes up the centimetres a turnout
    // crossing leaves behind - unless the points have taken a set away entirely, and
    // then the residual is metres and the train has been split.
    for (std::size_t i = 1; i < units_.size(); ++i) {
        Vehicle::TrackAnchor want;
        if (!units_[i - 1].anchorAtOffset(unitPitch(), want)) continue;
        if (want.pathIdx != units_[i].pathIdx()) {
            std::printf("[Consist] set %zu is no longer on the set in front's road - "
                        "the points have split the train\n", i + 1);
            continue;
        }
        const float slip = want.s - units_[i].s();
        if (std::abs(slip) > kCouplerSlack) {
            std::printf("[Consist] coupler between sets %zu and %zu is %.2f m out\n",
                        i, i + 1, slip);
            continue;
        }
        units_[i].nudge(slip);
    }
}

// --- the whole train ------------------------------------------------------------

VehicleState Consist::state() const {
    // The train is only on the rails while every set of it is.
    for (const Vehicle& u : units_)
        if (u.state() != VehicleState::OnRail) return u.state();
    return VehicleState::OnRail;
}

float Consist::mass() const {
    float m = 0.0f;
    for (const Vehicle& u : units_) m += u.mass();
    return m;
}

float Consist::length() const {
    const int n = unitCount();
    return static_cast<float>(n) * lead().length() +
           static_cast<float>(n - 1) * kCouplerGap;
}

float Consist::rollingResistance(float speed) const {
    // Davis: the rolling and flange terms go with the weight and so with the whole
    // train; the aerodynamic term does not, because only the leading set meets the
    // air. Taking one set's resistance and multiplying by the number of sets would
    // charge the train for pushing the same air twice.
    // Written as "one set, plus what each extra set adds" rather than as a multiple
    // with the drag taken back out, so a train of one comes to exactly the number a
    // lone set would give. Subtracting a term and adding it back does not round-trip
    // in floating point, and a consist of one has to be the old vehicle to the bit.
    const float one = lead().rollingResistance(speed);
    const float drag = lead().dragResistance(speed);
    return one + static_cast<float>(unitCount() - 1) * (one - drag);
}

GravityResolution Consist::gravity() const { return lead().gravity(); }

glm::vec3 Consist::inertia() const {
    return static_cast<float>(unitCount()) * lead().inertia();
}

TippingLimit Consist::tippingLimit(float curvature, float cant) const {
    return lead().tippingLimit(curvature, cant); // a set tips on its own dimensions
}

float Consist::mrPressure(int cab) const {
    const int u = (cab >= 0 && cab < cabCount()) ? cab / 2 : 0;
    return units_[u].mrPressure();
}
float Consist::bcPressure(int cab) const {
    const int u = (cab >= 0 && cab < cabCount()) ? cab / 2 : 0;
    return units_[u].bcPressure();
}

float Consist::bcRate() const {
    // The loudest airflow anywhere on the train: the sound is one sound, and a release
    // running through both sets is heard as the nearer of them.
    float r = 0.0f;
    for (const Vehicle& u : units_)
        if (std::abs(u.bcRate()) > std::abs(r)) r = u.bcRate();
    return r;
}

float Consist::bpRate() const {
    // The loudest length of pipe. Under EP every set vents at once so they agree, but a
    // pipe that has been cut at one end does not, and then the sound belongs to the end
    // that is actually losing air.
    float r = 0.0f;
    for (const Vehicle& u : units_)
        if (std::abs(u.bpRate()) > std::abs(r)) r = u.bpRate();
    return r;
}

float Consist::bpPressure(int cab) const {
    const int u = (cab >= 0 && cab < cabCount()) ? cab / 2 : 0;
    return units_[static_cast<std::size_t>(u)].bpPressure();
}

unsigned Consist::railImpacts() const {
    unsigned n = 0;
    for (const Vehicle& u : units_) n += u.railImpacts();
    return n;
}

int Consist::axleCount() const {
    int n = 0;
    for (const Vehicle& u : units_) n += static_cast<int>(u.axleOffsets().size());
    return n;
}

bool Consist::compressorRunning() const {
    for (const Vehicle& u : units_)
        if (u.compressorRunning()) return true;
    return false;
}

// --- engines --------------------------------------------------------------------

void Consist::toggleEngines() {
    // The link starts and stops the train's engines together, which is the point of
    // it. If any set has them running the command is "stop", otherwise "start" - so a
    // train found with one set shut down comes to agree with itself either way.
    bool anyOn = false;
    for (const Vehicle& u : units_) anyOn = anyOn || u.enginesOn();
    for (Vehicle& u : units_)
        if (u.enginesOn() == anyOn) u.toggleEngines();
}

int Consist::engineCount() const {
    int n = 0;
    for (const Vehicle& u : units_) n += u.engineCount();
    return n;
}

float Consist::engineRpm(int i) const {
    if (i < 0) return 0.0f;
    for (const Vehicle& u : units_) {
        if (i < u.engineCount()) return u.engineRpm(i);
        i -= u.engineCount();
    }
    return 0.0f;
}

EngineState Consist::engineState(int i) const {
    if (i < 0) return EngineState::Off;
    for (const Vehicle& u : units_) {
        if (i < u.engineCount()) return u.engineState(i);
        i -= u.engineCount();
    }
    return EngineState::Off;
}

bool Consist::enginesRunning() const {
    for (const Vehicle& u : units_)
        if (!u.enginesRunning()) return false;
    return true;
}

// --- geometry -------------------------------------------------------------------

std::vector<VehicleFrame> Consist::axleFrames() const {
    std::vector<VehicleFrame> out;
    for (const Vehicle& u : units_) {
        const std::vector<VehicleFrame> f = u.axleFrames();
        out.insert(out.end(), f.begin(), f.end());
    }
    return out;
}

std::vector<VehicleFrame> Consist::bogieFrames() const {
    std::vector<VehicleFrame> out;
    for (const Vehicle& u : units_) {
        const std::vector<VehicleFrame> f = u.bogieFrames();
        out.insert(out.end(), f.begin(), f.end());
    }
    return out;
}

std::vector<VehicleFrame> Consist::bodySectionFrames() const {
    std::vector<VehicleFrame> out;
    for (const Vehicle& u : units_) {
        const std::vector<VehicleFrame> f = u.bodySectionFrames();
        out.insert(out.end(), f.begin(), f.end());
    }
    return out;
}

bool Consist::occupiedSpans(std::vector<PathSpan>& out) const {
    out.clear();
    if (units_.empty()) return false;
    bool ok = true;

    // The whole train in one walk, from the first set's centre: back to its rear-most axle,
    // and forward past every set behind it to the last one's front-most axle. Done this way
    // rather than set by set because the rails *between* two sets are under the train too,
    // and only a walk that spans the coupler covers them.
    const std::vector<float> offs = units_.front().axleOffsets();
    if (!offs.empty()) {
        const auto mm = std::minmax_element(offs.begin(), offs.end());
        const float rear = *mm.first;
        const float front = static_cast<float>(unitCount() - 1) * unitPitch() + *mm.second;
        std::vector<PathSpan> whole;
        if (!units_.front().spansBetween(rear, front, whole)) ok = false;
        out.insert(out.end(), whole.begin(), whole.end());
    }

    // And each set on its own account. Normally this is already covered by the walk above,
    // but the points can split a train - the sets end up on roads that do not lead to one
    // another, which the coupler loop reports and does not repair - and then the single
    // walk follows the road the switches happen to be set for rather than the one each set
    // is actually standing on. Every set holding its own circuits is the safe reading.
    std::vector<PathSpan> one;
    for (const Vehicle& u : units_) {
        if (!u.occupiedSpans(one)) ok = false;
        out.insert(out.end(), one.begin(), one.end());
    }

    Vehicle::coalesceSpans(out);
    return ok;
}

std::vector<float> Consist::axleOffsets() const {
    // Each set's own offsets, shifted to where that set sits in the train.
    std::vector<float> out;
    const float pitch = unitPitch();
    const float first = -0.5f * static_cast<float>(unitCount() - 1) * pitch;
    for (int i = 0; i < unitCount(); ++i) {
        const float centre = first + static_cast<float>(i) * pitch;
        for (float o : units_[i].axleOffsets()) out.push_back(centre + o);
    }
    return out;
}
