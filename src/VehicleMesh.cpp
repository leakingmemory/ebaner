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

#include "VehicleMesh.h"

#include "Consist.h"

#include "Vehicle.h" // Vehicle, VehicleFrame

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace {
constexpr float kGauge = 1.435f;
constexpr float kWheelWidth = 0.135f;
constexpr float kAxleRadius = 0.06f;
constexpr float kAxleHalf = kGauge * 0.5f + kWheelWidth; // axle ends outside wheels
const glm::vec3 kAxleCol(0.36f, 0.36f, 0.38f);  // steel
const glm::vec3 kWheelCol(0.32f, 0.30f, 0.30f); // steel with a rusty tinge
const glm::vec3 kFrameCol(0.30f, 0.31f, 0.34f); // bogie frame steel
const glm::vec3 kUnderframeCol(0.24f, 0.25f, 0.27f); // carriage solebar/floor
constexpr int kSeg = 20;                        // segments per circle
constexpr float kPi = 3.14159265358979f;
// Bogie frame box, above the wheels.
constexpr float kFrameHalfWidth = 1.05f;  // across the track (m)
constexpr float kFrameHalfHeight = 0.18f; // vertical (m)
// Carriage underframe (floor plate) resting on the two bogies.
constexpr float kUnderframeHalfHeight = 0.15f; // thickness/2 (m)

// NSB Di 4 (Henschel, 1981) - the Nordlandsbanen locomotive. A Co'Co' with a full-width
// hood and a cab at each end, in the NSB red and black of its day.
namespace di4 {
const glm::vec3 kBody(0.62f, 0.11f, 0.12f);   // NSB red
const glm::vec3 kRoof(0.60f, 0.61f, 0.62f);   // light grey engine-room roof
const glm::vec3 kCabRoof(0.17f, 0.17f, 0.18f);// the cab roofs, near black
const glm::vec3 kSkirt(0.14f, 0.14f, 0.15f);  // solebar and valance
const glm::vec3 kGlass(0.13f, 0.15f, 0.18f);  // cab glazing
const glm::vec3 kFrame(0.10f, 0.10f, 0.11f);  // window surrounds, handrails
const glm::vec3 kLight(0.95f, 0.93f, 0.80f);  // headlight lens
const glm::vec3 kGrille(0.20f, 0.20f, 0.21f); // radiator and engine-room louvres
const glm::vec3 kStripe(0.91f, 0.86f, 0.70f);  // the cream bands along the sides
const glm::vec3 kPlough(0.86f, 0.70f, 0.09f); // the snowplough, yellow
// The end is raked in *elevation*: the plough end of the nose stands furthest forward and
// the front leans back as it rises, folding along horizontal lines. Not a wedge pointed in
// plan - that was the first attempt at this and it is the wrong axis altogether.
// The end profile, taken off a side elevation drawn over a render. The roof runs at one
// height for the whole length - there is no step down behind the cab, and what looks like
// a lower roof in photographs is the hatch panel *recessed* between the cantrails.
//
// The nose has a chin. Its furthest-forward point is the knee, a little over half way up:
// above it the screen rakes back to the roof edge, and below it the front tucks back in
// again to a short vertical skirt above the buffer beam. Getting that the other way up -
// furthest forward at the bottom, setting back all the way to the roof - is what makes it
// look like a wedge instead of a locomotive.
constexpr float kCabLen = 2.60f;    // cab, from the body end to the point of the nose
constexpr float kRoofEdgeAt = 0.91f; // roof edge, back from the nose point
constexpr float kChinAt = 0.55f;     // the chin's kink and the skirt, likewise
// Where the knee sits decides whether the cab works, because the knee *is* the windscreen
// sill: put it high and the glass ends up above a seated driver's eye, looking at bodywork.
// Scaled off the side elevation between the roof and the railhead - measuring to the buffer
// beam instead, as this first did, shortens the ruler and drives the whole nose up a metre.
// The winter three-quarter view agrees: sill 2.4 m over rail, screen head 3.4 m, and a good
// metre of red surround and grey cowl above that before the roof.
constexpr float kKneeFrac = 0.34f;   // knee height, of the body's above the floor
constexpr float kChinFrac = 0.04f;   // and the kink below it, just clear of the solebar
constexpr float kNoseWFrac = 0.96f;  // the front is a touch narrower than the body
// Seen head on the roof is much narrower than the body, with the shoulders chamfered down
// to the sides - the flanks tumble home at the top. Taken off a front elevation drawn over
// a render: the roof is about seven tenths of the body's width, and the chamfer takes a
// fifth of the body's height. It is also why the grey roof panel has red either side of it
// in an overhead photograph, which is the shoulder and not a stripe.
constexpr float kRoofWFrac = 0.69f;
constexpr float kShoulderFrac = 0.20f;
// The cab side glazing, off the side elevation: two lights a side between the windscreen
// and the cab bulkhead, level with the windscreen and with their heads a little under the
// shoulder, where the flank is still flat. Aft is the rectangular door window; forward of
// it, past a narrow pillar, a quarter-light whose leading edge rakes back as it rises, so
// it stands on the sill as a wedge. All measured forward from the bulkhead.
constexpr float kSideWinBack = 0.26f;  // door window, rear edge
constexpr float kSideWinFront = 0.88f; // and its front edge
constexpr float kQtrBack = 1.06f;      // quarter-light, rear edge - a pillar's width on
constexpr float kQtrFrontLo = 1.60f;   // its forward corner at the sill
constexpr float kQtrFrontHi = 1.36f;   // and at the head, which is the rake
constexpr float kSideWinLo = 1.06f;    // sill and head, above the cab floor
constexpr float kSideWinHi = 1.90f;
// Seated eye above the cab floor, and the seat under it. One constant, because the mesh
// and the camera both need it and a cab whose driver is drawn somewhere other than where
// he looks from is wrong in a way nothing in it can show.
constexpr float kEyeAboveFloor = 1.37f;
constexpr float kSeatCushion = 0.56f;
constexpr float kRoofThick = 0.10f;
constexpr float kBodyRise = 0.20f;   // underframe top to body floor
// The older coupling: side buffers on 1.75 m centres and a screw coupling between them,
// not the centre Scharfenberg the railcar carries.
// The windscreen within the raked panel, as a fraction of knee to roof. The cab reads
// these too - the sill is where the desk stops and the console starts, so the inside and
// the outside have to agree about it or the driver ends up looking at bodywork.
constexpr float kScreenLoV = 0.02f;
constexpr float kScreenHiV = 0.52f;
// Inside. White walls, pillars and ceiling, a dark floor and dark seats, and one flat desk
// in a mid steel blue that is the only real colour in the cab.
const glm::vec3 kCabWall(0.86f, 0.86f, 0.84f);
const glm::vec3 kCabFloor(0.15f, 0.15f, 0.16f);
const glm::vec3 kDesk(0.20f, 0.27f, 0.42f);
const glm::vec3 kDash(0.12f, 0.12f, 0.13f);
const glm::vec3 kSeat(0.11f, 0.11f, 0.12f);
const glm::vec3 kScreen(0.28f, 0.40f, 0.33f);
const glm::vec3 kGauge(0.85f, 0.85f, 0.82f);   // the two large faces, pale
const glm::vec3 kGaugeS(0.09f, 0.09f, 0.10f);  // the small ones, black
const glm::vec3 kNeedle(0.08f, 0.08f, 0.09f);
const glm::vec3 kNeedleL(0.90f, 0.90f, 0.86f); // a needle on a black face
const glm::vec3 kRed(0.78f, 0.12f, 0.10f);
const glm::vec3 kButton(0.26f, 0.26f, 0.28f);
const glm::vec3 kBlind(0.30f, 0.30f, 0.31f);   // the roller sun blinds over the screens
constexpr float kRevFull = 1000.0f; // rev counter full scale, over a 900 rpm governor
constexpr float kBufferHalfSpacing = 0.875f;
constexpr float kBufferR = 0.19f;
} // namespace di4

// NSB Class 93 (Bombardier Talent) exterior, classic NSB livery.
// NSB Type 5 (Strommens Vaerksted, 1977-81) passenger carriage, in the red-and-grey the
// class wore for most of its working life behind these locomotives. 25.3 m on two two-axle
// bogies. A slab-sided coach: flat flanks with the roof curving in above the cantrail, a
// continuous window band between two end vestibules, and a plug door at each end - the one
// on the left wider, which is what the wheelchair rebuild of a BC5-3 needed.
namespace t5 {
const glm::vec3 kBody(0.70f, 0.71f, 0.73f);  // silver-grey flanks
const glm::vec3 kBand(0.10f, 0.11f, 0.13f);  // dark window band
const glm::vec3 kRed(0.72f, 0.11f, 0.13f);   // NSB red: doors and the lower band
const glm::vec3 kRoof(0.52f, 0.53f, 0.55f);  // grey roof
const glm::vec3 kUnder(0.16f, 0.17f, 0.18f); // solebar and underfloor gear
const glm::vec3 kEnd(0.20f, 0.21f, 0.22f);   // the ends, in shadow between vehicles
constexpr float kFloorAbove = 0.15f; // underframe top to floor
constexpr float kWinLow = 1.00f;     // window band, above the floor
constexpr float kWinHigh = 1.90f;
constexpr float kCant = 2.15f;       // where the roof starts curving in
constexpr float kRoofHalf = 0.62f;   // roof half width, of the body's
constexpr float kRedBand = 0.34f;    // the red skirt, above the solebar
// Where the windows and the doors actually are, measured off the side elevation and the
// seat plan that Norske tog publish for this carriage - the SVGs behind the interactive
// diagrams, which are drawings and not pictures, so these are the real spacings rather
// than a guess at "a continuous band". Metres from the middle of the carriage; the body
// is 25.30 m, so the ends are at +/-12.65.
//
// Nine saloon windows in two groups, four and five, with 2.2 m of blank side between them
// at the centre. The blank is in the drawing and it is where the two table bays sit.
// Two variants of the same body, each with its own window and door spacing and its own
// furniture. Held as data rather than branches, because they differ only in where things
// are: a Type 5 is a Type 5, and the seat plan is what makes one a family carriage and
// another a cafe.
enum Inside {
    InsideSeats,   // a plain saloon end to end (B5-3)
    InsideFamily,  // the same, cut short for a playroom and wheelchair bays (BC5-3)
    InsideCafe,    // booths, a servery and a stowage bay (FR5-1)
    InsideSleeper, // compartments down one side and a corridor down the other (WLAB-2)
};

struct Layout {
    const float (*windows)[2]; // spans, metres from the middle of the carriage
    int windowCount;
    float doorAt[2];   // door centres; not at the ends on the cafe, which is the point
    float doorHalf[2]; // half widths - the BC5-3's wheelchair end is wider
    Inside inside;
    const float* seatRows; // where the rows are, off the seat plan
    int seatRowCount;
    float saloon0, saloon1; // partitions closing the saloon off
    float serviceAt;        // a steward's service point, 0 for none
};

// BC5-3: nine saloon windows in two groups, four and five, with 2.2 m of blank side
// between them at the centre where the two table bays sit.
constexpr float kWinBC53[][2] = {
    {-7.13f, -6.30f}, {-5.80f, -4.57f}, {-4.07f, -2.84f}, {-2.33f, -1.11f},
    {1.12f, 2.34f},   {2.85f, 4.08f},   {4.58f, 5.81f},   {6.32f, 7.54f},
    {8.03f, 9.26f},
};
// FR5-1: quite a different carriage from outside. Five windows over the cafe seating at
// one end, then a long blank flank where the servery and the counters are, three narrow
// ones over the far counter, and another blank over the bike and ski bay. Its two doors
// are nowhere near the ends - they are at -1.4 m and +7.6 m, which is where the plan puts
// its exits, either side of the servery.
constexpr float kWinFR51[][2] = {
    {-11.69f, -11.03f}, {-10.56f, -9.74f}, {-9.23f, -8.00f}, {-7.13f, -5.91f},
    {-4.79f, -3.56f},   {2.18f, 3.01f},    {3.18f, 4.01f},   {4.17f, 5.00f},
};
// The BC5-3 inside, from its seat plan: nine rows of four - two a side across a centre
// aisle - the middle two facing across tables.
constexpr float kRowsBC53[] = {-5.50f, -4.37f, -3.45f, -2.53f, -0.65f,
                               0.65f,  2.51f,  3.45f,  4.61f};
// And the B5-3's: the same shell with the whole of it given over to seating, so seventeen
// rows and sixty-eight seats where the family carriage has nine rows and thirty-six. The
// windows and doors are identical - the BC5-3 was rebuilt out of one of these - and what
// makes them different carriages is entirely what is inside.
constexpr float kRowsB53[] = {-7.17f, -6.20f, -5.28f, -4.37f, -3.45f, -2.53f,
                              -0.65f, 0.73f,  2.51f,  3.43f,  4.36f,  5.29f,
                              6.22f,  7.14f,  8.07f,  9.00f,  9.93f};

// A5-1, the 1st class comfort coach, and the shell is not quite the others': there is a
// window across the CENTRE where every 2nd class variant has 2.2 m of blank side, and an
// extra short one at the far end. Inside, twelve rows of four instead of seventeen - 48
// seats - spaced 1.27 m apart where 2nd class sits at 0.92, which is what the extra
// legroom looks like from a drawing. A steward's service point takes the middle.
constexpr float kWinA51[][2] = {
    {-7.13f, -6.31f}, {-5.81f, -4.58f}, {-4.08f, -2.85f}, {-2.34f, -1.11f},
    {-0.61f, 0.62f},  {1.12f, 2.35f},   {2.85f, 4.08f},   {4.59f, 5.82f},
    {6.32f, 7.54f},   {8.03f, 9.26f},   {9.86f, 10.46f},
};
constexpr float kRowsA51[] = {-6.89f, -5.63f, -4.36f, -3.04f, -1.77f, -0.51f,
                              3.03f,  4.36f,  5.63f,  6.89f,  8.16f,  9.48f};

constexpr Layout kBC53{kWinBC53, 9, {-11.36f, 11.36f}, {0.46f, 0.62f},
                       InsideFamily, kRowsBC53, 9, -8.30f, 5.30f, 0.0f};
constexpr Layout kB53{kWinBC53, 9, {-11.36f, 11.36f}, {0.46f, 0.46f},
                      InsideSeats, kRowsB53, 17, -8.30f, 10.60f, 0.0f};
constexpr Layout kA51{kWinA51, 11, {-11.36f, 11.36f}, {0.46f, 0.46f},
                      InsideSeats, kRowsA51, 12, -8.30f, 10.60f, 1.30f};
constexpr Layout kFR51{kWinFR51, 8, {-1.39f, 7.59f}, {0.46f, 0.46f},
                       InsideCafe, nullptr, 0, -12.0f, 12.0f, 0.0f};

// WLAB-2, the sleeping car - Strommen again, but 1986-87 and not a Type 5 at all: 27.0 m
// on a 3.24 m body, half a metre longer and wider than the coaches it runs with. Its side
// says exactly what it is. Fifteen small windows of 1.05 m at an even 1.385 m pitch, one
// for each compartment, which is the fourteen sovekupeer and the one HC sovekupe the
// builder lists; a door at each end at +/-12.09, which is where the drawing's door outline
// falls to the centimetre; and a short window beyond each door for the end vestibule.
constexpr float kWinWLAB[][2] = {
    {-13.15f, -12.76f}, {-8.93f, -7.88f},  {-7.55f, -6.50f}, {-6.14f, -5.10f},
    {-4.75f, -3.70f},   {-3.37f, -2.32f},  {-2.00f, -0.96f}, {-0.57f, 0.48f},
    {0.81f, 1.86f},     {2.18f, 3.22f},    {3.57f, 4.62f},   {4.94f, 5.98f},
    {6.33f, 7.38f},     {7.71f, 8.75f},    {9.12f, 10.16f},  {10.47f, 11.52f},
    {12.76f, 13.15f},
};
constexpr Layout kWLAB2{kWinWLAB, 17, {-12.09f, 12.09f}, {0.56f, 0.56f},
                        InsideSleeper, nullptr, 0, -9.70f, 12.20f, 0.0f};

constexpr float kWcAt = -10.40f; // the accessible WC, behind the partition at one end
// And the playroom, which on the family carriage takes the last four metres of saloon:
// the seats stop at +4.6 and it runs +5.8 to +10.2, with the wheelchair bays beside it.
// Set at +10.9 to begin with, which put it out in the vestibule and left the two windows
// over it looking into an empty carriage - visible only by comparing it against the B5-3,
// whose seating does run that far back.
constexpr float kPlayAt = 8.00f;
constexpr float kChairBays[] = {7.00f, 9.40f};
// And the FR5-1's, likewise: booth seating at one end, the servery and its counters
// through the middle with stools along them, and the bike, ski and luggage bay beyond.
constexpr float kBoothRows[] = {-11.30f, -10.20f, -9.10f, -8.00f, -6.90f};
constexpr float kServery0 = -6.50f, kServery1 = 4.00f;
constexpr float kStowFrom = 5.00f, kStowTo = 10.10f;
const glm::vec3 kSeat(0.55f, 0.16f, 0.18f);   // NSB seat moquette
const glm::vec3 kTable(0.78f, 0.76f, 0.71f);  // table tops
const glm::vec3 kLining(0.80f, 0.80f, 0.78f); // interior walls and partitions
const glm::vec3 kFloorIn(0.28f, 0.28f, 0.30f);
} // namespace t5

namespace c93 {
const glm::vec3 kBody(0.72f, 0.73f, 0.75f);  // silver-grey car body
const glm::vec3 kBand(0.11f, 0.12f, 0.14f);  // dark window band / glazing
const glm::vec3 kRed(0.74f, 0.10f, 0.12f);   // NSB red: doors + cab front
const glm::vec3 kRoof(0.55f, 0.56f, 0.58f);  // grey roof
const glm::vec3 kUnder(0.18f, 0.19f, 0.21f); // dark underframe/floor
const glm::vec3 kLight(0.93f, 0.92f, 0.84f); // headlight lens
constexpr float kHalfWidth = 1.36f;  // body half width (m)
constexpr float kHeight = 2.75f;     // floor-to-roof (m)
constexpr float kFloorAbove = 0.05f; // body floor above the bogie frame top (m)
constexpr float kWinLow = 1.00f;     // window band bottom above floor (m)
constexpr float kWinHigh = 2.05f;    // window band top above floor (m)
constexpr float kCantAbove = 2.20f;  // cantrail (shoulder base) above floor (m)
constexpr float kRoofHalf = 0.74f;   // domed-roof half width, fraction of body
constexpr float kTumble = 0.90f;     // floor-line half width (tumblehome), fraction
constexpr float kNoseLen = 3.60f;    // raked cab overhang (m)
constexpr float kDoorWidth = 1.30f;  // passenger door width (m)
constexpr float kGangGap = 0.45f;    // half the inter-car gap for the bellows (m)
const glm::vec3 kEquip(0.27f, 0.28f, 0.30f); // underfloor equipment box
const glm::vec3 kTank(0.32f, 0.32f, 0.34f);  // underfloor tank / lighter box
const glm::vec3 kRoofKit(0.40f, 0.41f, 0.43f); // roof exhaust / cooling boxes
const glm::vec3 kSkirt(0.09f, 0.09f, 0.10f);   // black coupler skirt / valance
const glm::vec3 kCoupler(0.22f, 0.23f, 0.25f); // steel automatic coupler head
const glm::vec3 kFloor(0.42f, 0.42f, 0.45f);   // interior saloon floor
const glm::vec3 kStep(0.32f, 0.32f, 0.34f);    // interior step / riser
const glm::vec3 kWall(0.74f, 0.70f, 0.62f);    // interior cab partition wall
const glm::vec3 kLining(0.82f, 0.80f, 0.75f);  // interior wall lining (neutral)
const glm::vec3 kCeiling(0.87f, 0.87f, 0.88f); // interior ceiling lining (light)
const glm::vec3 kGlass(0.17f, 0.19f, 0.23f);   // window glazing seen from inside
const glm::vec3 kWood(0.52f, 0.37f, 0.23f);    // wood-tone module panels
const glm::vec3 kSeat(0.30f, 0.32f, 0.38f);    // passenger seat
const glm::vec3 kDash(0.14f, 0.15f, 0.17f);    // driver's desk / console
const glm::vec3 kScreen(0.13f, 0.22f, 0.30f);  // MFD / LCD panel
const glm::vec3 kGauge(0.82f, 0.81f, 0.77f);   // analog dial face
const glm::vec3 kButton(0.34f, 0.35f, 0.38f);  // desk control buttons
const glm::vec3 kEngBar(0.30f, 0.78f, 0.36f);  // engine progress-bar fill (green)
} // namespace
} // namespace

// One set's geometry, appended to whatever is already there. Everything below is
// asked of that set alone, which is what puts the cab noses at each set's own two ends
// and keeps the gangway bellows inside a set: between two coupled sets there is no
// gangway, there are two nose ends facing each other over their couplers.
void VehicleMesh::emitUnit(const Vehicle& vehicle) {
    const glm::vec2 uv(0.0f);
    // This vehicle's wheels. A locomotive on 1.10 m wheels stands higher than a railcar
    // on 0.84 m ones, and everything above - the bogie frame, the underframe, the body -
    // is measured up from the axle centre, so the radius has to be the vehicle's own and
    // not a constant shared by all of them.
    const float wheelR = vehicle.wheelRadius();
    const float axleZ = wheelset::kRailTopZ + wheelR;
    auto push = [&](const glm::vec3& p, const glm::vec3& n, const glm::vec3& c) {
        vertices_.push_back({p, n, c, uv, -1.0f});
    };

    // One wheelset (axle + two wheels) at the given on-rail frame.
    auto emitWheelset = [&](const VehicleFrame& fr) {
        const glm::vec3 X = fr.right, Y = fr.tangent, Z = fr.up;
        const glm::vec3 origin = fr.pos + Z * axleZ;
        auto worldPt = [&](float lx, float ly, float lz) {
            return origin + X * lx + Y * ly + Z * lz;
        };
        auto worldNrm = [&](float nx, float ny, float nz) {
            return glm::normalize(X * nx + Y * ny + Z * nz);
        };
        // Cylinder with axis along local X, centred at x = cx, radius r,
        // half-length halfLen; side + two end caps.
        auto emitCyl = [&](float cx, float r, float halfLen, const glm::vec3& color) {
            const float x0 = cx - halfLen, x1 = cx + halfLen;
            for (int k = 0; k < kSeg; ++k) {
                const float a0 = 2.0f * kPi * k / kSeg;
                const float a1 = 2.0f * kPi * (k + 1) / kSeg;
                const float c0 = std::cos(a0), s0 = std::sin(a0);
                const float c1 = std::cos(a1), s1 = std::sin(a1);
                const glm::vec3 n0 = worldNrm(0.0f, c0, s0);
                const glm::vec3 n1 = worldNrm(0.0f, c1, s1);
                const std::uint32_t base =
                    static_cast<std::uint32_t>(vertices_.size());
                push(worldPt(x0, r * c0, r * s0), n0, color);
                push(worldPt(x1, r * c0, r * s0), n0, color);
                push(worldPt(x1, r * c1, r * s1), n1, color);
                push(worldPt(x0, r * c1, r * s1), n1, color);
                indices_.push_back(base + 0);
                indices_.push_back(base + 1);
                indices_.push_back(base + 2);
                indices_.push_back(base + 0);
                indices_.push_back(base + 2);
                indices_.push_back(base + 3);
            }
            for (int e = 0; e < 2; ++e) {
                const float xe = (e == 0) ? x0 : x1;
                const glm::vec3 an = worldNrm((e == 0) ? -1.0f : 1.0f, 0.0f, 0.0f);
                const std::uint32_t centre =
                    static_cast<std::uint32_t>(vertices_.size());
                push(worldPt(xe, 0.0f, 0.0f), an, color);
                for (int k = 0; k < kSeg; ++k) {
                    const float a = 2.0f * kPi * k / kSeg;
                    push(worldPt(xe, r * std::cos(a), r * std::sin(a)), an, color);
                }
                for (int k = 0; k < kSeg; ++k) {
                    indices_.push_back(centre);
                    indices_.push_back(centre + 1 + k);
                    indices_.push_back(centre + 1 + (k + 1) % kSeg);
                }
            }
        };
        emitCyl(0.0f, kAxleRadius, kAxleHalf, kAxleCol);
        emitCyl(-kGauge * 0.5f, wheelR, kWheelWidth * 0.5f, kWheelCol);
        emitCyl(kGauge * 0.5f, wheelR, kWheelWidth * 0.5f, kWheelCol);
    };

    // Solid box centred at c with half-extents (hx,hy,hz) along frame axes X,Y,Z.
    auto emitBox = [&](const glm::vec3& X, const glm::vec3& Y, const glm::vec3& Z,
                       const glm::vec3& c, float hx, float hy, float hz,
                       const glm::vec3& col) {
        auto P = [&](float sx, float sy, float sz) {
            return c + X * (sx * hx) + Y * (sy * hy) + Z * (sz * hz);
        };
        auto face = [&](const glm::vec3& p0, const glm::vec3& p1,
                        const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& n) {
            const std::uint32_t b = static_cast<std::uint32_t>(vertices_.size());
            push(p0, n, col);
            push(p1, n, col);
            push(p2, n, col);
            push(p3, n, col);
            indices_.push_back(b + 0);
            indices_.push_back(b + 1);
            indices_.push_back(b + 2);
            indices_.push_back(b + 0);
            indices_.push_back(b + 2);
            indices_.push_back(b + 3);
        };
        face(P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), P(1, -1, 1), X);
        face(P(-1, -1, -1), P(-1, 1, -1), P(-1, 1, 1), P(-1, -1, 1), -X);
        face(P(-1, 1, -1), P(1, 1, -1), P(1, 1, 1), P(-1, 1, 1), Y);
        face(P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), P(-1, -1, 1), -Y);
        face(P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1), Z);
        face(P(-1, -1, -1), P(1, -1, -1), P(1, 1, -1), P(-1, 1, -1), -Z);
    };

    for (const VehicleFrame& fr : vehicle.axleFrames()) emitWheelset(fr);

    // Height of a bogie frame box centre / its top above the pose bed.
    const float frameCentreZ = axleZ + wheelR;
    const float frameTopZ = frameCentreZ + kFrameHalfHeight;

    // A bogie frame box (low steel box above the wheels, spanning the wheelbase).
    auto emitBogieFrame = [&](const VehicleFrame& b) {
        emitBox(b.right, b.tangent, b.up, b.pos + b.up * frameCentreZ,
                kFrameHalfWidth, 0.5f * vehicle.wheelbase(), kFrameHalfHeight,
                kFrameCol);
    };

    // A bogie frame per bogie (none for a bare wheelset).
    for (const VehicleFrame& bf : vehicle.bogieFrames()) emitBogieFrame(bf);

    // A flat quad (p0..p3); its normal is oriented to point away from `inside`
    // so winding order doesn't matter (the body is roughly convex).
    auto quadN = [&](const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                     const glm::vec3& p3, const glm::vec3& col,
                     const glm::vec3& inside) {
        glm::vec3 n = glm::cross(p1 - p0, p3 - p0);
        const float l = glm::length(n);
        n = (l > 1e-9f) ? n / l : glm::vec3(0.0f, 0.0f, 1.0f);
        const glm::vec3 cen = (p0 + p1 + p2 + p3) * 0.25f;
        if (glm::dot(n, cen - inside) < 0.0f) n = -n;
        const std::uint32_t b = static_cast<std::uint32_t>(vertices_.size());
        push(p0, n, col); push(p1, n, col); push(p2, n, col); push(p3, n, col);
        indices_.push_back(b + 0); indices_.push_back(b + 1); indices_.push_back(b + 2);
        indices_.push_back(b + 0); indices_.push_back(b + 2); indices_.push_back(b + 3);
    };

    // --- Instruments ------------------------------------------------------------------
    // A dashboard is drawn the same way whoever built the locomotive: a flat panel, a round
    // dial face, and a needle swept across it. These sat inside the Class 93's cab block
    // and so could not be reached from anywhere else; they are up here now because there is
    // a second cab to draw and one copy of this is enough. `C,u,v,n` are the panel's centre
    // and its two in-plane axes with its outward normal - so a gauge is placed in the
    // panel's own coordinates and does not care how the panel is angled in the cab.
    auto rectFace = [&](const glm::vec3& C, const glm::vec3& u, const glm::vec3& v,
                        const glm::vec3& n, float cx, float cy, float hu, float hv, float pr,
                        const glm::vec3& col) {
        const glm::vec3 o = C + u * cx + v * cy + n * pr;
        quadN(o - u * hu - v * hv, o + u * hu - v * hv, o + u * hu + v * hv,
              o - u * hu + v * hv, col, o - n);
    };
    auto discFace = [&](const glm::vec3& C, const glm::vec3& u, const glm::vec3& v,
                        const glm::vec3& n, float cx, float cy, float r, float pr,
                        const glm::vec3& col) {
        const glm::vec3 o = C + u * cx + v * cy + n * pr;
        const int M = 16;
        glm::vec3 prev = o + u * r;
        for (int i = 1; i <= M; ++i) {
            const float a = 2.0f * kPi * i / M;
            const glm::vec3 cur = o + u * (r * std::cos(a)) + v * (r * std::sin(a));
            quadN(o, prev, cur, cur, col, o - n);
            prev = cur;
        }
    };
    // A gauge needle: value fraction in [0,1] sweeps a 240-degree arc.
    auto needle = [&](const glm::vec3& C, const glm::vec3& u, const glm::vec3& v,
                      const glm::vec3& n, float cx, float cy, float r, float frac,
                      const glm::vec3& col) {
        const glm::vec3 o = C + u * cx + v * cy + n * 0.016f;
        const float a = kPi * (210.0f - 240.0f * std::clamp(frac, 0.0f, 1.0f)) / 180.0f;
        const glm::vec3 dir = u * std::cos(a) + v * std::sin(a);
        const glm::vec3 perp = -u * std::sin(a) + v * std::cos(a);
        const glm::vec3 tip = o + dir * r;
        quadN(o - perp * 0.006f, o + perp * 0.006f, tip, tip, col, o - n);
    };

    // One NSB Class 93 car-body section. `f` is the section frame (X=right,
    // Y=tangent, Z=up); the body spans y in [-halfLen, halfLen] with the raked
    // cab at the outer end (cabNegY picks which) and the articulation gangway at
    // the other. Silver body, dark window band, red doors, red raked cab front.
    auto emitClass93 = [&](const VehicleFrame& f, float halfLen, bool cabNegY,
                           bool hasWC) {
        const glm::vec3 X = f.right, Y = f.tangent, Z = f.up;
        const float hw = c93::kHalfWidth;
        const float rhw = hw * c93::kRoofHalf;         // domed-roof half width
        const float z0 = frameTopZ + c93::kFloorAbove; // floor
        const float z1 = z0 + c93::kHeight;            // roof top
        const float zwl = z0 + c93::kWinLow, zwh = z0 + c93::kWinHigh; // window band
        const float zc = z0 + c93::kCantAbove;         // cantrail (shoulder base)
        auto P = [&](float x, float y, float z) { return f.pos + X * x + Y * y + Z * z; };

        const float tip = cabNegY ? -halfLen : halfLen;                // cab tip
        const float base = cabNegY ? -halfLen + c93::kNoseLen          // nose base
                                   : halfLen - c93::kNoseLen;
        // Gangway end, inset from the section boundary to leave a bellows gap.
        const float gang = (cabNegY ? 1.0f : -1.0f) * (halfLen - c93::kGangGap);
        const float ts = (tip < base) ? -1.0f : 1.0f;                  // forward sign
        const glm::vec3 fdir = Y * ts;
        const float bodyLo = std::min(base, gang), bodyHi = std::max(base, gang);
        const float allLo = std::min(tip, gang), allHi = std::max(tip, gang);
        const glm::vec3 in = P(0.0f, 0.5f * (allLo + allHi), 0.5f * (z0 + z1));

        // Shared cross-section: a rounded body profile (open at the bottom; the
        // floor pan closes it). Straight sides carry window-band boundary points,
        // then rounded shoulder arcs sweep in to a domed roof narrower than the
        // waist. `wS` scales the width and `drop` lowers the roof (used to taper
        // and rake the nose into a windscreen). Facets are coloured by geometry,
        // so the ring resolution can change freely.
        // Outer skin sill: the side sheeting hangs low almost everywhere (nose
        // included), with a tight rectangular cut-up over each bogie to clear the
        // wheels. The two bogies in this car (section frame) are the leading/outer
        // one and the shared middle bogie at the gangway end.
        const float bc = 0.5f * vehicle.bogieSpacing();
        const float yBogieOuter = ts * (bc - halfLen);
        const float yBogieMid = -ts * halfLen;
        // The nose and the skin over both bogies share one fairly low end level
        // (nose flush with the over-bogie skin); only the mid-car, between the
        // bogies, dips lower as a valance. A rectangular step joins them.
        const float zEnd = frameCentreZ - 0.10f;   // nose + over-bogie skin level
        const float zLow = frameCentreZ - 0.36f;   // mid-car valance (lower)
        const float dipInset = 0.5f * vehicle.wheelbase() + 0.42f;
        const float dipA = std::min(yBogieOuter, yBogieMid) + dipInset;
        const float dipB = std::max(yBogieOuter, yBogieMid) - dipInset;
        auto sillAt = [&](float y) { return (y > dipA && y < dipB) ? zLow : zEnd; };
        const int arcN = 4;
        auto ring = [&](float y, float wS, float drop, float zb, float zbL, float zbH) {
            const float w = hw * wS, rw = rhw * wS;
            const float zr = z1 - drop;
            const float zC = std::min(zc, zr - 0.04f);
            const float zH = std::min(zbH, zC - 0.02f), zL = std::min(zbL, zH - 0.02f);
            const float wb = w * c93::kTumble; // narrower at the sill line
            std::vector<glm::vec3> p;
            p.push_back(P(wb, y, zb));          // tumblehome: side leans in to sill
            p.push_back(P(w, y, zL));
            p.push_back(P(w, y, zH));
            for (int i = 0; i <= arcN; ++i) { // right shoulder (w,zC) -> (rw,zr)
                const float a = kPi * 0.5f * i / arcN;
                p.push_back(P(rw + (w - rw) * std::cos(a), y, zC + (zr - zC) * std::sin(a)));
            }
            for (int i = arcN; i >= 0; --i) { // left shoulder (-rw,zr) -> (-w,zC)
                const float a = kPi * 0.5f * i / arcN;
                p.push_back(P(-(rw + (w - rw) * std::cos(a)), y, zC + (zr - zC) * std::sin(a)));
            }
            p.push_back(P(-w, y, zH));
            p.push_back(P(-w, y, zL));
            p.push_back(P(-wb, y, zb));
            return p;
        };
        // Loft between two rings, colouring each facet by geometry: grey domed
        // roof (up-facing), the window band at cantrail height in `bandCol`, and
        // the rest of the side in `lowerCol`. Nose facets are coloured by
        // orientation (grey roof, dark forward windscreen, red sides).
        auto loft = [&](const std::vector<glm::vec3>& A, const std::vector<glm::vec3>& B,
                        bool nose, const glm::vec3& lowerCol,
                        const glm::vec3& bandColR, const glm::vec3& bandColL) {
            for (std::size_t k = 0; k + 1 < A.size(); ++k) {
                const glm::vec3 a = A[k], b = A[k + 1], c = B[k + 1], d = B[k];
                glm::vec3 nn = glm::cross(b - a, d - a);
                const float l = glm::length(nn);
                nn = (l > 1e-9f) ? nn / l : Z;
                const glm::vec3 cen = 0.25f * (a + b + c + d);
                if (glm::dot(nn, cen - in) < 0.0f) nn = -nn;
                const float locZ = glm::dot(cen - f.pos, Z);
                glm::vec3 col;
                if (nose) {
                    // Classify nose facets by orientation: the forward-raked front
                    // is the windscreen (glazed); the near-horizontal crown is
                    // opaque roof; the sideways-facing panels are the solid cab
                    // sides, glazed only over the window band (a small side
                    // window); red fascia below the glazing line.
                    const float fwd = glm::dot(nn, fdir);
                    const float side = std::abs(glm::dot(nn, X));
                    if (locZ < z0 + 0.65f)
                        col = c93::kRed;                       // fascia
                    else if (fwd > 0.28f)
                        col = c93::kBand;                      // windscreen (glass)
                    else if (side > 0.5f)
                        col = (k == 1 || k == 13) ? c93::kBand // small side window
                                                  : c93::kBody; // solid cab side
                    else
                        col = c93::kRoof;                      // crown (opaque roof)
                } else if (k >= 3 && k <= 11) {
                    col = c93::kRoof; // shoulders + domed roof (by ring index)
                } else if (k == 1) {
                    col = bandColR;   // +x window band (glazing or blank)
                } else if (k == 13) {
                    col = bandColL;   // -x window band
                } else {
                    col = lowerCol;   // lower body / cantrail (silver or door)
                }
                quadN(a, b, c, d, col, in);
            }
        };

        // Floor pan and the gangway end. The end wall has a central aisle
        // doorway (a low-floor walk-through to the next car via the bellows).
        const float wf = hw * c93::kTumble;
        quadN(P(-wf, allLo, z0), P(wf, allLo, z0), P(wf, allHi, z0), P(-wf, allHi, z0), c93::kUnder, in);
        {
            const float aisle = 0.52f;          // half aisle-doorway width
            const float zFlr = z0 + 0.03f;      // gangway floor
            const float zDoor = zFlr + 2.0f;    // doorway top
            quadN(P(-wf, gang, z0), P(-aisle, gang, z0), P(-aisle, gang, z1), P(-wf, gang, z1), c93::kUnder, in);
            quadN(P(aisle, gang, z0), P(wf, gang, z0), P(wf, gang, z1), P(aisle, gang, z1), c93::kUnder, in);
            quadN(P(-aisle, gang, zDoor), P(aisle, gang, zDoor), P(aisle, gang, z1), P(-aisle, gang, z1), c93::kUnder, in);
            quadN(P(-aisle, gang, z0), P(aisle, gang, z0), P(aisle, gang, zFlr), P(-aisle, gang, zFlr), c93::kUnder, in);
        }

        // Main body: one passenger door per car, set inboard of the cab. The
        // window band steps in level at the door — higher over the raised cab
        // vestibule, lower over the low-floor saloon. Windows are glazed panels
        // separated by body-colour pillars.
        const float Lb = bodyHi - bodyLo;
        const float dhw = 0.5f * c93::kDoorWidth;
        // The floor/window level steps at the stairs; the exterior door sits on
        // the low floor, past the stairs (so it doesn't open onto them).
        const float yStep = base + (gang - base) * 0.36f;
        const float yDoor = base + (gang - base) * 0.46f;
        auto cabSide = [&](float y) { return std::abs(y - base) < std::abs(yStep - base); };
        auto bandLo = [&](float y) { return z0 + (cabSide(y) ? 1.18f : 0.82f); };
        auto bandHi = [&](float y) { return z0 + (cabSide(y) ? 2.05f : 1.68f); };
        // Boxed interior areas have no windows, so the glazing is blanked over
        // them: the tech cabinets behind the cab, the stairs, and (one car only)
        // the WC/utility module on the gangway side.
        const float dirSaloon = (gang > base) ? 1.0f : -1.0f;
        // WC/utility module: one side only (+x), directly beside the door,
        // extending toward the gangway (a long box).
        const float wc0 = yDoor + dirSaloon * (dhw + 0.05f);
        const float wc1 = wc0 + dirSaloon * std::abs(gang - base) * 0.40f;
        // Tech cabinets and the stairs block both sides; the WC blocks its side.
        const std::pair<float, float> solidsBoth[] = {
            {base, base + dirSaloon * 2.0f}, {yStep - 0.4f, yStep + 0.4f}};
        auto inRange = [&](const std::pair<float, float>& s, float y) {
            return y > std::min(s.first, s.second) && y < std::max(s.first, s.second);
        };
        auto inBoth = [&](float y) {
            for (const auto& s : solidsBoth)
                if (inRange(s, y)) return true;
            return false;
        };
        auto inWC = [&](float y) { return hasWC && inRange({wc0, wc1}, y); };
        // Blank the glazing over boxed areas (per side): +x facet vs -x facet.
        auto bandR = [&](float y) { return (inBoth(y) || inWC(y)) ? c93::kBody : c93::kBand; };
        auto bandL = [&](float y) { return inBoth(y) ? c93::kBody : c93::kBand; };
        struct Panel { float a, b; glm::vec3 lower, bandR, bandL; };
        std::vector<Panel> panels;
        auto tile = [&](float a, float b) { // fill [a,b] with windows + pillars
            const float m = 0.18f, winW = 1.35f, pilW = 0.32f;
            float x = a;
            panels.push_back({x, x + m, c93::kBody, c93::kBody, c93::kBody}); // margin
            x += m;
            while (b - m - x >= winW - 1e-3f) {
                // A window each side unless blanked by a boxed interior area.
                const float wcen = x + 0.5f * winW;
                panels.push_back({x, x + winW, c93::kBody, bandR(wcen), bandL(wcen)});
                x += winW;
                if (b - m - x >= winW + pilW) {
                    panels.push_back({x, x + pilW, c93::kBody, c93::kBody, c93::kBody}); // pillar
                    x += pilW;
                }
            }
            panels.push_back({x, b, c93::kBody, c93::kBody, c93::kBody}); // end margin
        };
        tile(bodyLo, yDoor - dhw);
        panels.push_back({yDoor - dhw, yDoor + dhw, c93::kRed, c93::kRed, c93::kRed}); // door
        tile(yDoor + dhw, bodyHi);
        // Loft each panel, subdividing at the bogie-notch edges (doubled with a
        // tiny gap) so the sill steps up near-vertically over each bogie.
        const float eps = 0.02f;
        const float edges[] = {dipA, dipB};
        auto bodyRing = [&](float y) {
            return ring(y, 1.0f, 0.0f, sillAt(y), bandLo(y), bandHi(y));
        };
        for (const Panel& p : panels) {
            std::vector<float> ys = {p.a, p.b};
            for (const float e : edges)
                if (e > p.a + eps && e < p.b - eps) {
                    ys.push_back(e - eps);
                    ys.push_back(e + eps);
                }
            std::sort(ys.begin(), ys.end());
            for (std::size_t i = 0; i + 1 < ys.size(); ++i)
                loft(bodyRing(ys[i]), bodyRing(ys[i + 1]), false, p.lower, p.bandR, p.bandL);
        }

        // Nose: taper width to a rounded prow while the roof rakes down into a
        // deep windscreen. Ease-in width and eased roof drop keep it smooth.
        const int N = 12;
        auto noseStation = [&](int i) {
            const float u = static_cast<float>(i) / N;
            const float y = base + (tip - base) * u;
            const float wS = std::sqrt(std::max(0.12f, 1.0f - 0.90f * u * u));
            const float drop = (z1 - (zwl + 0.05f)) * (u * u); // roof → window level
            // Nose skin follows the same sill; band levels are unused (nose is red).
            return ring(y, wS, drop, sillAt(y), zwl, zwh);
        };
        std::vector<glm::vec3> prev = noseStation(0);
        for (int i = 1; i <= N; ++i) {
            const std::vector<glm::vec3> cur = noseStation(i);
            loft(prev, cur, true, c93::kRed, c93::kRed, c93::kRed); // colours unused for nose
            prev = cur;
        }
        // Prow cap (fan the last narrow ring closed).
        { glm::vec3 c(0.0f);
          for (const glm::vec3& q : prev) c += q;
          c /= static_cast<float>(prev.size());
          const float czc = glm::dot(c - f.pos, Z);
          for (std::size_t k = 0; k < prev.size(); ++k) {
              const glm::vec3& a = prev[k];
              const glm::vec3& b = prev[(k + 1) % prev.size()];
              quadN(a, b, c, c, (czc > z0 + 0.55f * (z1 - z0)) ? c93::kBand : c93::kRed, in);
          } }

        // Headlights / marker lights set into the lower corners of the
        // windscreen glazing: compact lamp blocks just above the red fascia.
        const float yh = base + (tip - base) * 0.90f;
        for (const float xh : {hw * 0.42f, -hw * 0.42f})
            emitBox(X, Y, Z, P(xh, yh, z0 + 0.82f), 0.14f, 0.09f, 0.07f, c93::kLight);

        // Underfloor systems: a shallow equipment raft under the whole body plus
        // a couple of deeper boxes (engine / tank) slung between the bogies.
        emitBox(X, Y, Z, P(0.0f, 0.5f * (bodyLo + bodyHi), z0 - 0.24f),
                hw * 0.86f, 0.5f * Lb * 0.96f, 0.22f, c93::kEquip);
        emitBox(X, Y, Z, P(0.0f, bodyLo + 0.34f * Lb, z0 - 0.56f),
                hw * 0.82f, 0.20f * Lb, 0.40f, c93::kEquip);
        emitBox(X, Y, Z, P(0.0f, bodyLo + 0.66f * Lb, z0 - 0.50f),
                hw * 0.74f, 0.15f * Lb, 0.34f, c93::kTank);

        // Roof equipment: exhaust / cooling boxes along the car roof, sitting on
        // the domed roof crown.
        emitBox(X, Y, Z, P(0.0f, bodyLo + 0.32f * Lb, z1 + 0.13f),
                rhw * 0.74f, 0.11f * Lb, 0.13f, c93::kRoofKit);
        emitBox(X, Y, Z, P(0.0f, bodyLo + 0.58f * Lb, z1 + 0.16f),
                rhw * 0.60f, 0.08f * Lb, 0.16f, c93::kRoofKit); // taller (exhaust)
        emitBox(X, Y, Z, P(0.0f, bodyLo + 0.80f * Lb, z1 + 0.11f),
                rhw * 0.72f, 0.09f * Lb, 0.11f, c93::kRoofKit);

        // Coupler skirt: a black valance under the cab front around the coupler.
        emitBox(X, Y, Z, P(0.0f, base + (tip - base) * 0.86f, z0 - 0.28f),
                hw * 0.52f, (tip - base) * 0.15f, 0.30f, c93::kSkirt);

        // Automatic (Scharfenberg) coupler poking out of the skirt: a draft-gear
        // block, the coupler shaft, and a wider knuckle head at the very front.
        const float cz = z0 - 0.30f;
        emitBox(X, Y, Z, P(0.0f, tip + ts * 0.02f, cz), hw * 0.26f, 0.12f, 0.16f, c93::kSkirt);
        emitBox(X, Y, Z, P(0.0f, tip + ts * 0.22f, cz), 0.07f, 0.12f, 0.09f, c93::kCoupler);
        emitBox(X, Y, Z, P(0.0f, tip + ts * 0.38f, cz), hw * 0.20f, 0.06f, 0.15f, c93::kCoupler);

        // Two obstacle deflectors hanging below the floor: a large upper snowplow
        // under the cab front, and a lower lifeguard set back just ahead of the
        // leading bogie's wheelset, dropping to near rail level. Each is a dark
        // V-blade wedge (top edge back, sweeping down-forward to a low V point).
        {
            auto deflector = [&](float yBack, float fwd, float zTop, float zBot,
                                 float wT, float wB) {
                const float yF = yBack + ts * fwd;
                const glm::vec3 TL = P(-wT, yBack, zTop), TR = P(wT, yBack, zTop);
                const glm::vec3 TC = P(0.0f, yBack, zTop);
                const glm::vec3 BL = P(-wB, yF, zBot), BR = P(wB, yF, zBot);
                const glm::vec3 BC = P(0.0f, yF + ts * 0.14f, zBot - 0.06f); // V point
                const glm::vec3 nref = P(0.0f, yBack - ts * 0.4f, zTop + 0.4f);
                const glm::vec3 dref = P(0.0f, yF, zBot - 2.0f);
                quadN(TL, BL, BC, TC, c93::kSkirt, nref); // blade faces
                quadN(TC, BC, BR, TR, c93::kSkirt, nref);
                quadN(TL, TC, BC, BL, c93::kSkirt, dref); // undersides
                quadN(TC, TR, BR, BC, c93::kSkirt, dref);
            };
            // Upper snowplow under the cab front (wide), lower lifeguard back at
            // the leading bogie (narrower, near rail).
            deflector(tip + ts * -0.9f, 0.45f, z0 - 0.34f, z0 - 0.86f, hw * 0.98f, hw * 0.66f);
            deflector(yBogieOuter + ts * 1.5f, 0.35f, z0 - 0.62f, z0 - 1.06f, hw * 0.52f, hw * 0.34f);
        }

        // ---- Interior (seen by flying the camera inside the shell). The low
        // saloon floor sits between two raised end vestibules (over the bogies,
        // in front of the doors), joined by two-step risers; a partition wall
        // with a central aisle doorway closes off the driver's cab. ----
        {
            const float ihw = hw * 0.90f;             // interior half width
            const float zFl = z0 + 0.03f;             // saloon floor
            const float zSt = zFl + 0.19f;            // mid step
            const float zFh = zFl + 0.38f;            // raised vestibule floor
            const float so = (base < gang) ? -1.0f : 1.0f; // toward the cab
            const float aisle = 0.52f;                // half aisle width
            auto plate = [&](float ya, float yb, float z, const glm::vec3& col) {
                emitBox(X, Y, Z, P(0.0f, 0.5f * (ya + yb), z - 0.025f), ihw,
                        0.5f * std::abs(yb - ya), 0.025f, col);
            };

            // Interior lining: an inset shell (walls + domed ceiling) facing the
            // aisle, so the inside carries its own colours — neutral walls, light
            // ceiling, dark glazing at the windows — independent of the exterior.
            auto liningRing = [&](float y) {
                return ring(y, 0.955f, 0.06f, zFl, bandLo(y), bandHi(y));
            };
            auto liningLoft = [&](const std::vector<glm::vec3>& A, const std::vector<glm::vec3>& B) {
                for (std::size_t k = 0; k + 1 < A.size(); ++k) {
                    const glm::vec3 a = A[k], b = A[k + 1], c = B[k + 1], d = B[k];
                    glm::vec3 n = glm::cross(b - a, d - a);
                    const float l = glm::length(n);
                    n = (l > 1e-9f) ? n / l : Z;
                    const glm::vec3 cen = 0.25f * (a + b + c + d);
                    const glm::vec3 target = P(0.0f, glm::dot(cen - f.pos, Y), 0.5f * (zFl + z1));
                    if (glm::dot(n, target - cen) < 0.0f) n = -n; // face the aisle
                    const glm::vec3 col = (k == 1 || k == 13) ? c93::kGlass
                                          : (k >= 3 && k <= 11) ? c93::kCeiling
                                                                : c93::kLining;
                    const glm::vec2 uv(0.0f);
                    const std::uint32_t vb = static_cast<std::uint32_t>(vertices_.size());
                    push(a, n, col); push(b, n, col); push(c, n, col); push(d, n, col);
                    indices_.push_back(vb + 0); indices_.push_back(vb + 1); indices_.push_back(vb + 2);
                    indices_.push_back(vb + 0); indices_.push_back(vb + 2); indices_.push_back(vb + 3);
                }
            };
            {
                const int steps = 24;
                std::vector<glm::vec3> prev = liningRing(bodyLo);
                for (int i = 1; i <= steps; ++i) {
                    std::vector<glm::vec3> cur =
                        liningRing(bodyLo + (bodyHi - bodyLo) * i / steps);
                    liningLoft(prev, cur);
                    prev = cur;
                }
            }

            // The cab end is raised (over the leading bogie); the low saloon floor
            // runs straight through the gangway (low-floor articulation). The step
            // is at the door; two risers with a mid tread step up toward the cab.
            const float yTop = yStep + so * 0.30f; // top of the steps (raised side)
            plate(base, yTop, zFh, c93::kFloor);   // raised cab vestibule
            plate(yStep, gang, zFl, c93::kFloor);  // saloon + low-floor gangway
            emitBox(X, Y, Z, P(0.0f, yStep, 0.5f * (zFl + zSt)), ihw, 0.02f, 0.5f * (zSt - zFl), c93::kStep);
            plate(yStep, yTop, zSt, c93::kStep);   // mid tread
            emitBox(X, Y, Z, P(0.0f, yTop, 0.5f * (zSt + zFh)), ihw, 0.02f, 0.5f * (zFh - zSt), c93::kStep);
            // Interior ceiling height at lateral x (follows the domed roof).
            auto ceilingAt = [&](float x) {
                const float ax = std::abs(x), zCrown = z1 - 0.12f;
                if (ax <= rhw) return zCrown;
                return zCrown + (zc - zCrown) * std::min(1.0f, (ax - rhw) / (hw - rhw));
            };
            // A floor-to-roof partition from a 4-point footprint; the top follows
            // the domed ceiling, walls oriented outward from the footprint centre.
            auto partition = [&](const std::array<glm::vec2, 4>& fp, float zFloor,
                                 const glm::vec3& col) {
                glm::vec2 c(0.0f);
                for (const glm::vec2& q : fp) c += q;
                const glm::vec3 ref = P(0.25f * c.x, 0.25f * c.y, zFloor + 1.0f);
                for (int i = 0; i < 4; ++i) {
                    const glm::vec2 a = fp[i], b = fp[(i + 1) % 4];
                    quadN(P(a.x, a.y, zFloor), P(b.x, b.y, zFloor),
                          P(b.x, b.y, ceilingAt(b.x)), P(a.x, a.y, ceilingAt(a.x)), col, ref);
                }
                quadN(P(fp[0].x, fp[0].y, ceilingAt(fp[0].x)), P(fp[1].x, fp[1].y, ceilingAt(fp[1].x)),
                      P(fp[2].x, fp[2].y, ceilingAt(fp[2].x)), P(fp[3].x, fp[3].y, ceilingAt(fp[3].x)), col, ref);
            };

            // Box in the stairs: full-height walls each side, right out to the
            // body sides (only the aisle steps through), saloon-facing end angled.
            for (const float sx : {1.0f, -1.0f})
                partition({glm::vec2(sx * aisle, yStep - so * 0.22f), glm::vec2(sx * ihw, yStep),
                           glm::vec2(sx * ihw, yTop), glm::vec2(sx * aisle, yTop)}, zFl, c93::kWall);

            // Cab partition: full-height side panels (domed top) either side of a
            // central aisle doorway, with a header over the door.
            const float wallY = base + so * 0.15f;
            for (const float sx : {1.0f, -1.0f})
                partition({glm::vec2(sx * aisle, wallY - 0.05f), glm::vec2(sx * ihw, wallY - 0.05f),
                           glm::vec2(sx * ihw, wallY + 0.05f), glm::vec2(sx * aisle, wallY + 0.05f)},
                          zFh, c93::kWall);
            const float doorTop = zFh + 1.95f, zCrown = z1 - 0.12f;
            emitBox(X, Y, Z, P(0.0f, wallY, 0.5f * (doorTop + zCrown)), aisle, 0.05f, 0.5f * (zCrown - doorTop), c93::kWall);

            // Full-height technical cabinets each side, behind the cab partition,
            // with the wall facing the door section angled a little.
            const float cabEnd = wallY - so * 0.1f, techLen = 1.7f, techDepth = 0.55f;
            const float doorEnd = cabEnd - so * techLen;
            for (const float sx : {1.0f, -1.0f})
                partition({glm::vec2(sx * ihw, cabEnd), glm::vec2(sx * ihw, doorEnd),
                           glm::vec2(sx * (ihw - techDepth), doorEnd + so * 0.35f),
                           glm::vec2(sx * (ihw - techDepth), cabEnd)}, zFh, c93::kWood);

            // WC / utility module (one car only): a full-height box on the +x
            // side of the low floor, directly beside the door; ends angled.
            if (hasWC)
                partition({glm::vec2(aisle, wc0 - so * 0.25f), glm::vec2(ihw, wc0),
                           glm::vec2(ihw, wc1), glm::vec2(aisle, wc1 + so * 0.25f)},
                          zFl, c93::kWood);

            // Seats: 2+2 rows through each saloon, oriented from the NSB seat
            // plans (the backrest is the darker end on the plan). Car A (no WC):
            // both aisle sides match, and only the end-most column of each section
            // is reversed to face inward (a facing bay against the cab / the
            // gangway). WC car: the two aisle sides face opposite ways — the +x
            // side toward the door, the -x side toward the car end except its
            // end-most column, which faces the door too.
            auto seat = [&](float x, float y, float zf, float face) {
                emitBox(X, Y, Z, P(x, y, zf + 0.42f), 0.22f, 0.24f, 0.05f, c93::kSeat);
                emitBox(X, Y, Z, P(x, y - face * 0.22f, zf + 0.72f), 0.22f, 0.05f, 0.28f,
                        c93::kSeat * 0.8f); // backrest a shade darker (shows facing)
            };
            {
                auto lerp = [&](float t) { return base + (gang - base) * t; };
                const float pitchT = 0.92f / std::abs(gang - base);
                std::vector<float> cabRows, gwRows; // seat rows, cab vs gangway section
                for (float t = 0.06f; t < 0.94f; t += pitchT) {
                    if (t < 0.15f) continue;              // tech behind cab
                    if (t > 0.32f && t < 0.40f) continue; // stairs
                    if (t > 0.42f && t < 0.51f) continue; // door
                    (t < 0.46f ? cabRows : gwRows).push_back(t);
                }
                // Facing for column `i` (0 = the section's end-most row) on aisle
                // side `sx`. toEnd faces the car end (cab / gangway); toDoor faces
                // the vestibule.
                auto faceFor = [&](int i, float sx, float toEnd, float toDoor) {
                    if (!hasWC)                          // car A: both sides match
                        return (i == 0) ? toDoor : toEnd;
                    return (sx > 0.0f) ? toDoor          // WC car +x: all to door
                                       : ((i == 0) ? toDoor : toEnd); // -x side
                };
                auto placeRow = [&](float t, int i, float toEnd, float toDoor) {
                    const float zf = (t < 0.34f) ? zFh : zFl;
                    const bool wcRegion = hasWC && t > 0.50f && t < 0.90f;
                    for (const float sx : {1.0f, -1.0f}) {
                        if (wcRegion && sx > 0.0f) continue; // WC on the +x side
                        if (wcRegion) {
                            // Opposite the WC: flip-up cushion on the window wall.
                            emitBox(X, Y, Z, P(-(ihw - 0.18f), lerp(t), zf + 0.42f),
                                    0.16f, 0.22f, 0.04f, c93::kSeat);
                            continue;
                        }
                        const float face = faceFor(i, sx, toEnd, toDoor);
                        seat(sx * 0.50f, lerp(t), zf, face); // aisle seat
                        seat(sx * 0.98f, lerp(t), zf, face); // window seat
                    }
                };
                for (std::size_t i = 0; i < cabRows.size(); ++i)
                    placeRow(cabRows[i], static_cast<int>(i), so, -so);
                for (std::size_t i = 0; i < gwRows.size(); ++i)
                    placeRow(gwRows[gwRows.size() - 1 - i], static_cast<int>(i), -so, so);
            }

            // Driver's cab (both ends, symmetric): a raised floor into the nose, a
            // central seat facing the windscreen and a raked desk in front of it.
            // No controls/gauges/screens yet. `so` points into the cab; the driver
            // faces `so`, toward the raked nose.
            {
                plate(base, base + so * 2.2f, zFh, c93::kFloor); // cab floor
                const float sy = base + so * 0.7f;               // seat centre
                seat(0.0f, sy, zFh, so);                         // driver's seat
                emitBox(X, Y, Z, P(0.0f, sy, zFh + 0.20f), 0.12f, 0.12f, 0.20f, c93::kDash); // pedestal
                for (const float sx : {1.0f, -1.0f})             // armrests
                    emitBox(X, Y, Z, P(sx * 0.30f, sy, zFh + 0.56f), 0.03f, 0.20f, 0.08f,
                            c93::kSeat * 0.7f);
                // Talent-style desk: a low flat table with the power/brake lever on
                // the right, and a taller three-facet instrument dome that wraps
                // toward the driver.
                const float dw = ihw * 0.78f;            // desk half-width
                const float xc = 0.30f;                  // centre-facet half-width
                const float zTab = zFh + 0.60f;          // table-top height (lowered)
                const float dyN = base + so * 1.02f;     // table edge near the driver
                const float dyD = base + so * 1.72f;     // table / dome junction
                const float dyF = base + so * 2.06f;     // dome back (toward glass)
                const float zDome = zFh + 1.12f;         // dome top (raised)
                const float rakeY = 0.16f;               // facet top forward rake
                const float wrap = 0.26f;                // side facets' wrap-back
                const glm::vec3 eye = P(0.0f, sy, zFh + 1.15f);            // driver's eye
                const glm::vec3 domeC = P(0.0f, dyD + so * 0.18f, 0.5f * (zTab + zDome));

                // Table top (solid dark console under it).
                emitBox(X, Y, Z, P(0.0f, 0.5f * (dyN + dyD), 0.5f * (zFh + zTab)),
                        dw, 0.5f * std::abs(dyD - dyN), 0.5f * (zTab - zFh), c93::kDash);

                // Live brake state drives the dials and the lever position.
                const float spd = vehicle.speed();
                const float bpP = vehicle.bpPressure();
                const float bcP = vehicle.bcPressure();
                const int cab = cabNegY ? 0 : 1;
                const int handle = vehicle.handlePosition(cab); // +brake / 0 / -power
                const float eng0 = vehicle.engineRpm(0) / Vehicle::kMaxRpm; // rev counter
                const float eng1 = vehicle.engineRpm(1) / Vehicle::kMaxRpm;

                // Combined power/brake lever on the driver's right of the table (the
                // two cabs face opposite ways, so -so keeps it right in both); the
                // stick swings toward the driver as the brake rises and away from the
                // driver (forward) as power rises.
                {
                    const float lx = -so * 0.54f, ly = dyN + so * 0.30f;
                    emitBox(X, Y, Z, P(lx, ly, zTab + 0.03f), 0.10f, 0.12f, 0.03f, c93::kButton); // base
                    const float tilt =
                        handle >= 0
                            ? 0.5f * static_cast<float>(handle) /
                                  static_cast<float>(Vehicle::kEmergencyNotch) // brake: toward driver
                            : -0.4f * static_cast<float>(-handle) /
                                  static_cast<float>(Vehicle::kMaxPowerNotch); // power: away (forward)
                    const glm::vec3 pv = P(lx, ly, zTab + 0.06f);
                    const glm::vec3 dir = Z * std::cos(tilt) - Y * (so * std::sin(tilt));
                    const glm::vec3 Yb = glm::normalize(glm::cross(dir, X));
                    const float L = 0.16f;
                    emitBox(X, Yb, dir, pv + dir * (0.5f * L), 0.025f, 0.025f, 0.5f * L, c93::kDash);   // stick
                    emitBox(X, Yb, dir, pv + dir * (L + 0.03f), 0.05f, 0.09f, 0.035f, c93::kButton);    // handle
                }

                // Reverser (R/N/F) handle on the driver's left (lx = so*0.54,
                // mirroring the brake). Smaller than the power lever; Neutral is
                // upright, Forward pushes away from the driver, Reverse pulls back
                // toward the driver (dir tilts fore/aft along ±Y).
                {
                    const int rev = vehicle.reverser(cab);
                    const float lx = so * 0.54f, ly = dyN + so * 0.24f;
                    emitBox(X, Y, Z, P(lx, ly, zTab + 0.03f), 0.07f, 0.09f, 0.03f, c93::kButton); // base
                    const float tilt = static_cast<float>(rev) * 0.35f; // +F away, -R toward driver
                    const glm::vec3 pv = P(lx, ly, zTab + 0.06f);
                    const glm::vec3 dir = Z * std::cos(tilt) + Y * (so * std::sin(tilt));
                    const glm::vec3 Yb = glm::normalize(glm::cross(dir, X));
                    const float L = 0.11f;
                    emitBox(X, Yb, dir, pv + dir * (0.5f * L), 0.02f, 0.02f, 0.5f * L, c93::kDash);   // stick
                    emitBox(X, Yb, dir, pv + dir * (L + 0.02f), 0.035f, 0.05f, 0.028f, c93::kButton); // knob
                }

                // Instrument dome: three distinct facets. The centre faces the
                // driver; the sides wrap back toward the driver so the panels are
                // clearly angled relative to one another.
                auto frontY = [&](float x) {
                    const float t = std::clamp((std::abs(x) - xc) / (dw - xc), 0.0f, 1.0f);
                    return dyD - so * wrap * t;
                };
                auto ptBot = [&](float x) { return P(x, frontY(x), zTab); };
                auto ptTop = [&](float x) { return P(x, frontY(x) + so * rakeY, zDome); };
                glm::vec3 C, u, v, n;
                auto facet = [&](float xa, float xb) {
                    const glm::vec3 BL = ptBot(xa), BR = ptBot(xb), TR = ptTop(xb), TL = ptTop(xa);
                    C = 0.25f * (BL + BR + TR + TL);
                    u = glm::normalize(BR - BL);
                    v = glm::normalize(TL - BL);
                    n = glm::normalize(glm::cross(u, v));
                    if (glm::dot(n, eye - C) < 0.0f) n = -n;
                    quadN(BL, BR, TR, TL, c93::kDash, C - n); // facet backing
                };

                // Close the dome shell behind the facets (top, back, sides, floor).
                const float xs[] = {-dw, -xc, xc, dw};
                for (int s = 0; s < 3; ++s)
                    quadN(ptTop(xs[s]), ptTop(xs[s + 1]), P(xs[s + 1], dyF, zDome),
                          P(xs[s], dyF, zDome), c93::kDash, domeC);
                quadN(P(-dw, dyF, zTab), P(dw, dyF, zTab), P(dw, dyF, zDome),
                      P(-dw, dyF, zDome), c93::kDash, domeC); // back
                quadN(P(-dw, dyD, zTab), P(dw, dyD, zTab), P(dw, dyF, zTab),
                      P(-dw, dyF, zTab), c93::kDash, domeC); // floor
                quadN(ptBot(-dw), ptTop(-dw), P(-dw, dyF, zDome), P(-dw, dyF, zTab), c93::kDash, domeC);
                quadN(ptBot(dw), ptTop(dw), P(dw, dyF, zDome), P(dw, dyF, zTab), c93::kDash, domeC);

                // Left facet: an MFD LCD screen framed by rectangular buttons; on the
                // screen, a vertical engine tachometer bar per engine (rpm as a
                // fraction of full power, filled from the bottom).
                facet(-dw, -xc);
                rectFace(C, u, v, n, 0.0f, 0.02f, 0.15f, 0.13f, 0.01f, c93::kScreen);
                {
                    const float y0 = -0.08f, y1 = 0.11f, bw = 0.035f; // bar extent / half-width
                    const std::pair<float, float> bars[] = {{-0.055f, eng0}, {0.055f, eng1}};
                    for (const auto& b : bars) {
                        rectFace(C, u, v, n, b.first, 0.5f * (y0 + y1), bw, 0.5f * (y1 - y0),
                                 0.013f, c93::kDash); // dark track
                        const float fill = glm::clamp(b.second, 0.0f, 1.0f) * (y1 - y0);
                        // Always emit (a zero-height, invisible quad when empty) so the
                        // vehicle mesh keeps a CONSTANT vertex/index count: the
                        // renderer's vehicle buffer is fixed-size and drops any update
                        // whose size changed, which would freeze the moving train.
                        rectFace(C, u, v, n, b.first, y0 + 0.5f * fill, bw * 0.8f,
                                 0.5f * fill, 0.015f, c93::kEngBar); // fill from bottom
                    }
                }
                for (int i = 0; i < 3; ++i) {
                    const float yy = 0.13f - 0.13f * i;
                    rectFace(C, u, v, n, -0.22f, yy, 0.02f, 0.025f, 0.012f, c93::kButton);
                    rectFace(C, u, v, n, 0.22f, yy, 0.02f, 0.025f, 0.012f, c93::kButton);
                }

                // Centre facet: two round analog dials over a row of buttons. Left
                // dial = speedometer; right dial = duplex air gauge (brake pipe red
                // needle + brake cylinder dark needle). Needles track the sim.
                //
                // Pipe and cylinder, not reservoir and cylinder: those are the two a
                // driver actually watches, because between them they say what the brake
                // is doing and what it has left to do it with. The reservoir is a
                // background quantity and lives on the HUD.
                facet(-xc, xc);
                discFace(C, u, v, n, -0.12f, 0.06f, 0.09f, 0.01f, c93::kGauge);
                discFace(C, u, v, n, 0.12f, 0.06f, 0.09f, 0.01f, c93::kGauge);
                needle(C, u, v, n, -0.12f, 0.06f, 0.076f, spd / 40.0f, c93::kDash);  // speed (0..40 m/s)
                needle(C, u, v, n, 0.12f, 0.06f, 0.072f, bpP / 10.0f, c93::kRed);    // brake pipe
                needle(C, u, v, n, 0.12f, 0.06f, 0.078f, bcP / 10.0f, c93::kDash);   // brake cylinder
                for (const float gx : {-0.12f, 0.12f})
                    discFace(C, u, v, n, gx, 0.06f, 0.015f, 0.02f, c93::kDash); // hubs (over needle roots)
                for (int i = 0; i < 5; ++i)
                    rectFace(C, u, v, n, -0.18f + 0.09f * i, -0.14f, 0.028f, 0.022f, 0.012f, c93::kButton);

                // Right facet: signalling / ATC panel — a screen and buttons.
                facet(xc, dw);
                rectFace(C, u, v, n, 0.0f, 0.05f, 0.14f, 0.09f, 0.01f, c93::kScreen);
                for (int i = 0; i < 4; ++i)
                    rectFace(C, u, v, n, -0.15f + 0.10f * i, -0.11f, 0.03f, 0.025f, 0.012f, c93::kButton);
            }
        }
    };

    // Body per section. A Class 93 draws a liveried car body (cab at each outer
    // end); everything else draws a bare underframe/floor plate. A carriage has
    // one full-length plate; a 3-bogie module two half plates hinging over the
    // shared middle bogie (each section oriented by its own bogie pair).
    // NSB Di 4. A full-width welded carbody, one roof height end to end, with a nose at
    // each end that has a chin: the front reaches furthest forward at a knee a little over
    // half way up, rakes back above it to the roof edge, and tucks back in below it to a
    // short skirt over the buffer beam. See the profile constants above.
    //
    // The roof looks lower between the cabs in photographs because the hatch and radiator
    // panels are recessed between the cantrails, not because the roof steps down.
    //
    // One rigid body on two bogies, so this is drawn once for the whole locomotive: `f` is
    // the body centre and `halfLen` reaches to the nose knee at each end.
    // One Type 5 carriage body.
    auto emitType5 = [&](const VehicleFrame& f, float halfLen, const t5::Layout& lay) {
        const glm::vec3 X = f.right, Y = f.tangent, Z = f.up;
        const float hw = 0.5f * vehicle.width();
        const float hwRoof = hw * t5::kRoofHalf;
        const float z0 = frameTopZ + t5::kFloorAbove;
        const float roofZ = wheelset::kRailTopZ + vehicle.height();
        const float zCant = z0 + t5::kCant;
        const float zLo = z0 + t5::kWinLow, zHi = z0 + t5::kWinHigh;
        auto P = [&](float lx, float ly, float lz) { return f.pos + X * lx + Y * ly + Z * lz; };
        const glm::vec3 core = P(0.0f, 0.0f, z0 + 0.5f * (zCant - z0));

        // Solebar, then the flanks in three bands: red skirt, silver, dark window band,
        // silver again to the cantrail. Drawn as strips rather than one box so the livery
        // is the geometry and not a texture.
        emitBox(X, Y, Z, P(0.0f, 0.0f, frameTopZ - 0.10f), hw * 0.94f, halfLen * 0.98f,
                0.16f, t5::kUnder);
        for (const float sx : {-1.0f, 1.0f}) {
            auto strip = [&](float a, float b, const glm::vec3& col) {
                quadN(P(sx * hw, -halfLen, a), P(sx * hw, halfLen, a),
                      P(sx * hw, halfLen, b), P(sx * hw, -halfLen, b), col, core);
            };
            strip(z0, z0 + t5::kRedBand, t5::kRed);
            strip(z0 + t5::kRedBand, zLo, t5::kBody);
            strip(zHi, zCant, t5::kBody);
            // The window band, window by window rather than as one dark stripe: body
            // between them, glass in them. Glazed in the translucent colour, so what is
            // behind the glass is what you see through it.
            {
                float at = -halfLen;
                auto pier = [&](float a, float b) {
                    if (b - a < 0.005f) return;
                    quadN(P(sx * hw, a, zLo), P(sx * hw, b, zLo), P(sx * hw, b, zHi),
                          P(sx * hw, a, zHi), t5::kBody, core);
                };
                for (int wi = 0; wi < lay.windowCount; ++wi) {
                    const float* win = lay.windows[wi];
                    pier(at, win[0]);
                    quadN(P(sx * (hw - 0.004f), win[0], zLo),
                          P(sx * (hw - 0.004f), win[1], zLo),
                          P(sx * (hw - 0.004f), win[1], zHi),
                          P(sx * (hw - 0.004f), win[0], zHi), di4::kGlass, core);
                    at = win[1];
                }
                pier(at, halfLen);
            }
            // The roof tumbling in above the cantrail.
            quadN(P(sx * hw, -halfLen, zCant), P(sx * hw, halfLen, zCant),
                  P(sx * hwRoof, halfLen, roofZ), P(sx * hwRoof, -halfLen, roofZ),
                  t5::kBody, core);
            // A plug door at each end, standing a little proud, floor to window head. The
            // wheelchair rebuild needed a wider one, and it is on the left.
            for (int di = 0; di < 2; ++di) {
                const float w = (sx < 0.0f) ? lay.doorHalf[di] : lay.doorHalf[0];
                emitBox(X, Y, Z, P(sx * (hw + 0.012f), lay.doorAt[di],
                                   z0 + 0.5f * (zHi - z0)),
                        0.012f, w, 0.5f * (zHi - z0), t5::kRed);
                // its window, in the upper half
                emitBox(X, Y, Z, P(sx * (hw + 0.026f), lay.doorAt[di],
                                   zLo + 0.55f * (zHi - zLo)),
                        0.010f, w * 0.62f, 0.30f * (zHi - zLo), di4::kGlass);
            }
        }
        emitBox(X, Y, Z, P(0.0f, 0.0f, roofZ - 0.05f), hwRoof, halfLen, 0.05f, t5::kRoof);

        // --- Inside -------------------------------------------------------------------
        //
        // Taken off the seat plan Norske tog publish for this carriage, which is a drawing
        // and not a photograph, so the arrangement is measured rather than supposed: nine
        // rows of four across a centre aisle, the middle two rows facing over tables, a
        // wheelchair-accessible WC and a service nook behind the partition at one end, and
        // the playroom and the wheelchair bays at the other. None of it can be walked into
        // - a carriage has no cab - but all of it is behind glass and can be seen through
        // the windows, which is the whole reason for drawing it.
        {
            const float ihw = hw - 0.06f;
            const float zCeil = z0 + 2.30f;
            const glm::vec3 in = P(0.0f, 0.0f, z0 + 1.0f);
            // Floor, ceiling and the lining behind the windows.
            emitBox(X, Y, Z, P(0.0f, 0.0f, z0 + 0.02f), ihw, halfLen * 0.99f, 0.02f,
                    t5::kFloorIn);
            emitBox(X, Y, Z, P(0.0f, 0.0f, zCeil), ihw, halfLen * 0.99f, 0.04f,
                    t5::kLining);
            // Side lining below the sill and above the window head, and nothing across
            // the window band itself. A wall run the full height sits six centimetres
            // behind the glass and is all you would ever see through it - the seats are
            // inboard of it, so the carriage would read as glazed and empty.
            for (const float sx : {-1.0f, 1.0f})
                for (const auto& band : {std::pair<float, float>{z0, zLo},
                                         std::pair<float, float>{zHi, zCeil}})
                    quadN(P(sx * ihw, -halfLen * 0.99f, band.first),
                          P(sx * ihw, halfLen * 0.99f, band.first),
                          P(sx * ihw, halfLen * 0.99f, band.second),
                          P(sx * ihw, -halfLen * 0.99f, band.second), t5::kLining,
                          in + X * (sx * 6.0f));
            if (lay.inside == t5::InsideCafe) {
                // Booth seating at one end, back to back across tables.
                for (const float row : t5::kBoothRows)
                    for (const float sx : {-1.0f, 1.0f}) {
                        emitBox(X, Y, Z, P(sx * 0.95f, row, z0 + 0.45f), 0.50f, 0.22f,
                                0.06f, t5::kSeat);
                        emitBox(X, Y, Z, P(sx * 0.95f, row + 0.20f, z0 + 0.80f), 0.50f,
                                0.05f, 0.35f, t5::kSeat);
                    }
                for (int k = 0; k < 4; ++k)
                    for (const float sx : {-1.0f, 1.0f})
                        emitBox(X, Y, Z,
                                P(sx * 0.95f, -10.75f + 1.10f * static_cast<float>(k),
                                  z0 + 0.72f),
                                0.45f, 0.30f, 0.03f, t5::kTable);
                // The servery through the middle: a counter down one side with the
                // kitchen block behind it, and stools along it.
                emitBox(X, Y, Z,
                        P(-1.00f, 0.5f * (t5::kServery0 + t5::kServery1), z0 + 0.55f),
                        0.42f, 0.5f * (t5::kServery1 - t5::kServery0), 0.55f, t5::kTable);
                emitBox(X, Y, Z, P(-1.20f, t5::kServery0 + 1.2f, z0 + 0.9f), 0.25f, 1.10f,
                        0.9f, t5::kLining);
                for (int k = 0; k < 9; ++k)
                    emitBox(X, Y, Z,
                            P(-0.35f, t5::kServery0 + 0.8f + 1.05f * static_cast<float>(k),
                              z0 + 0.35f),
                            0.16f, 0.16f, 0.35f, t5::kSeat);
                // And the bike, ski and luggage bay beyond it - racks down both sides.
                for (const float sx : {-1.0f, 1.0f})
                    for (int k = 0; k < 3; ++k)
                        emitBox(X, Y, Z,
                                P(sx * 1.10f,
                                  t5::kStowFrom + 0.9f + 1.70f * static_cast<float>(k),
                                  z0 + 0.60f),
                                0.28f, 0.60f, 0.60f, t5::kLining);
                emitBox(X, Y, Z, P(0.0f, t5::kStowTo + 0.3f, z0 + 0.5f * (zCeil - z0)), ihw,
                        0.05f, 0.5f * (zCeil - z0), t5::kLining);
            } else {
            // Partitions closing the saloon off from the two ends. A sleeper's are the
            // vestibule bulkheads, which is the same thing by another name.
            for (const float y : {lay.saloon0, lay.saloon1})
                emitBox(X, Y, Z, P(0.0f, y, z0 + 0.5f * (zCeil - z0)), ihw, 0.05f,
                        0.5f * (zCeil - z0), t5::kLining);
            // Nine rows of four: two seats a side, a 0.50 m aisle between them.
            for (int ri = 0; ri < lay.seatRowCount; ++ri) {
                const float row = lay.seatRows[ri];
                for (const float sx : {-1.0f, 1.0f})
                    for (int k = 0; k < 2; ++k) {
                        const float xc = sx * (0.30f + 0.55f * (static_cast<float>(k) + 0.5f));
                        emitBox(X, Y, Z, P(xc, row, z0 + 0.45f), 0.25f, 0.24f, 0.06f,
                                t5::kSeat); // cushion
                        emitBox(X, Y, Z, P(xc, row + 0.22f, z0 + 0.80f), 0.25f, 0.05f,
                                0.35f, t5::kSeat); // back, above the sill and visible
                    }
            }
            // The two table bays at the centre, one each side, against the blank panel -
            // and only on a 2nd class carriage: 1st class has a window across the middle
            // instead, and the sleeper has a compartment there like any other.
            if (lay.inside == t5::InsideSeats || lay.inside == t5::InsideFamily) {
                if (lay.serviceAt == 0.0f)
                    for (const float sx : {-1.0f, 1.0f})
                        emitBox(X, Y, Z, P(sx * 0.88f, 0.0f, z0 + 0.72f), 0.52f, 0.55f,
                                0.03f, t5::kTable);
                // The accessible WC: a closed cubicle, and why there is no window there.
                emitBox(X, Y, Z, P(-0.55f, t5::kWcAt, z0 + 0.5f * (zCeil - z0)), 0.90f,
                        0.85f, 0.5f * (zCeil - z0), t5::kLining);
            }
            // The playroom at the far end - an open floor with a couple of soft blocks on
            // it - and the wheelchair bays, which are floor kept clear beside it. Only on
            // the family carriage: the B5-3 has a service nook and luggage there instead,
            // which is what those ten metres of extra seating cost it.
            if (lay.inside == t5::InsideFamily) {
                // The play area is drawn as a raised platform with a padded surround and
                // a couple of soft blocks loose on it. The surround is what can be seen
                // from outside - the blocks sit below the window line, as they would.
                emitBox(X, Y, Z, P(0.0f, t5::kPlayAt, z0 + 0.12f), ihw - 0.25f, 2.10f,
                        0.12f, t5::kTable); // the platform
                emitBox(X, Y, Z, P(0.0f, t5::kPlayAt - 2.05f, z0 + 0.55f), ihw - 0.25f,
                        0.08f, 0.55f, t5::kSeat); // its padded surround, at the aisle end
                for (int k = 0; k < 2; ++k)
                    emitBox(X, Y, Z,
                            P(-0.45f + 0.9f * static_cast<float>(k), t5::kPlayAt,
                              z0 + 0.42f),
                            0.30f, 0.30f, 0.30f, t5::kSeat); // the soft blocks on it
                for (const float bay : t5::kChairBays)
                    for (const float sx : {-1.0f, 1.0f})
                        emitBox(X, Y, Z, P(sx * 1.05f, bay, z0 + 0.03f), 0.34f, 0.40f,
                                0.01f, t5::kTable); // the bays' marked floor
            } else if (lay.inside == t5::InsideSeats) {
                for (const float sx : {-1.0f, 1.0f})
                    emitBox(X, Y, Z, P(sx * 0.95f, lay.saloon1 + 0.70f, z0 + 0.55f), 0.42f,
                            0.55f, 0.55f, t5::kLining); // service nook and luggage
            }
            if (lay.inside == t5::InsideSleeper) {
                // Compartments down one side, a corridor down the other, and a berth
                // across each compartment at two heights. The upper one sits in the
                // window, which is what a sleeper's side looks like from a platform at
                // night; the lower is below the sill, as a bed is.
                const float corr = 0.55f; // the corridor wall, offset to one side
                emitBox(X, Y, Z,
                        P(corr, 0.5f * (lay.saloon0 + lay.saloon1), z0 + 0.5f * (zCeil - z0)),
                        0.05f, 0.5f * (lay.saloon1 - lay.saloon0), 0.5f * (zCeil - z0),
                        t5::kLining);
                // Compartments occupy the side from the far wall up to the corridor and
                // stop there. Centre them on the span they actually fill: getting this
                // wrong runs the berths straight through the corridor wall and out the
                // other side, which a ray cast through a window shows at once as berth,
                // wall, berth - a sleeper with beds in its corridor.
                const float cx = 0.5f * (corr - ihw);        // centre of the compartment
                const float chw = 0.5f * (corr + ihw);       // and half its width
                for (int wi = 1; wi + 1 < lay.windowCount; ++wi) {
                    const float* win = lay.windows[wi];
                    const float mid = 0.5f * (win[0] + win[1]);
                    // the partition on the far side of this compartment
                    emitBox(X, Y, Z, P(cx, win[1] + 0.17f, z0 + 0.5f * (zCeil - z0)), chw,
                            0.04f, 0.5f * (zCeil - z0), t5::kLining);
                    for (const float bz : {0.40f, 1.45f}) // lower berth and upper
                        emitBox(X, Y, Z, P(cx, mid, z0 + bz), chw - 0.06f, 0.44f, 0.06f,
                                t5::kSeat);
                }
            }
            // A steward's service point amidships, where 1st class has one.
            if (lay.serviceAt != 0.0f)
                emitBox(X, Y, Z, P(-0.80f, lay.serviceAt, z0 + 0.60f), 0.60f, 0.90f, 0.60f,
                        t5::kTable);
            }
        }
        // Floor pan and the two ends. The ends are plain: they are only ever seen from
        // between two vehicles, or at the end of the train.
        quadN(P(-hw, -halfLen, z0), P(hw, -halfLen, z0), P(hw, halfLen, z0),
              P(-hw, halfLen, z0), t5::kUnder, core + Z * 6.0f);
        for (const float sy : {-1.0f, 1.0f}) {
            quadN(P(-hw, sy * halfLen, z0), P(hw, sy * halfLen, z0),
                  P(hw, sy * halfLen, zCant), P(-hw, sy * halfLen, zCant), t5::kEnd, core);
            quadN(P(-hw, sy * halfLen, zCant), P(hw, sy * halfLen, zCant),
                  P(hwRoof, sy * halfLen, roofZ), P(-hwRoof, sy * halfLen, roofZ),
                  t5::kEnd, core);
            // Buffers and a gangway, so two of them look coupled rather than merely near.
            for (const float sx : {-1.0f, 1.0f})
                emitBox(X, Y, Z, P(sx * 0.875f, sy * (halfLen + 0.16f), frameTopZ + 0.02f),
                        0.19f, 0.16f, 0.19f, di4::kFrame);
            emitBox(X, Y, Z, P(0.0f, sy * (halfLen + 0.10f), z0 + 0.9f), 0.55f, 0.10f, 0.9f,
                    t5::kEnd);
        }
    };

    auto emitDi4 = [&](const VehicleFrame& f, float halfLen) {
        const glm::vec3 X = f.right, Y = f.tangent, Z = f.up;
        const float hw = 0.5f * vehicle.width();
        const float hwN = hw * di4::kNoseWFrac;
        const float z0 = frameTopZ + di4::kBodyRise; // underframe top / body floor
        // Quoted over the railhead, which is not where the mesh measures from: everything
        // here is above the pose bed, a railhead below that.
        const float roofZ = wheelset::kRailTopZ + vehicle.height();
        const float bodyH = roofZ - z0;
        const float zKnee = z0 + bodyH * di4::kKneeFrac;
        const float zChin = z0 + bodyH * di4::kChinFrac;
        const float zSh = roofZ - bodyH * di4::kShoulderFrac; // where the flanks tumble in
        const float hwRoof = hw * di4::kRoofWFrac;
        // The body's half-width at a height: square-sided up to the shoulder and then
        // chamfered in to the roof. Everything that touches the outside of the locomotive
        // goes through this, so the nose inherits the same shoulder the flanks have.
        auto hwAt = [&](float z, float scale) {
            const float w = z <= zSh ? hw
                                     : hw + (hwRoof - hw) * (z - zSh) / (roofZ - zSh);
            return w * scale;
        };
        auto P = [&](float lx, float ly, float lz) { return f.pos + X * lx + Y * ly + Z * lz; };
        const glm::vec3 core = P(0.0f, 0.0f, z0 + 0.5f * bodyH); // for outward normals

        // Underframe: a full-length solebar slab the body stands on.
        emitBox(X, Y, Z, P(0.0f, 0.0f, frameTopZ + 0.5f * di4::kBodyRise), hw * 0.98f,
                halfLen - di4::kChinAt, 0.5f * di4::kBodyRise, di4::kSkirt);

        // The body between the two roof edges, at one height, and its roof.
        const float yEdge = halfLen - di4::kRoofEdgeAt;
        // Sides, top and bottom, but no end caps. A box would put a red wall across each
        // cab at yEdge, half a metre in front of the driver's face; the caps were never
        // seen from outside anyway, because the nose panels close those ends.
        // The flanks, cut for the cab glazing at each end. One opening is cut spanning
        // both lights and the pillar between them is put back inside it, along with the
        // wedge of bodywork the quarter-light's rake leaves above its leading edge.
        // Painting glass on an uncut side gives a dark patch and no daylight.
        const float bulk = halfLen - di4::kCabLen;
        const float wy0 = bulk + di4::kSideWinBack;  // opening, toward the middle
        const float wy1 = bulk + di4::kSideWinFront; // door window's forward edge
        const float qy0 = bulk + di4::kQtrBack;      // quarter-light's rear edge
        const float qy1 = bulk + di4::kQtrFrontLo;   // and its sill corner: the opening's
        const float qy2 = bulk + di4::kQtrFrontHi;   // head corner, set back by the rake
        const float wzL = z0 + di4::kSideWinLo, wzH = z0 + di4::kSideWinHi;
        for (const float sx : {-1.0f, 1.0f}) {
            quadN(P(sx * hw, -yEdge, z0), P(sx * hw, yEdge, z0), P(sx * hw, yEdge, wzL),
                  P(sx * hw, -yEdge, wzL), di4::kBody, core);
            quadN(P(sx * hw, -yEdge, wzH), P(sx * hw, yEdge, wzH), P(sx * hw, yEdge, zSh),
                  P(sx * hw, -yEdge, zSh), di4::kBody, core);
            quadN(P(sx * hw, -wy0, wzL), P(sx * hw, wy0, wzL), P(sx * hw, wy0, wzH),
                  P(sx * hw, -wy0, wzH), di4::kBody, core);
            for (const float sy : {-1.0f, 1.0f}) {
                quadN(P(sx * hw, sy * qy1, wzL), P(sx * hw, sy * yEdge, wzL),
                      P(sx * hw, sy * yEdge, wzH), P(sx * hw, sy * qy1, wzH), di4::kBody,
                      core);
                quadN(P(sx * hw, sy * wy1, wzL), P(sx * hw, sy * qy0, wzL),
                      P(sx * hw, sy * qy0, wzH), P(sx * hw, sy * wy1, wzH), di4::kBody,
                      core); // the pillar between the two lights
                quadN(P(sx * hw, sy * qy1, wzL), P(sx * hw, sy * qy2, wzH),
                      P(sx * hw, sy * qy1, wzH), P(sx * hw, sy * qy1, wzH), di4::kBody,
                      core); // and the wedge over the rake
            }
        }
        quadN(P(-hw, -yEdge, z0), P(hw, -yEdge, z0), P(hw, yEdge, z0), P(-hw, yEdge, z0),
              di4::kBody, core);
        quadN(P(-hw, -yEdge, zSh), P(hw, -yEdge, zSh), P(hw, yEdge, zSh),
              P(-hw, yEdge, zSh), di4::kBody, core);
        for (const float sx : {-1.0f, 1.0f}) // the shoulders
            quadN(P(sx * hw, -yEdge, zSh), P(sx * hw, yEdge, zSh),
                  P(sx * hwRoof, yEdge, roofZ), P(sx * hwRoof, -yEdge, roofZ), di4::kBody,
                  core);
        emitBox(X, Y, Z, P(0.0f, 0.0f, roofZ - 0.5f * di4::kRoofThick), hwRoof, yEdge,
                0.5f * di4::kRoofThick, di4::kBody);
        // The hatch and radiator panels, recessed into that roof between the cantrails -
        // which is what reads as a lower roof from the side and from a bridge.
        const float flat = halfLen - di4::kCabLen;
        emitBox(X, Y, Z, P(0.0f, 0.0f, roofZ - di4::kRoofThick - 0.03f), hwRoof * 0.88f,
                flat, 0.04f, di4::kRoof);
        for (int i = -2; i <= 2; ++i)
            emitBox(X, Y, Z,
                    P(0.0f, static_cast<float>(i) * flat * 0.33f,
                      roofZ - di4::kRoofThick - 0.015f),
                    hwRoof * 0.74f, flat * 0.12f, 0.03f, di4::kGrille);

        // Cream bands along the red sides.
        for (const float sx : {-1.0f, 1.0f})
            for (const float fz : {0.40f, 0.52f, 0.60f})
                emitBox(X, Y, Z, P(sx * (hw + 0.006f), 0.0f, z0 + bodyH * fz), 0.010f,
                        flat * 0.99f, 0.050f, di4::kStripe);

        // --- The cab ---------------------------------------------------------------------
        //
        // Drawn from three interior photographs of a class of five. Two windscreens of
        // equal size either side of a narrow centre pillar, roller blinds above them; the
        // driver on the right, a second seat and a plain flat table on the left; and one
        // steel-blue desk running the full width of the cab. On it, in front of the driver,
        // a raised housing raked back at him - the dome - carrying, left to right, a
        // rectangular display, two large dials and a row of four small gauges.
        //
        // The large pair is a speedometer and a load meter. A diesel-electric is driven on
        // its ammeter: it is what says how hard the generator is being asked to work, and
        // it is the one gauge a Class 93 driver has no use for.
        auto emitDi4Cab = [&](float so) {
            // Inside the lining. Narrow enough to clear the nose, which is the tightest
            // thing the cab has to fit inside, and used everywhere so the walls are
            // straight - a wall that tapers has to be mitred into every window in it.
            const float ihw = hw * di4::kNoseWFrac - 0.01f;
            const float yB = so * flat;                    // rear bulkhead
            const float yK2 = so * halfLen;                // the knee, as the nose has it
            const float yE2 = so * (halfLen - di4::kRoofEdgeAt); // and the roof edge
            const float yC2 = so * (halfLen - di4::kChinAt);     // and the chin kink
            const float zGL = zKnee + di4::kScreenLoV * (roofZ - zKnee);  // screen sill
            const float zGH = zKnee + di4::kScreenHiV * (roofZ - zKnee);  // and its head
            // The cab floor is the body floor, 1.48 m over the railhead, which is where a
            // locomotive's is. Everything inside is measured up from it, and the number
            // that has to come out right is the eye: seated, it must land between the
            // windscreen sill and its head or the driver is looking at bodywork.
            const float zFl = z0;
            const float zC = zGH + 0.30f;         // ceiling, kept clear of the shoulder
            // The driver's right is at -so: `f.right` is cross(up, tangent), so the sign
            // that keeps a hand on the same side of the cab flips with the end. The Class
            // 93 works the same way round and for the same reason.
            auto dx = [&](float w) { return -so * w; };
            const float xD = dx(0.60f);                        // the driver's centreline
            const glm::vec3 mid = P(0.0f, 0.5f * (yB + yE2), 0.5f * (zFl + zC));
            const glm::vec3 out = Y * (so * 6.0f);             // a point beyond the nose

            // How far forward the skin is at a given height: the vertical skirt, then the
            // chin's rake, then the screen's. The floor, walls and ceiling all stop against
            // this rather than at one y, because the nose leans - a lining squared off at
            // the roof edge leaves the red skin bare beside the driver's shoulder, and one
            // squared off at the knee stands out through the front of the locomotive.
            auto yFront = [&](float z) {
                if (z <= zChin) return yC2;
                if (z <= zKnee) return yC2 + (yK2 - yC2) * (z - zChin) / (zKnee - zChin);
                return yK2 + (yE2 - yK2) * (z - zKnee) / (roofZ - zKnee);
            };
            const float yFl = yFront(zFl) - so * 0.05f, yCe = yFront(zC) - so * 0.05f;
            // Shell. A room is seen from within, so every reference point here is outside
            // its own face and the normals turn inward.
            // A centimetre and a half up: the body's own underside is at exactly this
            // height, and two coplanar faces fight for the pixel - which the floor loses,
            // so the driver stands on red paint.
            const float zPan = zFl + 0.015f;
            quadN(P(-ihw, yB, zPan), P(ihw, yB, zPan), P(ihw, yFl, zPan),
                  P(-ihw, yFl, zPan), di4::kCabFloor, mid + Z * 6.0f);
            quadN(P(-ihw, yB, zC), P(ihw, yB, zC), P(ihw, yCe, zC), P(-ihw, yCe, zC),
                  di4::kCabWall, mid - Z * 6.0f);
            // Two panels a side, folding at the knee. One would be a straight edge from
            // the floor to the ceiling, and the nose reaches half a metre further forward
            // than that line in between - which is precisely where the driver's eye is.
            const float yKn = yK2 - so * 0.05f;
            // Where the side glazing goes. It sits wholly above the knee, so only the
            // upper panel is cut; the lower one runs on whole.
            const float wyR = yB + so * di4::kSideWinBack;
            const float wyF = yB + so * di4::kSideWinFront;
            const float qyR = yB + so * di4::kQtrBack;
            const float qyLo = yB + so * di4::kQtrFrontLo;
            const float qyHi = yB + so * di4::kQtrFrontHi;
            const float wzL = zFl + di4::kSideWinLo, wzH = zFl + di4::kSideWinHi;
            for (const float sx : {-1.0f, 1.0f}) {
                const glm::vec3 outb = mid + X * (sx * 6.0f);
                quadN(P(sx * ihw, yB, zFl), P(sx * ihw, yFl, zFl), P(sx * ihw, yKn, zKnee),
                      P(sx * ihw, yB, zKnee), di4::kCabWall, outb);
                // Upper panel, as a frame round the opening: behind it, under it, over it,
                // and the rest forward to the slanted edge that follows the nose.
                quadN(P(sx * ihw, yB, zKnee), P(sx * ihw, wyR, zKnee), P(sx * ihw, wyR, zC),
                      P(sx * ihw, yB, zC), di4::kCabWall, outb);
                quadN(P(sx * ihw, wyR, zKnee), P(sx * ihw, qyLo, zKnee),
                      P(sx * ihw, qyLo, wzL), P(sx * ihw, wyR, wzL), di4::kCabWall, outb);
                quadN(P(sx * ihw, wyR, wzH), P(sx * ihw, qyLo, wzH), P(sx * ihw, qyLo, zC),
                      P(sx * ihw, wyR, zC), di4::kCabWall, outb);
                quadN(P(sx * ihw, qyLo, zKnee), P(sx * ihw, yKn, zKnee),
                      P(sx * ihw, yCe, zC), P(sx * ihw, qyLo, zC), di4::kCabWall, outb);
                quadN(P(sx * ihw, wyF, wzL), P(sx * ihw, qyR, wzL), P(sx * ihw, qyR, wzH),
                      P(sx * ihw, wyF, wzH), di4::kCabWall, outb); // pillar
                quadN(P(sx * ihw, qyLo, wzL), P(sx * ihw, qyHi, wzH), P(sx * ihw, qyLo, wzH),
                      P(sx * ihw, qyLo, wzH), di4::kCabWall, outb); // wedge over the rake
            }
            quadN(P(-ihw, yB, zFl), P(ihw, yB, zFl), P(ihw, yB, zC), P(-ihw, yB, zC),
                  di4::kCabWall, mid - out);
            // The white surround, drawn on the nose's own rake and just inside it. A
            // vertical plane here looks reasonable and is wrong: the skin leans back as it
            // rises, so a flat lining pokes straight out through the front of the
            // locomotive over most of its height.
            const float hwN2 = hw * di4::kNoseWFrac;
            const float gwI = hwN2 * 0.92f * 0.96f;
            auto fin = [&](float x, float vv) {
                return P(x, yK2 + (yE2 - yK2) * vv, zKnee + (roofZ - zKnee) * vv) -
                       Y * (so * 0.035f);
            };
            const float vLo = di4::kScreenLoV, vHi = di4::kScreenHiV;
            const float vC = (zC - zKnee) / (roofZ - zKnee);
            quadN(fin(-hwN2, vHi), fin(hwN2, vHi), fin(hwN2, vC), fin(-hwN2, vC),
                  di4::kCabWall, out);                        // header
            quadN(fin(-hwN2, 0.0f), fin(hwN2, 0.0f), fin(hwN2, vLo), fin(-hwN2, vLo),
                  di4::kCabWall, out);                        // sill
            // and on down the chin, which the desk hides all but a hand's width of.
            quadN(P(-hwN2, yC2 - so * 0.04f, zChin), P(hwN2, yC2 - so * 0.04f, zChin),
                  P(hwN2, yK2 - so * 0.04f, zKnee), P(-hwN2, yK2 - so * 0.04f, zKnee),
                  di4::kCabWall, out);
            for (const float sx : {-1.0f, 1.0f})              // jambs
                quadN(fin(sx * gwI, vLo), fin(sx * hwN2, vLo), fin(sx * hwN2, vHi),
                      fin(sx * gwI, vHi), di4::kCabWall, out);
            // The centre pillar. Narrow: two big equal panes and little between them.
            quadN(fin(-0.055f, vLo), fin(0.055f, vLo), fin(0.055f, vHi), fin(-0.055f, vHi),
                  di4::kCabWall, out);
            for (const float sx : {-1.0f, 1.0f}) {            // roller blinds
                const float a = sx > 0.0f ? 0.09f : -gwI, b = sx > 0.0f ? gwI : -0.09f;
                quadN(fin(a, vHi - 0.17f), fin(b, vHi - 0.17f), fin(b, vHi), fin(a, vHi),
                      di4::kBlind, out);
            }

            // One side light: the reveal carrying the lining out to the skin, and the
            // glass set in the skin. Seven centimetres of nothing between two apertures is
            // a slot a grazing line of sight goes straight through, which is the fault the
            // windscreen had; the glass is lapped over its frame here for the same reason.
            // Written for any four-cornered opening because the quarter-light is not a
            // rectangle - its leading edge rakes - and a rectangle-only version would have
            // to special-case it or leave it unlined.
            auto sideLight = [&](float sx, const float (&wy)[4], const float (&wz)[4]) {
                const float xi = sx * ihw, xo = sx * (hw - 0.004f);
                const float xm = 0.5f * (xi + xo);
                float cy = 0.0f, cz = 0.0f;
                for (int i = 0; i < 4; ++i) { cy += 0.25f * wy[i]; cz += 0.25f * wz[i]; }
                for (int i = 0; i < 4; ++i) {
                    const int j = (i + 1) % 4;
                    if (std::abs(wy[i] - wy[j]) < 1e-4f && std::abs(wz[i] - wz[j]) < 1e-4f)
                        continue;
                    // Each reveal face looks in at the opening, so its reference point is
                    // pushed well outside along that edge's own outward direction.
                    const float ey = 0.5f * (wy[i] + wy[j]), ez = 0.5f * (wz[i] + wz[j]);
                    const glm::vec3 ref =
                        P(xm, ey + (ey - cy) * 60.0f, ez + (ez - cz) * 60.0f);
                    quadN(P(xi, wy[i], wz[i]), P(xo, wy[i], wz[i]), P(xo, wy[j], wz[j]),
                          P(xi, wy[j], wz[j]), di4::kCabWall, ref);
                }
                const float gx = sx * (hw - 0.014f);
                glm::vec3 g[4];
                for (int i = 0; i < 4; ++i) {
                    const float dy = wy[i] - cy, dz = wz[i] - cz;
                    const float l = std::max(1e-4f, std::sqrt(dy * dy + dz * dz));
                    g[i] = P(gx, wy[i] + 0.016f * dy / l, wz[i] + 0.016f * dz / l);
                }
                quadN(g[0], g[1], g[2], g[3], di4::kGlass, mid + X * (sx * 6.0f));
            };
            for (const float sx : {-1.0f, 1.0f}) {
                const float dy[4] = {wyR, wyF, wyF, wyR};       // the door window
                const float dz[4] = {wzL, wzL, wzH, wzH};
                sideLight(sx, dy, dz);
                const float qy[4] = {qyR, qyLo, qyHi, qyR};     // and the quarter-light
                const float qz[4] = {wzL, wzL, wzH, wzH};
                sideLight(sx, qy, qz);
            }
            // A grab rail under the driver's window, which is in a door.
            emitBox(X, Y, Z, P(dx(1.46f), 0.5f * (wyR + wyF), wzL - 0.10f), 0.02f, 0.38f,
                    0.025f, di4::kFrame);

            // The desk: one flat slab across the cab at elbow height, its forward edge
            // under the windscreen sill. A plain table to the left, instruments right.
            const float zD = zFl + 0.86f;
            const float yDB = so * (halfLen - 1.02f);  // the edge nearest the driver
            const float yDF = so * (halfLen - 0.16f);  // and the one under the glass
            const float yDC = 0.5f * (yDB + yDF);
            emitBox(X, Y, Z, P(0.0f, yDC, zD - 0.025f), ihw - 0.02f,
                    0.5f * std::abs(yDF - yDB), 0.025f, di4::kDesk);
            quadN(P(-ihw, yDB, zFl), P(ihw, yDB, zFl), P(ihw, yDB, zD - 0.05f),
                  P(-ihw, yDB, zD - 0.05f), di4::kDash, out); // kick panel
            for (int i = -1; i <= 1; ++i)
                emitBox(X, Y, Z,
                        P(static_cast<float>(i) * 0.52f, yDB - so * 0.03f, zFl + 0.30f),
                        0.17f, 0.03f, 0.11f, di4::kButton);

            // The console, standing on the desk in front of the driver with its face raked
            // back at him. Its top rides just above the windscreen sill - which is why the
            // sill had to be at the right height before any of this could be placed.
            const float cW = 0.60f;                   // face half width
            const float yCB = so * (halfLen - 0.92f); // face, bottom edge
            const float yCF = so * (halfLen - 0.46f); // face, top edge
            const float zCB = zD + 0.05f, zCT = zD + 0.30f; // top just under the sightline
            const float xL = xD + dx(-cW), xR = xD + dx(cW);
            const glm::vec3 BL = P(xL, yCB, zCB), BR = P(xR, yCB, zCB);
            const glm::vec3 TR = P(xR, yCF, zCT), TL = P(xL, yCF, zCT);
            const glm::vec3 FL = P(xL, yDF, zCT), FR = P(xR, yDF, zCT);
            const glm::vec3 bBL = P(xL, yCB, zD), bBR = P(xR, yCB, zD);
            const glm::vec3 bFL = P(xL, yDF, zD), bFR = P(xR, yDF, zD);
            const glm::vec3 cC = 0.25f * (BL + BR + TR + TL);
            const glm::vec3 hC = 0.125f * (BL + BR + TL + TR + bBL + bBR + FL + FR);
            quadN(TL, TR, FR, FL, di4::kDash, hC);   // top
            quadN(FL, FR, bFR, bFL, di4::kDash, hC); // front, toward the glass
            quadN(bBL, bBR, BR, BL, di4::kDash, hC); // skirt under the face
            quadN(bBL, BL, TL, FL, di4::kDash, hC);  // cheeks, each a pentagon in two
            quadN(bBL, FL, bFL, bFL, di4::kDash, hC);
            quadN(bBR, BR, TR, FR, di4::kDash, hC);
            quadN(bBR, FR, bFR, bFR, di4::kDash, hC);

            // The eye this is all aimed at - and the same point drivercam::eyePose puts
            // the camera, which is the only reason any of these offsets are what they are.
            const glm::vec3 eye = P(xD, yCB - so * 0.95f, zFl + di4::kEyeAboveFloor);
            const glm::vec3 u = glm::normalize(BR - BL);
            const glm::vec3 v = glm::normalize(TL - BL);
            glm::vec3 n = glm::normalize(glm::cross(u, v));
            if (glm::dot(n, eye - cC) < 0.0f) n = -n;
            quadN(BL, BR, TR, TL, di4::kDash, cC - n); // the face itself

            // Live, off the locomotive.
            const int cab = so < 0.0f ? 0 : 1;
            const float spd = vehicle.speed();
            // The load meter is an ammeter, and the drive itself knows what fraction of
            // its current limit it is passing - better than dividing effort by a constant
            // kept in this file that has to be remembered to match the vehicle table.
            const float load = vehicle.tractionAmpsFrac();
            const float rpm = vehicle.engineRpm(0) / di4::kRevFull;
            const int power = vehicle.powerNotch(cab);
            const int brake = vehicle.brakeNotch(cab);
            const int rev = vehicle.reverser(cab);

            // Across the face as the photographs have it: a plain dial, the rectangular
            // display with its column of lamps, the speedometer, the load meter, and four
            // small gauges in a row. Every one of these is emitted whatever it reads - a
            // needle is a quad that turns, never a quad that appears - because the vehicle
            // index buffer is fixed at attach time and a mesh that changes size is dropped.
            discFace(cC, u, v, n, -0.47f, 0.03f, 0.046f, 0.012f, di4::kGaugeS);
            rectFace(cC, u, v, n, -0.28f, 0.03f, 0.095f, 0.072f, 0.012f, di4::kButton);
            rectFace(cC, u, v, n, -0.29f, 0.03f, 0.076f, 0.056f, 0.018f, di4::kScreen);
            for (int i = 0; i < 6; ++i)
                rectFace(cC, u, v, n, -0.180f, 0.085f - 0.024f * i, 0.009f, 0.009f, 0.016f,
                         i == 0 ? di4::kRed : di4::kButton);
            discFace(cC, u, v, n, -0.055f, 0.02f, 0.077f, 0.012f, di4::kGauge);
            discFace(cC, u, v, n, 0.135f, 0.02f, 0.077f, 0.012f, di4::kGauge);
            needle(cC, u, v, n, -0.055f, 0.02f, 0.065f, spd / 40.0f, di4::kNeedle);
            needle(cC, u, v, n, 0.135f, 0.02f, 0.065f, load, di4::kRed);
            for (const float gx : {-0.055f, 0.135f})
                discFace(cC, u, v, n, gx, 0.02f, 0.012f, 0.020f, di4::kNeedle);
            {
                // Main reservoir, brake pipe, brake cylinder, and the engine's revs.
                const std::pair<float, float> small[] = {{0.300f, vehicle.mrPressure() / 12.0f},
                                                         {0.375f, vehicle.bpPressure() / 10.0f},
                                                         {0.450f, vehicle.bcPressure() / 10.0f},
                                                         {0.525f, rpm}};
                for (const auto& g : small) {
                    discFace(cC, u, v, n, g.first, 0.085f, 0.032f, 0.012f, di4::kGaugeS);
                    needle(cC, u, v, n, g.first, 0.085f, 0.026f, g.second, di4::kNeedleL);
                }
            }
            for (int i = 0; i < 8; ++i) // switch rows, under the dials and along the top
                rectFace(cC, u, v, n, -0.50f + 0.085f * i, -0.155f, 0.026f, 0.020f, 0.012f,
                         di4::kButton);
            for (int i = 0; i < 6; ++i)
                rectFace(cC, u, v, n, 0.24f + 0.065f * i, 0.175f, 0.021f, 0.013f, 0.012f,
                         di4::kButton);

            // The driving controls, and this machine has two of them rather than the
            // railcar's one lever. The photographs are unambiguous: a short ball-topped
            // controller on a gated base under the left hand, and under the right - set
            // directly below the three air gauges, where a driver's eye goes when he is
            // braking - a long lever swinging fore and aft in a curved quadrant. That
            // quadrant plate is the signature of a driver's brake valve of the period,
            // and German practice of these years puts the Fahrschalter left and the
            // Führerbremsventil right, which is what a Henschel cab of 1981 would be.
            //
            // Where they stand is squeezed between two limits. A lever sits at a third of
            // the console's distance, so the same sideways offset throws it three times as
            // far out in the view: level with the console's edge in plan, and it lands at
            // the edge of the windscreen. Pulling it back toward the driver instead runs
            // into the camera's 0.5 m near plane, which clips it away silently - it is in
            // the mesh, it is in front of the eye, and it is simply not drawn.
            //
            // One lever, given its pivot, length and lean off the sim.
            auto lever = [&](float lx, float ly, float tilt, float L, float shaft,
                             float knob, const glm::vec3& col, bool collar) {
                const glm::vec3 pv = P(lx, ly, zD + 0.04f);
                const glm::vec3 dir = Z * std::cos(tilt) + Y * (so * std::sin(tilt));
                const glm::vec3 Yb = glm::normalize(glm::cross(dir, X));
                emitBox(X, Yb, dir, pv + dir * (0.5f * L), shaft, shaft, 0.5f * L,
                        di4::kDash);
                if (collar) // the yellow collar, standing clear of the stick it rings
                    emitBox(X, Yb, dir, pv + dir * (0.72f * L), 0.042f, 0.042f, 0.026f,
                            di4::kPlough);
                emitBox(X, Yb, dir, pv + dir * (L + knob * 0.8f), knob, knob, knob, col);
            };

            // Power controller, left hand. Off is upright; notching up leans it forward,
            // away from the driver, as the railcar's does on its power side.
            {
                const float lx = xD + dx(-0.38f), ly = yCB - so * 0.25f;
                emitBox(X, Y, Z, P(lx, ly, zD + 0.02f), 0.075f, 0.075f, 0.02f, di4::kButton);
                const float t = -0.42f * static_cast<float>(power) /
                                static_cast<float>(Vehicle::kMaxPowerNotch);
                lever(lx, ly, t, 0.20f, 0.022f, 0.040f, di4::kDash, true);
            }
            // Reverser, a short selector beside the controller: F away, R back, N upright.
            {
                const float lx = xD + dx(-0.20f), ly = yCB - so * 0.20f;
                emitBox(X, Y, Z, P(lx, ly, zD + 0.02f), 0.05f, 0.05f, 0.02f, di4::kButton);
                lever(lx, ly, -0.35f * static_cast<float>(rev), 0.11f, 0.015f, 0.028f,
                      di4::kButton, false);
            }
            // Driver's brake valve, right hand, in its quadrant. Release stands forward
            // and the handle comes back through the service range to emergency, so the
            // lever leans toward the driver as the brake goes on.
            {
                const float lx = xD + dx(0.38f), ly = yCB - so * 0.25f;
                const float t = -0.40f + 0.80f * static_cast<float>(brake) /
                                             static_cast<float>(Vehicle::kEmergencyNotch);
                const glm::vec3 pv = P(lx, ly, zD + 0.04f);
                const float R = 0.24f;
                for (int i = 0; i <= 7; ++i) { // the quadrant the handle swings in
                    const float a = -0.46f + 0.92f * static_cast<float>(i) / 7.0f;
                    emitBox(X, Y, Z,
                            pv + Y * (so * R * std::sin(a)) + Z * (R * std::cos(a) - R),
                            0.050f, 0.022f, 0.010f, di4::kFrame);
                }
                lever(lx, ly, t, 0.26f, 0.020f, 0.038f, di4::kDash, false);
            }
            // The writing pad on the desk to the driver's right, and the handset on its
            // cradle at the left of the console.
            emitBox(X, Y, Z, P(xD + dx(0.62f), yDB + so * 0.26f, zD + 0.006f), 0.16f, 0.12f,
                    0.006f, di4::kDash);
            emitBox(X, Y, Z, P(xD + dx(-0.72f), yCB - so * 0.10f, zD + 0.05f), 0.05f, 0.13f,
                    0.05f, di4::kDash);

            // Two seats: the driver's on the right, a second on the left, with the plain
            // flat table in front of it that the photographs show instead of a console.
            auto seat = [&](float sx) {
                const float sy = yCB - so * 0.95f; // where the eye is, by construction
                emitBox(X, Y, Z, P(sx, sy, zFl + 0.5f * di4::kSeatCushion), 0.10f, 0.10f,
                        0.5f * di4::kSeatCushion, di4::kDash);
                emitBox(X, Y, Z, P(sx, sy, zFl + di4::kSeatCushion), 0.26f, 0.25f, 0.06f,
                        di4::kSeat);
                emitBox(X, Y, Z, P(sx, sy - so * 0.23f, zFl + di4::kSeatCushion + 0.34f),
                        0.26f, 0.05f, 0.34f, di4::kSeat);
            };
            seat(xD);
            seat(dx(-0.86f));
        };

        for (const float so : {-1.0f, 1.0f}) { // a nose at each end
            const float yE = so * yEdge;                        // roof edge
            const float yK = so * halfLen;                      // the knee, furthest out
            const float yC = so * (halfLen - di4::kChinAt);     // chin kink and skirt

            // The three panels of the front, bottom up: skirt, chin, screen.
            quadN(P(-hwN, yC, z0), P(hwN, yC, z0), P(hwN, yC, zChin), P(-hwN, yC, zChin),
                  di4::kSkirt, core);
            quadN(P(-hwN, yC, zChin), P(hwN, yC, zChin), P(hwN, yK, zKnee),
                  P(-hwN, yK, zKnee), di4::kBody, core);
            // The screen, in two panels so it folds at the shoulder as the flanks do.
            const float nf = di4::kNoseWFrac;
            auto yAtZ = [&](float z) { // along the rake, from the knee up to the roof edge
                return yK + (yE - yK) * (z - zKnee) / (roofZ - zKnee);
            };
            const float ySh = yAtZ(zSh);
            // The raked panel, by (x across, v from the knee up to the roof edge) - the
            // same parametrisation the glass uses, so the two cannot drift apart.
            auto fp = [&](float x, float vv) {
                return P(x, yK + (yE - yK) * vv, zKnee + (roofZ - zKnee) * vv);
            };
            const float vSh = (zSh - zKnee) / (roofZ - zKnee);
            const float gw = hwN * 0.92f * 0.96f; // the aperture's half width
            // Knee to shoulder as a FRAME, not a panel: sill, head and a jamb each side.
            // Solid, it reads correctly from outside - the glass stands proud of it - and
            // is a red wall from the driver's seat, which is the side that matters now.
            quadN(fp(-hwN, 0.0f), fp(hwN, 0.0f), fp(hwN, di4::kScreenLoV),
                  fp(-hwN, di4::kScreenLoV), di4::kBody, core);
            quadN(fp(-hwN, di4::kScreenHiV), fp(hwN, di4::kScreenHiV), fp(hwN, vSh),
                  fp(-hwN, vSh), di4::kBody, core);
            for (const float sx : {-1.0f, 1.0f})
                quadN(fp(sx * gw, di4::kScreenLoV), fp(sx * hwN, di4::kScreenLoV),
                      fp(sx * hwN, di4::kScreenHiV), fp(sx * gw, di4::kScreenHiV),
                      di4::kBody, core);
            quadN(P(-hwAt(zSh, nf), ySh, zSh), P(hwAt(zSh, nf), ySh, zSh),
                  P(hwAt(roofZ, nf), yE, roofZ), P(-hwAt(roofZ, nf), yE, roofZ),
                  di4::kBody, core);
            // The windscreen, inset into that top panel and stood a little proud of it.
            {
                auto face = [&](float u, float v) {
                    const glm::vec3 lo = P((u * 2.0f - 1.0f) * hwN * 0.92f, yK, zKnee);
                    const glm::vec3 hi = P((u * 2.0f - 1.0f) * hwN * 0.92f, yE, roofZ);
                    return lo + (hi - lo) * v + Y * (so * 0.02f);
                };
                // Lapped a little over the frame all round rather than butted into it.
                // Butted, the glass stands two centimetres proud of an aperture exactly
                // its own size, and that leaves an open slot round the rim that a
                // grazing line of sight goes straight through into the cab.
                quadN(face(0.005f, di4::kScreenLoV - 0.012f),
                      face(0.995f, di4::kScreenLoV - 0.012f),
                      face(0.995f, di4::kScreenHiV + 0.012f),
                      face(0.005f, di4::kScreenHiV + 0.012f), di4::kGlass, core);
            }
            // Sides, tiled to follow the two folds rather than cutting across them.
            for (const float sx : {-1.0f, 1.0f}) {
                quadN(P(sx * hw, yE, z0), P(sx * hwN, yC, z0), P(sx * hwN, yC, zChin),
                      P(sx * hw, yE, zChin), di4::kBody, core);
                quadN(P(sx * hw, yE, zChin), P(sx * hwN, yC, zChin), P(sx * hwN, yK, zKnee),
                      P(sx * hw, yE, zKnee), di4::kBody, core);
                quadN(P(sx * hw, yE, zKnee), P(sx * hwAt(zKnee, nf), yK, zKnee),
                      P(sx * hwAt(zSh, nf), ySh, zSh), P(sx * hw, yE, zSh), di4::kBody,
                      core);
                quadN(P(sx * hw, yE, zSh), P(sx * hwAt(zSh, nf), ySh, zSh),
                      P(sx * hwAt(roofZ, nf), yE, roofZ), P(sx * hwRoof, yE, roofZ),
                      di4::kBody, core);
            }
            // The roof over the nose, and the floor pan under it.
            quadN(P(-hwRoof, yE, roofZ), P(hwRoof, yE, roofZ),
                  P(hwAt(roofZ, nf), yE, roofZ), P(-hwAt(roofZ, nf), yE, roofZ),
                  di4::kBody, core);
            quadN(P(-hw, yE, z0), P(hw, yE, z0), P(hwN, yC, z0), P(-hwN, yC, z0),
                  di4::kSkirt, core);

            // Marker lights: a cluster high under the roof edge, a pair on the chin.
            emitBox(X, Y, Z, P(0.0f, yE + so * 0.10f, roofZ - 0.16f), hw * 0.30f, 0.09f,
                    0.06f, di4::kLight);
            for (const float sx : {-1.0f, 1.0f})
                emitBox(X, Y, Z, P(sx * hwN * 0.60f, so * (halfLen - 0.22f),
                                   zChin + 0.30f * (zKnee - zChin)),
                        0.11f, 0.16f, 0.09f, di4::kLight);

            // A door a side behind the cab, where the side elevation has it, with the
            // handrail beside it. The cab side window used to be here too, as a pane laid
            // on the outside of an uncut flank - too high, too short, and see-through into
            // nothing. It is a real opening now, cut through skin and lining together
            // further forward, and drawn with the rest of the cab.
            for (const float sx : {-1.0f, 1.0f})
                emitBox(X, Y, Z, P(sx * (hw - 0.004f), so * (flat - 0.45f),
                                   z0 + 0.44f * bodyH),
                        0.014f, 0.40f, 0.44f * bodyH, di4::kFrame);

            // Buffer beam, coupler and the snowplough - which on this line is not an
            // ornament. It hangs below the skirt, ahead of the knee.
            emitBox(X, Y, Z, P(0.0f, yC + so * 0.06f, frameTopZ + 0.02f), hw, 0.07f, 0.18f,
                    di4::kSkirt);
            // Side buffers on 1.75 m centres with a screw coupling slung between them:
            // the older arrangement, which does not couple itself by being driven into.
            for (const float sx : {-1.0f, 1.0f})
                emitBox(X, Y, Z,
                        P(sx * di4::kBufferHalfSpacing, yC + so * 0.19f, frameTopZ + 0.02f),
                        di4::kBufferR, 0.16f, di4::kBufferR, di4::kFrame);
            emitBox(X, Y, Z, P(0.0f, yC + so * 0.14f, frameTopZ - 0.14f), 0.07f, 0.11f,
                    0.09f, di4::kFrame);
            quadN(P(-hw * 0.92f, so * (halfLen + 0.05f), frameTopZ - 0.62f),
                  P(hw * 0.92f, so * (halfLen + 0.05f), frameTopZ - 0.62f),
                  P(hw * 0.80f, yC, frameTopZ + 0.06f), P(-hw * 0.80f, yC, frameTopZ + 0.06f),
                  di4::kPlough, P(0.0f, so * halfLen, frameTopZ - 1.3f));
            emitBox(X, Y, Z, P(0.0f, so * (halfLen - 0.14f), frameTopZ - 0.44f), hw * 0.86f,
                    0.17f, 0.13f, di4::kPlough);

            emitDi4Cab(so);
        }
        // Fuel tank slung between the bogies: 5200 litres, and what makes a locomotive
        // read as heavy rather than as a coach.
        emitBox(X, Y, Z, P(0.0f, 0.0f, frameTopZ - 0.34f), hw * 0.62f, halfLen * 0.30f,
                0.32f, di4::kSkirt);
    };

    const std::vector<VehicleFrame> sections = vehicle.bodySectionFrames();
    if (!sections.empty()) {
        const float halfLen =
            0.5f * vehicle.length() / static_cast<float>(sections.size());
        for (std::size_t i = 0; i < sections.size(); ++i) {
            if (vehicle.bodyStyle() == BodyClass93) {
                emitClass93(sections[i], halfLen, i == 0, i == 1); // WC in the 2nd car
            } else if (vehicle.bodyStyle() == BodyDi4) {
                emitDi4(sections[i], halfLen);
            } else if (vehicle.bodyStyle() == BodyType5) {
                emitType5(sections[i], halfLen, t5::kBC53);
            } else if (vehicle.bodyStyle() == BodyType5Fr) {
                emitType5(sections[i], halfLen, t5::kFR51);
            } else if (vehicle.bodyStyle() == BodyType5B) {
                emitType5(sections[i], halfLen, t5::kB53);
            } else if (vehicle.bodyStyle() == BodyType5A) {
                emitType5(sections[i], halfLen, t5::kA51);
            } else if (vehicle.bodyStyle() == BodyWlab2) {
                emitType5(sections[i], halfLen, t5::kWLAB2);
            } else {
                const glm::vec3 centre =
                    sections[i].pos + sections[i].up * (frameTopZ + kUnderframeHalfHeight);
                emitBox(sections[i].right, sections[i].tangent, sections[i].up,
                        centre, 0.5f * vehicle.width(), halfLen,
                        kUnderframeHalfHeight, kUnderframeCol);
            }
        }

        // Gangway bellows: a dark concertina tube bridging each pair of adjacent
        // Class 93 car bodies over the shared middle bogie. Its ends follow the
        // two sections' inner faces, so it flexes with the articulation.
        if (vehicle.bodyStyle() == BodyClass93 && sections.size() >= 2) {
            const float z0b = frameTopZ + c93::kFloorAbove;
            const float zCb = z0b + c93::kCantAbove;
            const float bw = c93::kHalfWidth * 0.70f;
            auto gwEnd = [&](const VehicleFrame& f, float sign) {
                const glm::vec3 e = f.pos + f.tangent * (sign * (halfLen - c93::kGangGap));
                return std::array<glm::vec3, 4>{
                    e + f.right * bw + f.up * z0b, e + f.right * bw + f.up * zCb,
                    e - f.right * bw + f.up * zCb, e - f.right * bw + f.up * z0b};
            };
            for (std::size_t i = 0; i + 1 < sections.size(); ++i) {
                const std::array<glm::vec3, 4> A = gwEnd(sections[i], +1.0f);
                const std::array<glm::vec3, 4> B = gwEnd(sections[i + 1], -1.0f);
                glm::vec3 mid(0.0f);
                for (int k = 0; k < 4; ++k) mid += A[k] + B[k];
                mid *= 0.125f;
                const int M = 6; // concertina segments (alternate in/out)
                std::array<glm::vec3, 4> prev = A;
                for (int j = 1; j <= M; ++j) {
                    const float t = static_cast<float>(j) / M;
                    std::array<glm::vec3, 4> cur;
                    glm::vec3 cen(0.0f);
                    for (int k = 0; k < 4; ++k) cen += glm::mix(A[k], B[k], t);
                    cen *= 0.25f;
                    const float s = (j % 2 == 0 || j == M) ? 1.0f : 0.82f; // ribs
                    for (int k = 0; k < 4; ++k)
                        cur[k] = cen + (glm::mix(A[k], B[k], t) - cen) * s;
                    for (int k = 0; k < 4; ++k)
                        quadN(prev[k], prev[(k + 1) % 4], cur[(k + 1) % 4], cur[k],
                              c93::kSkirt, mid);
                    prev = cur;
                }
                // Low-floor bridge across the gap so the aisle walks through.
                quadN(A[3], A[0], B[0], B[3], c93::kFloor,
                      mid - glm::vec3(0.0f, 0.0f, 2.0f));
            }
        }
    }

}

// The whole train: every set's geometry into one buffer, then the glazing sorted to
// the back of it. The renderer holds one vehicle buffer and draws it in two runs, so
// the split has to be over the finished train and not per set.
void VehicleMesh::build(const Consist& consist) {
    vertices_.clear();
    indices_.clear();
    for (int i = 0; i < consist.unitCount(); ++i) emitUnit(consist.unit(i));
    sortGlass();
}

// Every train in the world, into the same one buffer. There is a single vehicle buffer
// and its index buffer is fixed from the moment it is attached, so this ordering is
// only safe under a live index buffer while the trains and their sets stay as they
// were: parting one appends a portion at the end, which moves sets past each other in
// this loop, and that has to be followed by a fresh attachVehicle rather than another
// vertex refresh. Otherwise the old indices are read against new vertices and the
// trains draw as a heap of triangles, silently.
void VehicleMesh::build(const std::deque<Consist>& trains) {
    vertices_.clear();
    indices_.clear();
    for (const Consist& t : trains)
        for (int i = 0; i < t.unitCount(); ++i) emitUnit(t.unit(i));
    sortGlass();
}

void VehicleMesh::sortGlass() {
    // Split the glazing out into a trailing, transparent run so it can be drawn
    // after the opaque geometry with alpha blending. Glass is the exterior window
    // band / windscreen (kBand) and the interior glazing (kGlass); its vertices are
    // tagged with a texLayer sentinel the track shader reads as "translucent".
    {
        constexpr float kGlassLayer = -2.0f;
        auto isGlass = [](const glm::vec3& c) {
            auto eq = [&](const glm::vec3& g) {
                return std::abs(c.x - g.x) < 0.005f && std::abs(c.y - g.y) < 0.005f &&
                       std::abs(c.z - g.z) < 0.005f;
            };
            return eq(c93::kBand) || eq(c93::kGlass) || eq(di4::kGlass);
        };
        std::vector<std::uint32_t> opaque, glass;
        opaque.reserve(indices_.size());
        for (std::size_t i = 0; i + 2 < indices_.size(); i += 3) {
            const std::uint32_t a = indices_[i], b = indices_[i + 1], c = indices_[i + 2];
            if (isGlass(vertices_[a].color) && isGlass(vertices_[b].color) &&
                isGlass(vertices_[c].color)) {
                vertices_[a].texLayer = vertices_[b].texLayer = vertices_[c].texLayer =
                    kGlassLayer;
                glass.push_back(a); glass.push_back(b); glass.push_back(c);
            } else {
                opaque.push_back(a); opaque.push_back(b); opaque.push_back(c);
            }
        }
        glassFirstIndex_ = static_cast<std::uint32_t>(opaque.size());
        opaque.insert(opaque.end(), glass.begin(), glass.end());
        indices_ = std::move(opaque);
    }
}

namespace drivercam {

// Whether a given unit offers cabs to sit in, and how many. Two cabs either way on the
// machines that have them, but for different reasons: the railcar's are the outer ends of
// its two body sections, the locomotive's are the two ends of one rigid body. A Di 4
// reported none until it got a drawn cab, purely because it has a single section.
namespace {
int cabsOn(const Vehicle& v) {
    if (v.cabCount() == 0) return 0; // a carriage: no driving position at all
    if (v.bodyStyle() == BodyDi4) return 2;
    if (v.bodyStyle() == BodyClass93 && v.bodySectionFrames().size() >= 2) return 2;
    return 0;
}
} // namespace

int count(const Consist& c) {
    int n = 0;
    for (int i = 0; i < c.unitCount(); ++i) n += cabsOn(c.unit(i));
    return n;
}

// Mirrors the cab geometry in emitClass93: the seat sits at base + so*0.7 on the
// raised vestibule floor, facing the nose (so). Uses the same frame-height stack.
bool eyePose(const Consist& c, int position, glm::vec3& eye, glm::vec3& forward) {
    // Cab `position` counts along the whole train, two to a set; within its set it is
    // the cab at that section's outer end, which is the same geometry as before.
    if (position < 0 || position >= count(c)) return false;
    // Walk to the unit this cab belongs to, skipping the ones that have none. A train may
    // be a locomotive and five carriages, and then cab 1 is the locomotive's other end and
    // not the first carriage's imaginary front.
    int unit = 0;
    for (; unit < c.unitCount(); ++unit) {
        const int k = cabsOn(c.unit(unit));
        if (position < k) break;
        position -= k;
    }
    if (unit >= c.unitCount()) return false;
    const Vehicle& v = c.unit(unit);
    const std::vector<VehicleFrame> sections = v.bodySectionFrames();
    if (sections.empty()) return false;

    if (v.bodyStyle() == BodyDi4) {
        // One rigid body with a cab at each end, so both cabs share section 0 and are told
        // apart by `so` alone. Every offset below is copied from emitDi4Cab and means
        // nothing on its own: the eye has to land where that put the seat, or the driver
        // sits inside his own console.
        const VehicleFrame& f = sections[0];
        const float halfLen = 0.5f * v.length();
        const float so = position == 0 ? -1.0f : 1.0f;
        // The same stack emitUnit builds, and it has to be copied exactly: the bogie frame
        // centre sits a wheel radius above the axle centre, which is itself a radius above
        // the railhead, so the radius is counted TWICE. Counting it once puts the eye 55 cm
        // low - under the desk, inside the console - and the view from there is convincing
        // enough to be read as a modelling mistake rather than a camera one.
        const float zFl = wheelset::kRailTopZ + 2.0f * v.wheelRadius() + kFrameHalfHeight +
                          di4::kBodyRise; // cab floor = body floor
        const float sy = so * (halfLen - 0.92f) - so * 0.95f; // seat, as in emitDi4Cab
        eye = f.pos + f.tangent * sy + f.right * (-so * 0.60f) +
              f.up * (zFl + di4::kEyeAboveFloor);
        forward = f.tangent * so;
        return true;
    }

    if (v.bodyStyle() != BodyClass93 || position < 0 ||
        position >= static_cast<int>(sections.size()))
        return false;
    const VehicleFrame& f = sections[position];
    const float halfLen = 0.5f * v.length() / static_cast<float>(sections.size());
    const bool cabNegY = (position == 0);
    const float so = cabNegY ? -1.0f : 1.0f;                       // toward this cab
    const float base = cabNegY ? -halfLen + c93::kNoseLen : halfLen - c93::kNoseLen;
    const float sy = base + so * 0.7f;                             // seat centre
    const float frameCentreZ = wheelset::kAxleCentreAboveBed + wheelset::kWheelRadius;
    const float zFh = frameCentreZ + kFrameHalfHeight + c93::kFloorAbove + 0.41f; // cab floor
    eye = f.pos + f.tangent * sy + f.up * (zFh + 1.4f);            // seated eye height
    forward = f.tangent * so;                                     // out the windscreen
    return true;
}

} // namespace drivercam
