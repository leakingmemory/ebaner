#pragma once

#include "TrackPath.h" // TrackPose

#include <glm/glm.hpp>

class TrackPath;

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

// A rail vehicle riding a TrackPath at arc-length s. For now a single wheelset
// with a mass; velocity/integration comes later.
class Vehicle {
public:
    Vehicle(const TrackPath* path, float s, float massKg);

    TrackPose pose() const;
    GravityResolution gravity() const;

    float s() const { return s_; }
    float mass() const { return mass_; }
    const TrackPath* path() const { return path_; }

private:
    const TrackPath* path_;
    float s_;
    float mass_;
};
