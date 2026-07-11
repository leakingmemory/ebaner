#pragma once

#include <glm/glm.hpp>

// Free-fly camera. World is right-handed with +z up (matches terrain coords).
class Camera {
public:
    // Places the camera at `pos` looking along horizontal `dir`.
    void init(const glm::vec3& pos, const glm::vec3& dir);

    // Advances position from held movement keys (dt in seconds).
    void move(float forward, float right, float up, float dt, bool fast);

    // Applies mouse-look deltas (pixels).
    void look(float dx, float dy);

    glm::mat4 viewMatrix() const;
    glm::mat4 projMatrix(float aspect) const;

    glm::vec3 position() const { return pos_; }
    glm::vec3 forward() const;

private:
    glm::vec3 pos_{0.0f};
    float yaw_ = 0.0f;    // radians, around +z; 0 = +x
    float pitch_ = 0.0f;  // radians, clamped to avoid gimbal flip

    float speed_ = 40.0f;       // metres/second (base)
    float mouseSens_ = 0.0022f; // radians per pixel
    float fovY_ = glm::radians(60.0f);
    float nearZ_ = 0.5f;
    float farZ_ = 60000.0f;
};
