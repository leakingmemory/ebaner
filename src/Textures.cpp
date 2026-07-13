#include "Textures.h"

#include <cmath>

namespace landtex {
namespace {

// Deterministic integer hash -> [0,1).
float hash2(int x, int y) {
    std::uint32_t h = static_cast<std::uint32_t>(x) * 374761393u +
                      static_cast<std::uint32_t>(y) * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return (h & 0xFFFFFFu) / static_cast<float>(0x1000000u);
}

float smooth(float t) { return t * t * (3.0f - 2.0f * t); }

// Value noise tiling with period `p` (p divides SIZE so the layer is seamless).
float valueNoise(float x, float y, int p) {
    int xi = static_cast<int>(std::floor(x));
    int yi = static_cast<int>(std::floor(y));
    float xf = x - xi, yf = y - yi;
    auto lat = [&](int a, int b) { return hash2((a % p + p) % p, (b % p + p) % p); };
    float v00 = lat(xi, yi), v10 = lat(xi + 1, yi);
    float v01 = lat(xi, yi + 1), v11 = lat(xi + 1, yi + 1);
    float sx = smooth(xf), sy = smooth(yf);
    float a = v00 + (v10 - v00) * sx;
    float b = v01 + (v11 - v01) * sx;
    return a + (b - a) * sy;
}

// Fractal noise (tileable): octaves at periods SIZE/8, SIZE/4, SIZE/2 ...
float fbm(int px, int py, int basePeriod) {
    float sum = 0.0f, amp = 0.5f, total = 0.0f;
    int p = basePeriod;
    for (int o = 0; o < 4 && p <= SIZE; ++o) {
        float scale = static_cast<float>(p) / SIZE;
        sum += amp * valueNoise(px * scale, py * scale, p);
        total += amp;
        amp *= 0.5f;
        p *= 2;
    }
    return sum / total; // [0,1]
}

struct RGB { float r, g, b; };

RGB clampRGB(RGB c) {
    auto cl = [](float v) { return v < 0 ? 0.0f : (v > 1 ? 1.0f : v); };
    return {cl(c.r), cl(c.g), cl(c.b)};
}

void putPixel(std::vector<std::uint8_t>& buf, int layer, int x, int y, RGB c) {
    c = clampRGB(c);
    std::size_t i = (static_cast<std::size_t>(layer) * SIZE * SIZE +
                     static_cast<std::size_t>(y) * SIZE + x) * 4;
    buf[i + 0] = static_cast<std::uint8_t>(c.r * 255.0f + 0.5f);
    buf[i + 1] = static_cast<std::uint8_t>(c.g * 255.0f + 0.5f);
    buf[i + 2] = static_cast<std::uint8_t>(c.b * 255.0f + 0.5f);
    buf[i + 3] = 255;
}

// Mixes a base colour with a mottled variation driven by fractal noise.
void fillMottled(std::vector<std::uint8_t>& buf, int layer, RGB base, RGB var,
                 int period, float amount) {
    for (int y = 0; y < SIZE; ++y)
        for (int x = 0; x < SIZE; ++x) {
            float n = (fbm(x, y, period) - 0.5f) * 2.0f; // [-1,1]
            float t = n * amount;
            putPixel(buf, layer, x, y,
                     {base.r + var.r * t, base.g + var.g * t, base.b + var.b * t});
        }
}

} // namespace

int layerForArtype(int artype) {
    switch (artype) {
        case 10: return BUILTUP;
        case 20: return AGRI;
        case 30: return FOREST;
        case 50: return OPEN;
        case 60: return BOG;
        case 70: return GLACIER;
        case 81: return FRESH;
        case 82: return OCEAN;
        default: return OTHER; // 0, 99, or anything unmapped
    }
}

std::vector<std::uint8_t> generate() {
    std::vector<std::uint8_t> buf(static_cast<std::size_t>(LAYERS) * SIZE * SIZE * 4,
                                  255);

    // OTHER — neutral grey-green.
    fillMottled(buf, OTHER, {0.42f, 0.45f, 0.38f}, {1, 1, 1}, 16, 0.10f);

    // BUILT-UP — grey with blocky variation.
    for (int y = 0; y < SIZE; ++y)
        for (int x = 0; x < SIZE; ++x) {
            float block = fbm((x / 12) * 12, (y / 12) * 12, 32); // quantised
            float n = fbm(x, y, 64) * 0.15f;
            float v = 0.45f + (block - 0.5f) * 0.25f + n;
            putPixel(buf, BUILTUP, x, y, {v, v, v * 0.98f});
        }

    // AGRICULTURE — warm green with faint field striping.
    for (int y = 0; y < SIZE; ++y)
        for (int x = 0; x < SIZE; ++x) {
            float field = fbm((x / 40) * 40, (y / 40) * 40, 8); // large parcels
            float stripe = 0.04f * std::sin((x + field * 30.0f) * 0.5f);
            float n = (fbm(x, y, 32) - 0.5f) * 0.10f;
            RGB base{0.55f, 0.60f, 0.30f};
            float shift = (field - 0.5f) * 0.18f;
            putPixel(buf, AGRI, x, y,
                     {base.r + shift + stripe + n, base.g + shift * 0.5f + n,
                      base.b + shift * 0.3f + n});
        }

    // FOREST — dark green, canopy mottling with occasional darker clumps.
    for (int y = 0; y < SIZE; ++y)
        for (int x = 0; x < SIZE; ++x) {
            float canopy = fbm(x, y, 32);
            float clump = fbm(x, y, 8);
            float d = (canopy - 0.5f) * 0.12f - (clump < 0.35f ? 0.06f : 0.0f);
            putPixel(buf, FOREST, x, y,
                     {0.14f + d, 0.34f + d * 1.2f, 0.16f + d});
        }

    // OPEN LAND — khaki heath, fine noise.
    fillMottled(buf, OPEN, {0.55f, 0.51f, 0.37f}, {1, 0.95f, 0.8f}, 24, 0.12f);

    // BOG — olive, patchy wet spots.
    for (int y = 0; y < SIZE; ++y)
        for (int x = 0; x < SIZE; ++x) {
            float wet = fbm(x, y, 16);
            float d = (wet - 0.5f) * 0.14f;
            RGB base{0.38f, 0.42f, 0.26f};
            if (wet < 0.4f) base = {0.30f, 0.34f, 0.26f}; // darker wet patches
            putPixel(buf, BOG, x, y, {base.r + d, base.g + d, base.b + d * 0.5f});
        }

    // GLACIER — white ice with faint bluish crevasse noise.
    for (int y = 0; y < SIZE; ++y)
        for (int x = 0; x < SIZE; ++x) {
            float cr = fbm(x, y, 32);
            float d = (cr - 0.5f) * 0.10f;
            putPixel(buf, GLACIER, x, y,
                     {0.90f + d, 0.93f + d, 0.98f + d * 0.5f});
        }

    // FRESHWATER — lake blue with gentle ripples.
    for (int y = 0; y < SIZE; ++y)
        for (int x = 0; x < SIZE; ++x) {
            float w = 0.5f + 0.5f * std::sin(x * 0.13f + fbm(x, y, 16) * 6.0f);
            float d = (w - 0.5f) * 0.08f;
            putPixel(buf, FRESH, x, y, {0.16f + d, 0.34f + d, 0.52f + d});
        }

    // OCEAN — darker blue, larger swell.
    for (int y = 0; y < SIZE; ++y)
        for (int x = 0; x < SIZE; ++x) {
            float w = 0.5f + 0.5f * std::sin((x + y) * 0.06f + fbm(x, y, 8) * 5.0f);
            float d = (w - 0.5f) * 0.07f;
            putPixel(buf, OCEAN, x, y, {0.09f + d, 0.24f + d, 0.42f + d});
        }

    return buf;
}

} // namespace landtex
