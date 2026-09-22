// The C side of the lanes benchmark: what bench/lanes.ae is measured
// against, written the way aephysics/native/aephysics_native.c writes its
// vector code (GCC vector extensions, fp-contract off).
//
// Does Aether's std.lanes emit what that vector code emits?
// The kernel is the contact solve's per-point body, four contacts to a
// pass: rotate both anchors by the bodies' delta rotations, the
// separation and its bias, the relative normal velocity, the clamped
// impulse, and the two bodies' velocity updates.
#pragma GCC optimize("fp-contract=off")
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef float v4 __attribute__((vector_size(16)));
typedef int m4 __attribute__((vector_size(16)));
typedef struct { v4 x, y, z; } vec3w;
typedef struct { vec3w v; v4 s; } quatw;

static inline v4 splat(float x) { return (v4){ x, x, x, x }; }
static inline vec3w add_v(vec3w a, vec3w b) { return (vec3w){ a.x + b.x, a.y + b.y, a.z + b.z }; }
static inline vec3w sub_v(vec3w a, vec3w b) { return (vec3w){ a.x - b.x, a.y - b.y, a.z - b.z }; }
static inline vec3w mul_sv(v4 s, vec3w a) { return (vec3w){ s * a.x, s * a.y, s * a.z }; }
static inline v4 dot(vec3w a, vec3w b) { return (a.x * b.x + a.y * b.y) + a.z * b.z; }
static inline vec3w cross(vec3w a, vec3w b) { return (vec3w){ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
static inline vec3w rotate(quatw q, vec3w a)
{
    vec3w t1 = cross(q.v, a);
    vec3w t2 = { t1.x + q.s * a.x, t1.y + q.s * a.y, t1.z + q.s * a.z };
    vec3w t3 = cross(q.v, t2);
    v4 two = splat(2.0f);
    return (vec3w){ a.x + two * t3.x, a.y + two * t3.y, a.z + two * t3.z };
}
static inline v4 pick(v4 a, v4 b, m4 m) { return (v4)(((m4)a & ~m) | ((m4)b & m)); }
static inline v4 vmax(v4 a, v4 b) { m4 m = a > b; return (v4)(((m4)a & m) | ((m4)b & ~m)); }

#define BUNDLES 4096
#define POINTS 4
#define PASSES 200

static float buf[BUNDLES * 56 * 4];

static inline v4 ld(int bundle, int field) { return *(v4*)(buf + (bundle * 56 + field) * 4); }
static inline void st(int bundle, int field, v4 value) { *(v4*)(buf + (bundle * 56 + field) * 4) = value; }

int main(void)
{
    for (int i = 0; i < BUNDLES * 56 * 4; ++i) buf[i] = 0.1f + (float)((i * 37) % 19) * 0.01f;
    // The impulse a point starts the pass with, so the clamp is not the
    // only branch the data ever takes.
    for (int b = 0; b < BUNDLES; ++b)
        for (int lane = 0; lane < 4; ++lane) buf[b * 224 + 212 + lane] = 5.0f;
    v4 inv_h = splat(240.0f), contact_speed = splat(-3.0f), zero = splat(0.0f), one = splat(1.0f);
    clock_t start = clock();
    v4 sink = zero;
    for (int pass = 0; pass < PASSES; ++pass) {
        for (int b = 0; b < BUNDLES; ++b) {
            vec3w normal = { ld(b, 0), ld(b, 1), ld(b, 2) };
            vec3w dp = { ld(b, 3), ld(b, 4), ld(b, 5) };
            quatw dqa = { { ld(b, 6), ld(b, 7), ld(b, 8) }, ld(b, 9) };
            quatw dqb = { { ld(b, 10), ld(b, 11), ld(b, 12) }, ld(b, 13) };
            vec3w va = { ld(b, 14), ld(b, 15), ld(b, 16) };
            vec3w wa = { ld(b, 17), ld(b, 18), ld(b, 19) };
            vec3w vb = { ld(b, 20), ld(b, 21), ld(b, 22) };
            vec3w wb = { ld(b, 23), ld(b, 24), ld(b, 25) };
            v4 inv_mass_a = ld(b, 26), inv_mass_b = ld(b, 27);
            v4 bias_rate = ld(b, 28), mass_scale = ld(b, 29), impulse_scale = ld(b, 30);
            for (int j = 0; j < POINTS; ++j) {
                int base = 31 + j * 6;
                vec3w ra = { ld(b, base), ld(b, base + 1), ld(b, base + 2) };
                vec3w rb = { ld(b, base + 3), ld(b, base + 4), ld(b, base + 5) };
                v4 base_separation = ld(b, 55);
                v4 normal_mass = ld(b, 54);
                v4 normal_impulse = ld(b, 53);
                vec3w rsa = rotate(dqa, ra);
                vec3w rsb = rotate(dqb, rb);
                vec3w ds = add_v(dp, sub_v(rsb, rsa));
                v4 s = dot(normal, ds) + base_separation;
                m4 apart = s > zero;
                v4 bias = pick(vmax(bias_rate * s, contact_speed), s * inv_h, apart);
                v4 point_mass_scale = pick(mass_scale, one, apart);
                v4 point_impulse_scale = pick(impulse_scale, zero, apart);
                vec3w vra = add_v(va, cross(wa, ra));
                vec3w vrb = add_v(vb, cross(wb, rb));
                v4 vn = dot(sub_v(vrb, vra), normal);
                v4 neg_impulse = normal_mass * (point_mass_scale * vn + bias) + point_impulse_scale * normal_impulse;
                v4 new_impulse = vmax(normal_impulse - neg_impulse, zero);
                v4 delta = new_impulse - normal_impulse;
                vec3w p = mul_sv(delta, normal);
                wa = sub_v(wa, mul_sv(inv_mass_a, cross(ra, p)));
                va = sub_v(va, mul_sv(inv_mass_a, p));
                wb = add_v(wb, mul_sv(inv_mass_b, cross(rb, p)));
                vb = add_v(vb, mul_sv(inv_mass_b, p));
                sink = sink + new_impulse;
            }
            st(b, 14, va.x); st(b, 15, va.y); st(b, 16, va.z);
            st(b, 17, wa.x); st(b, 18, wa.y); st(b, 19, wa.z);
            st(b, 20, vb.x); st(b, 21, vb.y); st(b, 22, vb.z);
            st(b, 23, wb.x); st(b, 24, wb.y); st(b, 25, wb.z);
        }
    }
    double ms = (double)(clock() - start) * 1000.0 / CLOCKS_PER_SEC;
    printf("c lanes: %d bundles x %d points x %d passes in %.1f ms (sink %.4f)\n",
           BUNDLES, POINTS, PASSES, ms, (double)(sink[0] + sink[1] + sink[2] + sink[3]));
    return 0;
}
