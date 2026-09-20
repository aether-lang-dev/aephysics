// The lanes of aephysics.contact_solver_wide as vector code: the warm
// start, solve and restitution over the module's WideConstraint, four
// contacts per operation through GCC's vector extensions (SSE on
// x86-64, NEON on arm64, both baseline). The constraints are packed
// into single precision for the solve, as the reference computes them:
// four floats are one register, where four doubles are two, and the
// gathered bodies of a constraint then fit the register file. The
// module's double layout is declared again here for the packing and
// checked by aephysics_wide_constraint_size() against
// sizeof(WideConstraint) on the Aether side; the arithmetic is the
// module's, operation for operation.
//
// The multiply-adds are written as two operations: -ffp-contract=off
// keeps the compiler from fusing them, so the lanes match each other
// across machines.
#pragma GCC optimize("fp-contract=off")
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// --- the module's layout, doubles ----------------------------------------------------------------------------

typedef struct { double x, y, z, w; } d4;
typedef struct { d4 x, y, z; } vec3d;
typedef struct { d4 x, y; } vec2d;
typedef struct { d4 cxx, cxy, cyy; } sym2d;
typedef struct { d4 cxx, cxy, cxz, cyy, cyz, czz; } sym3d;
typedef struct { int32_t x, y, z, w; } intw;
typedef struct { void *x, *y, *z, *w; } ptrw;

typedef struct {
    vec3d anchor_a, anchor_b;
    d4 base_separation, normal_impulse, total_normal_impulse, normal_mass, lever_arm, relative_velocity;
} wide_point_d;

typedef struct {
    intw index_a, index_b, point_counts;
    d4 inv_mass_a, inv_mass_b;
    sym3d inv_inertia_a, inv_inertia_b;
    vec3d normal, tangent1, tangent2, center_a, center_b;
    d4 twist_mass, twist_impulse;
    sym2d tangent_mass;
    vec2d friction_impulse;
    sym3d rolling_mass;
    vec3d rolling_impulse;
    d4 friction, rolling_resistance, tangent_velocity1, tangent_velocity2, bias_rate, mass_scale, impulse_scale, restitution;
    ptrw manifolds;
    wide_point_d points[4];
} wide_constraint_d;

// aephysics.dynamics's BodyState.
typedef struct {
    double v[3], w[3], dp[3], dq_v[3], dq_s;
    int32_t flags;
} body_state;

#define DYNAMIC_FLAG 0x1000

// --- the solve's layout, float lanes --------------------------------------------------------------------------

typedef float v4 __attribute__((vector_size(16)));
typedef int32_t m4 __attribute__((vector_size(16)));   // a comparison's lanes: all bits or none
typedef struct { v4 x, y, z; } vec3w;
typedef struct { v4 x, y; } vec2w;
typedef struct { vec3w v; v4 s; } quatw;
typedef struct { v4 cxx, cxy, cyy; } sym2w;
typedef struct { v4 cxx, cxy, cxz, cyy, cyz, czz; } sym3w;

typedef struct {
    vec3w anchor_a, anchor_b;
    v4 base_separation, normal_impulse, total_normal_impulse, normal_mass, lever_arm, relative_velocity;
} wide_point;

typedef struct {
    intw index_a, index_b, point_counts;
    v4 inv_mass_a, inv_mass_b;
    sym3w inv_inertia_a, inv_inertia_b;
    vec3w normal, tangent1, tangent2, center_a, center_b;
    v4 twist_mass, twist_impulse;
    sym2w tangent_mass;
    vec2w friction_impulse;
    sym3w rolling_mass;
    vec3w rolling_impulse;
    v4 friction, rolling_resistance, tangent_velocity1, tangent_velocity2, bias_rate, mass_scale, impulse_scale, restitution;
    wide_point points[4];
} wide_constraint;

typedef struct { vec3w v, w, dp; quatw dq; } body_state_w;

// The step's packed constraints: the block the module prepared, and its
// float copy, grown to the largest step so far.
static const wide_constraint_d* g_block = NULL;
static wide_constraint* g_packed = NULL;
static int g_packed_capacity = 0;

static inline v4 narrow(d4 a) { return (v4){ (float)a.x, (float)a.y, (float)a.z, (float)a.w }; }
static inline d4 widen(v4 a) { return (d4){ a[0], a[1], a[2], a[3] }; }
static inline vec3w narrow3(vec3d a) { return (vec3w){ narrow(a.x), narrow(a.y), narrow(a.z) }; }
static inline vec3d widen3(vec3w a) { return (vec3d){ widen(a.x), widen(a.y), widen(a.z) }; }
static inline sym3w narrow_sym3(sym3d a)
{
    return (sym3w){ narrow(a.cxx), narrow(a.cxy), narrow(a.cxz), narrow(a.cyy), narrow(a.cyz), narrow(a.czz) };
}

// The module's block into floats, before the step's first stage.
void aephysics_wide_pack(const void* block, int count)
{
    g_block = block;
    if (count > g_packed_capacity) {
        free(g_packed);
        g_packed_capacity = count + count / 2 + 1;
        g_packed = malloc((size_t)g_packed_capacity * sizeof(wide_constraint));
    }
    for (int i = 0; i < count; ++i) {
        const wide_constraint_d* d = g_block + i;
        wide_constraint* f = g_packed + i;
        f->index_a = d->index_a;
        f->index_b = d->index_b;
        f->point_counts = d->point_counts;
        f->inv_mass_a = narrow(d->inv_mass_a);
        f->inv_mass_b = narrow(d->inv_mass_b);
        f->inv_inertia_a = narrow_sym3(d->inv_inertia_a);
        f->inv_inertia_b = narrow_sym3(d->inv_inertia_b);
        f->normal = narrow3(d->normal);
        f->tangent1 = narrow3(d->tangent1);
        f->tangent2 = narrow3(d->tangent2);
        f->center_a = narrow3(d->center_a);
        f->center_b = narrow3(d->center_b);
        f->twist_mass = narrow(d->twist_mass);
        f->twist_impulse = narrow(d->twist_impulse);
        f->tangent_mass = (sym2w){ narrow(d->tangent_mass.cxx), narrow(d->tangent_mass.cxy), narrow(d->tangent_mass.cyy) };
        f->friction_impulse = (vec2w){ narrow(d->friction_impulse.x), narrow(d->friction_impulse.y) };
        f->rolling_mass = narrow_sym3(d->rolling_mass);
        f->rolling_impulse = narrow3(d->rolling_impulse);
        f->friction = narrow(d->friction);
        f->rolling_resistance = narrow(d->rolling_resistance);
        f->tangent_velocity1 = narrow(d->tangent_velocity1);
        f->tangent_velocity2 = narrow(d->tangent_velocity2);
        f->bias_rate = narrow(d->bias_rate);
        f->mass_scale = narrow(d->mass_scale);
        f->impulse_scale = narrow(d->impulse_scale);
        f->restitution = narrow(d->restitution);
        for (int j = 0; j < 4; ++j) {
            const wide_point_d* dp = d->points + j;
            wide_point* fp = f->points + j;
            fp->anchor_a = narrow3(dp->anchor_a);
            fp->anchor_b = narrow3(dp->anchor_b);
            fp->base_separation = narrow(dp->base_separation);
            fp->normal_impulse = narrow(dp->normal_impulse);
            fp->total_normal_impulse = narrow(dp->total_normal_impulse);
            fp->normal_mass = narrow(dp->normal_mass);
            fp->lever_arm = narrow(dp->lever_arm);
            fp->relative_velocity = narrow(dp->relative_velocity);
        }
    }
}

// The impulses back into the module's block, before the store.
void aephysics_wide_unpack(void* block, int count)
{
    wide_constraint_d* base = block;
    for (int i = 0; i < count; ++i) {
        const wide_constraint* f = g_packed + i;
        wide_constraint_d* d = base + i;
        d->twist_impulse = widen(f->twist_impulse);
        d->friction_impulse = (vec2d){ widen(f->friction_impulse.x), widen(f->friction_impulse.y) };
        d->rolling_impulse = widen3(f->rolling_impulse);
        for (int j = 0; j < 4; ++j) {
            d->points[j].normal_impulse = widen(f->points[j].normal_impulse);
            d->points[j].total_normal_impulse = widen(f->points[j].total_normal_impulse);
        }
    }
    g_block = NULL;
}

// A stage's constraints: the module passes a pointer into its block; the
// same offset into the packed copy.
static inline wide_constraint* packed_of(const void* constraints)
{
    return g_packed + ((const wide_constraint_d*)constraints - g_block);
}

int aephysics_wide_constraint_size(void) { return (int)sizeof(wide_constraint_d); }
int aephysics_wide_body_state_size(void) { return (int)sizeof(body_state); }

static inline v4 splat(float s) { return (v4){ s, s, s, s }; }
// Per lane: b where the comparison held, a elsewhere.
static inline v4 select(v4 a, v4 b, m4 m) { return (v4)(((m4)b & m) | ((m4)a & ~m)); }
static inline v4 vmax(v4 a, v4 b) { return select(b, a, a >= b); }
static inline v4 vmin(v4 a, v4 b) { return select(b, a, a <= b); }
static inline v4 vsqrt(v4 a) { return (v4){ sqrtf(a[0]), sqrtf(a[1]), sqrtf(a[2]), sqrtf(a[3]) }; }
static inline vec3w mul_sv(v4 s, vec3w a) { return (vec3w){ s * a.x, s * a.y, s * a.z }; }
static inline vec3w add_v(vec3w a, vec3w b) { return (vec3w){ a.x + b.x, a.y + b.y, a.z + b.z }; }
static inline vec3w sub_v(vec3w a, vec3w b) { return (vec3w){ a.x - b.x, a.y - b.y, a.z - b.z }; }
static inline vec3w mul_sub_sv(vec3w a, v4 s, vec3w b) { return (vec3w){ a.x - s * b.x, a.y - s * b.y, a.z - s * b.z }; }
static inline vec3w mul_add_sv(vec3w a, v4 s, vec3w b) { return (vec3w){ a.x + s * b.x, a.y + s * b.y, a.z + s * b.z }; }
static inline vec3w mul_mv(sym3w m, vec3w a)
{
    return (vec3w){ m.cxx * a.x + (m.cxy * a.y + m.cxz * a.z), m.cxy * a.x + (m.cyy * a.y + m.cyz * a.z),
                    m.cxz * a.x + (m.cyz * a.y + m.czz * a.z) };
}
static inline vec3w mul_sub_mv(vec3w a, sym3w m, vec3w b) { return sub_v(a, mul_mv(m, b)); }
static inline vec3w mul_add_mv(vec3w a, sym3w m, vec3w b) { return add_v(a, mul_mv(m, b)); }
static inline vec2w mul_mv2(sym2w m, vec2w a) { return (vec2w){ m.cxx * a.x + m.cxy * a.y, m.cxy * a.x + m.cyy * a.y }; }
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
// Masks are lanes of 0 or 1, as the Aether lanes have them.
static inline v4 greater(v4 a, v4 b) { return (v4)((a > b) & (m4)splat(1.0f)); }
static inline v4 equals(v4 a, v4 b) { return (v4)((a == b) & (m4)splat(1.0f)); }
static inline v4 or_mask(v4 a, v4 b) { return (v4)(((a != splat(0.0f)) | (b != splat(0.0f))) & (m4)splat(1.0f)); }
static inline v4 blend(v4 a, v4 b, v4 m) { return select(a, b, m != splat(0.0f)); }
static inline int all_zero(v4 a) { return a[0] == 0.0f && a[1] == 0.0f && a[2] == 0.0f && a[3] == 0.0f; }
static inline int max_point_count(const wide_constraint* c)
{
    int m = c->point_counts.x;
    if (c->point_counts.y > m) m = c->point_counts.y;
    if (c->point_counts.z > m) m = c->point_counts.z;
    if (c->point_counts.w > m) m = c->point_counts.w;
    return m;
}

static const body_state identity = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, 1.0, 0 };

static inline body_state_w gather(const body_state* states, intw indices)
{
    const body_state* s0 = indices.x == 0 ? &identity : states + indices.x - 1;
    const body_state* s1 = indices.y == 0 ? &identity : states + indices.y - 1;
    const body_state* s2 = indices.z == 0 ? &identity : states + indices.z - 1;
    const body_state* s3 = indices.w == 0 ? &identity : states + indices.w - 1;
    body_state_w b;
    for (int k = 0; k < 3; ++k) {
        (&b.v.x)[k] = (v4){ (float)s0->v[k], (float)s1->v[k], (float)s2->v[k], (float)s3->v[k] };
        (&b.w.x)[k] = (v4){ (float)s0->w[k], (float)s1->w[k], (float)s2->w[k], (float)s3->w[k] };
        (&b.dp.x)[k] = (v4){ (float)s0->dp[k], (float)s1->dp[k], (float)s2->dp[k], (float)s3->dp[k] };
        (&b.dq.v.x)[k] = (v4){ (float)s0->dq_v[k], (float)s1->dq_v[k], (float)s2->dq_v[k], (float)s3->dq_v[k] };
    }
    b.dq.s = (v4){ (float)s0->dq_s, (float)s1->dq_s, (float)s2->dq_s, (float)s3->dq_s };
    return b;
}

static inline void scatter_lane(body_state* states, int index1, const body_state_w* b, int lane)
{
    if (index1 == 0) return;
    body_state* s = states + index1 - 1;
    if ((s->flags & DYNAMIC_FLAG) == 0) return;
    s->v[0] = b->v.x[lane]; s->v[1] = b->v.y[lane]; s->v[2] = b->v.z[lane];
    s->w[0] = b->w.x[lane]; s->w[1] = b->w.y[lane]; s->w[2] = b->w.z[lane];
}

static inline void scatter(body_state* states, intw indices, const body_state_w* b)
{
    scatter_lane(states, indices.x, b, 0);
    scatter_lane(states, indices.y, b, 1);
    scatter_lane(states, indices.z, b, 2);
    scatter_lane(states, indices.w, b, 3);
}

void aephysics_wide_warm_start(void* states_block, void* constraints, int count)
{
    body_state* states = states_block;
    wide_constraint* packed = packed_of(constraints);
    for (int index = 0; index < count; ++index) {
        wide_constraint* c = packed + index;
        body_state_w ba = gather(states, c->index_a);
        body_state_w bb = gather(states, c->index_b);
        int point_count = max_point_count(c);
        for (int j = 0; j < point_count; ++j) {
            wide_point* cp = c->points + j;
            vec3w impulse = mul_sv(cp->normal_impulse, c->normal);
            ba.w = mul_sub_mv(ba.w, c->inv_inertia_a, cross(cp->anchor_a, impulse));
            ba.v = mul_sub_sv(ba.v, c->inv_mass_a, impulse);
            bb.w = mul_add_mv(bb.w, c->inv_inertia_b, cross(cp->anchor_b, impulse));
            bb.v = mul_add_sv(bb.v, c->inv_mass_b, impulse);
        }
        vec3w friction = mul_add_sv(mul_sv(c->friction_impulse.x, c->tangent1), c->friction_impulse.y, c->tangent2);
        ba.w = mul_sub_mv(ba.w, c->inv_inertia_a, cross(c->center_a, friction));
        ba.v = mul_sub_sv(ba.v, c->inv_mass_a, friction);
        bb.w = mul_add_mv(bb.w, c->inv_inertia_b, cross(c->center_b, friction));
        bb.v = mul_add_sv(bb.v, c->inv_mass_b, friction);
        vec3w twist = mul_sv(c->twist_impulse, c->normal);
        ba.w = mul_sub_mv(ba.w, c->inv_inertia_a, twist);
        bb.w = mul_add_mv(bb.w, c->inv_inertia_b, twist);
        ba.w = mul_sub_mv(ba.w, c->inv_inertia_a, c->rolling_impulse);
        bb.w = mul_add_mv(bb.w, c->inv_inertia_b, c->rolling_impulse);
        scatter(states, c->index_a, &ba);
        scatter(states, c->index_b, &bb);
    }
}

void aephysics_wide_solve(void* states_block, void* constraints, int count, int use_bias, double inv_h_scalar,
                          double contact_speed_scalar, double epsilon_scalar)
{
    body_state* states = states_block;
    wide_constraint* packed = packed_of(constraints);
    v4 inv_h = splat((float)inv_h_scalar);
    v4 contact_speed = splat((float)-contact_speed_scalar);
    v4 one = splat(1.0f);
    v4 zero = splat(0.0f);
    v4 epsilon = splat((float)epsilon_scalar);
    for (int index = 0; index < count; ++index) {
        wide_constraint* c = packed + index;
        int point_count = max_point_count(c);
        body_state_w ba = gather(states, c->index_a);
        body_state_w bb = gather(states, c->index_b);
        v4 bias_rate = zero, mass_scale = one, impulse_scale = zero;
        if (use_bias) {
            bias_rate = c->mass_scale * c->bias_rate;
            mass_scale = c->mass_scale;
            impulse_scale = c->impulse_scale;
        }
        vec3w dp = sub_v(bb.dp, ba.dp);
        v4 total_normal_impulse = zero;
        v4 total_twist_limit = zero;
        for (int j = 0; j < point_count; ++j) {
            wide_point* cp = c->points + j;
            vec3w ra = cp->anchor_a;
            vec3w rb = cp->anchor_b;
            vec3w rsa = rotate(ba.dq, ra);
            vec3w rsb = rotate(bb.dq, rb);
            vec3w ds = add_v(dp, sub_v(rsb, rsa));
            v4 s = dot(c->normal, ds) + cp->base_separation;
            v4 apart = greater(s, zero);
            v4 spec_bias = s * inv_h;
            v4 soft_bias = vmax(bias_rate * s, contact_speed);
            v4 bias = blend(soft_bias, spec_bias, apart);
            v4 point_mass_scale = blend(mass_scale, one, apart);
            v4 point_impulse_scale = blend(impulse_scale, zero, apart);
            vec3w vra = add_v(ba.v, cross(ba.w, ra));
            vec3w vrb = add_v(bb.v, cross(bb.w, rb));
            v4 vn = dot(sub_v(vrb, vra), c->normal);
            v4 neg_impulse = cp->normal_mass * (point_mass_scale * vn + bias) + point_impulse_scale * cp->normal_impulse;
            v4 new_impulse = vmax(cp->normal_impulse - neg_impulse, zero);
            v4 delta_impulse = new_impulse - cp->normal_impulse;
            cp->normal_impulse = new_impulse;
            cp->total_normal_impulse = cp->total_normal_impulse + new_impulse;
            total_normal_impulse = total_normal_impulse + new_impulse;
            total_twist_limit = total_twist_limit + cp->lever_arm * new_impulse;
            vec3w p = mul_sv(delta_impulse, c->normal);
            ba.w = mul_sub_mv(ba.w, c->inv_inertia_a, cross(ra, p));
            ba.v = mul_sub_sv(ba.v, c->inv_mass_a, p);
            bb.w = mul_add_mv(bb.w, c->inv_inertia_b, cross(rb, p));
            bb.v = mul_add_sv(bb.v, c->inv_mass_b, p);
        }
        if (!use_bias) {
            if (!all_zero(c->rolling_resistance)) {
                vec3w delta_rolling = mul_mv(c->rolling_mass, sub_v(ba.w, bb.w));
                vec3w old_rolling = c->rolling_impulse;
                c->rolling_impulse = add_v(old_rolling, delta_rolling);
                v4 max_rolling = c->rolling_resistance * total_normal_impulse;
                v4 length_squared = dot(c->rolling_impulse, c->rolling_impulse);
                v4 over = greater(length_squared, epsilon + max_rolling * max_rolling);
                v4 normalize = max_rolling / (vsqrt(length_squared) + epsilon);
                v4 scale = blend(one, normalize, over);
                scale = blend(zero, scale, greater(c->rolling_resistance, zero));
                c->rolling_impulse = mul_sv(scale, c->rolling_impulse);
                delta_rolling = sub_v(c->rolling_impulse, old_rolling);
                ba.w = mul_sub_mv(ba.w, c->inv_inertia_a, delta_rolling);
                bb.w = mul_add_mv(bb.w, c->inv_inertia_b, delta_rolling);
            }
            v4 twist_speed = dot(c->normal, sub_v(bb.w, ba.w));
            v4 max_twist = c->friction * total_twist_limit;
            v4 old_twist = c->twist_impulse;
            c->twist_impulse = vmax(-max_twist, vmin(old_twist - c->twist_mass * twist_speed, max_twist));
            v4 delta_twist = c->twist_impulse - old_twist;
            vec3w twist = mul_sv(delta_twist, c->normal);
            ba.w = mul_sub_mv(ba.w, c->inv_inertia_a, twist);
            bb.w = mul_add_mv(bb.w, c->inv_inertia_b, twist);
            vec3w ca = c->center_a;
            vec3w cb = c->center_b;
            vec3w vra = add_v(ba.v, cross(ba.w, ca));
            vec3w vrb = add_v(bb.v, cross(bb.w, cb));
            vec3w vr = sub_v(vrb, vra);
            vec2w vt = { dot(vr, c->tangent1) - c->tangent_velocity1, dot(vr, c->tangent2) - c->tangent_velocity2 };
            vec2w tm = mul_mv2(c->tangent_mass, vt);
            vec2w new_friction = { c->friction_impulse.x - tm.x, c->friction_impulse.y - tm.y };
            v4 max_friction = c->friction * total_normal_impulse;
            v4 length_squared = new_friction.x * new_friction.x + new_friction.y * new_friction.y;
            v4 over = greater(length_squared, max_friction * max_friction);
            v4 normalize = max_friction / (vsqrt(length_squared) + epsilon);
            v4 scale = blend(one, normalize, over);
            new_friction = (vec2w){ scale * new_friction.x, scale * new_friction.y };
            vec2w delta_friction = { new_friction.x - c->friction_impulse.x, new_friction.y - c->friction_impulse.y };
            c->friction_impulse = new_friction;
            vec3w p = add_v(mul_sv(delta_friction.x, c->tangent1), mul_sv(delta_friction.y, c->tangent2));
            ba.w = mul_sub_mv(ba.w, c->inv_inertia_a, cross(ca, p));
            ba.v = mul_sub_sv(ba.v, c->inv_mass_a, p);
            bb.w = mul_add_mv(bb.w, c->inv_inertia_b, cross(cb, p));
            bb.v = mul_add_sv(bb.v, c->inv_mass_b, p);
        }
        scatter(states, c->index_a, &ba);
        scatter(states, c->index_b, &bb);
    }
}

void aephysics_wide_restitution(void* states_block, void* constraints, int count, double threshold_scalar)
{
    body_state* states = states_block;
    wide_constraint* packed = packed_of(constraints);
    v4 threshold = splat((float)threshold_scalar);
    v4 zero = splat(0.0f);
    for (int index = 0; index < count; ++index) {
        wide_constraint* c = packed + index;
        if (all_zero(c->restitution)) continue;
        int point_count = max_point_count(c);
        body_state_w ba = gather(states, c->index_a);
        body_state_w bb = gather(states, c->index_b);
        v4 no_restitution = equals(c->restitution, zero);
        for (int j = 0; j < point_count; ++j) {
            wide_point* cp = c->points + j;
            v4 slow = greater(cp->relative_velocity + threshold, zero);
            v4 untouched = equals(cp->total_normal_impulse, zero);
            v4 skip = or_mask(or_mask(slow, untouched), no_restitution);
            v4 mass = blend(cp->normal_mass, zero, skip);
            vec3w ra = cp->anchor_a;
            vec3w rb = cp->anchor_b;
            vec3w vra = add_v(ba.v, cross(ba.w, ra));
            vec3w vrb = add_v(bb.v, cross(bb.w, rb));
            v4 vn = dot(sub_v(vrb, vra), c->normal);
            v4 neg_impulse = mass * (vn + c->restitution * cp->relative_velocity);
            v4 new_impulse = vmax(cp->normal_impulse - neg_impulse, zero);
            v4 delta_impulse = new_impulse - cp->normal_impulse;
            cp->normal_impulse = new_impulse;
            cp->total_normal_impulse = cp->total_normal_impulse + delta_impulse;
            vec3w p = mul_sv(delta_impulse, c->normal);
            ba.w = mul_sub_mv(ba.w, c->inv_inertia_a, cross(ra, p));
            ba.v = mul_sub_sv(ba.v, c->inv_mass_a, p);
            bb.w = mul_add_mv(bb.w, c->inv_inertia_b, cross(rb, p));
            bb.v = mul_add_sv(bb.v, c->inv_mass_b, p);
        }
        scatter(states, c->index_a, &ba);
        scatter(states, c->index_b, &bb);
    }
}
