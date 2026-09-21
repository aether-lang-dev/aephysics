// The native side of aephysics, one C file built with every program
// (`ae build --extra aephysics/native/aephysics_native.c`): what Aether
// cannot express yet, and nothing else.
//
// 1. The threads' side of aephysics.parallel and the solver's stages: a
//    thread-local worker index (Aether has no thread-local variables), a
//    yield and a pause, the processor count, atomic operations on an int
//    in place (std.sync's atomics are cells of their own; the solver's
//    blocks and the tree's nodes carry theirs in the struct, as the
//    reference does), and a counting semaphore the scheduler's threads
//    wait on between steps.
// 2. The lanes of aephysics.contact_solver_wide as vector code: the
// prepare, warm start, solve and restitution over the module's
// WideConstraint, four contacts per operation through GCC's vector
// extensions (SSE on x86-64, NEON on arm64, both baseline). The
// constraints are prepared in single precision for the solve, as the
// reference computes them: four floats are one register, where four
// doubles are two, and the gathered bodies of a constraint then fit
// the register file. The module's double layout is declared again here
// for the impulses handed back and checked by
// aephysics_wide_constraint_size() against sizeof(WideConstraint) on
// the Aether side; the module's other structures are read through the
// field offsets it measures (aephysics_wide_layout); the arithmetic is
// the module's, operation for operation.
//
// The multiply-adds are written as two operations: -ffp-contract=off
// keeps the compiler from fusing them, so the lanes match each other
// across machines.
#pragma GCC optimize("fp-contract=off")
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#include <limits.h>
#elif defined(__APPLE__)
#include <dispatch/dispatch.h>
#include <sched.h>
#include <unistd.h>
#else
#include <sched.h>
#include <semaphore.h>
#include <unistd.h>
#endif

// --- threads --------------------------------------------------------------------------------------------------

// Which worker the calling thread is, set by the task running on it: the
// per-worker scratch every module keeps is chosen by this. The main
// thread is worker 0 until a task says otherwise.
static _Thread_local int g_worker_index = 0;

int aephysics_worker_index(void) { return g_worker_index; }
void aephysics_set_worker_index(int index) { g_worker_index = index; }

// The calling thread gives up the rest of its slice, for a spin that waits on another worker.
void aephysics_yield(void)
{
#ifdef _WIN32
    SwitchToThread();
#else
    sched_yield();
#endif
}

// The processors the machine offers, at least one.
int aephysics_processor_count(void)
{
#ifdef _WIN32
    SYSTEM_INFO info;
    GetSystemInfo(&info);
    return info.dwNumberOfProcessors > 0 ? (int)info.dwNumberOfProcessors : 1;
#else
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    return n > 0 ? (int)n : 1;
#endif
}

// A counting semaphore (the reference's b3Semaphore): the scheduler's
// threads wait on it for work and are signalled one per task, or once
// each to shut down. Win32's, Apple's dispatch one, POSIX's elsewhere.
#ifdef _WIN32
void *aephysics_semaphore_new(int initial) { return CreateSemaphoreExW(NULL, initial, INT_MAX, NULL, 0, SEMAPHORE_ALL_ACCESS); }
void aephysics_semaphore_free(void *s) { CloseHandle((HANDLE)s); }
void aephysics_semaphore_wait(void *s) { WaitForSingleObjectEx((HANDLE)s, INFINITE, FALSE); }
void aephysics_semaphore_signal(void *s, int count) { ReleaseSemaphore((HANDLE)s, count, NULL); }
#elif defined(__APPLE__)
void *aephysics_semaphore_new(int initial) { return (void *)dispatch_semaphore_create(initial); }
void aephysics_semaphore_free(void *s) { dispatch_release((dispatch_semaphore_t)s); }
void aephysics_semaphore_wait(void *s) { dispatch_semaphore_wait((dispatch_semaphore_t)s, DISPATCH_TIME_FOREVER); }
void aephysics_semaphore_signal(void *s, int count)
{
    for (int i = 0; i < count; ++i) dispatch_semaphore_signal((dispatch_semaphore_t)s);
}
#else
void *aephysics_semaphore_new(int initial)
{
    sem_t *s = malloc(sizeof(sem_t));
    if (s != NULL && sem_init(s, 0, (unsigned int)initial) != 0) {
        free(s);
        return NULL;
    }
    return s;
}
void aephysics_semaphore_free(void *s)
{
    sem_destroy((sem_t *)s);
    free(s);
}
void aephysics_semaphore_wait(void *s)
{
    while (sem_wait((sem_t *)s) != 0) {
    }
}
void aephysics_semaphore_signal(void *s, int count)
{
    for (int i = 0; i < count; ++i) sem_post((sem_t *)s);
}
#endif

// A spin's pause: the processor's hint that the thread is waiting.
void aephysics_pause(void)
{
#if defined(__x86_64__) || defined(__i386__)
    __builtin_ia32_pause();
#elif defined(__aarch64__)
    __asm__ __volatile__("yield");
#endif
}

// Atomics on an int in place, sequentially consistent like the
// reference's C11 atomics: the load and store, fetch-add and fetch-or
// (the value before), and compare-and-swap (whether it swapped).
int aephysics_atomic_load(int *p) { return __atomic_load_n(p, __ATOMIC_SEQ_CST); }
void aephysics_atomic_store(int *p, int value) { __atomic_store_n(p, value, __ATOMIC_SEQ_CST); }
int aephysics_atomic_add(int *p, int value) { return __atomic_fetch_add(p, value, __ATOMIC_SEQ_CST); }
int aephysics_atomic_or(int *p, int value) { return __atomic_fetch_or(p, value, __ATOMIC_SEQ_CST); }
int aephysics_atomic_cas(int *p, int expected, int desired)
{
    return __atomic_compare_exchange_n(p, &expected, desired, 0, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
}

// --- the contact lanes ----------------------------------------------------------------------------------------

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

// The step's constraints: the module's block, and the float copy the
// solve reads at the same offsets, grown to the largest step so far. The
// block is set once a step (aephysics_wide_begin); the prepare below
// writes a lane's floats straight into the copy, and the impulses are
// unpacked by ranges before the store, by whichever worker has the
// range.
static const wide_constraint_d* g_block = NULL;
static wide_constraint* g_packed = NULL;
static int g_packed_capacity = 0;

// The step's block, and room for its float copy.
void aephysics_wide_begin(const void* block, int count)
{
    g_block = block;
    if (count > g_packed_capacity) {
        free(g_packed);
        g_packed_capacity = count + count / 2 + 1;
        g_packed = malloc((size_t)g_packed_capacity * sizeof(wide_constraint));
    }
}

void aephysics_wide_end(void) { g_block = NULL; }

static inline d4 widen(v4 a) { return (d4){ a[0], a[1], a[2], a[3] }; }
static inline vec3d widen3(vec3w a) { return (vec3d){ widen(a.x), widen(a.y), widen(a.z) }; }

// A range's impulses back into the module's block, before its store.
void aephysics_wide_unpack(void* constraints, int count)
{
    wide_constraint_d* base = constraints;
    const wide_constraint* packed = g_packed + (base - g_block);
    for (int i = 0; i < count; ++i) {
        const wide_constraint* f = packed + i;
        wide_constraint_d* d = base + i;
        d->twist_impulse = widen(f->twist_impulse);
        d->friction_impulse = (vec2d){ widen(f->friction_impulse.x), widen(f->friction_impulse.y) };
        d->rolling_impulse = widen3(f->rolling_impulse);
        for (int j = 0; j < 4; ++j) {
            d->points[j].normal_impulse = widen(f->points[j].normal_impulse);
            d->points[j].total_normal_impulse = widen(f->points[j].total_normal_impulse);
        }
    }
}

// A stage's constraints: the module passes a pointer into its block; the
// same offset into the packed copy.
static inline wide_constraint* packed_of(const void* constraints)
{
    return g_packed + ((const wide_constraint_d*)constraints - g_block);
}

int aephysics_wide_constraint_size(void) { return (int)sizeof(wide_constraint_d); }
int aephysics_wide_body_state_size(void) { return (int)sizeof(body_state); }

// --- the prepare, straight into the lanes ---------------------------------------------------------------------
//
// b3PrepareContacts_Convex's lane body, in doubles as the module computes
// it (operation for operation, so a lane prepared here is the lane the
// module prepares) and stored as the floats the solve reads. The module
// used to prepare a lane at a time into its double block and this file
// packed the block to floats: a millisecond a step of packing on the
// large pyramid, and a prepare writing doubles a lane at a time at twice
// the reference's cost. The module's structures are read through the
// field offsets it measures at start (aephysics_wide_layout), so this
// file declares none of them.
//
// What the store reads afterwards goes into the double block too: the
// bodies' indices, the point counts, the manifolds, the tangents and
// each point's relative velocity; the impulses come back from the solve
// through aephysics_wide_unpack.

enum {
    OFF_C_INDEX_A, OFF_C_INDEX_B, OFF_C_MANIFOLDS, OFF_C_FRICTION, OFF_C_RESTITUTION, OFF_C_ROLLING, OFF_C_TANGENT_VELOCITY,
    OFF_M_POINTS, OFF_M_POINT_STRIDE, OFF_M_NORMAL, OFF_M_TWIST_IMPULSE, OFF_M_FRICTION_IMPULSE, OFF_M_ROLLING_IMPULSE, OFF_M_POINT_COUNT,
    OFF_P_ANCHOR_A, OFF_P_ANCHOR_B, OFF_P_SEPARATION, OFF_P_NORMAL_IMPULSE,
    OFF_S_INV_MASS, OFF_S_INV_INERTIA_WORLD, OFF_S_SIZE,
    OFF_B_LINEAR, OFF_B_ANGULAR, OFF_B_SIZE,
    OFF_COUNT
};

static int g_off[OFF_COUNT];
static int g_layout_known = 0;

// The step's inputs the prepare shares: the body arrays and the softness.
static const char* g_sims = NULL;
static const char* g_states = NULL;
static double g_soft[3], g_static_soft[3];
static double g_warm_start_scale = 0.0;

int aephysics_wide_layout_count(void) { return OFF_COUNT; }

void aephysics_wide_layout(const int* offsets)
{
    memcpy(g_off, offsets, sizeof(g_off));
    g_layout_known = 1;
}

void aephysics_wide_prepare_begin(const void* sims, const void* states,
                                  double bias_rate, double mass_scale, double impulse_scale,
                                  double static_bias_rate, double static_mass_scale, double static_impulse_scale,
                                  int enable_warm_starting)
{
    g_sims = sims;
    g_states = states;
    g_soft[0] = bias_rate; g_soft[1] = mass_scale; g_soft[2] = impulse_scale;
    g_static_soft[0] = static_bias_rate; g_static_soft[1] = static_mass_scale; g_static_soft[2] = static_impulse_scale;
    g_warm_start_scale = enable_warm_starting ? 1.0 : 0.0;
}

// A slot's float lanes zeroed: the tail slot of a colour, whose spare
// lanes must reach no body.
void aephysics_wide_zero_packed(const void* slot)
{
    memset(packed_of(slot), 0, sizeof(wide_constraint));
}

typedef struct { double x, y, z; } dv3;
typedef struct { dv3 cx, cy, cz; } dm3;

#define FIELD(base, off, type) (*(const type*)((const char*)(base) + (off)))
#define FIELD_DV3(base, off) (*(const dv3*)((const char*)(base) + (off)))

static inline dv3 dv3_zero(void) { return (dv3){ 0.0, 0.0, 0.0 }; }
static inline dv3 dv3_add(dv3 a, dv3 b) { return (dv3){ a.x + b.x, a.y + b.y, a.z + b.z }; }
static inline dv3 dv3_sub(dv3 a, dv3 b) { return (dv3){ a.x - b.x, a.y - b.y, a.z - b.z }; }
static inline double dv3_dot(dv3 a, dv3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline dv3 dv3_cross(dv3 a, dv3 b)
{
    return (dv3){ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
static inline dv3 dv3_mul_add(dv3 a, double s, dv3 b) { return (dv3){ a.x + s * b.x, a.y + s * b.y, a.z + s * b.z }; }
static inline dv3 dv3_mul_sv(double s, dv3 a) { return (dv3){ s * a.x, s * a.y, s * a.z }; }
static inline double dv3_length(dv3 v) { return sqrt(dv3_dot(v, v)); }
// math.normalize: a unit vector, or zero for an input too small to have a direction.
#define AE_TINY 0.0000000000000000000000000000000000000117549435
static inline dv3 dv3_normalize(dv3 a)
{
    double ls = a.x * a.x + a.y * a.y + a.z * a.z;
    if (ls > 1000.0 * AE_TINY) {
        double s = 1.0 / sqrt(ls);
        return (dv3){ s * a.x, s * a.y, s * a.z };
    }
    return dv3_zero();
}
// math.perp
static inline dv3 dv3_perp(dv3 a)
{
    if (a.x < 0.0 - 0.5 || 0.5 < a.x) return dv3_normalize((dv3){ a.y, 0.0 - a.x, 0.0 });
    return dv3_normalize((dv3){ 0.0, a.z, 0.0 - a.y });
}
static inline dm3 dm3_zero(void) { return (dm3){ dv3_zero(), dv3_zero(), dv3_zero() }; }
static inline dv3 dm3_mul_mv(dm3 m, dv3 a)
{
    return (dv3){ m.cx.x * a.x + m.cy.x * a.y + m.cz.x * a.z,
                  m.cx.y * a.x + m.cy.y * a.y + m.cz.y * a.z,
                  m.cx.z * a.x + m.cy.z * a.y + m.cz.z * a.z };
}
static inline dm3 dm3_add(dm3 a, dm3 b) { return (dm3){ dv3_add(a.cx, b.cx), dv3_add(a.cy, b.cy), dv3_add(a.cz, b.cz) }; }
static inline double dm3_det(dm3 m) { return dv3_dot(m.cx, dv3_cross(m.cy, m.cz)); }
static inline double ae_abs(double a) { return a < 0.0 ? 0.0 - a : a; }
// math.invert_matrix: the inverse, or zero when singular.
static inline dm3 dm3_invert(dm3 m)
{
    double d = dm3_det(m);
    if (ae_abs(d) > 1000.0 * AE_TINY) {
        double inv = 1.0 / d;
        dm3 out = { dv3_mul_sv(inv, dv3_cross(m.cy, m.cz)), dv3_mul_sv(inv, dv3_cross(m.cz, m.cx)), dv3_mul_sv(inv, dv3_cross(m.cx, m.cy)) };
        return (dm3){ { out.cx.x, out.cy.x, out.cz.x }, { out.cx.y, out.cy.y, out.cz.y }, { out.cx.z, out.cy.z, out.cz.z } };
    }
    return dm3_zero();
}
static inline double ae_clamp(double a, double lower, double upper)
{
    if (a < lower) return lower;
    if (upper < a) return upper;
    return a;
}

static inline void lane_set(v4* f, int l, double v) { (*f)[l] = (float)v; }
static inline void lane_set3(vec3w* f, int l, dv3 v) { lane_set(&f->x, l, v.x); lane_set(&f->y, l, v.y); lane_set(&f->z, l, v.z); }
static inline void lane_set_sym3(sym3w* f, int l, dm3 m)
{
    lane_set(&f->cxx, l, m.cx.x); lane_set(&f->cxy, l, m.cx.y); lane_set(&f->cxz, l, m.cx.z);
    lane_set(&f->cyy, l, m.cy.y); lane_set(&f->cyz, l, m.cy.z); lane_set(&f->czz, l, m.cz.z);
}
static inline void dlane_set(d4* d, int l, double v) { ((double*)d)[l] = v; }
static inline void dlane_set3(vec3d* d, int l, dv3 v) { dlane_set(&d->x, l, v.x); dlane_set(&d->y, l, v.y); dlane_set(&d->z, l, v.z); }

#define MAX_MANIFOLD_POINTS 4
#define SPECULATIVE_DISTANCE 0.02
#define MIN_FRICTION_WEIGHT 0.0000000001
#define NULL_INDEX (-1)

// One convex contact into lane `l` of the slot the module points at.
void aephysics_wide_prepare(const void* contact, void* slot, int l)
{
    wide_constraint_d* d = slot;
    wide_constraint* f = packed_of(slot);
    const char* c = contact;
    const char* manifold = FIELD(c, g_off[OFF_C_MANIFOLDS], const char*);
    int index_a = FIELD(c, g_off[OFF_C_INDEX_A], int32_t);
    int index_b = FIELD(c, g_off[OFF_C_INDEX_B], int32_t);
    double inv_tau = 1.0 / SPECULATIVE_DISTANCE;

    ((int32_t*)&d->index_a)[l] = index_a + 1;
    ((int32_t*)&d->index_b)[l] = index_b + 1;
    ((int32_t*)&f->index_a)[l] = index_a + 1;
    ((int32_t*)&f->index_b)[l] = index_b + 1;
    ((void**)&d->manifolds)[l] = (void*)manifold;

    double m_a = 0.0, m_b = 0.0;
    dm3 i_a = dm3_zero(), i_b = dm3_zero();
    dv3 v_a = dv3_zero(), w_a = dv3_zero(), v_b = dv3_zero(), w_b = dv3_zero();
    if (index_a != NULL_INDEX) {
        const char* sim = g_sims + (size_t)index_a * g_off[OFF_S_SIZE];
        const char* state = g_states + (size_t)index_a * g_off[OFF_B_SIZE];
        m_a = FIELD(sim, g_off[OFF_S_INV_MASS], double);
        i_a = FIELD(sim, g_off[OFF_S_INV_INERTIA_WORLD], dm3);
        v_a = FIELD_DV3(state, g_off[OFF_B_LINEAR]);
        w_a = FIELD_DV3(state, g_off[OFF_B_ANGULAR]);
    }
    if (index_b != NULL_INDEX) {
        const char* sim = g_sims + (size_t)index_b * g_off[OFF_S_SIZE];
        const char* state = g_states + (size_t)index_b * g_off[OFF_B_SIZE];
        m_b = FIELD(sim, g_off[OFF_S_INV_MASS], double);
        i_b = FIELD(sim, g_off[OFF_S_INV_INERTIA_WORLD], dm3);
        v_b = FIELD_DV3(state, g_off[OFF_B_LINEAR]);
        w_b = FIELD_DV3(state, g_off[OFF_B_ANGULAR]);
    }
    lane_set(&f->inv_mass_a, l, m_a);
    lane_set(&f->inv_mass_b, l, m_b);
    lane_set_sym3(&f->inv_inertia_a, l, i_a);
    lane_set_sym3(&f->inv_inertia_b, l, i_b);
    const double* soft = (index_a == NULL_INDEX || index_b == NULL_INDEX) ? g_static_soft : g_soft;
    dv3 normal = FIELD_DV3(manifold, g_off[OFF_M_NORMAL]);
    dv3 tangent1 = dv3_perp(normal);
    dv3 tangent2 = dv3_cross(tangent1, normal);
    lane_set3(&f->normal, l, normal);
    lane_set3(&f->tangent1, l, tangent1);
    lane_set3(&f->tangent2, l, tangent2);
    dlane_set3(&d->tangent1, l, tangent1);
    dlane_set3(&d->tangent2, l, tangent2);
    dv3 tangent_velocity = FIELD_DV3(c, g_off[OFF_C_TANGENT_VELOCITY]);
    lane_set(&f->friction, l, FIELD(c, g_off[OFF_C_FRICTION], double));
    lane_set(&f->restitution, l, FIELD(c, g_off[OFF_C_RESTITUTION], double));
    lane_set(&f->rolling_resistance, l, FIELD(c, g_off[OFF_C_ROLLING], double));
    lane_set(&f->tangent_velocity1, l, dv3_dot(tangent_velocity, tangent1));
    lane_set(&f->tangent_velocity2, l, dv3_dot(tangent_velocity, tangent2));
    lane_set(&f->bias_rate, l, soft[0]);
    lane_set(&f->mass_scale, l, soft[1]);
    lane_set(&f->impulse_scale, l, soft[2]);
    int point_count = FIELD(manifold, g_off[OFF_M_POINT_COUNT], int32_t);
    ((int32_t*)&d->point_counts)[l] = point_count;
    ((int32_t*)&f->point_counts)[l] = point_count;
    dv3 center_a = dv3_zero(), center_b = dv3_zero();
    double total_weight = 0.0;
    for (int p = 0; p < point_count; ++p) {
        const char* mp = manifold + g_off[OFF_M_POINTS] + (size_t)p * g_off[OFF_M_POINT_STRIDE];
        wide_point* cp = f->points + p;
        dv3 r_a = FIELD_DV3(mp, g_off[OFF_P_ANCHOR_A]);
        dv3 r_b = FIELD_DV3(mp, g_off[OFF_P_ANCHOR_B]);
        double s = FIELD(mp, g_off[OFF_P_SEPARATION], double);
        // The friction centre decays with separation (the scalar prepare says why).
        double weight = ae_clamp(2.0 - s * inv_tau, MIN_FRICTION_WEIGHT, 1.0);
        center_a = dv3_mul_add(center_a, weight, r_a);
        center_b = dv3_mul_add(center_b, weight, r_b);
        total_weight = total_weight + weight;
        lane_set3(&cp->anchor_a, l, r_a);
        lane_set3(&cp->anchor_b, l, r_b);
        lane_set(&cp->base_separation, l, s - dv3_dot(dv3_sub(r_b, r_a), normal));
        lane_set(&cp->normal_impulse, l, g_warm_start_scale * FIELD(mp, g_off[OFF_P_NORMAL_IMPULSE], double));
        lane_set(&cp->total_normal_impulse, l, 0.0);
        dv3 rn_a = dv3_cross(r_a, normal);
        dv3 rn_b = dv3_cross(r_b, normal);
        double k_normal = m_a + m_b + dv3_dot(rn_a, dm3_mul_mv(i_a, rn_a)) + dv3_dot(rn_b, dm3_mul_mv(i_b, rn_b));
        double normal_mass = 0.0;
        if (k_normal > 0.0) normal_mass = 1.0 / k_normal;
        lane_set(&cp->normal_mass, l, normal_mass);
        dv3 vr_a = dv3_add(v_a, dv3_cross(w_a, r_a));
        dv3 vr_b = dv3_add(v_b, dv3_cross(w_b, r_b));
        double relative_velocity = dv3_dot(normal, dv3_sub(vr_b, vr_a));
        lane_set(&cp->relative_velocity, l, relative_velocity);
        dlane_set(&d->points[p].relative_velocity, l, relative_velocity);
    }
    double inv_weight = 1.0 / total_weight;
    center_a = dv3_mul_sv(inv_weight, center_a);
    center_b = dv3_mul_sv(inv_weight, center_b);
    lane_set3(&f->center_a, l, center_a);
    lane_set3(&f->center_b, l, center_b);
    for (int p = 0; p < point_count; ++p) {
        const char* mp = manifold + g_off[OFF_M_POINTS] + (size_t)p * g_off[OFF_M_POINT_STRIDE];
        dv3 r_a = FIELD_DV3(mp, g_off[OFF_P_ANCHOR_A]);
        lane_set(&f->points[p].lever_arm, l, dv3_length(dv3_sub(center_a, r_a)));
    }
    dv3 rt_a1 = dv3_cross(center_a, tangent1);
    dv3 rt_a2 = dv3_cross(center_a, tangent2);
    dv3 rt_b1 = dv3_cross(center_b, tangent1);
    dv3 rt_b2 = dv3_cross(center_b, tangent2);
    double kxx = m_a + m_b + dv3_dot(rt_a1, dm3_mul_mv(i_a, rt_a1)) + dv3_dot(rt_b1, dm3_mul_mv(i_b, rt_b1));
    double kyy = m_a + m_b + dv3_dot(rt_a2, dm3_mul_mv(i_a, rt_a2)) + dv3_dot(rt_b2, dm3_mul_mv(i_b, rt_b2));
    double kxy = dv3_dot(rt_a1, dm3_mul_mv(i_a, rt_a2)) + dv3_dot(rt_b1, dm3_mul_mv(i_b, rt_b2));
    // math.invert2 of the 2x2 { cx: {kxx, kxy}, cy: {kxy, kyy} }: the inverse, or zero when singular.
    double det2 = kxx * kyy - kxy * kxy;
    double t_cxx = 0.0, t_cxy = 0.0, t_cyy = 0.0;
    if (ae_abs(det2) > 1000.0 * AE_TINY) {
        double inv = 1.0 / det2;
        t_cxx = inv * kyy;
        t_cxy = 0.0 - inv * kxy;
        t_cyy = inv * kxx;
    }
    lane_set(&f->tangent_mass.cxx, l, t_cxx);
    lane_set(&f->tangent_mass.cxy, l, t_cxy);
    lane_set(&f->tangent_mass.cyy, l, t_cyy);
    dv3 friction_impulse = FIELD_DV3(manifold, g_off[OFF_M_FRICTION_IMPULSE]);
    lane_set(&f->friction_impulse.x, l, g_warm_start_scale * dv3_dot(friction_impulse, tangent1));
    lane_set(&f->friction_impulse.y, l, g_warm_start_scale * dv3_dot(friction_impulse, tangent2));
    dm3 i_sum = dm3_add(i_a, i_b);
    double k_twist = dv3_dot(normal, dm3_mul_mv(i_sum, normal));
    double twist_mass = 0.0;
    if (k_twist > 0.0) twist_mass = 1.0 / k_twist;
    lane_set(&f->twist_mass, l, twist_mass);
    lane_set(&f->twist_impulse, l, g_warm_start_scale * FIELD(manifold, g_off[OFF_M_TWIST_IMPULSE], double));
    lane_set_sym3(&f->rolling_mass, l, dm3_invert(i_sum));
    lane_set3(&f->rolling_impulse, l, dv3_mul_sv(g_warm_start_scale, FIELD_DV3(manifold, g_off[OFF_M_ROLLING_IMPULSE])));
    // The points the manifold lacks are zero: nothing reaches the bodies through them.
    for (int p = point_count; p < MAX_MANIFOLD_POINTS; ++p) {
        wide_point* cp = f->points + p;
        lane_set3(&cp->anchor_a, l, dv3_zero());
        lane_set3(&cp->anchor_b, l, dv3_zero());
        lane_set(&cp->base_separation, l, 0.0);
        lane_set(&cp->normal_impulse, l, 0.0);
        lane_set(&cp->total_normal_impulse, l, 0.0);
        lane_set(&cp->normal_mass, l, 0.0);
        lane_set(&cp->relative_velocity, l, 0.0);
        lane_set(&cp->lever_arm, l, 0.0);
        dlane_set(&d->points[p].relative_velocity, l, 0.0);
    }
}

static inline v4 splat(float s) { return (v4){ s, s, s, s }; }
// Per lane: b where the comparison held, a elsewhere.
static inline v4 pick(v4 a, v4 b, m4 m) { return (v4)(((m4)b & m) | ((m4)a & ~m)); }
static inline v4 vmax(v4 a, v4 b) { return pick(b, a, a >= b); }
static inline v4 vmin(v4 a, v4 b) { return pick(b, a, a <= b); }
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
static inline v4 blend(v4 a, v4 b, v4 m) { return pick(a, b, m != splat(0.0f)); }
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
