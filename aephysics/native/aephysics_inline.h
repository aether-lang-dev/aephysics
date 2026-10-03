// The native helpers small enough that a call costs more than they do,
// as static inline definitions the generated C includes (@c_include in
// aephysics.native, with @c_import on their externs): the prefetch the
// collide pass, the prepare and the store issue per contact, the spin's
// pause and the atomics the solver's block claims and sync bits use.
// Out of line in aephysics_native.c they were a call each, and the
// prefetch alone was 2% of the large pyramid's step.
#ifndef AEPHYSICS_INLINE_H
#define AEPHYSICS_INLINE_H

// A hint to fetch the cache line at `p` ahead of its use (the reference's
// narrow phase and contact prepare prefetch the contacts to come); Aether
// has no prefetch of its own. A null or stale pointer is harmless: a
// prefetch never faults.
static inline void aephysics_prefetch(const void *p)
{
#if defined(__GNUC__) || defined(__clang__)
    __builtin_prefetch(p, 0, 3);
#else
    (void)p;
#endif
}

// A spin's pause: the processor's hint that the thread is waiting.
static inline void aephysics_pause(void)
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
static inline int aephysics_atomic_load(void *p) { return __atomic_load_n((int *)p, __ATOMIC_SEQ_CST); }
static inline void aephysics_atomic_store(void *p, int value) { __atomic_store_n((int *)p, value, __ATOMIC_SEQ_CST); }
static inline int aephysics_atomic_add(void *p, int value) { return __atomic_fetch_add((int *)p, value, __ATOMIC_SEQ_CST); }
static inline int aephysics_atomic_or(void *p, int value) { return __atomic_fetch_or((int *)p, value, __ATOMIC_SEQ_CST); }
static inline int aephysics_atomic_cas(void *p, int expected, int desired)
{
    return __atomic_compare_exchange_n((int *)p, &expected, desired, 0, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
}

#endif
