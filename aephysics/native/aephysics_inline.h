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

// The 8-byte word at `index` words into `p`, read through memcpy so a
// block written as doubles, ints and structs can be read as words without
// breaking C's aliasing rule (a plain int64_t* read let GCC move the read
// ahead of the stores it follows once the hash was inlined).
static inline long long aephysics_load_word(const void *p, long long index)
{
    long long word;
    __builtin_memcpy(&word, (const char *)p + 8 * index, 8);
    return word;
}

// The byte at `index` into `p`, unsigned (a hashed block's tail).
static inline int aephysics_load_u8(const void *p, long long index)
{
    return ((const unsigned char *)p)[index];
}

// The unsigned 16-bit value at `index` halves into `p` (a height field's
// quanta), one load where two byte reads and a shift were the field's
// queries' extra cost.
static inline int aephysics_load_u16(const void *p, long long index)
{
    unsigned short value;
    __builtin_memcpy(&value, (const char *)p + 2 * index, 2);
    return value;
}

// A single's and a double's bits, unsigned in the single's case (the
// world state hash, which mixes every value as its bits).
static inline long long aephysics_f32_bits(float value)
{
    unsigned int bits;
    __builtin_memcpy(&bits, &value, 4);
    return bits;
}

static inline long long aephysics_f64_bits(double value)
{
    long long bits;
    __builtin_memcpy(&bits, &value, 8);
    return bits;
}

// A double narrowed to the nearest float at or below it, and at or above
// it: the bounds of a single-precision box rounded outward (the
// reference's b3RoundDownFloat and b3RoundUpFloat). The step to the next
// float is the one on its bits that nextafterf takes, inline: a box is
// rounded on every tree query, and the libm call was most of the cost.
// Zero steps to the smallest denormal of the other sign, the largest
// float to infinity and infinity to the largest float, as nextafterf's do;
// a NaN is never stepped. Written without branches: whether a coordinate
// needs the step is a coin flip, and a branch on it mispredicted half the
// time (the continuous pass rounds a box for every fast shape).
static inline float aephysics_round_down_f32(double x)
{
    float f = (float)x;
    unsigned int bits;
    __builtin_memcpy(&bits, &f, 4);
    unsigned int need = (double)f > x;
    unsigned int stepped = bits + ((bits >> 31) ? 1u : 0xFFFFFFFFu);
    stepped = f == 0.0f ? 0x80000001u : stepped;
    bits = need ? stepped : bits;
    __builtin_memcpy(&f, &bits, 4);
    return f;
}

static inline float aephysics_round_up_f32(double x)
{
    float f = (float)x;
    unsigned int bits;
    __builtin_memcpy(&bits, &f, 4);
    unsigned int need = (double)f < x;
    unsigned int stepped = bits + ((bits >> 31) ? 0xFFFFFFFFu : 1u);
    stepped = f == 0.0f ? 0x00000001u : stepped;
    bits = need ? stepped : bits;
    __builtin_memcpy(&f, &bits, 4);
    return f;
}

#endif
