// The native side of aephysics, one C file built with every program
// (`ae build --extra aephysics/native/aephysics_native.c`): what Aether
// cannot express yet, and nothing else.
//
// The threads' side of aephysics.parallel and the solver's stages: a
// thread-local worker index (Aether has no thread-local variables), a
// yield and a pause, the processor count, atomic operations on an int
// in place (std.sync's atomics are cells of their own; the solver's
// blocks and the tree's nodes carry theirs in the struct, as the
// reference does), and a counting semaphore the scheduler's threads wait
// on between steps.
//
// The contact solver's lanes lived here too, as GCC vector code, while
// Aether had no lane type. It has std.lanes now, the module's lanes are
// f32x4, they computed the same record as this file's to the bit and ran
// faster on the pyramids (aephysics#48), so they went (#46).
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
