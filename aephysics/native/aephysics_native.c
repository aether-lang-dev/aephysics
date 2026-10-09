// The native side of aephysics, one C file built with every program
// (`ae build --extra aephysics/native/aephysics_native.c`): what Aether
// cannot express yet, and nothing else.
//
// The threads' side of aephysics.parallel and the solver's stages: a
// thread-local worker index (Aether has no thread-local variables), a
// yield, the processor count, and a counting semaphore the scheduler's
// threads wait on between steps. The atomics on an int in place
// (std.sync's atomics are cells of their own; the solver's blocks and
// the tree's nodes carry theirs in the struct, as the reference does),
// the pause and the prefetch are in aephysics_inline.h, inlined.
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

// --- names ----------------------------------------------------------------------------------------------------

// A name the engine keeps (the world's name cache, a recording's tags) as a
// plain zero-ended buffer, read back as a string: Aether frees only what
// carries its own header, so a plain buffer handed back is read as a
// literal and left alone. Empty for none.
const char *aephysics_name_text(const void *name) { return name != NULL ? (const char *)name : ""; }

// --- threads --------------------------------------------------------------------------------------------------

// --- worker slots ---------------------------------------------------------------------------------------------

// The worker slots a program's threads and worlds hold, a bit each, and
// MAX_WORKERS (64) of them: every module keeps a scratch block per slot,
// chosen by the calling thread's index, so two threads running at once
// must never hold the same one. A world claims a run of its worker count
// when made and gives it back when destroyed; a thread that is not one of
// a world's workers (the main thread, a host's own threads) claims one
// the first time it asks for its index, and gives it back when it exits.
// The reference keeps the same scratch on each thread's stack, which is
// what makes its queries safe from any thread; this is that, by slot.
static volatile uint64_t g_slot_mask = 0;

// The first slot of a free run of `count`, claimed; -1 when there is none.
int aephysics_claim_slots(int count)
{
    if (count < 1 || count > 64) return -1;
    uint64_t run = count == 64 ? ~(uint64_t)0 : (((uint64_t)1 << count) - 1);
    for (;;) {
        uint64_t mask = __atomic_load_n(&g_slot_mask, __ATOMIC_SEQ_CST);
        int start = -1;
        for (int i = 0; i + count <= 64; ++i) {
            if ((mask & (run << i)) == 0) {
                start = i;
                break;
            }
        }
        if (start < 0) return -1;
        if (__atomic_compare_exchange_n(&g_slot_mask, &mask, mask | (run << start), 0, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) return start;
    }
}

void aephysics_release_slots(int start, int count)
{
    if (start < 0 || count < 1 || start + count > 64) return;
    uint64_t run = count == 64 ? ~(uint64_t)0 : (((uint64_t)1 << count) - 1);
    __atomic_fetch_and(&g_slot_mask, ~(run << start), __ATOMIC_SEQ_CST);
}

// --- threads --------------------------------------------------------------------------------------------------

// Which worker the calling thread is: the slot it claimed, or the one a
// task running on it set (a world's step and its tasks run in the world's
// own slots). Kept as the index plus one, so a thread that has not asked
// yet reads zero and claims a slot of its own. When every slot is held a
// thread shares slot 0, as every thread did before slots were claimed:
// the reference caps its workers at 64 the same way (B3_MAX_WORKERS).
//
// On Windows, MinGW's gcc builds _Thread_local as emulated TLS: every read
// is a call to __emutls_get_address, and every scratch block of every
// module is chosen through this read, many times a query. A native TLS
// slot (TlsAlloc) reads the thread's own slot instead, and a fiber-local
// slot (FlsAlloc) gives the claimed worker slot back when the thread
// exits. Elsewhere the compiler's TLS is native already, and a pthread
// key's destructor gives the slot back.
#ifdef _WIN32
static DWORD g_worker_slot = TLS_OUT_OF_INDEXES;
static DWORD g_owned_slot = FLS_OUT_OF_INDEXES;

static void WINAPI release_owned_slot(void *value)
{
    if (value != NULL) aephysics_release_slots((int)((intptr_t)value - 1), 1);
}

static DWORD worker_slot(void)
{
    DWORD slot = g_worker_slot;
    if (slot != TLS_OUT_OF_INDEXES) return slot;
    DWORD made = TlsAlloc();
    LONG raced = InterlockedCompareExchange((volatile LONG *)&g_worker_slot, (LONG)made, (LONG)TLS_OUT_OF_INDEXES);
    if (raced != (LONG)TLS_OUT_OF_INDEXES) {
        TlsFree(made);
        return (DWORD)raced;
    }
    return made;
}

static DWORD owned_slot(void)
{
    DWORD slot = g_owned_slot;
    if (slot != FLS_OUT_OF_INDEXES) return slot;
    DWORD made = FlsAlloc(release_owned_slot);
    LONG raced = InterlockedCompareExchange((volatile LONG *)&g_owned_slot, (LONG)made, (LONG)FLS_OUT_OF_INDEXES);
    if (raced != (LONG)FLS_OUT_OF_INDEXES) {
        FlsFree(made);
        return (DWORD)raced;
    }
    return made;
}

static int claim_thread_slot(void)
{
    int index = aephysics_claim_slots(1);
    if (index < 0) {
        index = 0;
    } else {
        FlsSetValue(owned_slot(), (void *)(intptr_t)(index + 1));
    }
    TlsSetValue(worker_slot(), (LPVOID)(intptr_t)(index + 1));
    return index;
}

int aephysics_worker_index(void)
{
    intptr_t stored = (intptr_t)TlsGetValue(worker_slot());
    if (stored != 0) return (int)(stored - 1);
    return claim_thread_slot();
}

void aephysics_set_worker_index(int index) { TlsSetValue(worker_slot(), (LPVOID)(intptr_t)(index + 1)); }
#else
#include <pthread.h>
static _Thread_local int g_worker_index = 0;   // the index plus one; zero until claimed or set
static pthread_key_t g_owned_key;
static pthread_once_t g_owned_once = PTHREAD_ONCE_INIT;

static void release_owned_slot(void *value)
{
    if (value != NULL) aephysics_release_slots((int)((intptr_t)value - 1), 1);
}

static void make_owned_key(void) { pthread_key_create(&g_owned_key, release_owned_slot); }

static int claim_thread_slot(void)
{
    int index = aephysics_claim_slots(1);
    if (index < 0) {
        index = 0;
    } else {
        pthread_once(&g_owned_once, make_owned_key);
        pthread_setspecific(g_owned_key, (void *)(intptr_t)(index + 1));
    }
    g_worker_index = index + 1;
    return index;
}

int aephysics_worker_index(void)
{
    int stored = g_worker_index;
    if (stored != 0) return stored - 1;
    return claim_thread_slot();
}

void aephysics_set_worker_index(int index) { g_worker_index = index + 1; }
#endif

// How deep the calling thread's slot is in traversals of one set of
// stacks (a tree's, the meshes', the compounds'): a query's visitor may
// run another query on the same thread (an overlap checking a line of
// sight), and the slot's stacks belong to the outermost traversal of
// them. `depths` is the set's int[64], one count a slot. A traversal
// entered at depth 0 uses the slot's stacks; a nested one takes stacks of
// its own for its length, as each of the reference's queries has its
// stack on the C stack. A slot is one running thread's, so the counts
// need no atomics.
int aephysics_enter_traversal(void *depths) { return ((int *)depths)[aephysics_worker_index()]++; }
void aephysics_leave_traversal(void *depths) { ((int *)depths)[aephysics_worker_index()]--; }

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

// Whether this CPU runs AVX2 (the reference's b3IsAVX2Available): the
// instruction set and the operating system's saving of its registers, as
// GCC's and Clang's CPU check reads them. Never on a CPU that is not x86.
int aephysics_has_avx2(void)
{
#if defined(__x86_64__) || defined(__i386__)
    __builtin_cpu_init();
    return __builtin_cpu_supports("avx2") ? 1 : 0;
#else
    return 0;
#endif
}

// A counting semaphore (the reference's b3Semaphore): the scheduler's
// threads wait on it for work and are signalled one per task, or once
// each to shut down. Win32's, Apple's dispatch one, POSIX's elsewhere.
// Its block is the caller's (allocated through the engine's allocator, as
// the reference's b3CreateSemaphore allocates its own through b3Alloc):
// aephysics_semaphore_bytes says how big, init makes it, destroy undoes
// init and leaves the block to its owner.
#ifdef _WIN32
typedef struct { HANDLE handle; } AephysicsSemaphore;
int aephysics_semaphore_init(void *block, int initial)
{
    AephysicsSemaphore *s = (AephysicsSemaphore *)block;
    s->handle = CreateSemaphoreExW(NULL, initial, INT_MAX, NULL, 0, SEMAPHORE_ALL_ACCESS);
    return s->handle != NULL;
}
void aephysics_semaphore_destroy(void *block) { CloseHandle(((AephysicsSemaphore *)block)->handle); }
void aephysics_semaphore_wait(void *block) { WaitForSingleObjectEx(((AephysicsSemaphore *)block)->handle, INFINITE, FALSE); }
void aephysics_semaphore_signal(void *block, int count) { ReleaseSemaphore(((AephysicsSemaphore *)block)->handle, count, NULL); }
#elif defined(__APPLE__)
typedef struct { dispatch_semaphore_t handle; } AephysicsSemaphore;
int aephysics_semaphore_init(void *block, int initial)
{
    AephysicsSemaphore *s = (AephysicsSemaphore *)block;
    s->handle = dispatch_semaphore_create(initial);
    return s->handle != NULL;
}
void aephysics_semaphore_destroy(void *block) { dispatch_release(((AephysicsSemaphore *)block)->handle); }
void aephysics_semaphore_wait(void *block) { dispatch_semaphore_wait(((AephysicsSemaphore *)block)->handle, DISPATCH_TIME_FOREVER); }
void aephysics_semaphore_signal(void *block, int count)
{
    for (int i = 0; i < count; ++i) dispatch_semaphore_signal(((AephysicsSemaphore *)block)->handle);
}
#else
typedef struct { sem_t handle; } AephysicsSemaphore;
int aephysics_semaphore_init(void *block, int initial)
{
    return sem_init(&((AephysicsSemaphore *)block)->handle, 0, (unsigned int)initial) == 0;
}
void aephysics_semaphore_destroy(void *block) { sem_destroy(&((AephysicsSemaphore *)block)->handle); }
void aephysics_semaphore_wait(void *block)
{
    while (sem_wait(&((AephysicsSemaphore *)block)->handle) != 0) {
    }
}
void aephysics_semaphore_signal(void *block, int count)
{
    for (int i = 0; i < count; ++i) sem_post(&((AephysicsSemaphore *)block)->handle);
}
#endif
int aephysics_semaphore_bytes(void) { return (int)sizeof(AephysicsSemaphore); }

// The prefetch, the pause and the atomics are static inline in
// aephysics_inline.h, so the generated C inlines them.
