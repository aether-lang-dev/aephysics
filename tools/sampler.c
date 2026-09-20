// A sampling profiler for MinGW: a thread suspends the main thread every
// ~0.5 ms and records its instruction pointer; a destructor dumps the
// samples as "addr count" lines (with the module base) to sample_out.txt.
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#define MAXS (1 << 22)
static HANDLE g_main;
static DWORD64 *g_samples;
static volatile LONG g_count;
static volatile int g_stop;
static DWORD WINAPI sampler(LPVOID p)
{
    (void)p;
    LARGE_INTEGER f, t0, t1; QueryPerformanceFrequency(&f);
    while (!g_stop && g_count < MAXS) {
        QueryPerformanceCounter(&t0);
        do { QueryPerformanceCounter(&t1); } while ((t1.QuadPart - t0.QuadPart) * 2000 < f.QuadPart);
        if (SuspendThread(g_main) == (DWORD)-1) break;
        CONTEXT c; c.ContextFlags = CONTEXT_CONTROL;
        if (GetThreadContext(g_main, &c)) g_samples[g_count++] = c.Rip;
        ResumeThread(g_main);
    }
    return 0;
}
__attribute__((constructor)) static void start(void)
{
    DuplicateHandle(GetCurrentProcess(), GetCurrentThread(), GetCurrentProcess(), &g_main, 0, FALSE, DUPLICATE_SAME_ACCESS);
    g_samples = malloc(MAXS * sizeof(DWORD64));
    CreateThread(NULL, 0, sampler, NULL, 0, NULL);
}
__attribute__((destructor)) static void stop(void)
{
    g_stop = 1; Sleep(5);
    FILE *f = fopen("sample_out.txt", "w");
    fprintf(f, "BASE %llx\n", (unsigned long long)(ULONG_PTR)GetModuleHandle(NULL));
    for (LONG i = 0; i < g_count; ++i) fprintf(f, "%llx\n", (unsigned long long)g_samples[i]);
    fclose(f);
}
