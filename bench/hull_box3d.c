// The hull builder of the reference on the same scenes as bench/hull.ae:
// 200 hulls of 64 sphere points capped at 32 vertices, 20 hulls of 4,096
// points inside a cube (the merge churn), and 2,000 boxes. Single
// thread, wall time per phase.
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static double now_ms( void )
{
	struct timespec ts;
	timespec_get( &ts, TIME_UTC );
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static uint32_t s_seed = 12345;
static float unit_random( void )
{
	s_seed ^= s_seed << 13;
	s_seed ^= s_seed >> 17;
	s_seed ^= s_seed << 5;
	return (float)( s_seed & 32767u ) / 32767.0f;
}

static void fill_sphere( b3Vec3* points, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		float u1 = unit_random();
		float u2 = 2.0f * B3_PI * unit_random();
		float u3 = 2.0f * B3_PI * unit_random();
		float s1 = sqrtf( 1.0f - u1 ), s2 = sqrtf( u1 );
		points[i] = (b3Vec3){ s1 * sinf( u2 ), s1 * cosf( u2 ), s2 * sinf( u3 ) };
	}
}

static void fill_cube( b3Vec3* points, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		points[i] = (b3Vec3){ 2.0f * unit_random() - 1.0f, 2.0f * unit_random() - 1.0f, 2.0f * unit_random() - 1.0f };
	}
	for ( int c = 0; c < 8; ++c )
	{
		points[count - 8 + c] = (b3Vec3){ ( c & 1 ) ? 1.0f : -1.0f, ( c & 2 ) ? 1.0f : -1.0f, ( c & 4 ) ? 1.0f : -1.0f };
	}
}

int main( void )
{
	b3Vec3* points = malloc( 4096 * sizeof( b3Vec3 ) );
	int vertices = 0, faces = 0;

	s_seed = 12345;
	double t0 = now_ms();
	for ( int i = 0; i < 200; ++i )
	{
		fill_sphere( points, 64 );
		b3HullData* hull = b3CreateHull( points, 64, 32 );
		vertices += hull->vertexCount;
		faces += hull->faceCount;
		b3DestroyHull( hull );
	}
	double t1 = now_ms();
	for ( int i = 0; i < 20; ++i )
	{
		fill_cube( points, 4096 );
		b3HullData* hull = b3CreateHull( points, 4096, 64 );
		vertices += hull->vertexCount;
		faces += hull->faceCount;
		b3DestroyHull( hull );
	}
	double t2 = now_ms();
	float volume = 0.0f;
	for ( int i = 0; i < 2000; ++i )
	{
		b3BoxHull box = b3MakeBoxHull( 0.5f + 0.001f * i, 0.5f, 0.5f );
		volume += box.base.volume;
	}
	double t3 = now_ms();
	printf( "box3d hull: 200 spheres (64 -> 32) %.2f ms, 20 cubes (4096 -> 8) %.2f ms, 2000 boxes %.2f ms, %d vertices, %d faces, box volume %.1f\n",
			t1 - t0, t2 - t1, t3 - t2, vertices, faces, volume );
	free( points );
	return 0;
}
