// The dynamic tree of the reference on the same scene as bench/tree.ae:
// 10,000 boxes inserted one by one, 100 full rebuilds, 100,000 box queries,
// 10,000 ray casts, then every proxy moved and a partial rebuild, ten
// rounds. Single thread, wall time per phase.
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
static uint32_t next_random( void )
{
	s_seed = 1664525u * s_seed + 1013904223u;
	return s_seed;
}
static float random_float( float lo, float hi )
{
	float t = (float)( next_random() >> 8 ) / (float)( 1 << 24 );
	return lo + t * ( hi - lo );
}
static b3AABB random_box( float extent, float size )
{
	b3Vec3 c = { random_float( -extent, extent ), random_float( -extent, extent ), random_float( -extent, extent ) };
	b3Vec3 h = { random_float( 0.1f, size ), random_float( 0.1f, size ), random_float( 0.1f, size ) };
	return (b3AABB){ b3Sub( c, h ), b3Add( c, h ) };
}

static int g_hits;
static bool count_hit( int proxyId, uint64_t userData, void* context )
{
	(void)proxyId; (void)userData; (void)context;
	g_hits += 1;
	return true;
}
static float ray_hit( const b3RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)proxyId; (void)userData; (void)context;
	g_hits += 1;
	return input->maxFraction;
}

#define N 10000
#define QUERIES 100000
#define RAYS 10000

int main( void )
{
	b3DynamicTree tree = b3DynamicTree_Create( 16 );
	b3AABB* boxes = malloc( N * sizeof( b3AABB ) );
	int* ids = malloc( N * sizeof( int ) );

	double t0 = now_ms();
	for ( int i = 0; i < N; ++i )
	{
		boxes[i] = random_box( 100.0f, 1.0f );
		ids[i] = b3DynamicTree_CreateProxy( &tree, boxes[i], B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
	}
	double t1 = now_ms();
	for ( int k = 0; k < 100; ++k ) b3DynamicTree_Rebuild( &tree, true );
	double t2 = now_ms();

	g_hits = 0;
	for ( int q = 0; q < QUERIES; ++q )
	{
		b3AABB box = random_box( 100.0f, 2.0f );
		b3DynamicTree_Query( &tree, box, B3_DEFAULT_MASK_BITS, false, count_hit, NULL );
	}
	double t3 = now_ms();
	int queryHits = g_hits;

	g_hits = 0;
	for ( int r = 0; r < RAYS; ++r )
	{
		b3RayCastInput input = {
			.origin = { random_float( -100.0f, 100.0f ), random_float( -100.0f, 100.0f ), random_float( -100.0f, 100.0f ) },
			.translation = { random_float( -200.0f, 200.0f ), random_float( -200.0f, 200.0f ), random_float( -200.0f, 200.0f ) },
			.maxFraction = 1.0f,
		};
		b3DynamicTree_RayCast( &tree, &input, B3_DEFAULT_MASK_BITS, false, ray_hit, NULL );
	}
	double t4 = now_ms();
	int rayHits = g_hits;

	double moveMs = 0.0, rebuildMs = 0.0;
	for ( int round = 0; round < 10; ++round )
	{
		double a = now_ms();
		for ( int i = 0; i < N; ++i )
		{
			b3Vec3 d = { random_float( -1.0f, 1.0f ), random_float( -1.0f, 1.0f ), random_float( -1.0f, 1.0f ) };
			boxes[i].lowerBound = b3Add( boxes[i].lowerBound, d );
			boxes[i].upperBound = b3Add( boxes[i].upperBound, d );
			b3DynamicTree_EnlargeProxy( &tree, ids[i], b3AABB_Union( b3DynamicTree_GetAABB( &tree, ids[i] ), boxes[i] ) );
		}
		double b = now_ms();
		b3DynamicTree_Rebuild( &tree, false );
		double c = now_ms();
		moveMs += b - a;
		rebuildMs += c - b;
	}

	printf( "box3d tree: insert %d %.2f ms, 100x full rebuild %.2f ms, %d queries %.2f ms (%d hits), %d rays %.2f ms (%d hits), "
			"10x enlarge-all %.2f ms, 10x partial rebuild %.2f ms, height %d, area ratio %.2f\n",
			N, t1 - t0, t2 - t1, QUERIES, t3 - t2, queryHits, RAYS, t4 - t3, rayHits, moveMs, rebuildMs,
			b3DynamicTree_GetHeight( &tree ), b3DynamicTree_GetAreaRatio( &tree ) );
	b3DynamicTree_Destroy( &tree );
	free( boxes );
	free( ids );
	return 0;
}
