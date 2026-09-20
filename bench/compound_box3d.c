// The compound of the reference on the same scenes as bench/compound.ae:
// a compound of 2,000 children (1,000 spheres, 500 capsules and 500
// instances of one box hull on a 20 x 10 x 10 grid, two materials)
// built ten times; 100,000 rays through it; 100,000 box queries over
// it; 10,000 sphere shape casts; 10,000 overlaps; 10,000 mover planes.
// Single thread, wall time per phase, with the hit counts and sums as
// the checksum.
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "box3d/types.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Internal to the reference (compound.h in src) but linked from its library.
int b3CollideMoverAndCompound( b3PlaneResult* planes, int capacity, const b3CompoundData* shape, const b3Capsule* mover );

static double now_ms( void )
{
	struct timespec ts;
	timespec_get( &ts, TIME_UTC );
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static int g_queryHits;
static bool count_child( const b3CompoundData* compound, int childIndex, void* context )
{
	(void)compound; (void)childIndex; (void)context;
	g_queryHits += 1;
	return true;
}

#define RAYS 100000
#define QUERIES 100000
#define CASTS 10000
#define OVERLAPS 10000
#define MOVERS 10000

int main( void )
{
	b3SurfaceMaterial matA = b3DefaultSurfaceMaterial();
	b3SurfaceMaterial matB = b3DefaultSurfaceMaterial();
	matB.friction = 0.3f;
	b3BoxHull box = b3MakeBoxHull( 0.4f, 0.4f, 0.4f );

	b3CompoundSphereDef* spheres = calloc( 1000, sizeof( b3CompoundSphereDef ) );
	b3CompoundCapsuleDef* capsules = calloc( 500, sizeof( b3CompoundCapsuleDef ) );
	b3CompoundHullDef* hulls = calloc( 500, sizeof( b3CompoundHullDef ) );
	for ( int i = 0; i < 2000; ++i )
	{
		int x = i % 20, y = ( i / 20 ) % 10, z = i / 200;
		b3Vec3 c = { 2.0f * x, 2.0f * y, 2.0f * z };
		b3SurfaceMaterial mat = ( i & 1 ) ? matB : matA;
		if ( i < 1000 )
		{
			spheres[i] = (b3CompoundSphereDef){ .sphere = { c, 0.5f }, .material = mat };
		}
		else if ( i < 1500 )
		{
			capsules[i - 1000] = (b3CompoundCapsuleDef){ .capsule = { { c.x - 0.4f, c.y, c.z }, { c.x + 0.4f, c.y, c.z }, 0.3f }, .material = mat };
		}
		else
		{
			hulls[i - 1500] = (b3CompoundHullDef){ .hull = &box.base, .transform = { c, b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.3f ) }, .material = mat };
		}
	}
	b3CompoundDef def = { .capsules = capsules, .capsuleCount = 500, .hulls = hulls, .hullCount = 500, .spheres = spheres, .sphereCount = 1000 };

	double t0 = now_ms();
	b3CompoundData* compound = NULL;
	for ( int i = 0; i < 10; ++i )
	{
		if ( compound ) b3DestroyCompound( compound );
		compound = b3CreateCompound( &def );
	}
	double t1 = now_ms();

	int rayHits = 0;
	double raySum = 0.0;
	for ( int i = 0; i < RAYS; ++i )
	{
		float t = (float)i / (float)RAYS;
		b3Vec3 origin = { -3.0f, 1.0f + 17.0f * t, 9.0f + 8.0f * sinf( 40.0f * t ) };
		b3Vec3 translation = { 45.0f, 2.0f * sinf( 7.0f * t ), 2.0f * cosf( 9.0f * t ) };
		b3RayCastInput input = { origin, translation, 1.0f };
		b3CastOutput out = b3RayCastCompound( compound, &input );
		if ( out.hit )
		{
			rayHits += 1;
			raySum += out.fraction + out.childIndex;
		}
	}
	double t2 = now_ms();

	g_queryHits = 0;
	for ( int i = 0; i < QUERIES; ++i )
	{
		float t = (float)i / (float)QUERIES;
		b3Vec3 c = { 1.0f + 37.0f * t, 9.0f + 8.0f * sinf( 3.0f * t ), 9.0f + 8.0f * cosf( 30.0f * t ) };
		b3Vec3 h = { 1.5f, 1.5f, 1.5f };
		b3AABB bounds = { b3Sub( c, h ), b3Add( c, h ) };
		b3QueryCompound( compound, bounds, count_child, NULL );
	}
	double t3 = now_ms();

	int castHits = 0;
	double castSum = 0.0;
	for ( int i = 0; i < CASTS; ++i )
	{
		float t = (float)i / (float)CASTS;
		b3Vec3 start = { 1.0f + 37.0f * t, 25.0f, 9.0f + 8.0f * sinf( 50.0f * t ) };
		b3ShapeCastInput input = { { &start, 1, 0.3f }, { 0.5f * sinf( 11.0f * t ), -30.0f, 0.0f }, 1.0f, false };
		b3CastOutput out = b3ShapeCastCompound( compound, &input );
		if ( out.hit )
		{
			castHits += 1;
			castSum += out.fraction + out.childIndex;
		}
	}
	double t4 = now_ms();

	b3Transform transform = { { 10.0f, 20.0f, 30.0f }, b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI ) };
	int overlapHits = 0;
	for ( int i = 0; i < OVERLAPS; ++i )
	{
		float t = (float)i / (float)OVERLAPS;
		b3Vec3 local = { 1.0f + 37.0f * t, 9.0f + 8.0f * sinf( 17.0f * t ), 9.0f + 8.0f * cosf( 23.0f * t ) };
		b3Vec3 center = b3TransformPoint( transform, local );
		b3ShapeProxy proxy = { &center, 1, 0.4f };
		if ( b3OverlapCompound( compound, transform, &proxy ) ) overlapHits += 1;
	}
	double t5 = now_ms();

	int moverPlanes = 0;
	double moverSum = 0.0;
	for ( int i = 0; i < MOVERS; ++i )
	{
		float t = (float)i / (float)MOVERS;
		b3Vec3 a = { 1.0f + 37.0f * t, 9.5f + 8.0f * sinf( 17.0f * t ), 9.0f + 8.0f * cosf( 23.0f * t ) };
		b3Capsule mover = { a, { a.x, a.y + 1.0f, a.z }, 0.35f };
		b3PlaneResult planes[8];
		int count = b3CollideMoverAndCompound( planes, 8, compound, &mover );
		moverPlanes += count;
		for ( int k = 0; k < count; ++k ) moverSum += planes[k].plane.offset;
	}
	double t6 = now_ms();

	printf( "box3d compound: 10 builds %.1f ms (%d children, %d bytes, %d materials), %d rays %.2f ms (%d hits, sum %.3f), "
			"%d queries %.2f ms (%d hits), %d casts %.2f ms (%d hits, sum %.3f), %d overlaps %.2f ms (%d hits), %d movers %.2f ms (%d planes, sum %.3f)\n",
			t1 - t0, compound->tree.proxyCount, compound->byteCount, compound->materialCount, RAYS, t2 - t1, rayHits, raySum, QUERIES, t3 - t2,
			g_queryHits, CASTS, t4 - t3, castHits, castSum, OVERLAPS, t5 - t4, overlapHits, MOVERS, t6 - t5, moverPlanes, moverSum );
	b3DestroyCompound( compound );
	free( hulls );
	free( capsules );
	free( spheres );
	return 0;
}
