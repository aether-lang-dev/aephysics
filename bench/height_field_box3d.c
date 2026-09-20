// The height field of the reference on the same scenes as
// bench/height_field.ae: a 512 x 512 wave field (522,242 triangles, a
// hole every sixteenth cell) built ten times; 100,000 ray casts down onto
// it; 100,000 box queries over it; 10,000 sphere shape casts onto it;
// 10,000 sphere overlaps at its surface. Single thread, wall time per
// phase, with the hit counts and sums as the checksum.
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

static double now_ms( void )
{
	struct timespec ts;
	timespec_get( &ts, TIME_UTC );
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static int g_queryHits;
static bool count_triangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	(void)a; (void)b; (void)c; (void)triangleIndex; (void)context;
	g_queryHits += 1;
	return true;
}

#define RAYS 100000
#define QUERIES 100000
#define CASTS 10000
#define OVERLAPS 10000

int main( void )
{
	b3Vec3 scale = { 0.5f, 2.0f, 0.5f };
	double t0 = now_ms();
	b3HeightFieldData* data = NULL;
	for ( int i = 0; i < 10; ++i )
	{
		if ( data ) b3DestroyHeightField( data );
		data = b3CreateWave( 512, 512, scale, 0.02f, 0.031f, true );
	}
	double t1 = now_ms();

	int rayHits = 0;
	double raySum = 0.0;
	for ( int i = 0; i < RAYS; ++i )
	{
		float t = (float)i / (float)RAYS;
		b3RayCastInput input = { { 1.0f + 254.0f * t, 5.0f, 128.0f + 120.0f * sinf( 40.0f * t ) }, { 3.0f * cosf( 7.0f * t ), -10.0f, 3.0f * sinf( 9.0f * t ) }, 1.0f };
		b3CastOutput out = b3RayCastHeightField( data, &input );
		if ( out.hit )
		{
			rayHits += 1;
			raySum += out.point.y;
		}
	}
	double t2 = now_ms();

	g_queryHits = 0;
	for ( int i = 0; i < QUERIES; ++i )
	{
		float t = (float)i / (float)QUERIES;
		b3Vec3 c = { 2.0f + 252.0f * t, 0.3f * sinf( 3.0f * t ), 128.0f + 120.0f * cosf( 30.0f * t ) };
		b3Vec3 h = { 0.6f, 2.5f, 0.6f };
		b3AABB bounds = { b3Sub( c, h ), b3Add( c, h ) };
		b3QueryHeightField( data, bounds, count_triangle, NULL );
	}
	double t3 = now_ms();

	int castHits = 0;
	double castSum = 0.0;
	for ( int i = 0; i < CASTS; ++i )
	{
		float t = (float)i / (float)CASTS;
		b3Vec3 start = { 2.0f + 252.0f * t, 5.0f, 128.0f + 120.0f * sinf( 50.0f * t ) };
		b3ShapeCastInput input = { { &start, 1, 0.3f }, { 4.0f * cosf( 11.0f * t ), -10.0f, 4.0f * sinf( 13.0f * t ) }, 1.0f, false };
		b3CastOutput out = b3ShapeCastHeightField( data, &input );
		if ( out.hit )
		{
			castHits += 1;
			castSum += out.fraction;
		}
	}
	double t4 = now_ms();

	int overlapHits = 0;
	for ( int i = 0; i < OVERLAPS; ++i )
	{
		float t = (float)i / (float)OVERLAPS;
		b3Vec3 center = { 2.0f + 252.0f * t, 1.0f * sinf( 17.0f * t ), 128.0f + 120.0f * cosf( 23.0f * t ) };
		b3ShapeProxy proxy = { &center, 1, 0.5f };
		if ( b3OverlapHeightField( data, b3Transform_identity, &proxy ) ) overlapHits += 1;
	}
	double t5 = now_ms();

	printf( "box3d height_field: 10 builds %.1f ms (%d triangles, %d bytes), %d rays %.2f ms (%d hits, sum %.3f), %d queries %.2f ms (%d hits), "
			"%d casts %.2f ms (%d hits, sum %.3f), %d overlaps %.2f ms (%d hits)\n",
			t1 - t0, 2 * ( data->rowCount - 1 ) * ( data->columnCount - 1 ), data->byteCount, RAYS, t2 - t1, rayHits, raySum, QUERIES, t3 - t2, g_queryHits,
			CASTS, t4 - t3, castHits, castSum, OVERLAPS, t5 - t4, overlapHits );
	b3DestroyHeightField( data );
	return 0;
}
