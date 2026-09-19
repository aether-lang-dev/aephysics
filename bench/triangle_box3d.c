// The triangle manifolds of the reference on the same scenes as
// bench/triangle.ae: 20,000 triangle-hull collisions of a tipped box
// rolling over a triangle, cold cache, then the same with the cache
// carried; 20,000 triangle-capsule and 20,000 triangle-sphere collisions
// along a sweep. Single thread, wall time per phase, with the point
// counts and separation sums as the checksum.
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

static b3Quat exact_quat( b3Vec3 axis, float radians )
{
	float h = 0.5f * radians, s = sinf( h );
	return (b3Quat){ { s * axis.x, s * axis.y, s * axis.z }, cosf( h ) };
}

#define N 20000

int main( void )
{
	b3Vec3 axis = b3Normalize( (b3Vec3){ 1.0f, 0.3f, 0.5f } );
	b3LocalManifoldPoint points[8];

	// A box at a hundred attitudes (built outside the timing) sinking onto a
	// big triangle, the triangle in the hull's frame.
	static b3BoxHull hulls[100];
	for ( int i = 0; i < 100; ++i )
	{
		hulls[i] = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, (b3Transform){ b3Vec3_zero, exact_quat( axis, 0.015f * i ) } );
	}
	int pointsCold = 0;
	double sepCold = 0.0;
	double t0 = now_ms();
	for ( int i = 0; i < N; ++i )
	{
		float t = (float)i / (float)N;
		float y = -0.8f + 0.1f * sinf( 20.0f * t );
		b3Vec3 v1 = { -3.0f + 0.5f * t, y, -2.0f }, v2 = { 0.0f, y, 4.0f }, v3 = { 3.0f, y + 0.02f * t, -2.0f };
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideTriangleAndHull( &manifold, 8, v1, v2, v3, 0, &hulls[i % 100].base, &cache, true );
		pointsCold += manifold.pointCount;
		for ( int k = 0; k < manifold.pointCount; ++k ) sepCold += points[k].separation;
	}
	double t1 = now_ms();

	int pointsWarm = 0, hits = 0;
	double sepWarm = 0.0;
	b3SATCache warm = { 0 };
	b3BoxHull hullW = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, (b3Transform){ b3Vec3_zero, exact_quat( axis, 0.3f ) } );
	for ( int i = 0; i < N; ++i )
	{
		float t = (float)i / (float)N;
		// Resting on the triangle, the triangle creeping: the cache's steady state.
		float y = -0.66f + 0.004f * sinf( 20.0f * t );
		b3Vec3 v1 = { -3.0f + 0.5f * t, y, -2.0f }, v2 = { 0.0f, y, 4.0f }, v3 = { 3.0f, y + 0.002f * t, -2.0f };
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3CollideTriangleAndHull( &manifold, 8, v1, v2, v3, 0, &hullW.base, &warm, true );
		pointsWarm += manifold.pointCount;
		hits += warm.hit;
		for ( int k = 0; k < manifold.pointCount; ++k ) sepWarm += points[k].separation;
	}
	double t2 = now_ms();

	int pointsCapsule = 0;
	double sepCapsule = 0.0;
	b3Vec3 tv1 = { -2.0f, 0.0f, 0.0f }, tv2 = { 2.0f, 0.0f, 0.0f }, tv3 = { 0.0f, 0.0f, -2.0f };
	for ( int i = 0; i < N; ++i )
	{
		float t = (float)i / (float)N;
		b3Vec3 dir = b3Normalize( (b3Vec3){ sinf( 2.0f * t ), 0.3f * sinf( 7.0f * t ), cosf( 2.0f * t ) } );
		b3Vec3 mid = { 0.3f * cosf( 3.0f * t ), 0.25f - 0.3f * t, -0.6f + 0.6f * t };
		b3Capsule capsule = { b3MulAdd( mid, -0.6f, dir ), b3MulAdd( mid, 0.6f, dir ), 0.1f };
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SimplexCache cache = { 0 };
		b3CollideTriangleAndCapsule( &manifold, 8, (b3Vec3[]){ tv1, tv2, tv3 }, &capsule, &cache );
		pointsCapsule += manifold.pointCount;
		for ( int k = 0; k < manifold.pointCount; ++k ) sepCapsule += points[k].separation;
	}
	double t3 = now_ms();

	int pointsSphere = 0;
	double sepSphere = 0.0;
	for ( int i = 0; i < N; ++i )
	{
		float t = (float)i / (float)N;
		b3Sphere sphere = { { 2.5f * sinf( 5.0f * t ), 0.3f - 0.25f * t, -2.5f + 3.0f * t }, 0.2f };
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3CollideTriangleAndSphere( &manifold, 8, (b3Vec3[]){ tv1, tv2, tv3 }, &sphere );
		pointsSphere += manifold.pointCount;
		for ( int k = 0; k < manifold.pointCount; ++k ) sepSphere += points[k].separation;
	}
	double t4 = now_ms();

	printf( "box3d triangle: %d triangle-hull cold %.2f ms (%d points, sum %.3f), warm %.2f ms (%d points, sum %.3f, %d cache hits), "
			"%d triangle-capsule %.2f ms (%d points, sum %.3f), %d triangle-sphere %.2f ms (%d points, sum %.3f)\n",
			N, t1 - t0, pointsCold, sepCold, t2 - t1, pointsWarm, sepWarm, hits, N, t3 - t2, pointsCapsule, sepCapsule, N, t4 - t3,
			pointsSphere, sepSphere );
	return 0;
}
