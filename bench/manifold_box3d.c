// The contact manifolds of the reference on the same scenes as
// bench/manifold.ae: 20,000 hull-hull collisions of two rotated boxes
// along a sweep through overlap, cold cache; the same 20,000 with the
// cache carried between poses; 20,000 hull-capsule and 20,000
// hull-sphere collisions along a sweep. Single thread, wall time per
// phase, with the point counts and separation sums as the checksum.
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
	b3BoxHull hullA = b3MakeBoxHull( 1.0f, 0.5f, 0.75f );
	b3BoxHull hullB = b3MakeBoxHull( 0.6f, 0.8f, 0.4f );
	b3Vec3 axis = b3Normalize( (b3Vec3){ 1.0f, 2.0f, 0.5f } );
	b3LocalManifoldPoint points[8];

	int pointsCold = 0;
	double sepCold = 0.0;
	double t0 = now_ms();
	for ( int i = 0; i < N; ++i )
	{
		float t = (float)i / (float)N;
		b3Transform xf = { { 2.2f - 2.0f * t, 0.3f * sinf( 4.0f * t ), 0.4f * cosf( 2.0f * t ) }, exact_quat( axis, 3.0f * t ) };
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, xf, &cache );
		pointsCold += manifold.pointCount;
		for ( int k = 0; k < manifold.pointCount; ++k ) sepCold += points[k].separation;
	}
	double t1 = now_ms();

	int pointsWarm = 0, hits = 0;
	double sepWarm = 0.0;
	b3SATCache warm = { 0 };
	for ( int i = 0; i < N; ++i )
	{
		float t = (float)i / (float)N;
		b3Transform xf = { { 2.2f - 2.0f * t, 0.3f * sinf( 4.0f * t ), 0.4f * cosf( 2.0f * t ) }, exact_quat( axis, 3.0f * t ) };
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, xf, &warm );
		pointsWarm += manifold.pointCount;
		hits += warm.hit;
		for ( int k = 0; k < manifold.pointCount; ++k ) sepWarm += points[k].separation;
	}
	double t2 = now_ms();

	int pointsCapsule = 0;
	double sepCapsule = 0.0;
	for ( int i = 0; i < N; ++i )
	{
		float t = (float)i / (float)N;
		b3Capsule capsule = { { -0.4f, 0.0f, 0.0f }, { 0.4f, 0.0f, 0.0f }, 0.15f };
		b3Transform xf = { { 0.3f * sinf( 3.0f * t ), 1.4f - 1.6f * t, 0.2f }, exact_quat( axis, 2.0f * t ) };
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SimplexCache cache = { 0 };
		b3CollideHullAndCapsule( &manifold, 8, &hullA.base, &capsule, xf, &cache );
		pointsCapsule += manifold.pointCount;
		for ( int k = 0; k < manifold.pointCount; ++k ) sepCapsule += points[k].separation;
	}
	double t3 = now_ms();

	int pointsSphere = 0;
	double sepSphere = 0.0;
	for ( int i = 0; i < N; ++i )
	{
		float t = (float)i / (float)N;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.2f };
		b3Transform xf = { { 0.5f * sinf( 5.0f * t ), 1.4f - 1.6f * t, 0.3f * cosf( 3.0f * t ) }, b3Quat_identity };
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SimplexCache cache = { 0 };
		b3CollideHullAndSphere( &manifold, 8, &hullA.base, &sphere, xf, &cache );
		pointsSphere += manifold.pointCount;
		for ( int k = 0; k < manifold.pointCount; ++k ) sepSphere += points[k].separation;
	}
	double t4 = now_ms();

	printf( "box3d manifold: %d hull-hull cold %.2f ms (%d points, sum %.3f), warm %.2f ms (%d points, sum %.3f, %d cache hits), "
			"%d hull-capsule %.2f ms (%d points, sum %.3f), %d hull-sphere %.2f ms (%d points, sum %.3f)\n",
			N, t1 - t0, pointsCold, sepCold, t2 - t1, pointsWarm, sepWarm, hits, N, t3 - t2, pointsCapsule, sepCapsule, N, t4 - t3,
			pointsSphere, sepSphere );
	return 0;
}
