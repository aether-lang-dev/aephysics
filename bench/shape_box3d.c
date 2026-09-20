// The shapes of the reference on the same scenes as bench/shape.ae:
// 1,000,000 rays at a unit sphere and 1,000,000 at a capsule from a fan
// of origins (a third missing), 100,000 sphere shape casts at a capsule
// under a transform through the dispatch, 100,000 overlaps, 100,000
// mover planes against a capsule, and 100,000 capsule masses. Single
// thread, wall time per phase, with the hit counts and sums as the
// checksum.
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "box3d/types.h"

// Internal to the reference (shape.h in src) but linked from its library.
int b3CollideMoverAndCapsule( b3PlaneResult* result, const b3Capsule* shape, const b3Capsule* mover );

#include <math.h>
#include <stdio.h>
#include <time.h>

static double now_ms( void )
{
	struct timespec ts;
	timespec_get( &ts, TIME_UTC );
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

#define RAYS 1000000
#define CASTS 100000
#define OVERLAPS 100000
#define MOVERS 100000
#define MASSES 100000

int main( void )
{
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 1.0f };
	b3Capsule capsule = { { -2.0f, 0.0f, 0.0f }, { 2.0f, 0.0f, 0.0f }, 1.0f };

	double t0 = now_ms();
	int sphereHits = 0;
	double sphereSum = 0.0;
	for ( int i = 0; i < RAYS; ++i )
	{
		float t = (float)i / (float)RAYS;
		float a = 6.2831853f * t;
		b3Vec3 origin = { 5.0f * cosf( 7.0f * a ), 5.0f * sinf( 7.0f * a ), 5.0f * sinf( 3.0f * a ) };
		b3Vec3 target = { 1.5f * sinf( 11.0f * a ), 1.5f * cosf( 13.0f * a ), 0.0f };
		b3RayCastInput input = { origin, b3Sub( target, origin ), 1.0f };
		b3CastOutput out = b3RayCastSphere( &sphere, &input );
		if ( out.hit )
		{
			sphereHits += 1;
			sphereSum += out.fraction;
		}
	}
	double t1 = now_ms();

	int capsuleHits = 0;
	double capsuleSum = 0.0;
	for ( int i = 0; i < RAYS; ++i )
	{
		float t = (float)i / (float)RAYS;
		float a = 6.2831853f * t;
		b3Vec3 origin = { 6.0f * cosf( 7.0f * a ), 5.0f * sinf( 7.0f * a ), 5.0f * sinf( 3.0f * a ) };
		b3Vec3 target = { 3.0f * sinf( 11.0f * a ), 1.5f * cosf( 13.0f * a ), 0.0f };
		b3RayCastInput input = { origin, b3Sub( target, origin ), 1.0f };
		b3CastOutput out = b3RayCastCapsule( &capsule, &input );
		if ( out.hit )
		{
			capsuleHits += 1;
			capsuleSum += out.fraction;
		}
	}
	double t2 = now_ms();

	// b3ShapeCastShape, b3OverlapShape and b3CollideMover are internal to the
	// reference (shape.h in src); their dispatch is done here as they do it:
	// the input into the shape's frame, the output back out.
	b3Quat q = b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ 1.0f, 2.0f, 3.0f } ), 0.7f );
	b3Transform transform = { { 10.0f, 20.0f, 30.0f }, q };

	int castHits = 0;
	double castSum = 0.0;
	for ( int i = 0; i < CASTS; ++i )
	{
		float t = (float)i / (float)CASTS;
		float a = 6.2831853f * t;
		b3Vec3 local = { 2.5f * cosf( 5.0f * a ), 4.0f, 1.2f * sinf( 5.0f * a ) };
		b3Vec3 start = b3TransformPoint( transform, local );
		b3Vec3 translation = b3RotateVector( q, (b3Vec3){ 0.0f, -8.0f, 0.5f * sinf( 9.0f * a ) } );
		b3Vec3 localStart = b3InvTransformPoint( transform, start );
		b3ShapeCastInput input = { { &localStart, 1, 0.3f }, b3InvRotateVector( q, translation ), 1.0f, false };
		b3CastOutput out = b3ShapeCastCapsule( &capsule, &input );
		if ( out.hit )
		{
			out.point = b3TransformPoint( transform, out.point );
			out.normal = b3RotateVector( q, out.normal );
			castHits += 1;
			castSum += out.fraction;
		}
	}
	double t3 = now_ms();

	int overlapHits = 0;
	for ( int i = 0; i < OVERLAPS; ++i )
	{
		float t = (float)i / (float)OVERLAPS;
		float a = 6.2831853f * t;
		b3Vec3 local = { 3.0f * cosf( 5.0f * a ), 1.6f * sinf( 3.0f * a ), 1.2f * sinf( 5.0f * a ) };
		b3Vec3 center = b3TransformPoint( transform, local );
		b3ShapeProxy proxy = { &center, 1, 0.5f };
		if ( b3OverlapCapsule( &capsule, transform, &proxy ) ) overlapHits += 1;
	}
	double t4 = now_ms();

	int moverPlanes = 0;
	double moverSum = 0.0;
	for ( int i = 0; i < MOVERS; ++i )
	{
		float t = (float)i / (float)MOVERS;
		float a = 6.2831853f * t;
		b3Vec3 local = { 3.0f * cosf( 5.0f * a ), 1.0f + 0.6f * sinf( 3.0f * a ), 1.0f * sinf( 5.0f * a ) };
		b3Capsule mover = { b3TransformPoint( transform, local ), b3TransformPoint( transform, b3Add( local, (b3Vec3){ 0.0f, 1.0f, 0.0f } ) ), 0.3f };
		b3Capsule localMover = { b3InvTransformPoint( transform, mover.center1 ), b3InvTransformPoint( transform, mover.center2 ), mover.radius };
		b3PlaneResult planes[4];
		int count = b3CollideMoverAndCapsule( planes, &capsule, &localMover );
		for ( int k = 0; k < count; ++k )
		{
			planes[k].plane.normal = b3RotateVector( q, planes[k].plane.normal );
			planes[k].point = b3TransformPoint( transform, planes[k].point );
		}
		moverPlanes += count;
		if ( count > 0 ) moverSum += planes[0].plane.offset;
	}
	double t5 = now_ms();

	double massSum = 0.0;
	for ( int i = 0; i < MASSES; ++i )
	{
		float t = (float)i / (float)MASSES;
		b3Capsule c = { { -1.0f - t, 0.0f, 0.0f }, { 1.0f, t, 0.0f }, 0.5f + 0.5f * t };
		b3MassData md = b3ComputeCapsuleMass( &c, 1.0f );
		massSum += md.mass + md.inertia.cx.x + md.inertia.cy.y + md.inertia.cz.z;
	}
	double t6 = now_ms();

	printf( "box3d shape: %d sphere rays %.2f ms (%d hits, sum %.3f), %d capsule rays %.2f ms (%d hits, sum %.3f), "
			"%d shape casts %.2f ms (%d hits, sum %.3f), %d overlaps %.2f ms (%d hits), %d movers %.2f ms (%d planes, sum %.3f), "
			"%d masses %.2f ms (sum %.3f)\n",
			RAYS, t1 - t0, sphereHits, sphereSum, RAYS, t2 - t1, capsuleHits, capsuleSum, CASTS, t3 - t2, castHits, castSum,
			OVERLAPS, t4 - t3, overlapHits, MOVERS, t5 - t4, moverPlanes, moverSum, MASSES, t6 - t5, massSum );
	return 0;
}
