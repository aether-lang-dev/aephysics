// The plane solver of the reference on the same scene as bench/mover.ae:
// 1,000,000 solves of a target against six planes tilted around it (a
// floor, four walls leaning in, a soft ceiling), each followed by a
// velocity clip. Single thread, wall time, with the sums of the deltas
// and iteration counts as the checksum.
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

static double now_ms( void )
{
	struct timespec ts;
	timespec_get( &ts, TIME_UTC );
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

#define SOLVES 1000000

int main( void )
{
	b3CollisionPlane planes[6] = { 0 };
	planes[0].plane = ( b3Plane ){ { 0.0f, 1.0f, 0.0f }, 0.0f };
	planes[1].plane = ( b3Plane ){ b3Normalize( ( b3Vec3 ){ 1.0f, 0.3f, 0.0f } ), -1.0f };
	planes[2].plane = ( b3Plane ){ b3Normalize( ( b3Vec3 ){ -1.0f, 0.3f, 0.0f } ), -1.0f };
	planes[3].plane = ( b3Plane ){ b3Normalize( ( b3Vec3 ){ 0.0f, 0.3f, 1.0f } ), -1.0f };
	planes[4].plane = ( b3Plane ){ b3Normalize( ( b3Vec3 ){ 0.0f, 0.3f, -1.0f } ), -1.0f };
	planes[5].plane = ( b3Plane ){ { 0.0f, -1.0f, 0.0f }, -2.0f };
	for ( int i = 0; i < 5; ++i )
	{
		planes[i].pushLimit = FLT_MAX;
		planes[i].clipVelocity = true;
	}
	planes[5].pushLimit = 0.1f;
	planes[5].clipVelocity = false;

	double t0 = now_ms();
	double deltaSum = 0.0, clipSum = 0.0;
	long iterations = 0;
	for ( int i = 0; i < SOLVES; ++i )
	{
		float t = (float)i / (float)SOLVES;
		b3Vec3 target = { 1.6f * sinf( 13.0f * t ), -0.5f + 3.0f * cosf( 7.0f * t ), 1.6f * sinf( 17.0f * t ) };
		b3PlaneSolverResult result = b3SolvePlanes( target, planes, 6 );
		deltaSum += result.delta.x + result.delta.y + result.delta.z;
		iterations += result.iterationCount;
		b3Vec3 v = b3ClipVector( ( b3Vec3 ){ 2.0f * sinf( 5.0f * t ), -3.0f, 2.0f * cosf( 5.0f * t ) }, planes, 6 );
		clipSum += v.x + v.y + v.z;
	}
	double t1 = now_ms();
	printf( "box3d mover: %d solves %.2f ms (delta sum %.3f, %ld iterations, clip sum %.3f)\n", SOLVES, t1 - t0, deltaSum, iterations, clipSum );
	return 0;
}
