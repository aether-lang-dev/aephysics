// GJK, the shape cast and the time of impact of the reference on the same
// scenes as bench/distance.ae: two boxes over 100,000 poses along a
// sweep, cold cache then warm; 10,000 shape casts; 10,000 times of
// impact of a falling, turning box. Single thread, wall time per phase.
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

static void box_points( b3Vec3* p, float hx, float hy, float hz )
{
	for ( int i = 0; i < 8; ++i )
	{
		p[i] = (b3Vec3){ ( i & 1 ) ? -hx : hx, ( i & 2 ) ? -hy : hy, ( i & 4 ) ? -hz : hz };
	}
}

static b3Quat exact_quat( b3Vec3 axis, float radians )
{
	float h = 0.5f * radians, s = sinf( h );
	return (b3Quat){ { s * axis.x, s * axis.y, s * axis.z }, cosf( h ) };
}

#define N 100000
#define CASTS 10000
#define TOIS 10000

int main( void )
{
	b3Vec3 a[8], b[8];
	box_points( a, 1.0f, 0.5f, 0.75f );
	box_points( b, 0.6f, 0.8f, 0.4f );
	b3ShapeProxy proxyA = { a, 8, 0.0f };
	b3ShapeProxy proxyB = { b, 8, 0.0f };
	b3Vec3 axis = b3Normalize( (b3Vec3){ 1.0f, 2.0f, 0.5f } );

	double sum = 0.0;
	int iterations = 0;
	double t0 = now_ms();
	for ( int step = 0; step < N; ++step )
	{
		float t = (float)step / (float)N;
		b3DistanceInput input = {
			.proxyA = proxyA,
			.proxyB = proxyB,
			.transform = { { 3.0f - 4.0f * t, 0.5f * sinf( 4.0f * t ), 1.5f * cosf( 2.0f * t ) }, exact_quat( axis, 3.0f * t ) },
			.useRadii = false,
		};
		b3SimplexCache cache = { 0 };
		b3DistanceOutput o = b3ShapeDistance( &input, &cache, NULL, 0 );
		sum += o.distance;
		iterations += o.iterations;
	}
	double t1 = now_ms();
	double sumWarm = 0.0;
	b3SimplexCache warm = { 0 };
	for ( int step = 0; step < N; ++step )
	{
		float t = (float)step / (float)N;
		b3DistanceInput input = {
			.proxyA = proxyA,
			.proxyB = proxyB,
			.transform = { { 3.0f - 4.0f * t, 0.5f * sinf( 4.0f * t ), 1.5f * cosf( 2.0f * t ) }, exact_quat( axis, 3.0f * t ) },
			.useRadii = false,
		};
		b3DistanceOutput o = b3ShapeDistance( &input, &warm, NULL, 0 );
		sumWarm += o.distance;
	}
	double t2 = now_ms();

	double castSum = 0.0;
	for ( int i = 0; i < CASTS; ++i )
	{
		float t = (float)i / (float)CASTS;
		b3ShapeCastPairInput input = {
			.proxyA = proxyA,
			.proxyB = proxyB,
			.transform = { { 4.0f, 0.5f * sinf( 4.0f * t ), 0.0f }, exact_quat( axis, 3.0f * t ) },
			.translationB = { -8.0f, 0.0f, 0.0f },
			.maxFraction = 1.0f,
			.canEncroach = false,
		};
		b3CastOutput o = b3ShapeCast( &input );
		castSum += o.hit ? o.fraction : -1.0;
	}
	double t3 = now_ms();

	double toiSum = 0.0;
	b3Vec3 ground[8];
	box_points( ground, 2.0f, 0.25f, 2.0f );
	b3ShapeProxy proxyG = { ground, 8, 0.0f };
	for ( int i = 0; i < TOIS; ++i )
	{
		float t = (float)i / (float)TOIS;
		b3TOIInput input = {
			.proxyA = proxyG,
			.proxyB = proxyB,
			.sweepA = { b3Vec3_zero, b3Vec3_zero, b3Vec3_zero, b3Quat_identity, b3Quat_identity },
			.sweepB = { b3Vec3_zero, { 0.0f, 4.0f, 0.0f }, { 0.5f, -1.0f, 0.0f }, b3Quat_identity, exact_quat( b3Vec3_axisZ, 0.5f + t ) },
			.maxFraction = 1.0f,
		};
		b3TOIOutput o = b3TimeOfImpact( &input );
		toiSum += o.fraction;
	}
	double t4 = now_ms();

	printf( "box3d distance: %d cold queries %.2f ms (sum %.3f, %d iterations), %d warm %.2f ms (sum %.3f), %d casts %.2f ms (sum %.3f), %d tois %.2f ms (sum %.3f)\n",
			N, t1 - t0, sum, iterations, N, t2 - t1, sumWarm, CASTS, t3 - t2, castSum, TOIS, t4 - t3, toiSum );
	return 0;
}
