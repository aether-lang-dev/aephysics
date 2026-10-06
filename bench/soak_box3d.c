// bench/soak.ae's scene, Box3D side: 2,048 mixed bodies thrown into a
// walled pen, the same dice in the same order, 600 steps, single thread.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static double now_ms( void )
{
	struct timespec ts;
	timespec_get( &ts, TIME_UTC );
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

#define RAND_LIMIT 32767
static unsigned long long g_seed = 1;

static int random_int( void )
{
	unsigned long long x = g_seed;
	x = ( x ^ ( x << 13 ) ) & 4294967295ull;
	x = x ^ ( x >> 17 );
	x = ( x ^ ( x << 5 ) ) & 4294967295ull;
	g_seed = x;
	return (int)( x % ( RAND_LIMIT + 1 ) );
}

static float rf( float lo, float hi )
{
	float r = (float)( random_int() & RAND_LIMIT );
	return ( hi - lo ) * ( r / (float)RAND_LIMIT ) + lo;
}

static b3Vec3 rv( float lo, float hi )
{
	float x = rf( lo, hi );
	float y = rf( lo, hi );
	float z = rf( lo, hi );
	return (b3Vec3){ x, y, z };
}

static b3Quat rq( void )
{
	b3Vec3 v = rv( -1.0f, 1.0f );
	float w = rf( -1.0f, 1.0f );
	b3Quat q = { v, w };
	return b3NormalizeQuat( q );
}

#define SIDE 16
#define LAYERS 8
#define BODY_COUNT 2048
#define PEN 16.0f
#define GRAVITY 10.0f
#define STEPS 600

static b3BodyId bodies[BODY_COUNT];
static b3HullData* kept[BODY_COUNT];
static int keptCount = 0;

static void addBody( b3WorldId worldId, b3Vec3 p, int index )
{
	b3BodyDef bdef = b3DefaultBodyDef();
	bdef.type = b3_dynamicBody;
	bdef.position = (b3Pos){ p.x, p.y, p.z };
	bdef.rotation = rq();
	bdef.linearVelocity = rv( -2.0f, 2.0f );
	bdef.angularVelocity = rv( -3.0f, 3.0f );
	b3BodyId body = b3CreateBody( worldId, &bdef );
	b3ShapeDef sdef = b3DefaultShapeDef();
	sdef.baseMaterial.restitution = rf( 0.0f, 0.6f );
	sdef.baseMaterial.rollingResistance = 0.05f;
	int kind = random_int() % 4;
	if ( kind == 0 )
	{
		float hx = rf( 0.1f, 0.5f );
		float hy = rf( 0.1f, 0.5f );
		float hz = rf( 0.1f, 0.5f );
		b3BoxHull box = b3MakeBoxHull( hx, hy, hz );
		b3CreateHullShape( body, &sdef, &box.base );
	}
	else if ( kind == 1 )
	{
		float radius = rf( 0.1f, 0.4f );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, radius };
		b3CreateSphereShape( body, &sdef, &sphere );
	}
	else if ( kind == 2 )
	{
		float half = rf( 0.1f, 0.4f );
		float radius = rf( 0.1f, 0.3f );
		b3Capsule capsule = { { 0.0f, -half, 0.0f }, { 0.0f, half, 0.0f }, radius };
		b3CreateCapsuleShape( body, &sdef, &capsule );
	}
	else
	{
		b3Vec3 points[12];
		for ( int k = 0; k < 12; ++k )
			points[k] = rv( -0.4f, 0.4f );
		b3HullData* h = b3CreateHull( points, 12, 12 );
		if ( h == NULL )
		{
			b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.3f };
			b3CreateSphereShape( body, &sdef, &sphere );
		}
		else
		{
			b3CreateHullShape( body, &sdef, h );
			kept[keptCount++] = h;
		}
	}
	bodies[index] = body;
}

static void run( const char* name, bool sleep )
{
	g_seed = 1;
	b3WorldDef wdef = b3DefaultWorldDef();
	wdef.gravity = (b3Vec3){ 0.0f, -GRAVITY, 0.0f };
	wdef.enableSleep = sleep;
	b3WorldId worldId = b3CreateWorld( &wdef );
	b3BodyDef gdef = b3DefaultBodyDef();
	gdef.position = (b3Pos){ 0.0f, -0.5f, 0.0f };
	b3BodyId ground = b3CreateBody( worldId, &gdef );
	b3ShapeDef gs = b3DefaultShapeDef();
	b3BoxHull floor = b3MakeBoxHull( PEN, 0.5f, PEN );
	b3CreateHullShape( ground, &gs, &floor.base );
	b3BoxHull w1 = b3MakeOffsetBoxHull( 0.5f, 3.0f, PEN, (b3Vec3){ PEN + 0.5f, 3.0f, 0.0f } );
	b3CreateHullShape( ground, &gs, &w1.base );
	b3BoxHull w2 = b3MakeOffsetBoxHull( 0.5f, 3.0f, PEN, (b3Vec3){ -PEN - 0.5f, 3.0f, 0.0f } );
	b3CreateHullShape( ground, &gs, &w2.base );
	b3BoxHull w3 = b3MakeOffsetBoxHull( PEN, 3.0f, 0.5f, (b3Vec3){ 0.0f, 3.0f, PEN + 0.5f } );
	b3CreateHullShape( ground, &gs, &w3.base );
	b3BoxHull w4 = b3MakeOffsetBoxHull( PEN, 3.0f, 0.5f, (b3Vec3){ 0.0f, 3.0f, -PEN - 0.5f } );
	b3CreateHullShape( ground, &gs, &w4.base );
	int n = 0;
	float half = 0.5f * ( SIDE - 1 );
	for ( int layer = 0; layer < LAYERS; ++layer )
		for ( int row = 0; row < SIDE; ++row )
			for ( int column = 0; column < SIDE; ++column )
			{
				b3Vec3 p = { 1.8f * ( column - half ), 1.0f + 1.8f * layer, 1.8f * ( row - half ) };
				addBody( worldId, p, n++ );
			}
	b3Profile sum_profile = { 0 };
	double t0 = now_ms();
	for ( int s = 0; s < STEPS; ++s )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		b3Profile p = b3World_GetProfile( worldId );
		sum_profile.pairs += p.pairs;
		sum_profile.collide += p.collide;
		sum_profile.solve += p.solve;
		sum_profile.constraints += p.constraints;
		sum_profile.transforms += p.transforms;
		sum_profile.sleepIslands += p.sleepIslands;
	}
	double t1 = now_ms();
	double sum = 0.0;
	for ( int i = 0; i < BODY_COUNT; ++i )
		sum += b3Body_GetPosition( bodies[i] ).y;
	b3Counters counters = b3World_GetCounters( worldId );
	printf( "box3d soak %s: %d steps of %d mixed bodies %g ms, %g ms per step (height sum %g, %d contacts, %d awake)\n", name, STEPS, BODY_COUNT,
			t1 - t0, ( t1 - t0 ) / STEPS, sum, counters.contactCount, b3World_GetAwakeBodyCount( worldId ) );
	printf( "box3d phases ms/step: pairs %g collide %g solve %g (constraints %g transforms %g sleep %g)\n", sum_profile.pairs / STEPS,
			sum_profile.collide / STEPS, sum_profile.solve / STEPS, sum_profile.constraints / STEPS, sum_profile.transforms / STEPS,
			sum_profile.sleepIslands / STEPS );
	b3DestroyWorld( worldId );
	for ( int i = 0; i < keptCount; ++i )
		b3DestroyHull( kept[i] );
	keptCount = 0;
}

int main( void )
{
	run( "sleeping", true );
	run( "awake", false );
	return 0;
}
