// The reference's benchmark scenes (its shared/benchmarks.c, copied
// here so the pair builds alone) stepped by b3World_Step, as
// bench/physics_world.ae steps them with ours: the large pyramid (a
// base of 100 cubes, 200 steps), many pyramids (14 by 14 pyramids of
// base 10, 100 steps), the joint grid (100 by 100 spheres hung by
// 19,800 spherical joints, 100 steps), sleeping off. Single thread,
// wall time of the steps and the average step, the sum of the bodies'
// heights as the checksum.
#include "box3d/box3d.h"
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

static b3BoxHull g_box;

static void ground( b3WorldId worldId, float extent )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( extent, 1.0f, extent );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &shapeDef, &box.base );
}

static void pyramid( b3WorldId worldId, int baseCount, float extent, float centerX, float baseZ, bool allowSleep )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = allowSleep;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 100.0f;
	for ( int i = 0; i < baseCount; ++i )
	{
		float y = ( 2.0f * i + 1.0f ) * extent;
		for ( int j = i; j < baseCount; ++j )
		{
			float x = ( i + 1.0f ) * extent + 2.0f * ( j - i ) * extent + centerX;
			bodyDef.position = (b3Pos){ x, y, baseZ };
			b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &g_box.base );
		}
	}
}

static double heightSum( b3WorldId worldId, b3BodyId* ids, int count )
{
	double sum = 0.0;
	for ( int i = 0; i < count; ++i )
	{
		if ( b3Body_GetType( ids[i] ) == b3_dynamicBody )
		{
			sum += b3Body_GetPosition( ids[i] ).y;
		}
	}
	return sum;
}

static int g_idCount;
static b3BodyId* g_ids;

static void run( const char* name, b3WorldId worldId, int steps )
{
	// Gather the body ids through the move events after one zero step would miss sleeping ones; walk our own list instead.
	double t0 = now_ms();
	for ( int s = 0; s < steps; ++s )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	double t1 = now_ms();
	b3Counters counters = b3World_GetCounters( worldId );
	printf( "box3d physics_world: %s: %d steps of %d bodies %.1f ms, %.3f ms per step (height sum %.3f, %d contacts, %d joints)\n", name,
			steps, counters.bodyCount, t1 - t0, ( t1 - t0 ) / steps, heightSum( worldId, g_ids, g_idCount ), counters.contactCount,
			counters.jointCount );
	b3DestroyWorld( worldId );
	g_idCount = 0;
}

int main( void )
{
	g_box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	g_ids = calloc( 20000, sizeof( b3BodyId ) );

	// The large pyramid.
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_EnableSleeping( worldId, false );
	ground( worldId, 400.0f );
	int baseCount = 100;
	pyramid( worldId, baseCount, 0.5f, -0.5f * baseCount, 0.0f, true );
	// The ids: the ground is body 1, the cubes 2.. in order of creation.
	for ( int i = 0; i < baseCount * ( baseCount + 1 ) / 2; ++i )
	{
		g_ids[g_idCount++] = (b3BodyId){ 2 + i, worldId.index1 - 1, 1 };
	}
	run( "large pyramid", worldId, 200 );

	// Many pyramids.
	worldId = b3CreateWorld( &worldDef );
	baseCount = 10;
	float extent = 0.5f;
	int rows = 14, columns = 14;
	float groundExtent = extent * columns * ( baseCount + 1.0f );
	ground( worldId, groundExtent );
	float baseWidth = 2.0f * extent * baseCount;
	float baseZ = -groundExtent + 2.0f * extent;
	float deltaZ = 2.0f * ( groundExtent - 2.0f * extent ) / ( rows - 1.0f );
	for ( int i = 0; i < rows; ++i )
	{
		for ( int j = 0; j < columns; ++j )
		{
			float centerX = -groundExtent + j * ( baseWidth + 2.0f * extent ) + 2.0f * extent;
			pyramid( worldId, baseCount, extent, centerX - 0.5f, baseZ, false );
		}
		baseZ += deltaZ;
	}
	for ( int i = 0; i < rows * columns * baseCount * ( baseCount + 1 ) / 2; ++i )
	{
		g_ids[g_idCount++] = (b3BodyId){ 2 + i, worldId.index1 - 1, 1 };
	}
	run( "many pyramids", worldId, 100 );

	// The joint grid.
	worldId = b3CreateWorld( &worldDef );
	b3World_EnableSleeping( worldId, false );
	int n = 100;
	b3BodyId* bodies = malloc( n * n * sizeof( b3BodyId ) );
	int index = 0;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = ~2u;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.4f };
	b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.enableSleep = false;
	for ( int k = 0; k < n; ++k )
	{
		for ( int i = 0; i < n; ++i )
		{
			bodyDef.type = i == 0 ? b3_staticBody : b3_dynamicBody;
			bodyDef.position = (b3Pos){ (float)k, -(float)i, 0.0f };
			b3BodyId body = b3CreateBody( worldId, &bodyDef );
			b3CreateSphereShape( body, &shapeDef, &sphere );
			if ( i > 0 )
			{
				jointDef.base.bodyIdA = bodies[index - 1];
				jointDef.base.bodyIdB = body;
				jointDef.base.localFrameA.p = (b3Vec3){ 0.0f, -0.5f, 0.0f };
				jointDef.base.localFrameB.p = (b3Vec3){ 0.0f, 0.5f, 0.0f };
				b3CreateSphericalJoint( worldId, &jointDef );
			}
			if ( k > 0 )
			{
				jointDef.base.bodyIdA = bodies[index - n];
				jointDef.base.bodyIdB = body;
				jointDef.base.localFrameA.p = (b3Vec3){ 0.5f, 0.0f, 0.0f };
				jointDef.base.localFrameB.p = (b3Vec3){ -0.5f, 0.0f, 0.0f };
				b3CreateSphericalJoint( worldId, &jointDef );
			}
			bodies[index++] = body;
			g_ids[g_idCount++] = body;
		}
	}
	run( "joint grid", worldId, 100 );
	free( bodies );
	free( g_ids );
	return 0;
}
