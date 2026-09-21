// The reference's benchmark scenes by worker count, as bench/parallel.ae
// steps them with ours: the large pyramid, many pyramids and the joint
// grid (bench/physics_world_box3d.c has each), sleeping off, at 1, 2, 4
// and 8 workers and the machine's processor count, on the reference's
// built-in scheduler (b3WorldDef.workerCount above one with no task
// callbacks). The same line per run as ours, the height sum as the
// checksum.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

static double now_ms( void )
{
	struct timespec ts;
	timespec_get( &ts, TIME_UTC );
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static int processor_count( void )
{
#ifdef _WIN32
	SYSTEM_INFO info;
	GetSystemInfo( &info );
	return info.dwNumberOfProcessors > 0 ? (int)info.dwNumberOfProcessors : 1;
#else
	long n = sysconf( _SC_NPROCESSORS_ONLN );
	return n > 0 ? (int)n : 1;
#endif
}

static b3BoxHull g_box;
static int g_idCount;
static b3BodyId* g_ids;

static void ground( b3WorldId worldId, float extent )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( extent, 1.0f, extent );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &shapeDef, &box.base );
}

static void pyramid( b3WorldId worldId, int baseCount, float extent, float centerX, float baseZ )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
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

static double heightSum( void )
{
	double sum = 0.0;
	for ( int i = 0; i < g_idCount; ++i )
	{
		if ( b3Body_GetType( g_ids[i] ) == b3_dynamicBody )
		{
			sum += b3Body_GetPosition( g_ids[i] ).y;
		}
	}
	return sum;
}

static double run( const char* name, b3WorldId worldId, int steps, int workers )
{
	double t0 = now_ms();
	for ( int s = 0; s < steps; ++s )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	double t1 = now_ms();
	double sum = heightSum();
	printf( "box3d parallel: %s, %d workers: %.4f ms per step (height sum %.3f)\n", name, workers, ( t1 - t0 ) / steps, sum );
	b3DestroyWorld( worldId );
	g_idCount = 0;
	return sum;
}

static b3WorldId world( int workers )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workers;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_EnableSleeping( worldId, false );
	return worldId;
}

static double largePyramid( int workers )
{
	b3WorldId worldId = world( workers );
	ground( worldId, 400.0f );
	int baseCount = 100;
	pyramid( worldId, baseCount, 0.5f, -0.5f * baseCount, 0.0f );
	// The ids: the ground is body 1, the cubes 2.. in order of creation.
	for ( int i = 0; i < baseCount * ( baseCount + 1 ) / 2; ++i )
	{
		g_ids[g_idCount++] = (b3BodyId){ 2 + i, worldId.index1 - 1, 1 };
	}
	return run( "large pyramid", worldId, 200, workers );
}

static double manyPyramids( int workers )
{
	b3WorldId worldId = world( workers );
	int baseCount = 10;
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
			pyramid( worldId, baseCount, extent, centerX - 0.5f, baseZ );
		}
		baseZ += deltaZ;
	}
	for ( int i = 0; i < rows * columns * baseCount * ( baseCount + 1 ) / 2; ++i )
	{
		g_ids[g_idCount++] = (b3BodyId){ 2 + i, worldId.index1 - 1, 1 };
	}
	return run( "many pyramids", worldId, 100, workers );
}

static double jointGrid( int workers )
{
	b3WorldId worldId = world( workers );
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
	double sum = run( "joint grid", worldId, 100, workers );
	free( bodies );
	return sum;
}

int main( void )
{
	g_box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	g_ids = calloc( 20000, sizeof( b3BodyId ) );
	int counts[5] = { 1, 2, 4, 8, processor_count() };
	int n = counts[4] <= 8 ? 4 : 5;
	double first = 0.0;
	for ( int i = 0; i < n; ++i )
	{
		double sum = largePyramid( counts[i] );
		if ( i == 0 ) first = sum; else if ( sum != first ) printf( "box3d parallel: NOTE the large pyramid's height sum changed with the worker count\n" );
	}
	for ( int i = 0; i < n; ++i )
	{
		double sum = manyPyramids( counts[i] );
		if ( i == 0 ) first = sum; else if ( sum != first ) printf( "box3d parallel: NOTE the many pyramids' height sum changed with the worker count\n" );
	}
	for ( int i = 0; i < n; ++i )
	{
		double sum = jointGrid( counts[i] );
		if ( i == 0 ) first = sum; else if ( sum != first ) printf( "box3d parallel: NOTE the joint grid's height sum changed with the worker count\n" );
	}
	free( g_ids );
	return 0;
}
