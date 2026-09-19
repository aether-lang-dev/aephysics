// The bake-off scenes on Box3D, single thread. The same scenes as
// bake_jolt.cpp, step for step: a 1/60 s step with 4 sub-steps (the
// reference's own benchmark setting), no sleeping, and every scene reports
// its wall time over the steps and one number for how it held together.
//
//   bake_box3d pile|pyramid|chain [steps]
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static double now_ms( void )
{
	struct timespec ts;
	timespec_get( &ts, TIME_UTC );
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static b3BodyId ground( b3WorldId world )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
	b3BodyId id = b3CreateBody( world, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 400.0f, 1.0f, 400.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( id, &shapeDef, &box.base );
	return id;
}

// 10,000 unit boxes in a 20 x 25 x 20 block, dropped from 2 m onto the ground.
static int g_pile_count;
static b3BodyId g_pile[10000];
static void create_pile( b3WorldId world )
{
	ground( world );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	g_pile_count = 0;
	for ( int y = 0; y < 25; ++y )
		for ( int x = 0; x < 20; ++x )
			for ( int z = 0; z < 20; ++z )
			{
				bodyDef.position = (b3Pos){ 1.1f * ( x - 10 ), 2.0f + 1.1f * y, 1.1f * ( z - 10 ) };
				b3BodyId id = b3CreateBody( world, &bodyDef );
				b3CreateHullShape( id, &shapeDef, &box.base );
				g_pile[g_pile_count++] = id;
			}
}
// How it held: the lowest body's height (should stay at or above the ground).
static double measure_pile( void )
{
	double lowest = 1e9;
	for ( int i = 0; i < g_pile_count; ++i )
	{
		b3Pos p = b3Body_GetPosition( g_pile[i] );
		if ( p.y < lowest ) lowest = p.y;
	}
	return lowest;
}

// A pyramid of 100 rows (5,050 boxes), the reference's large_pyramid.
static int g_pyr_count;
static b3BodyId g_pyr[5050];
static b3Pos g_pyr_start[5050];
static void create_pyramid( b3WorldId world )
{
	ground( world );
	int baseCount = 100;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 100.0f;
	float h = 0.5f;
	b3BoxHull box = b3MakeBoxHull( h, h, h );
	float shift = 1.0f * h;
	g_pyr_count = 0;
	for ( int i = 0; i < baseCount; ++i )
	{
		float y = ( 2.0f * i + 1.0f ) * shift;
		for ( int j = i; j < baseCount; ++j )
		{
			float x = ( i + 1.0f ) * shift + 2.0f * ( j - i ) * shift - h * baseCount;
			bodyDef.position = (b3Pos){ x, y, 0.0f };
			b3BodyId id = b3CreateBody( world, &bodyDef );
			b3CreateHullShape( id, &shapeDef, &box.base );
			g_pyr_start[g_pyr_count] = bodyDef.position;
			g_pyr[g_pyr_count++] = id;
		}
	}
}
// How it held: the largest drift of any box from where it was placed.
static double measure_pyramid( void )
{
	double worst = 0.0;
	for ( int i = 0; i < g_pyr_count; ++i )
	{
		b3Pos p = b3Body_GetPosition( g_pyr[i] );
		double dx = p.x - g_pyr_start[i].x, dy = p.y - g_pyr_start[i].y, dz = p.z - g_pyr_start[i].z;
		double d = dx * dx + dy * dy + dz * dz;
		if ( d > worst ) worst = d;
	}
	return worst > 0.0 ? __builtin_sqrt( worst ) : 0.0;
}

// A 100 x 100 grid of spheres joined by spherical joints, hanging from
// its top row: the reference's joint_grid.
static b3BodyId g_chain_last;
static void create_chain( b3WorldId world )
{
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
		for ( int i = 0; i < n; ++i )
		{
			bodyDef.type = i == 0 ? b3_staticBody : b3_dynamicBody;
			bodyDef.position = (b3Pos){ (float)k, -(float)i, 0.0f };
			b3BodyId body = b3CreateBody( world, &bodyDef );
			b3CreateSphereShape( body, &shapeDef, &sphere );
			if ( i > 0 )
			{
				jointDef.base.bodyIdA = bodies[index - 1];
				jointDef.base.bodyIdB = body;
				jointDef.base.localFrameA.p = (b3Vec3){ 0.0f, -0.5f, 0.0f };
				jointDef.base.localFrameB.p = (b3Vec3){ 0.0f, 0.5f, 0.0f };
				b3CreateSphericalJoint( world, &jointDef );
			}
			if ( k > 0 )
			{
				jointDef.base.bodyIdA = bodies[index - n];
				jointDef.base.bodyIdB = body;
				jointDef.base.localFrameA.p = (b3Vec3){ 0.5f, 0.0f, 0.0f };
				jointDef.base.localFrameB.p = (b3Vec3){ -0.5f, 0.0f, 0.0f };
				b3CreateSphericalJoint( world, &jointDef );
			}
			bodies[index++] = body;
		}
	g_chain_last = bodies[n * n - 1];
	free( bodies );
}
// How it held: how far the bottom corner sphere has fallen below where a
// 99-link chain can reach (each link is a metre long).
static double measure_chain( void )
{
	b3Pos p = b3Body_GetPosition( g_chain_last );
	double reach = -99.0;
	return reach - p.y;
}

int main( int argc, char** argv )
{
	const char* scene = argc > 1 ? argv[1] : "pile";
	int steps = argc > 2 ? atoi( argv[2] ) : 300;

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId world = b3CreateWorld( &worldDef );
	b3World_EnableSleeping( world, false );

	double ( *measure )( void ) = NULL;
	if ( strcmp( scene, "pile" ) == 0 ) { create_pile( world ); measure = measure_pile; }
	else if ( strcmp( scene, "pyramid" ) == 0 ) { create_pyramid( world ); measure = measure_pyramid; }
	else if ( strcmp( scene, "chain" ) == 0 ) { create_chain( world ); measure = measure_chain; }
	else { fprintf( stderr, "scene?\n" ); return 2; }

	float dt = 1.0f / 60.0f;
	int substeps = 4;
	// The first step builds structures and is not the steady cost.
	b3World_Step( world, dt, substeps );
	double start = now_ms();
	for ( int i = 1; i < steps; ++i ) b3World_Step( world, dt, substeps );
	double ms = now_ms() - start;
	printf( "box3d %s: %d steps in %.1f ms, %.3f ms/step, measure %.4f\n", scene, steps, ms, ms / ( steps - 1 ), measure() );
	b3DestroyWorld( world );
	return 0;
}
