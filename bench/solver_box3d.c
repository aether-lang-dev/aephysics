// The reference's step on the same scenes as bench/solver.ae: 5,000
// cubes dropped from a metre onto a slab in a 100 by 50 grid for 120
// steps of 1/60 with four sub-steps; then 100 stacks of 10 cubes for
// 120 steps. Single thread, wall time of the steps, the sum of the
// cubes' heights and the awake count as the checksums.
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

#define STEPS 120

static void run( const char* name, int count, int columns, int rows, float spacing, float baseHeight, float layerHeight, b3BoxHull* box,
				 b3BoxHull* slab )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = (b3Pos){ 100.0f, -0.5f, 50.0f };
	b3BodyId ground = b3CreateBody( worldId, &groundDef );
	b3CreateHullShape( ground, &shapeDef, &slab->base );
	b3BodyId* ids = calloc( count, sizeof( b3BodyId ) );
	for ( int i = 0; i < count; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		int column = i % ( columns * rows );
		int layer = i / ( columns * rows );
		bodyDef.position = (b3Pos){ spacing * ( column % columns ), baseHeight + layerHeight * layer, spacing * ( column / columns ) };
		ids[i] = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( ids[i], &shapeDef, &box->base );
	}
	double t0 = now_ms();
	for ( int s = 0; s < STEPS; ++s )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	double t1 = now_ms();
	double heightSum = 0.0;
	for ( int i = 0; i < count; ++i )
	{
		heightSum += b3Body_GetPosition( ids[i] ).y;
	}
	b3Counters counters = b3World_GetCounters( worldId );
	int awake = 0;
	for ( int i = 0; i < count; ++i )
	{
		awake += b3Body_IsAwake( ids[i] ) ? 1 : 0;
	}
	printf( "box3d solver: %s: %d steps of %d cubes %.1f ms (height sum %.3f, %d awake, %d contacts)\n", name, STEPS, count, t1 - t0,
			heightSum, awake, counters.contactCount );
	free( ids );
	b3DestroyWorld( worldId );
}

int main( void )
{
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3BoxHull slab = b3MakeBoxHull( 110.0f, 0.5f, 60.0f );
	run( "falling grid", 5000, 100, 50, 2.0f, 1.5f, 0.0f, &box, &slab );
	run( "stacks of ten", 1000, 10, 10, 4.0f, 0.5f, 1.0f, &box, &slab );
	return 0;
}
