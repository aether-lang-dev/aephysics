// The world's bookkeeping of the reference on the same scene as
// bench/dynamics.ae: ten rounds of a world with 5,000 dynamic bodies
// (a box hull and an offset sphere each) and 500 static ones, the mass
// summed, every body's transform set once, then the world destroyed.
// No step. Single thread, wall time per phase, with the mass sum as
// the checksum.
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

#define ROUNDS 10
#define DYNAMIC 5000
#define STATIC 500

int main( void )
{
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3BodyId* ids = calloc( DYNAMIC + STATIC, sizeof( b3BodyId ) );
	double createMs = 0.0, moveMs = 0.0, destroyMs = 0.0, massSum = 0.0;
	for ( int round = 0; round < ROUNDS; ++round )
	{
		double t0 = now_ms();
		b3WorldDef worldDef = b3DefaultWorldDef();
		b3WorldId worldId = b3CreateWorld( &worldDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 1.0f;
		for ( int i = 0; i < DYNAMIC; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = (b3Pos){ 2.0f * ( i % 100 ), 2.0f + 2.0f * ( ( i / 100 ) % 50 ), 2.0f * ( i / 5000 ) };
			ids[i] = b3CreateBody( worldId, &bodyDef );
			b3CreateHullShape( ids[i], &shapeDef, &box.base );
			b3Sphere sphere = { { 0.6f, 0.0f, 0.0f }, 0.25f };
			b3CreateSphereShape( ids[i], &shapeDef, &sphere );
			massSum += b3Body_GetMass( ids[i] );
		}
		for ( int i = 0; i < STATIC; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = (b3Pos){ 2.0f * ( i % 100 ), 0.0f, 20.0f + 2.0f * ( i / 100 ) };
			ids[DYNAMIC + i] = b3CreateBody( worldId, &bodyDef );
			b3CreateHullShape( ids[DYNAMIC + i], &shapeDef, &box.base );
		}
		double t1 = now_ms();
		for ( int i = 0; i < DYNAMIC; ++i )
		{
			b3Pos p = b3Body_GetPosition( ids[i] );
			p.y += 0.1f;
			b3Body_SetTransform( ids[i], p, b3Body_GetRotation( ids[i] ) );
		}
		double t2 = now_ms();
		b3DestroyWorld( worldId );
		double t3 = now_ms();
		createMs += t1 - t0;
		moveMs += t2 - t1;
		destroyMs += t3 - t2;
	}
	printf( "box3d dynamics: %d rounds: create %d bodies and %d shapes %.1f ms, move %d bodies %.1f ms, destroy %.1f ms (mass sum %.3f)\n",
			ROUNDS, DYNAMIC + STATIC, 2 * DYNAMIC + STATIC, createMs, DYNAMIC, moveMs, destroyMs, massSum );
	free( ids );
	return 0;
}
