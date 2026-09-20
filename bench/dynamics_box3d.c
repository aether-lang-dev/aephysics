// The world's bookkeeping of the reference on the same scenes as
// bench/dynamics.ae: ten rounds of a world with 5,000 dynamic bodies
// (a box hull and an offset sphere each) and 500 static ones, the mass
// summed, every body's transform set once, then the world destroyed;
// then 5,000 cubes resting on a slab through eleven zero-length steps
// (the reference's step with no time collides and does not solve): the
// first finds the pairs and begins the contacts, the rest recycle them;
// then those cubes chained by 4,999 revolute joints (created, then
// destroyed); then 2,500 sensor spheres over 2,500 static boxes through
// eleven zero-length steps (which run the sensor pass). Single thread,
// wall time per phase, with the mass sum, the contact count, the island
// count and the begin event count as the checksums.
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
#define SENSORS 2500

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

	// Contacts: cubes resting on a slab, collided without solving.
	{
		b3WorldDef worldDef = b3DefaultWorldDef();
		b3WorldId worldId = b3CreateWorld( &worldDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.position = (b3Pos){ 100.0f, -0.5f, 50.0f };
		b3BodyId ground = b3CreateBody( worldId, &groundDef );
		b3BoxHull slab = b3MakeBoxHull( 110.0f, 0.5f, 60.0f );
		b3CreateHullShape( ground, &shapeDef, &slab.base );
		for ( int i = 0; i < DYNAMIC; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = (b3Pos){ 2.0f * ( i % 100 ), 0.49f, 2.0f * ( i / 100 ) };
			ids[i] = b3CreateBody( worldId, &bodyDef );
			b3CreateHullShape( ids[i], &shapeDef, &box.base );
		}
		double t0 = now_ms();
		b3World_Step( worldId, 0.0f, 1 );
		double t1 = now_ms();
		for ( int k = 0; k < 10; ++k )
		{
			b3World_Step( worldId, 0.0f, 1 );
		}
		double t2 = now_ms();
		b3Counters counters = b3World_GetCounters( worldId );
		printf( "box3d dynamics: first collide of %d cubes on a slab %.2f ms (%d contacts, %d islands), 10 recycling collides %.2f ms\n",
				DYNAMIC, t1 - t0, counters.contactCount, counters.islandCount, t2 - t1 );

		// Joints: the cubes chained, then unchained.
		b3JointId* joints = calloc( DYNAMIC, sizeof( b3JointId ) );
		t0 = now_ms();
		for ( int i = 0; i < DYNAMIC - 1; ++i )
		{
			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = ids[i];
			jointDef.base.bodyIdB = ids[i + 1];
			jointDef.base.localFrameA.p = (b3Vec3){ 1.0f, 0.0f, 0.0f };
			jointDef.base.localFrameB.p = (b3Vec3){ -1.0f, 0.0f, 0.0f };
			joints[i] = b3CreateRevoluteJoint( worldId, &jointDef );
		}
		t1 = now_ms();
		counters = b3World_GetCounters( worldId );
		int jointCount = counters.jointCount;
		int islandCount = counters.islandCount;
		for ( int i = 0; i < DYNAMIC - 1; ++i )
		{
			b3DestroyJoint( joints[i], false );
		}
		t2 = now_ms();
		printf( "box3d dynamics: chain %d cubes with %d revolute joints %.2f ms (%d island), destroy them %.2f ms\n", DYNAMIC, jointCount,
				t1 - t0, islandCount, t2 - t1 );
		free( joints );
		b3DestroyWorld( worldId );
	}

	// Sensors: spheres over static boxes.
	{
		b3WorldDef worldDef = b3DefaultWorldDef();
		b3WorldId worldId = b3CreateWorld( &worldDef );
		for ( int i = 0; i < SENSORS; ++i )
		{
			b3BodyDef boxDef = b3DefaultBodyDef();
			boxDef.position = (b3Pos){ 2.0f * ( i % 50 ), 0.0f, 2.0f * ( i / 50 ) };
			b3BodyId boxId = b3CreateBody( worldId, &boxDef );
			b3ShapeDef boxShapeDef = b3DefaultShapeDef();
			boxShapeDef.enableSensorEvents = true;
			b3CreateHullShape( boxId, &boxShapeDef, &box.base );
			b3BodyDef sensorDef = b3DefaultBodyDef();
			sensorDef.type = b3_dynamicBody;
			sensorDef.position = (b3Pos){ 2.0f * ( i % 50 ) + 0.5f, 0.0f, 2.0f * ( i / 50 ) };
			b3BodyId sensorId = b3CreateBody( worldId, &sensorDef );
			b3ShapeDef sensorShapeDef = b3DefaultShapeDef();
			sensorShapeDef.isSensor = true;
			sensorShapeDef.enableSensorEvents = true;
			b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.6f };
			b3CreateSphereShape( sensorId, &sensorShapeDef, &sphere );
		}
		double t0 = now_ms();
		b3World_Step( worldId, 0.0f, 1 );
		double t1 = now_ms();
		int beginCount = b3World_GetSensorEvents( worldId ).beginCount;
		for ( int k = 0; k < 10; ++k )
		{
			b3World_Step( worldId, 0.0f, 1 );
		}
		double t2 = now_ms();
		printf( "box3d dynamics: first pass of %d sensors over %d boxes %.2f ms (%d begin events), 10 more passes %.2f ms\n", SENSORS, SENSORS,
				t1 - t0, beginCount, t2 - t1 );
		b3DestroyWorld( worldId );
	}
	free( ids );
	return 0;
}
