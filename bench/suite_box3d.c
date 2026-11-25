// SPDX-License-Identifier: MIT
// The rest of the reference's benchmark suite (#141): its own scene
// functions (shared/benchmarks.c, e77352c) run as benchmark/main.c runs
// them, one thread: the scene's own step before every step, the first
// step untimed, the rest timed. Prints the line bench/suite.ae prints, so
// the two are read side by side.
//
// Then the SAT runs, the reference's own (shared/sat_benchmark.c).
//
//   AEPHYSICS_BENCH_SCENE=<name>|sat runs one; unset, all of them.

#include "benchmarks.h"
#include "sat_benchmark.h"

#include "box3d/box3d.h"

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

typedef struct Scene
{
	const char* name;
	void ( *capacity )( b3Capacity* capacity );
	void ( *create )( b3WorldId worldId );
	void ( *destroy )( void );
	void ( *step )( b3WorldId worldId, int stepIndex );
	int steps;
} Scene;

// The dynamic bodies' heights summed in id order. These scenes destroy no
// bodies, so the ids are 1..count, each of its first generation.
static double HeightSum( b3WorldId worldId )
{
	b3Counters counters = b3World_GetCounters( worldId );
	double sum = 0.0;
	for ( int i = 1; i <= counters.bodyCount; ++i )
	{
		b3BodyId id = { i, (uint16_t)( worldId.index1 - 1 ), 1 };
		if ( b3Body_IsValid( id ) && b3Body_GetType( id ) == b3_dynamicBody )
		{
			sum += b3Body_GetPosition( id ).y;
		}
	}
	return sum;
}

static void Run( const Scene* scene )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	if ( scene->capacity != NULL )
	{
		scene->capacity( &worldDef.capacity );
	}

	b3WorldId worldId = b3CreateWorld( &worldDef );
	scene->create( worldId );
	// AEPHYSICS_BENCH_NO_SLEEP=1, as bench/suite.ae reads it.
	const char* noSleep = getenv( "AEPHYSICS_BENCH_NO_SLEEP" );
	if ( noSleep != NULL && strcmp( noSleep, "1" ) == 0 )
	{
		b3World_EnableSleeping( worldId, false );
	}

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	if ( scene->step != NULL )
	{
		scene->step( worldId, 0 );
	}
	b3World_Step( worldId, timeStep, subStepCount );

	// AEPHYSICS_BENCH_STEPS=n: every scene n steps (bench/suite.ae's run_steps).
	int steps = scene->steps;
	const char* askedSteps = getenv( "AEPHYSICS_BENCH_STEPS" );
	if ( askedSteps != NULL && atoi( askedSteps ) > 1 )
	{
		steps = atoi( askedSteps );
	}

	double pairs = 0.0, collide = 0.0, solve = 0.0, transforms = 0.0, refit = 0.0, sleep = 0.0, sensors = 0.0;
	// The solve's own stages, as bench/physics_world_box3d.c sums them, to
	// read beside ours (AEPHYSICS_STAGES=1).
	double prepare = 0.0, integrateV = 0.0, warm = 0.0, impulses = 0.0, integrateP = 0.0, relax = 0.0, restitution = 0.0, store = 0.0;
	double t0 = now_ms();
	for ( int stepIndex = 1; stepIndex < steps; ++stepIndex )
	{
		if ( scene->step != NULL )
		{
			scene->step( worldId, stepIndex );
		}
		b3World_Step( worldId, timeStep, subStepCount );
		b3Profile p = b3World_GetProfile( worldId );
		pairs += p.pairs;
		collide += p.collide;
		solve += p.solve;
		transforms += p.transforms;
		refit += p.refit;
		sleep += p.sleepIslands;
		sensors += p.sensors;
		prepare += p.prepareConstraints;
		integrateV += p.integrateVelocities;
		warm += p.warmStart;
		impulses += p.solveImpulses;
		integrateP += p.integratePositions;
		relax += p.relaxImpulses;
		restitution += p.restitution;
		store += p.storeImpulses;
	}
	double t1 = now_ms();

	double timed = steps - 1;
	b3Counters counters = b3World_GetCounters( worldId );
	printf( "box3d suite: %s: %d steps of %d bodies %.3f ms, %.6f ms per step (height sum %.3f, %d contacts, %d joints, %d awake)\n",
			scene->name, steps, counters.bodyCount, t1 - t0, ( t1 - t0 ) / timed, HeightSum( worldId ), counters.contactCount,
			counters.jointCount, b3World_GetAwakeBodyCount( worldId ) );
	printf( "box3d phases ms/step: pairs %.6f collide %.6f\n", pairs / timed, collide / timed );
	printf( "box3d stages ms/step: solve %.6f transforms %.6f refit %.6f sleep %.6f sensors %.6f\n", solve / timed, transforms / timed,
			refit / timed, sleep / timed, sensors / timed );
	printf( "box3d colors:" );
	for ( int i = 0; i < 24; ++i )
	{
		if ( counters.colorCounts[i] > 0 )
		{
			printf( " %d:%d", i, counters.colorCounts[i] );
		}
	}
	printf( "\n" );
	printf( "box3d solve ms/step: prepare %.6f integrate_v %.6f warm %.6f solve %.6f integrate_p %.6f relax %.6f restitution %.6f store %.6f\n",
			prepare / timed, integrateV / timed, warm / timed, impulses / timed, integrateP / timed, relax / timed, restitution / timed,
			store / timed );
	if ( strcmp( scene->name, "spinner" ) == 0 )
	{
		printf( "box3d spinner angle %.6f\n", GetSpinnerAngle() );
	}

	b3DestroyWorld( worldId );
	if ( scene->destroy != NULL )
	{
		scene->destroy();
	}
	ResetGroundShapeId();
}

// Not the reference's: a ground box and eight spheres resting on it, never
// sleeping, as bench/suite.ae's tiny builds it, so a step's fixed cost is
// most of what is timed.
static void CreateTiny( b3WorldId worldId )
{
	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull box = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &shapeDef, &box.base );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	for ( int i = 0; i < 8; ++i )
	{
		bodyDef.position = (b3Pos){ 2.0f * i - 7.0f, 0.5f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}
}

// benchmark/main.c's RunSatBenchmarks: each hull pair's GJK cold and warm
// and SAT with and without the inscribed-sphere bound, the best of four
// runs of twenty repeats, in microseconds a query.
static void RunSat( void )
{
	int runCount = 4;
	int repeatCount = 20;
	for ( int typeA = 0; typeA < satHull_count; ++typeA )
	{
		for ( int typeB = typeA; typeB < satHull_count; ++typeB )
		{
			SatBenchmarkData* data = CreateSatBenchmark( (SatHullType)typeA, (SatHullType)typeB );
			float coldMs = 1e30f, warmMs = 1e30f, satMs = 1e30f, fullMs = 1e30f;
			int queryCount = 0, earlyReturnCount = 0;
			for ( int runIndex = 0; runIndex < runCount; ++runIndex )
			{
				EnableSatInscribedSphere( data, true );
				SatBenchmarkResult cold = RunSatBenchmark( data, repeatCount, false );
				SatBenchmarkResult warm = RunSatBenchmark( data, repeatCount, true );
				EnableSatInscribedSphere( data, false );
				SatBenchmarkResult full = RunSatBenchmark( data, repeatCount, false );
				coldMs = cold.distanceMs < coldMs ? cold.distanceMs : coldMs;
				warmMs = warm.distanceMs < warmMs ? warm.distanceMs : warmMs;
				float sat = cold.satMs < warm.satMs ? cold.satMs : warm.satMs;
				satMs = sat < satMs ? sat : satMs;
				fullMs = full.satMs < fullMs ? full.satMs : fullMs;
				queryCount = cold.queryCount;
				earlyReturnCount += cold.earlyReturnCount + full.earlyReturnCount;
			}
			float scale = 1000.0f / queryCount;
			printf( "box3d sat: %s/%s: gjk cold %.3f us, gjk warm %.3f, sat %.3f, sat full %.3f (early returns %d, worst placement error %g)\n",
					GetSatHullName( (SatHullType)typeA ), GetSatHullName( (SatHullType)typeB ), scale * coldMs, scale * warmMs, scale * satMs,
					scale * fullMs, earlyReturnCount, data->maxPlacementError );
			DestroySatBenchmark( data );
		}
	}
}

int main( void )
{
	Scene scenes[] = {
		{ "convex_pile", GetConvexPileCapacity, CreateConvexPile, NULL, NULL, 500 },
		{ "junkyard", GetJunkyardCapacity, CreateJunkyard, NULL, StepJunkyard, 500 },
		{ "large_world", GetLargeWorldCapacity, CreateLargeWorld, NULL, StepLargeWorld, 500 },
		{ "sleep", GetSleepCapacity, CreateSleep, NULL, StepSleep, 300 },
		{ "spinner", GetSpinnerCapacity, CreateSpinner, DestroySpinner, NULL, 800 },
		{ "trees25", NULL, CreateTrees25, DestroyTrees, NULL, 500 },
		{ "trees50", NULL, CreateTrees50, DestroyTrees, NULL, 500 },
		{ "trees100", NULL, CreateTrees100, DestroyTrees, NULL, 500 },
		{ "washer", GetWasherCapacity, CreateWasher, NULL, NULL, 1000 },
		{ "tiny", NULL, CreateTiny, NULL, NULL, 20000 },
	};

	const char* asked = getenv( "AEPHYSICS_BENCH_SCENE" );
	for ( int i = 0; i < (int)( sizeof( scenes ) / sizeof( scenes[0] ) ); ++i )
	{
		if ( asked == NULL || strcmp( asked, scenes[i].name ) == 0 )
		{
			Run( scenes + i );
		}
	}
	if ( asked == NULL || strcmp( asked, "sat" ) == 0 )
	{
		RunSat();
	}
	return 0;
}
