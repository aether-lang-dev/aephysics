// The reference's "rain" benchmark (shared/benchmarks.c): a ten by ten
// grid of cells, each a grid mesh with a torus on it, and every 48
// steps a column of cells gets three ragdolls (shared/human.c) dropped
// from twenty metres; once every column has its group the columns are
// recycled in turn. 400 steps of 1/60 with four sub-steps, one thread,
// against bench/human.ae. The checksums: the body, joint and contact
// counts, the awake bodies, and the heights of the bodies that moved in
// the last step (the reference offers no walk over its bodies).
#include "box3d/box3d.h"
#include "benchmarks.h"

#include <stdio.h>
#include <time.h>

static double now_ms( void )
{
	struct timespec ts;
	timespec_get( &ts, TIME_UTC );
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

int main( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateRain( worldId );
	int steps = 400;
	double t0 = now_ms();
	for ( int s = 0; s < steps; ++s )
	{
		StepRain( worldId, s );
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	double t1 = now_ms();
	b3BodyEvents events = b3World_GetBodyEvents( worldId );
	double movedHeights = 0.0;
	for ( int i = 0; i < events.moveCount; ++i )
	{
		movedHeights += events.moveEvents[i].transform.p.y;
	}
	b3Counters counters = b3World_GetCounters( worldId );
	printf( "box3d human: rain: %d steps %.1f ms, %.3f ms per step (%d bodies, %d joints, %d contacts, %d awake, %d moved, height sum %.3f)\n",
			steps, t1 - t0, ( t1 - t0 ) / steps, counters.bodyCount, counters.jointCount, counters.contactCount,
			b3World_GetAwakeBodyCount( worldId ), events.moveCount, movedHeights );
	DestroyRain();
	b3DestroyWorld( worldId );
	return 0;
}
