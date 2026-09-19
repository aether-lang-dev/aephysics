// The mesh of the reference on the same scenes as bench/mesh.ae: a
// 200 x 200 wave mesh (80,000 triangles) built ten times with the median
// split and ten with the SAH, with edges identified; 100,000 ray casts
// down onto it; 100,000 box queries over it; 10,000 sphere shape casts
// onto it. Single thread, wall time per phase, with the hit counts and
// sums as the checksum.
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

static int g_queryHits;
static bool count_triangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	(void)a; (void)b; (void)c; (void)triangleIndex; (void)context;
	g_queryHits += 1;
	return true;
}

#define RAYS 100000
#define QUERIES 100000
#define CASTS 10000

int main( void )
{
	double t0 = now_ms();
	b3MeshData* data = NULL;
	int nodeCount = 0;
	for ( int i = 0; i < 10; ++i )
	{
		if ( data ) b3DestroyMesh( data );
		data = b3CreateWaveMesh( 200, 200, 0.5f, 0.4f, 0.3f, 0.7f );
		nodeCount = data->nodeCount;
	}
	double t1 = now_ms();
	b3MeshData* sahData = NULL;
	{
		// The wave mesh uses the median split; the same triangles through the SAH.
		for ( int i = 0; i < 10; ++i )
		{
			if ( sahData ) b3DestroyMesh( sahData );
			const b3Vec3* vertices = b3GetMeshVertices( data );
			const b3MeshTriangle* triangles = b3GetMeshTriangles( data );
			b3MeshDef def = { 0 };
			def.vertices = (b3Vec3*)vertices;
			def.vertexCount = data->vertexCount;
			def.indices = (int32_t*)triangles;
			def.triangleCount = data->triangleCount;
			def.identifyEdges = true;
			sahData = b3CreateMesh( &def, NULL, 0 );
		}
	}
	double t2 = now_ms();

	b3Mesh mesh = { data, { 1.0f, 1.0f, 1.0f } };
	int rayHits = 0;
	double raySum = 0.0;
	for ( int i = 0; i < RAYS; ++i )
	{
		float t = (float)i / (float)RAYS;
		b3RayCastInput input = { { -49.0f + 98.0f * t, 3.0f, 40.0f * sinf( 40.0f * t ) }, { 0.0f, -6.0f, 0.0f }, 1.0f };
		b3CastOutput out = b3RayCastMesh( &mesh, &input );
		if ( out.hit )
		{
			rayHits += 1;
			raySum += out.point.y;
		}
	}
	double t3 = now_ms();

	g_queryHits = 0;
	for ( int i = 0; i < QUERIES; ++i )
	{
		float t = (float)i / (float)QUERIES;
		b3Vec3 c = { -48.0f + 96.0f * t, 0.2f * sinf( 3.0f * t ), 40.0f * cosf( 30.0f * t ) };
		b3Vec3 h = { 0.6f, 0.5f, 0.6f };
		b3AABB bounds = { b3Sub( c, h ), b3Add( c, h ) };
		b3QueryMesh( &mesh, bounds, count_triangle, NULL );
	}
	double t4 = now_ms();

	int castHits = 0;
	double castSum = 0.0;
	for ( int i = 0; i < CASTS; ++i )
	{
		float t = (float)i / (float)CASTS;
		b3Vec3 start = { -48.0f + 96.0f * t, 3.0f, 40.0f * sinf( 50.0f * t ) };
		b3ShapeCastInput input = { { &start, 1, 0.3f }, { 0.0f, -6.0f, 0.0f }, 1.0f, false };
		b3CastOutput out = b3ShapeCastMesh( &mesh, &input );
		if ( out.hit )
		{
			castHits += 1;
			castSum += out.fraction;
		}
	}
	double t5 = now_ms();

	printf( "box3d mesh: 10 median builds %.1f ms (%d triangles, %d nodes, height %d), 10 SAH builds %.1f ms (%d nodes, height %d), "
			"%d rays %.2f ms (%d hits, sum %.3f), %d queries %.2f ms (%d hits), %d casts %.2f ms (%d hits, sum %.3f)\n",
			t1 - t0, data->triangleCount, nodeCount, data->treeHeight, t2 - t1, sahData->nodeCount, sahData->treeHeight,
			RAYS, t3 - t2, rayHits, raySum, QUERIES, t4 - t3, g_queryHits, CASTS, t5 - t4, castHits, castSum );
	b3DestroyMesh( sahData );
	b3DestroyMesh( data );
	return 0;
}
