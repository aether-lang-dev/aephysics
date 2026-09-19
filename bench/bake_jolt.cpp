// The bake-off scenes on Jolt, single thread: the same scenes as
// bake_box3d.c. Jolt's recommended 60 Hz setting is one collision step
// with its default solver iterations (10 velocity, 2 position); Box3D's is
// four sub-steps. Each engine runs at its own recommended setting for a
// game's 60 Hz step, which is the comparison a game cares about.
//
//   bake_jolt pile|pyramid|chain [steps]
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace JPH;

namespace Layers
{
static constexpr ObjectLayer NON_MOVING = 0;
static constexpr ObjectLayer MOVING = 1;
static constexpr ObjectLayer NUM_LAYERS = 2;
} // namespace Layers

class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter
{
public:
	bool ShouldCollide( ObjectLayer a, ObjectLayer b ) const override
	{
		return a == Layers::MOVING || b == Layers::MOVING;
	}
};

namespace BroadPhaseLayers
{
static constexpr BroadPhaseLayer NON_MOVING( 0 );
static constexpr BroadPhaseLayer MOVING( 1 );
static constexpr uint NUM_LAYERS( 2 );
} // namespace BroadPhaseLayers

class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface
{
public:
	uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::NUM_LAYERS; }
	BroadPhaseLayer GetBroadPhaseLayer( ObjectLayer layer ) const override
	{
		return layer == Layers::NON_MOVING ? BroadPhaseLayers::NON_MOVING : BroadPhaseLayers::MOVING;
	}
#if defined( JPH_EXTERNAL_PROFILE ) || defined( JPH_PROFILE_ENABLED )
	const char* GetBroadPhaseLayerName( BroadPhaseLayer ) const override { return "layer"; }
#endif
};

class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
{
public:
	bool ShouldCollide( ObjectLayer a, BroadPhaseLayer b ) const override
	{
		return a == Layers::MOVING || b == BroadPhaseLayers::MOVING;
	}
};

static double now_ms()
{
	using namespace std::chrono;
	return duration<double, std::milli>( steady_clock::now().time_since_epoch() ).count();
}

static BodyInterface* g_bodies;

static void ground()
{
	BodyCreationSettings s( new BoxShape( Vec3( 400.0f, 1.0f, 400.0f ) ), RVec3( 0, -1, 0 ), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING );
	g_bodies->CreateAndAddBody( s, EActivation::DontActivate );
}

static BodyID dynamic_box( RVec3Arg p, float half, float density )
{
	BoxShape* shape = new BoxShape( Vec3( half, half, half ) );
	shape->SetDensity( density );
	BodyCreationSettings s( shape, p, Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING );
	s.mAllowSleeping = false;
	return g_bodies->CreateAndAddBody( s, EActivation::Activate );
}

static std::vector<BodyID> g_pile;
static void create_pile()
{
	ground();
	for ( int y = 0; y < 25; ++y )
		for ( int x = 0; x < 20; ++x )
			for ( int z = 0; z < 20; ++z )
				g_pile.push_back( dynamic_box( RVec3( 1.1f * ( x - 10 ), 2.0f + 1.1f * y, 1.1f * ( z - 10 ) ), 0.5f, 1000.0f ) );
}
static double measure_pile()
{
	double lowest = 1e9;
	for ( BodyID id : g_pile )
	{
		RVec3 p = g_bodies->GetPosition( id );
		if ( p.GetY() < lowest ) lowest = p.GetY();
	}
	return lowest;
}

static std::vector<BodyID> g_pyr;
static std::vector<RVec3> g_pyr_start;
static void create_pyramid()
{
	ground();
	int baseCount = 100;
	float h = 0.5f;
	float shift = 1.0f * h;
	for ( int i = 0; i < baseCount; ++i )
	{
		float y = ( 2.0f * i + 1.0f ) * shift;
		for ( int j = i; j < baseCount; ++j )
		{
			float x = ( i + 1.0f ) * shift + 2.0f * ( j - i ) * shift - h * baseCount;
			RVec3 p( x, y, 0.0f );
			g_pyr.push_back( dynamic_box( p, h, 100.0f ) );
			g_pyr_start.push_back( p );
		}
	}
}
static double measure_pyramid()
{
	double worst = 0.0;
	for ( size_t i = 0; i < g_pyr.size(); ++i )
	{
		RVec3 p = g_bodies->GetPosition( g_pyr[i] );
		double d = ( p - g_pyr_start[i] ).LengthSq();
		if ( d > worst ) worst = d;
	}
	return std::sqrt( worst );
}

static BodyID g_chain_last;
static void create_chain( PhysicsSystem& system )
{
	int n = 100;
	std::vector<BodyID> bodies;
	bodies.reserve( n * n );
	SphereShape* sphere = new SphereShape( 0.4f );
	for ( int k = 0; k < n; ++k )
		for ( int i = 0; i < n; ++i )
		{
			RVec3 p( (float)k, -(float)i, 0.0f );
			BodyCreationSettings s( sphere, p, Quat::sIdentity(),
									i == 0 ? EMotionType::Static : EMotionType::Dynamic,
									i == 0 ? Layers::NON_MOVING : Layers::MOVING );
			s.mAllowSleeping = false;
			// The spheres of the grid do not collide with each other, as in the reference scene.
			s.mCollisionGroup.SetGroupID( 1 );
			BodyID id = g_bodies->CreateAndAddBody( s, EActivation::Activate );
			if ( i > 0 )
			{
				PointConstraintSettings c;
				c.mSpace = EConstraintSpace::WorldSpace;
				c.mPoint1 = p + RVec3( 0, 0.5f, 0 );
				c.mPoint2 = c.mPoint1;
				Body* a = nullptr; Body* b = nullptr;
				{
					BodyLockWrite la( system.GetBodyLockInterface(), bodies[bodies.size() - 1] ); a = &la.GetBody();
					BodyLockWrite lb( system.GetBodyLockInterface(), id ); b = &lb.GetBody();
					system.AddConstraint( c.Create( *a, *b ) );
				}
			}
			if ( k > 0 )
			{
				PointConstraintSettings c;
				c.mSpace = EConstraintSpace::WorldSpace;
				c.mPoint1 = p + RVec3( -0.5f, 0, 0 );
				c.mPoint2 = c.mPoint1;
				BodyLockWrite la( system.GetBodyLockInterface(), bodies[bodies.size() - n] );
				BodyLockWrite lb( system.GetBodyLockInterface(), id );
				system.AddConstraint( c.Create( la.GetBody(), lb.GetBody() ) );
			}
			bodies.push_back( id );
		}
	g_chain_last = bodies.back();
}
static double measure_chain()
{
	RVec3 p = g_bodies->GetPosition( g_chain_last );
	return -99.0 - p.GetY();
}

int main( int argc, char** argv )
{
	const char* scene = argc > 1 ? argv[1] : "pile";
	int steps = argc > 2 ? atoi( argv[2] ) : 300;

	RegisterDefaultAllocator();
	Factory::sInstance = new Factory();
	RegisterTypes();
	TempAllocatorImpl temp_allocator( 1024 * 1024 * 1024 );
	JobSystemSingleThreaded job_system( cMaxPhysicsJobs );

	BPLayerInterfaceImpl bp_layers;
	ObjectVsBroadPhaseLayerFilterImpl object_vs_bp;
	ObjectLayerPairFilterImpl object_vs_object;
	PhysicsSystem system;
	system.Init( 20000, 0, 200000, 200000, bp_layers, object_vs_bp, object_vs_object );
	g_bodies = &system.GetBodyInterface();

	double ( *measure )() = nullptr;
	if ( strcmp( scene, "pile" ) == 0 ) { create_pile(); measure = measure_pile; }
	else if ( strcmp( scene, "pyramid" ) == 0 ) { create_pyramid(); measure = measure_pyramid; }
	else if ( strcmp( scene, "chain" ) == 0 ) { create_chain( system ); measure = measure_chain; }
	else { fprintf( stderr, "scene?\n" ); return 2; }
	system.OptimizeBroadPhase();

	float dt = 1.0f / 60.0f;
	int collisionSteps = 1;
	system.Update( dt, collisionSteps, &temp_allocator, &job_system );
	double start = now_ms();
	for ( int i = 1; i < steps; ++i ) system.Update( dt, collisionSteps, &temp_allocator, &job_system );
	double ms = now_ms() - start;
	printf( "jolt %s: %d steps in %.1f ms, %.3f ms/step, measure %.4f\n", scene, steps, ms, ms / ( steps - 1 ), measure() );

	UnregisterTypes();
	delete Factory::sInstance;
	Factory::sInstance = nullptr;
	return 0;
}
