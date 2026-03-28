/*
 * main.cpp
 */

// std
#include <iostream>
#include <cstdlib>
#include <atomic>
#include <new>
#include <vector>

// BulletPhysics
#include "math/Integrator.h"
#include "math/Angles.h"
#include "builtin/bodies/RigidBody.h"
#include "ballistics/external/PhysicsWorld.h"
#include "ballistics/external/forces/Gravity.h"
#include "ballistics/external/forces/Coriolis.h"
#include "ballistics/external/forces/SpinDrift.h"
#include "ballistics/external/forces/drag/Drag.h"
#include "ballistics/external/environments/Atmosphere.h"
#include "ballistics/external/environments/Geographic.h"
#include "ballistics/external/environments/Humidity.h"
#include "ballistics/external/environments/Wind.h"
#include "geography/CoordinateMapping.h"

using namespace BulletPhysics;

// allocation tracker
static std::atomic<bool> g_tracking{false};
static std::atomic<std::size_t> g_allocCount{0};
static std::atomic<std::size_t> g_allocBytes{0};
static std::atomic<std::size_t> g_freeCount{0};

void* operator new(std::size_t size)
{
    if (g_tracking.load(std::memory_order_relaxed))
    {
        g_allocCount.fetch_add(1, std::memory_order_relaxed);
        g_allocBytes.fetch_add(size, std::memory_order_relaxed);
    }

    void* ptr = std::malloc(size);
    if (!ptr)
        throw std::bad_alloc();

    return ptr;
}

void operator delete(void* ptr) noexcept
{
    if (ptr && g_tracking.load(std::memory_order_relaxed))
    {
        g_freeCount.fetch_add(1, std::memory_order_relaxed);
    }

    std::free(ptr);
}

void operator delete(void* ptr, std::size_t) noexcept
{
    if (ptr && g_tracking.load(std::memory_order_relaxed))
    {
        g_freeCount.fetch_add(1, std::memory_order_relaxed);
    }

    std::free(ptr);
}

static void resetCounters()
{
    g_allocCount.store(0, std::memory_order_relaxed);
    g_allocBytes.store(0, std::memory_order_relaxed);
    g_freeCount.store(0, std::memory_order_relaxed);
}

static void startTracking()
{
    resetCounters();
    g_tracking.store(true, std::memory_order_seq_cst);
}

static void stopTracking()
{
    g_tracking.store(false, std::memory_order_seq_cst);
}

// simulation parameters
static constexpr double DT = 0.001;
static constexpr int HOT_STEPS = 1000;
static constexpr int BODY_COUNT = 8;

static builtin::bodies::ProjectileRigidBody makeBody()
{
    auto specs = projectile::ProjectileSpecs::create(0.01, 0.00762)
        .withDragModel(ballistics::external::forces::drag::DragCurveModel::G7)
        .withMuzzle(750.0, projectile::Direction::RIGHT, 12.0);

    builtin::bodies::ProjectileRigidBody body(specs);
    body.setPosition({0.0, 1.5, 0.0});
    body.setAngles(0.0, 90.0);
    return body;
}

struct TestResult
{
    const char* integrator;
    std::size_t allocations;
    std::size_t bytes;
    std::size_t frees;
};

static TestResult runTest(const char* name, math::IIntegrator& integrator, ballistics::external::PhysicsWorld& world)
{
    // setup phase (allocations allowed)
    std::vector<builtin::bodies::ProjectileRigidBody> bodies;
    bodies.reserve(BODY_COUNT);
    for (int i = 0; i < BODY_COUNT; ++i)
    {
        bodies.push_back(makeBody());
    }

    // hot loop (allocations tracked)
    startTracking();

    for (int step = 0; step < HOT_STEPS; ++step)
    {
        for (auto& body : bodies)
        {
            integrator.step(body, &world, DT);
        }
    }

    stopTracking();

    return {
        name,
        g_allocCount.load(),
        g_allocBytes.load(),
        g_freeCount.load()
    };
}

int main()
{
    geography::CoordinateMapping::set(geography::mappings::OpenGL());

    // integrators
    math::EulerIntegrator euler;
    math::MidpointIntegrator midpoint;
    math::RK4Integrator rk4;

    // full world configuration
    ballistics::external::PhysicsWorld world;

    world.addEnvironment(std::make_unique<ballistics::external::environments::Atmosphere>(280.0, 100000.0));
    world.addEnvironment(std::make_unique<ballistics::external::environments::Humidity>(60));
    world.addEnvironment(std::make_unique<ballistics::external::environments::Geographic>(math::deg2rad(48.1482), math::deg2rad(17.1067)));
    world.addEnvironment(std::make_unique<ballistics::external::environments::Wind>(math::Vec3{0.0, 0.0, 2.0}));
    world.addForce(std::make_unique<ballistics::external::forces::Gravity>());
    world.addForce(std::make_unique<ballistics::external::forces::Drag>());
    world.addForce(std::make_unique<ballistics::external::forces::Coriolis>());
    ballistics::external::forces::SpinDrift::addTo(world);

    struct TestEntry
    {
        const char* name;
        math::IIntegrator* integrator;
    };

    std::vector<TestEntry> tests = {
        {"euler", &euler},
        {"midpoint", &midpoint},
        {"rk4", &rk4},
    };

    // run tests
    std::vector<TestResult> results;
    results.reserve(tests.size());

    for (auto& test : tests)
    {
        results.push_back(runTest(test.name, *test.integrator, world));
    }

    // report
    std::cout << "bodies: " << BODY_COUNT << ", steps: " << HOT_STEPS << "\n\n";

    for (const auto& result : results)
    {
        std::cout << result.integrator << ": " << result.allocations << " allocations, " << result.bytes << " bytes, " << result.frees << " frees\n";
    }

    return 0;
}
