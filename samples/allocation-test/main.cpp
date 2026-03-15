/*
 * main.cpp
 */

// std
#include <iostream>
#include <cstdlib>
#include <cstdint>
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

using namespace BulletPhysics;
using namespace BulletPhysics::math;
using namespace BulletPhysics::builtin::bodies;
using namespace BulletPhysics::ballistics::external;
using namespace BulletPhysics::ballistics::external::forces;
using namespace BulletPhysics::ballistics::external::environments;

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

static constexpr double LAUNCH_SPEED = 750.0;
static constexpr double DT = 0.001;
static constexpr int HOT_STEPS = 1000;
static constexpr int BODY_COUNT = 8;

static ProjectileRigidBody makeBody()
{
    auto specs = projectile::ProjectileSpecs::create(0.01, 0.00762)
        .withDragModel(drag::DragCurveModel::G7)
        .withMuzzle(LAUNCH_SPEED, projectile::Direction::RIGHT, 12.0);

    ProjectileRigidBody body(specs);
    body.setPosition({0.0, 1.5, 0.0});
    body.setVelocityFromAngles(LAUNCH_SPEED, 0.0, 90.0);
    return body;
}

struct TestResult
{
    const char* integrator;
    const char* config;
    int bodies;
    int steps;
    std::size_t allocations;
    std::size_t bytes;
    std::size_t frees;
};

static TestResult runTest(const char* integratorName, IIntegrator& integrator, const char* configName, PhysicsWorld& world)
{
    // setup phase (allocations allowed)
    std::vector<ProjectileRigidBody> bodies;
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
        integratorName,
        configName,
        BODY_COUNT,
        HOT_STEPS,
        g_allocCount.load(),
        g_allocBytes.load(),
        g_freeCount.load()
    };
}

// ./AllocationTest

int main()
{
    // integrators
    EulerIntegrator euler;
    MidpointIntegrator midpoint;
    RK4Integrator rk4;

    // world configurations
    // 1. gravity only
    PhysicsWorld gravityWorld;
    gravityWorld.addForce(std::make_unique<Gravity>());

    // 2. full config (gravity + drag + atmosphere + humidity + coriolis + spin)
    PhysicsWorld fullWorld;
    fullWorld.addForce(std::make_unique<Gravity>());
    fullWorld.addEnvironment(std::make_unique<Atmosphere>(280.0, 100000.0));
    fullWorld.addEnvironment(std::make_unique<Humidity>(60));
    fullWorld.addEnvironment(std::make_unique<Geographic>(deg2rad(48.1482), deg2rad(17.1067)));
    fullWorld.addForce(std::make_unique<Drag>());
    fullWorld.addForce(std::make_unique<Coriolis>());
    SpinDrift::addTo(fullWorld);

    // test matrix
    struct TestCase
    {
        const char* integratorName;
        IIntegrator* integrator;
        const char* configName;
        PhysicsWorld* world;
    };

    std::vector<TestCase> tests = {
        {"Euler",    &euler,    "gravity", &gravityWorld},
        {"Euler",    &euler,    "full",    &fullWorld},
        {"Midpoint", &midpoint, "gravity", &gravityWorld},
        {"Midpoint", &midpoint, "full",    &fullWorld},
        {"RK4",      &rk4,      "gravity", &gravityWorld},
        {"RK4",      &rk4,      "full",    &fullWorld},
    };

    // run tests
    std::vector<TestResult> results;
    results.reserve(tests.size());

    for (auto& test : tests)
    {
        results.push_back(runTest(test.integratorName, *test.integrator, test.configName, *test.world));
    }

    // report
    std::cout << "Bodies: " << BODY_COUNT << ", Steps: " << HOT_STEPS
              << ", Total iterations: " << BODY_COUNT * HOT_STEPS << "\n\n";

    std::cout << "integrator,config,allocations,bytes,frees" << "\n";

    for (const auto& r : results)
    {
        std::cout << r.integrator << "," << r.config << ","
                  << r.allocations << "," << r.bytes << "," << r.frees << "\n";
    }

    return 0;
}
