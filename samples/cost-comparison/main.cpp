/*
 * main.cpp
 */

// std
#include <iostream>
#include <chrono>
#include <random>

// BulletPhysics
#include "math/Integrator.h"
#include "math/Angles.h"
#include "dynamics/PhysicsBody.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/forces/Gravity.h"
#include "dynamics/forces/Coriolis.h"
#include "dynamics/forces/SpinDrift.h"
#include "dynamics/forces/drag/Drag.h"
#include "dynamics/environment/Atmosphere.h"
#include "dynamics/environment/Geographic.h"
#include "dynamics/environment/Humidity.h"
#include "dynamics/environment/Wind.h"

using namespace BulletPhysics;
using namespace BulletPhysics::math;
using namespace BulletPhysics::dynamics;
using namespace BulletPhysics::dynamics::forces;
using namespace BulletPhysics::dynamics::environment;

static constexpr double LAUNCH_SPEED = 750.0;
static constexpr double DT = 0.001;

// measurement params
static constexpr int WARMUP_STEPS = 2000;
static constexpr int MEASURE_STEPS = 8000;
static constexpr int REPS = 9;

static projectile::ProjectileRigidBody makeBody()
{
    projectile::ProjectileSpecs specs{};
    specs.mass = 0.01;
    specs.diameter = 0.00762;
    specs.dragModel = drag::DragCurveModel::G7;
    specs.spinSpecs = projectile::SpinSpecs{};
    specs.spinSpecs->riflingSpecs = projectile::RiflingSpecs{
        projectile::RiflingSpecs::Direction::RIGHT,
        12.0
    };

    projectile::ProjectileRigidBody body(specs);
    body.setPosition({0.0, 1.5, 0.0});
    body.setVelocityFromAngles(LAUNCH_SPEED, 0.0, 90.0);
    return body;
}

static void thrashCache()
{
    static std::vector<std::uint8_t> buf(64 * 1024 * 1024, 1);
    volatile std::uint64_t sum = 0;
    for (size_t i = 0; i < buf.size(); i += 64)
        sum += buf[i];
    (void)sum;
}

struct ConfigRun
{
    const char* name;
    PhysicsWorld* world;
};

static void runOneConfig(const char* config, PhysicsWorld& world, IIntegrator& integrator, int rep)
{
    thrashCache();

    // warmup
    {
        auto body = makeBody();
        for (int i = 0; i < WARMUP_STEPS; ++i)
            integrator.step(body, &world, DT);
    }

    thrashCache();

    // measure
    auto body = makeBody();

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < MEASURE_STEPS; ++i)
        integrator.step(body, &world, DT);
    auto t1 = std::chrono::high_resolution_clock::now();

    long long total_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    double avg_step_ns = double(total_ns) / double(MEASURE_STEPS);

    std::cout << config << "," << rep << "," << MEASURE_STEPS << "," << avg_step_ns << "\n";
}

// ./CostComparison > midpoint.csv

int main()
{
    MidpointIntegrator integrator;

    // 1. g only
    PhysicsWorld idealWorld;
    idealWorld.addForce(std::make_unique<Gravity>());

    // 2. g + drag
    PhysicsWorld dragWorld;
    dragWorld.addForce(std::make_unique<Gravity>());
    dragWorld.addEnvironment(std::make_unique<Atmosphere>(280.0, 100000.0)); // t_0 = 280 K, p_0 = 100.000 Pa
    dragWorld.addEnvironment(std::make_unique<Humidity>(60));                  // relative humidity = 60%
    dragWorld.addForce(std::make_unique<drag::Drag>());

    // 3. g + drag + coriolis
    PhysicsWorld coriolisWorld;
    coriolisWorld.addForce(std::make_unique<Gravity>());
    coriolisWorld.addEnvironment(std::make_unique<Atmosphere>(280.0, 100000.0)); // t_0 = 280 K, p_0 = 100.000 Pa
    coriolisWorld.addEnvironment(std::make_unique<Humidity>(60));                  // relative humidity = 60%
    coriolisWorld.addEnvironment(std::make_unique<Geographic>(deg2rad(48.1482), deg2rad(17.1067))); // Bratislava coordinates
    coriolisWorld.addForce(std::make_unique<drag::Drag>());
    coriolisWorld.addForce(std::make_unique<Coriolis>());

    // 4. g + drag + coriolis + spin
    PhysicsWorld spinWorld;
    spinWorld.addForce(std::make_unique<Gravity>());
    spinWorld.addEnvironment(std::make_unique<Atmosphere>(280.0, 100000.0)); // t_0 = 280 K, p_0 = 100.000 Pa
    spinWorld.addEnvironment(std::make_unique<Humidity>(60));                  // relative humidity = 60%
    spinWorld.addEnvironment(std::make_unique<Geographic>(deg2rad(48.1482), deg2rad(17.1067))); // Bratislava coordinates
    spinWorld.addForce(std::make_unique<drag::Drag>());
    spinWorld.addForce(std::make_unique<Coriolis>());
    SpinDrift::addTo(spinWorld);

    std::vector<ConfigRun> configs = {
        {"gravity", &idealWorld},
        {"+drag", &dragWorld},
        {"+coriolis", &coriolisWorld},
        {"+spin", &spinWorld},
    };

    std::mt19937 rng(12345);

    std::cout << "config,rep,steps,avg_step_ns\n";

    for (int rep = 0; rep < REPS; ++rep)
    {
        std::shuffle(configs.begin(), configs.end(), rng);

        for (auto& c : configs)
            runOneConfig(c.name, *c.world, integrator, rep);
    }

    return 0;
}
