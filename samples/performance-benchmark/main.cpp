/*
 * main.cpp
 */

// std
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>

// linux
#include <sched.h>
#include <cstring>

// BulletPhysics
#include "math/Integrator.h"
#include "math/Angles.h"
#include "dynamics/PhysicsBody.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/forces/Gravity.h"
#include "dynamics/forces/Coriolis.h"
#include "dynamics/forces/drag/Drag.h"
#include "dynamics/environment/Atmosphere.h"
#include "dynamics/environment/Geographic.h"
#include "dynamics/environment/Humidity.h"

using namespace BulletPhysics;
using namespace BulletPhysics::math;
using namespace BulletPhysics::dynamics;
using namespace BulletPhysics::dynamics::forces;
using namespace BulletPhysics::dynamics::environment;

static constexpr int BODY_COUNT = 1000;
static constexpr double DT = 0.001;

static projectile::ProjectileRigidBody makeBody(double speed, double elevation, double azimuth)
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
    body.setVelocityFromAngles(speed, elevation, azimuth);
    return body;
}

int main()
{
    // pin to CPU 0
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0)
    {
        std::cerr << "Warning: sched_setaffinity failed: " << std::strerror(errno) << "\n";
    }

    // raise priority (requires root)
    sched_param param{};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
    {
        std::cerr << "Warning: sched_setscheduler failed: " << std::strerror(errno) << "\n";
    }

    // physics world: full config
    PhysicsWorld physicsWorld;
    physicsWorld.addForce(std::make_unique<Gravity>());
    physicsWorld.addEnvironment(std::make_unique<Atmosphere>(280.0, 100000.0));
    physicsWorld.addEnvironment(std::make_unique<Humidity>(60));
    physicsWorld.addEnvironment(std::make_unique<Geographic>(deg2rad(48.1482), deg2rad(17.1067)));
    physicsWorld.addForce(std::make_unique<drag::Drag>());
    physicsWorld.addForce(std::make_unique<Coriolis>());

    // integrator
    MidpointIntegrator integrator;

    // create projectiles: same speed and elevation, spread azimuth evenly
    static constexpr double SPEED = 750.0;
    static constexpr double ELEVATION = 5.0;
    std::vector<projectile::ProjectileRigidBody> bodies;
    bodies.reserve(BODY_COUNT);

    for (int i = 0; i < BODY_COUNT; ++i)
    {
        double azimuth = 360.0 * i / BODY_COUNT;
        bodies.push_back(makeBody(SPEED, ELEVATION, azimuth));
    }

    std::cout << "Bodies: " << BODY_COUNT << ", dt: " << DT << "\n";
    std::cout << "Integrator: Midpoint\n";
    std::cout << "Config: Gravity + Atmosphere + Humidity + Geographic + Drag + Coriolis\n\n";

    // buffer for results (avoid I/O in hot loop)
    struct Sample
    {
        int step;
        int active;
        long long step_ns;
    };

    std::vector<Sample> samples;
    samples.reserve(16000);

    // simulate
    int step = 0;
    int active = BODY_COUNT;

    while (active == BODY_COUNT)
    {
        auto t0 = std::chrono::high_resolution_clock::now();

        for (auto& body : bodies)
        {
            integrator.step(body, &physicsWorld, DT);
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        long long step_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();

        // count active (above ground)
        active = 0;
        for (const auto& body : bodies)
        {
            if (body.getPosition().y > 0.0)
            {
                ++active;
            }
        }

        samples.push_back({step, active, step_ns});
        ++step;
    }

    // write csv after simulation
    std::ofstream csv("performance.csv");
    csv << "step,active,step_ns\n";
    for (const auto& s : samples)
    {
        csv << s.step << "," << s.active << "," << s.step_ns << "\n";
    }
    csv.close();

    std::cout << "Simulation complete: " << step << " steps\n";
    std::cout << "Results written to performance.csv\n";

    return 0;
}
