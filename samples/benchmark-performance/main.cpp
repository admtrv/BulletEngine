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
#include "builtin/bodies/RigidBody.h"
#include "ballistics/external/PhysicsWorld.h"
#include "ballistics/external/forces/Gravity.h"
#include "ballistics/external/forces/Coriolis.h"
#include "ballistics/external/forces/drag/Drag.h"
#include "ballistics/external/environments/Atmosphere.h"
#include "ballistics/external/environments/Geographic.h"
#include "ballistics/external/environments/Humidity.h"
#include "geography/CoordinateMapping.h"

using namespace BulletPhysics;

// exit file
static constexpr std::string_view FILE_NAME = "performance.csv";

// simulation parameters
static constexpr int BODY_COUNT = 1000;
static constexpr double DT = 0.001;
static constexpr double ELEVATION = 5.0;

// configuration
static constexpr double TEMPERATURE = 280.0;        // K
static constexpr double PRESSURE = 100000.0;        // Pa
static constexpr double REL_HUMIDITY = 60.0;        // %
static constexpr double LATITUDE = 48.1482;         // deg
static constexpr double LONGITUDE = 17.1067;        // deg

static builtin::bodies::ProjectileRigidBody makeBody(double azimuth)
{
    auto specs = projectile::ProjectileSpecs::create(0.01, 0.00762)
        .withDragModel(ballistics::external::forces::drag::DragCurveModel::G7)
        .withMuzzle(750.0, projectile::Direction::RIGHT, 12.0);

    builtin::bodies::ProjectileRigidBody body(specs);
    body.setPosition({0.0, 1.5, 0.0});
    body.setAngles(ELEVATION, azimuth);
    return body;
}

struct Sample
{
    int step;
    int active;
    long long stepNs;
};

int main()
{
    geography::CoordinateMapping::set(geography::mappings::OpenGL());

    // pin to CPU 0
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0)
        std::cerr << "warning: sched_setaffinity failed: " << std::strerror(errno) << "\n";     // use sudo

    // raise priority
    sched_param param{};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
        std::cerr << "warning: sched_setscheduler failed: " << std::strerror(errno) << "\n";

    // physics world
    ballistics::external::PhysicsWorld physicsWorld;

    physicsWorld.addForce(std::make_unique<ballistics::external::forces::Gravity>());
    physicsWorld.addEnvironment(std::make_unique<ballistics::external::environments::Atmosphere>(TEMPERATURE, PRESSURE));
    physicsWorld.addEnvironment(std::make_unique<ballistics::external::environments::Humidity>(REL_HUMIDITY));
    physicsWorld.addEnvironment(std::make_unique<ballistics::external::environments::Geographic>(math::deg2rad(LATITUDE), math::deg2rad(LONGITUDE)));
    physicsWorld.addForce(std::make_unique<ballistics::external::forces::Drag>());
    physicsWorld.addForce(std::make_unique<ballistics::external::forces::Coriolis>());

    // integrator
    math::MidpointIntegrator integrator;

    // create projectiles: same speed and elevation, spread azimuth evenly
    std::vector<builtin::bodies::ProjectileRigidBody> bodies;
    bodies.reserve(BODY_COUNT);

    for (int i = 0; i < BODY_COUNT; ++i)
    {
        double azimuth = 360.0 * i / BODY_COUNT;
        bodies.push_back(makeBody(azimuth));
    }

    std::cout << "bodies: " << BODY_COUNT << "\n";

    // simulate
    std::vector<Sample> samples;
    samples.reserve(16000);

    int step = 0;
    int active = BODY_COUNT;

    while (active == BODY_COUNT)
    {
        auto t0 = std::chrono::high_resolution_clock::now();

        for (auto& body : bodies)
            integrator.step(body, &physicsWorld, DT);

        auto t1 = std::chrono::high_resolution_clock::now();
        long long stepNs = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();

        // count active (above ground)
        active = 0;
        for (const auto& body : bodies)
        {
            if (body.getPosition().y > 0.0)
                ++active;
        }

        samples.push_back({step, active, stepNs});
        ++step;
    }

    // write csv

    std::ofstream file(FILE_NAME.data());
    file << "step,active,step_ns\n";

    for (const auto& sample : samples)
        file << sample.step << "," << sample.active << "," << sample.stepNs << "\n";

    std::cout << "done " << FILE_NAME << "\n";

    return 0;
}
