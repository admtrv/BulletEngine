/*
 * main.cpp
 */

// std
#include <iostream>
#include <fstream>
#include <chrono>
#include <random>

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
#include "geography/CoordinateMapping.h"

using namespace BulletPhysics;

// time step
static constexpr double DT = 0.001;

// measurement params
static constexpr int WARMUP_STEPS = 2000;
static constexpr int MEASURE_STEPS = 8000;
static constexpr int REPS = 9;

// configuration
static constexpr double TEMPERATURE = 280.0;        // K
static constexpr double PRESSURE = 100000.0;        // Pa
static constexpr double REL_HUMIDITY = 60.0;        // %
static constexpr double LATITUDE = 48.1482;         // deg
static constexpr double LONGITUDE = 17.1067;        // deg

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

static void thrashCache()
{
    static std::vector<std::uint8_t> buf(64 * 1024 * 1024, 1);  // 64 MB
    volatile std::uint64_t sum = 0;
    for (size_t i = 0; i < buf.size(); i += 64)
        sum += buf[i];
    (void)sum;
}

static void runConfig(const char* config, ballistics::external::PhysicsWorld& world, math::IIntegrator& integrator, int rep, std::ostream& out)
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

    out << config << "," << rep << "," << MEASURE_STEPS << "," << avg_step_ns << "\n";
}

int main()
{
    geography::CoordinateMapping::set(geography::mappings::OpenGL());

    // gravity only
    ballistics::external::PhysicsWorld gravityWorld;

    gravityWorld.addForce(std::make_unique<ballistics::external::forces::Gravity>());

    // + drag
    ballistics::external::PhysicsWorld dragWorld;

    dragWorld.addEnvironment(std::make_unique<ballistics::external::environments::Atmosphere>(TEMPERATURE, PRESSURE));
    dragWorld.addEnvironment(std::make_unique<ballistics::external::environments::Humidity>(REL_HUMIDITY));
    dragWorld.addForce(std::make_unique<ballistics::external::forces::Gravity>());
    dragWorld.addForce(std::make_unique<ballistics::external::forces::Drag>());

    // + coriolis
    ballistics::external::PhysicsWorld coriolisWorld;

    coriolisWorld.addEnvironment(std::make_unique<ballistics::external::environments::Atmosphere>(TEMPERATURE, PRESSURE));
    coriolisWorld.addEnvironment(std::make_unique<ballistics::external::environments::Humidity>(REL_HUMIDITY));
    coriolisWorld.addEnvironment(std::make_unique<ballistics::external::environments::Geographic>(math::deg2rad(LATITUDE), math::deg2rad(LONGITUDE)));
    coriolisWorld.addForce(std::make_unique<ballistics::external::forces::Gravity>());
    coriolisWorld.addForce(std::make_unique<ballistics::external::forces::Drag>());
    coriolisWorld.addForce(std::make_unique<ballistics::external::forces::Coriolis>());

    // + spin
    ballistics::external::PhysicsWorld spinWorld;

    spinWorld.addEnvironment(std::make_unique<ballistics::external::environments::Atmosphere>(TEMPERATURE, PRESSURE));
    spinWorld.addEnvironment(std::make_unique<ballistics::external::environments::Humidity>(REL_HUMIDITY));
    spinWorld.addEnvironment(std::make_unique<ballistics::external::environments::Geographic>(math::deg2rad(LATITUDE), math::deg2rad(LONGITUDE)));
    spinWorld.addForce(std::make_unique<ballistics::external::forces::Gravity>());
    spinWorld.addForce(std::make_unique<ballistics::external::forces::Drag>());
    spinWorld.addForce(std::make_unique<ballistics::external::forces::Coriolis>());
    ballistics::external::forces::SpinDrift::addTo(spinWorld);

    struct ConfigEntry {
        const char* name;
        ballistics::external::PhysicsWorld* world;
    };

    std::vector<ConfigEntry> configs = {
        {"gravity", &gravityWorld},
        {"+drag", &dragWorld},
        {"+coriolis", &coriolisWorld},
        {"+spin", &spinWorld},
    };

    struct IntegratorEntry {
        const char* name;
        math::IIntegrator* integrator;
    };

    math::EulerIntegrator euler;
    math::MidpointIntegrator midpoint;
    math::RK4Integrator rk4;

    IntegratorEntry integrators[] = {
        {"euler", &euler},
        {"midpoint", &midpoint},
        {"rk4", &rk4},
    };

    std::mt19937 rng(12345);

    for (auto& integrator : integrators)
    {
        std::string filename = std::string(integrator.name) + ".csv";
        std::ofstream file(filename);
        file << "config,rep,steps,avg_step_ns\n";

        for (int rep = 0; rep < REPS; ++rep)
        {
            std::shuffle(configs.begin(), configs.end(), rng);

            for (auto& c : configs)
                runConfig(c.name, *c.world, *integrator.integrator, rep, file);
        }

        std::cout << "done " << filename << "\n";
    }

    return 0;
}
