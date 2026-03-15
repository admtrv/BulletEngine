/*
 * main.cpp
 */

// std
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <chrono>
#include <algorithm>
#include <random>
#include <cstdint>

// BulletPhysics
#include "math/Integrator.h"
#include "builtin/bodies/RigidBody.h"
#include "ballistics/external/PhysicsWorld.h"
#include "ballistics/external/forces/Gravity.h"
#include "ballistics/external/forces/Force.h"
#include "ballistics/external/PhysicsContext.h"

using namespace BulletPhysics;

// exit files
static constexpr std::string_view TRAJECTORY_FILE_NAME = "trajectory.csv";
static constexpr std::string_view TIMING_FILE_NAME = "timing.csv";

// simulation parameters
static constexpr double MASS = 1.0;
static constexpr double INIT_X = 0.0;
static constexpr double INIT_Y = 0.0;
static constexpr double INIT_VX = 20.0;
static constexpr double INIT_VY = 30.0;
static constexpr double G = 9.80665;
static constexpr double K = 2.0;
static constexpr double DT = 0.25;

// measurement controls
static constexpr int REPS = 15;
static constexpr int WARMUP_STEPS = 64;

// linear drag: F = -k * v
class LinearDrag : public ballistics::external::forces::IForce
{
public:
    void apply(IPhysicsBody& body, ballistics::external::PhysicsContext& /*context*/) override
    {
        body.addForce(-K * body.getVelocity());
    }

    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Linear Drag";
    std::string m_symbol = "Fd";
};

// analytical solution (gravity + linear drag)
static math::Vec3 analytical(double t)
{
    double km = K / MASS;
    double ekt = std::exp(-km * t);

    double x = (INIT_VX / K) * (1.0 - ekt);
    double y = (INIT_VY / K + G * MASS / (K * K)) * (1.0 - ekt) - (G * MASS / K) * t;
    return {x, y, 0.0};
}

struct SimResult
{
    std::vector<math::Vec3> points;
    std::vector<long long> stepTimesNs;
};

static void thrashCache()
{
    static std::vector<std::uint8_t> buf(64 * 1024 * 1024, 1);  // 64 MB
    volatile std::uint64_t sum = 0;
    for (size_t i = 0; i < buf.size(); i += 64)
        sum += buf[i];
    (void)sum;
}

static void init(ballistics::external::PhysicsWorld& world, builtin::bodies::RigidBody& body)
{
    world = ballistics::external::PhysicsWorld{};
    world.addForce(std::make_unique<ballistics::external::forces::Gravity>());
    world.addForce(std::make_unique<LinearDrag>());

    body = builtin::bodies::RigidBody{};
    body.setMass(MASS);
    body.setPosition({INIT_X, INIT_Y, 0.0});
    body.setVelocity({INIT_VX, INIT_VY, 0.0});
}

static SimResult simulate(math::IIntegrator& integrator, bool measure)
{
    ballistics::external::PhysicsWorld world;
    builtin::bodies::RigidBody body;
    init(world, body);

    // warmup
    for (int i = 0; i < WARMUP_STEPS; ++i)
        integrator.step(body, &world, DT);

    // reset
    init(world, body);

    std::vector<math::Vec3> points;
    std::vector<long long> times;

    while (true)
    {
        auto pos = body.getPosition();
        points.push_back(pos);

        if (pos.y <= 0.0 && points.size() > 1)
            break;

        if (measure)
        {
            auto t0 = std::chrono::high_resolution_clock::now();
            integrator.step(body, &world, DT);
            auto t1 = std::chrono::high_resolution_clock::now();
            times.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
        }
        else
        {
            integrator.step(body, &world, DT);
        }
    }

    return {points, times};
}

static double average(const std::vector<long long>& samples)
{
    if (samples.empty())
        return 0.0;

    long double sum = 0.0;

    for (auto sample : samples)
        sum += (long double)sample;

    return (double)(sum / (long double)samples.size());
}

int main()
{
    math::EulerIntegrator euler;
    math::MidpointIntegrator midpoint;
    math::RK4Integrator rk4;

    struct Entry {
        const char* name;
        math::IIntegrator* integrator;
        std::vector<double> avgs;
    };

    std::vector<Entry> entries = {
        {"euler", &euler, {}},
        {"midpoint", &midpoint, {}},
        {"rk4", &rk4, {}},
    };

    // reference runs
    thrashCache();
    auto eulerRef = simulate(euler, false);

    thrashCache();
    auto midpointRef = simulate(midpoint, false);

    thrashCache();
    auto rk4Ref = simulate(rk4, false);

    std::mt19937 rng(12345);

    for (int rep = 0; rep < REPS; ++rep)
    {
        std::shuffle(entries.begin(), entries.end(), rng);

        for (auto& entry : entries)
        {
            thrashCache();
            auto res = simulate(*entry.integrator, true);
            entry.avgs.push_back(average(res.stepTimesNs));
        }
    }

    // trajectory
    {
        std::ofstream file(TRAJECTORY_FILE_NAME.data());
        file << std::fixed << std::setprecision(8);
        file << "euler_x,euler_y,midpoint_x,midpoint_y,rk4_x,rk4_y,analytical_x,analytical_y\n";

        size_t steps = std::min({eulerRef.points.size(), midpointRef.points.size(), rk4Ref.points.size()});
        for (size_t i = 0; i < steps; ++i)
        {
            double t = i * DT;
            auto an = analytical(t);

            file << eulerRef.points[i].x    << "," << eulerRef.points[i].y    << ","
                 << midpointRef.points[i].x << "," << midpointRef.points[i].y << ","
                 << rk4Ref.points[i].x      << "," << rk4Ref.points[i].y      << ","
                 << an.x << "," << an.y << "\n";
        }

        std::cout << "done " << TRAJECTORY_FILE_NAME << "\n";
    }

    // timing
    {
        std::ofstream file(TIMING_FILE_NAME.data());
        file << "integrator,rep,avg_step_ns\n";

        for (const auto& entry : entries)
        {
            for (int i = 0; i < (int)entry.avgs.size(); ++i)
                file << entry.name << "," << i << "," << entry.avgs[i] << "\n";
        }

        std::cout << "done " << TIMING_FILE_NAME << "\n";
    }

    return 0;
}
