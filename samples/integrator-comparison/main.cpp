/*
 * main.cpp
 */

// BulletPhysics
#include "math/Integrator.h"
#include "builtin/bodies/RigidBody.h"
#include "ballistics/external/PhysicsWorld.h"
#include "ballistics/external/forces/Gravity.h"
#include "ballistics/external/forces/Force.h"
#include "ballistics/external/PhysicsContext.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <chrono>
#include <algorithm>
#include <random>
#include <cstdint>

using namespace BulletPhysics;
using namespace BulletPhysics::math;
using namespace BulletPhysics::builtin::bodies;
using namespace BulletPhysics::ballistics::external;
using namespace BulletPhysics::ballistics::external::forces;

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
class LinearDrag : public IForce
{
public:
    void apply(IPhysicsBody& body, PhysicsContext& /*context*/) override
    {
        body.addForce(-K * body.getVelocity());
    }

    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Linear Drag";
    std::string m_symbol = "Fd";
};

struct Point
{
    double x, y;
};

static Point analytical(double t)
{
    double km = K / MASS;
    double ekt = std::exp(-km * t);

    double x = (INIT_VX / K) * (1.0 - ekt);
    double y = (INIT_VY / K + G * MASS / (K * K)) * (1.0 - ekt) - (G * MASS / K) * t;
    return {x, y};
}

struct SimResult
{
    std::vector<Point> points;
    std::vector<long long> stepTimesNs;
};

// crude cache thrash
static inline void thrashCache()
{
    static std::vector<std::uint8_t> buf(64 * 1024 * 1024, 1); // 64 MB
    volatile std::uint64_t sum = 0;
    for (size_t i = 0; i < buf.size(); i += 64) sum += buf[i];
    (void)sum;
}

static void initWorldAndBody(PhysicsWorld& world, RigidBody& body)
{
    world = PhysicsWorld{};
    world.addForce(std::make_unique<Gravity>());
    world.addForce(std::make_unique<LinearDrag>());

    body = RigidBody{};
    body.setMass(MASS);
    body.setPosition({INIT_X, INIT_Y, 0.0});
    body.setVelocity({INIT_VX, INIT_VY, 0.0});
}

static SimResult simulateOne(IIntegrator& integrator, bool measure)
{
    PhysicsWorld world;
    RigidBody body;
    initWorldAndBody(world, body);

    // warmup: do a few steps to stabilize branches / icache for THIS integrator+world
    for (int i = 0; i < WARMUP_STEPS; ++i)
        integrator.step(body, &world, DT);

    // reset to identical initial state for actual simulation
    initWorldAndBody(world, body);

    std::vector<Point> points;
    std::vector<long long> times;

    while (true)
    {
        auto pos = body.getPosition();
        points.push_back({pos.x, pos.y});

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

static double avgNs(const std::vector<long long>& v)
{
    if (v.empty())
        return 0.0;

    long double s = 0.0;

    for (auto x : v)
        s += (long double)x;

    return (double)(s / (long double)v.size());
}

// ./IntegratorComparison > trajectory.csv 2> time.csv

int main()
{
    EulerIntegrator euler;
    MidpointIntegrator midpoint;
    RK4Integrator rk4;

    struct Entry { int id; const char* name; IIntegrator* integ; };
    std::vector<Entry> entries = {
        {0, "euler", &euler},
        {1, "midpoint", &midpoint},
        {2, "rk4", &rk4},
    };

    // reference runs for trajectory export (no timing)
    thrashCache();
    auto eulerRef = simulateOne(euler, false);
    thrashCache();
    auto midpointRef = simulateOne(midpoint, false);
    thrashCache();
    auto rk4Ref = simulateOne(rk4, false);

    std::vector<double> euler_avgs, midpoint_avgs, rk4_avgs;
    euler_avgs.reserve(REPS);
    midpoint_avgs.reserve(REPS);
    rk4_avgs.reserve(REPS);

    std::mt19937 rng(12345);

    for (int rep = 0; rep < REPS; ++rep)
    {
        std::shuffle(entries.begin(), entries.end(), rng);

        for (auto& e : entries)
        {
            thrashCache();
            auto res = simulateOne(*e.integ, true);
            double a = avgNs(res.stepTimesNs);

            switch (e.id)
            {
                case 0: euler_avgs.push_back(a); break;
                case 1: midpoint_avgs.push_back(a); break;
                case 2: rk4_avgs.push_back(a); break;
                default: break;
            }
        }
    }

    std::cout << "euler_x,euler_y,midpoint_x,midpoint_y,rk4_x,rk4_y,analytical_x,analytical_y\n";
    std::cout << std::fixed << std::setprecision(8);

    int steps = static_cast<int>(std::min({eulerRef.points.size(), midpointRef.points.size(), rk4Ref.points.size()}));
    for (int i = 0; i < steps; ++i)
    {
        double t = i * DT;
        auto an = analytical(t);

        std::cout << eulerRef.points[i].x    << "," << eulerRef.points[i].y    << ","
                  << midpointRef.points[i].x << "," << midpointRef.points[i].y << ","
                  << rk4Ref.points[i].x      << "," << rk4Ref.points[i].y      << ","
                  << an.x << "," << an.y << "\n";
    }

    std::cerr << "integrator,rep,avg_step_ns\n";
    for (int i = 0; i < (int)euler_avgs.size(); ++i)
        std::cerr << "euler," << i << "," << euler_avgs[i] << "\n";
    for (int i = 0; i < (int)midpoint_avgs.size(); ++i)
        std::cerr << "midpoint," << i << "," << midpoint_avgs[i] << "\n";
    for (int i = 0; i < (int)rk4_avgs.size(); ++i)
        std::cerr << "rk4," << i << "," << rk4_avgs[i] << "\n";

    return 0;
}
