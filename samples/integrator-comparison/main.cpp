/*
 * main.cpp
 */

// BulletPhysics
#include "math/Integrator.h"
#include "dynamics/PhysicsBody.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/forces/Gravity.h"
#include "dynamics/forces/Force.h"
#include "dynamics/PhysicsContext.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <chrono>

using namespace BulletPhysics;
using namespace BulletPhysics::math;
using namespace BulletPhysics::dynamics;
using namespace BulletPhysics::dynamics::forces;

// simulation parameters
static constexpr float MASS = 1.0f;
static constexpr float INIT_X = 0.0f;
static constexpr float INIT_Y = 0.0f;
static constexpr float INIT_VX = 20.0f;
static constexpr float INIT_VY = 30.0f;
static constexpr float G = 9.80665f;
static constexpr float K = 2.0f;
static constexpr float DT = 0.25f;

// linear drag: F = -k * v
class LinearDrag : public IForce
{
public:
    void apply(IPhysicsBody& body, PhysicsContext& /*context*/, float /*dt*/) override
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
    float x, y;
};

// analytical solution with linear drag and gravity
static Point analytical(float t)
{
    float km = K / MASS;
    float ekt = std::exp(-km * t);

    float x = (INIT_VX / K) * (1.0f - ekt);
    float y = (INIT_VY / K + G * MASS / (K * K)) * (1.0f - ekt) - (G * MASS / K) * t;

    return {x, y};
}

struct SimResult
{
    std::vector<Point> points;
    std::vector<long long> stepTimesNs;
};

static SimResult simulate(IIntegrator& integrator)
{
    PhysicsWorld world;
    world.addForce(std::make_unique<Gravity>());
    world.addForce(std::make_unique<LinearDrag>());

    RigidBody body;
    body.setMass(MASS);
    body.setPosition({INIT_X, INIT_Y, 0.0f});
    body.setVelocity({INIT_VX, INIT_VY, 0.0f});

    std::vector<Point> points;
    std::vector<long long> times;

    while (true)
    {
        auto pos = body.getPosition();
        points.push_back({pos.x, pos.y});

        if (pos.y <= 0.0f && points.size() > 1)
            break;

        auto t0 = std::chrono::high_resolution_clock::now();
        integrator.step(body, &world, DT);
        auto t1 = std::chrono::high_resolution_clock::now();

        times.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    }

    return {points, times};
}

// ./IntegratorComparison > data.csv

int main()
{
    EulerIntegrator euler;
    MidpointIntegrator midpoint;
    RK4Integrator rk4;

    auto eulerRes = simulate(euler);
    auto midpointRes = simulate(midpoint);
    auto rk4Res = simulate(rk4);

    std::cout << "euler_x,euler_y,midpoint_x,midpoint_y,rk4_x,rk4_y,analytical_x,analytical_y,euler_step_ns,midpoint_step_ns,rk4_step_ns\n";
    std::cout << std::fixed << std::setprecision(8);

    int steps = static_cast<int>(std::min({eulerRes.points.size(), midpointRes.points.size(), rk4Res.points.size()}));
    for (int i = 0; i < steps; ++i)
    {
        float t = i * DT;
        auto an = analytical(t);

        long long etns = i < (int)eulerRes.stepTimesNs.size() ? eulerRes.stepTimesNs[i] : 0;
        long long mtns = i < (int)midpointRes.stepTimesNs.size() ? midpointRes.stepTimesNs[i] : 0;
        long long rtns = i < (int)rk4Res.stepTimesNs.size() ? rk4Res.stepTimesNs[i] : 0;

        std::cout << eulerRes.points[i].x    << "," << eulerRes.points[i].y    << ","
                  << midpointRes.points[i].x << "," << midpointRes.points[i].y << ","
                  << rk4Res.points[i].x      << "," << rk4Res.points[i].y      << ","
                  << an.x << "," << an.y << ","
                  << etns << "," << mtns << "," << rtns << "\n";
    }

    return 0;
}
