/*
 * main.cpp
 */

// std
#include <iostream>
#include <cmath>
#include <vector>

// BulletPhysics
#include "math/Integrator.h"
#include "dynamics/PhysicsBody.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/forces/Gravity.h"
#include "dynamics/forces/Force.h"

using namespace BulletPhysics;
using namespace BulletPhysics::math;
using namespace BulletPhysics::dynamics;
using namespace BulletPhysics::dynamics::forces;

// simulation parameters
static constexpr double MASS = 1.0;
static constexpr double INIT_VX = 20.0;
static constexpr double INIT_VY = 10.0;
static constexpr double G = 9.80665;
static constexpr double K = 2.0;

static constexpr double T = 1.0;

static const std::vector<double> TEST_DT = {
    0.04,
    0.02,
    0.01,
    0.005,
    0.0025,
    0.00125,
    0.000625,
};

// linear drag: F = -k * v
class LinearDrag : public IForce
{
public:
    void apply(IPhysicsBody& body, PhysicsContext& /*context*/, double /*dt*/) override
    {
        body.addForce(-K * body.getVelocity());
    }

    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Linear Drag";
    std::string m_symbol = "Fd";
};

// analytical solution for gravity + linear drag
static Vec3 analytical(double t)
{
    double km = K / MASS;
    double ekt = std::exp(-km * t);

    double x = (INIT_VX / K) * (1.0 - ekt);
    double y = (INIT_VY / K + G * MASS / (K * K)) * (1.0 - ekt) - (G * MASS / K) * t;
    return {x, y, 0.0};
}

static Vec3 simulate(IIntegrator& integrator, double dt)
{
    PhysicsWorld world;
    world.addForce(std::make_unique<Gravity>());
    world.addForce(std::make_unique<LinearDrag>());

    RigidBody body;
    body.setMass(MASS);
    body.setPosition({0.0, 0.0, 0.0});
    body.setVelocity({INIT_VX, INIT_VY, 0.0});

    double t = 0.0;
    while (t < T) {
        double h = std::min(dt, T - t);
        integrator.step(body, &world, h);
        t += h;
    }

    return body.getPosition();
}

// ./ConvergenceTest > convergence.csv

int main()
{
    Vec3 ref = analytical(T);

    auto error = [&](const Vec3& p) {
        double dx = p.x - ref.x;
        double dy = p.y - ref.y;
        double dz = p.z - ref.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    std::cout << "dt,method,error\n";

    for (double dt : TEST_DT)
    {
        EulerIntegrator euler;
        MidpointIntegrator midpoint;
        RK4Integrator rk4;

        double eEuler = error(simulate(euler, dt));
        double eMidpoint = error(simulate(midpoint, dt));
        double eRK4 = error(simulate(rk4, dt));

        std::cout << dt << ",Euler," << eEuler << "\n";
        std::cout << dt << ",Midpoint," << eMidpoint << "\n";
        std::cout << dt << ",RK4," << eRK4 << "\n";
    }

    return 0;
}
