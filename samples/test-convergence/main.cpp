/*
 * main.cpp
 */

// std
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

// BulletPhysics
#include "math/Integrator.h"
#include "builtin/bodies/RigidBody.h"
#include "ballistics/external/PhysicsWorld.h"
#include "ballistics/external/forces/Gravity.h"
#include "ballistics/external/forces/Force.h"
#include "geography/CoordinateMapping.h"

using namespace BulletPhysics;

// exit file
static constexpr std::string_view FILE_NAME = "convergence.csv";

// simulation parameters
static constexpr double MASS = 1.0;
static constexpr double INIT_VX = 20.0;
static constexpr double INIT_VY = 10.0;
static constexpr double G = 9.80665;
static constexpr double K = 2.0;

// time steps
static constexpr double DT = 1.0;
static const std::vector<double> DTS = {
    0.04,
    0.02,
    0.01,
    0.005,
    0.0025,
    0.00125,
    0.000625,
    0.0003125
};

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

static math::Vec3 simulate(math::IIntegrator& integrator, double dt)
{
    ballistics::external::PhysicsWorld world;

    world.addForce(std::make_unique<ballistics::external::forces::Gravity>());
    world.addForce(std::make_unique<LinearDrag>());

    builtin::bodies::RigidBody body;
    body.setMass(MASS);
    body.setPosition({0.0, 0.0, 0.0});
    body.setVelocity({INIT_VX, INIT_VY, 0.0});

    double t = 0.0;
    while (t < DT)
    {
        double h = std::min(dt, DT - t);
        integrator.step(body, &world, h);
        t += h;
    }

    return body.getPosition();
}

static double error(const math::Vec3& result, const math::Vec3& reference)
{
    double dx = result.x - reference.x;
    double dy = result.y - reference.y;
    double dz = result.z - reference.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int main()
{
    geography::CoordinateMapping::set(geography::mappings::OpenGL());

    math::Vec3 reference = analytical(DT);

    std::ofstream file((FILE_NAME.data()));
    file << "dt,method,error\n";

    for (double dt : DTS)
    {
        math::EulerIntegrator euler;
        math::MidpointIntegrator midpoint;
        math::RK4Integrator rk4;

        file << dt << ",euler," << error(simulate(euler, dt), reference) << "\n";
        file << dt << ",midpoint," << error(simulate(midpoint, dt), reference) << "\n";
        file << dt << ",rk4," << error(simulate(rk4, dt), reference) << "\n";
    }

    std::cout << "done " << FILE_NAME <<"\n";

    return 0;
}
