/*
 * main.cpp
 */

// std
#include <iostream>
#include <iomanip>

// BulletPhysics
#include "math/Integrator.h"
#include "math/Angles.h"
#include "builtin/bodies/RigidBody.h"
#include "ballistics/external/PhysicsWorld.h"
#include "ballistics/external/environments/Atmosphere.h"
#include "ballistics/external/environments/Geographic.h"
#include "ballistics/external/environments/Humidity.h"
#include "ballistics/external/environments/Wind.h"
#include "ballistics/external/forces/Gravity.h"
#include "ballistics/external/forces/Coriolis.h"
#include "ballistics/external/forces/SpinDrift.h"
#include "ballistics/external/forces/drag/Drag.h"
#include "geography/CoordinateMapping.h"

using namespace BulletPhysics;

// unit conversion
static constexpr double inchToMeter(double in) { return in / 39.3700787; }
static constexpr double meterToInch(double m) { return m * 39.3700787; }
static constexpr double grainToKilogram(double gr) { return gr / 15432.358; }
static constexpr double feetPerSecondToMeterPerSecond(double fps) { return fps / 3.28084; }
static constexpr double meterPerSecondToMilesPerHour(double mps) { return mps * 2.23693629; }
static constexpr double kelvinToFahrenheit(double k) { return (k - 273.15) * 9.0 / 5.0 + 32.0; }
static constexpr double pascalToInchesHg(double pa) { return pa / 3386.389; }
static constexpr double milesPerHourToMeterPerSecond(double mph) { return mph / 2.23693629; }
static constexpr double fahrenheitToKelvin(double f) { return (f - 32.0) * 5.0 / 9.0 + 273.15; }
static constexpr double inchesHgToPascal(double inHg) { return inHg * 3386.389; }
static constexpr double inchesPerTurnToCalibersPerTurn(double twistInchesPerTurn, double caliberInches) { return twistInchesPerTurn / caliberInches; }

static const char* dragFunctionName(ballistics::external::forces::drag::DragCurveModel model)
{
    switch (model)
    {
        case ballistics::external::forces::drag::DragCurveModel::G1: return "G1";
        case ballistics::external::forces::drag::DragCurveModel::G7: return "G7";
        default: return "Other";
    }
}

// projectile parameters
static constexpr double CALIBER = 0.308;            // in
static constexpr double BULLET_WEIGHT = 168.0;      // gr
static constexpr double MUZZLE_VELOCITY = 2650.0;   // ft/s

static constexpr double BARREL_TWIST = 12.0;
static constexpr projectile::Direction TWIST_DIRECTION = projectile::Direction::RIGHT;
static constexpr ballistics::external::forces::drag::DragCurveModel DRAG_FUNCTION = ballistics::external::forces::drag::DragCurveModel::G7;

// tuned to match
static constexpr double DRAG_SCALE_FACTOR = 1.168;
static constexpr double OVERTUNING_COEFFICIENT = 4.0;
static constexpr double LIFT_COEFFICIENT = 2.88;
static constexpr double MAGNUS_COEFFICIENT = -0.4;

// launch parameters
static constexpr double LAUNCH_ELEVATION = 0.0;         // deg
static constexpr double LAUNCH_AZIMUTH = 90.0;          // deg

// world parameters
static constexpr double TEMPERATURE = 59;           // F
static constexpr double PRESSURE = 29.92;           // in Hg
static constexpr double REL_HUMIDITY = 50.0;        // %
static constexpr double LATITUDE = 45.0;            // deg
static const math::Vec3 WIND = {-22.37, 0.0, 0.0};     // mph

// sampling parameters
static constexpr double MIN_RANGE = 0.0;                // m
static constexpr double MAX_RANGE = 1000.0;             // m
static constexpr double RANGE_INCREMENT = 50.0;         // m
static constexpr double DT = 0.0005;                    // s


int main()
{
    geography::CoordinateMapping::set(geography::mappings::OpenGL());

    // world (maximum physics)
    ballistics::external::PhysicsWorld world;
    world.addEnvironment(std::make_unique<ballistics::external::environments::Atmosphere>(fahrenheitToKelvin(TEMPERATURE), inchesHgToPascal(PRESSURE)));
    world.addEnvironment(std::make_unique<ballistics::external::environments::Humidity>(REL_HUMIDITY));
    world.addEnvironment(std::make_unique<ballistics::external::environments::Geographic>(math::deg2rad(LATITUDE), math::deg2rad(0.0)));
    world.addEnvironment(std::make_unique<ballistics::external::environments::Wind>(math::Vec3{milesPerHourToMeterPerSecond(WIND.x), milesPerHourToMeterPerSecond(WIND.y), milesPerHourToMeterPerSecond(WIND.z)}));
    world.addForce(std::make_unique<ballistics::external::forces::Gravity>());
    world.addForce(std::make_unique<ballistics::external::forces::Drag>());
    world.addForce(std::make_unique<ballistics::external::forces::Coriolis>());
    ballistics::external::forces::SpinDrift::addTo(world);

    // projectile
    auto specs = projectile::ProjectileSpecs::create(grainToKilogram(BULLET_WEIGHT), inchToMeter(CALIBER))
        .withDragModel(DRAG_FUNCTION, DRAG_SCALE_FACTOR)
        .withMuzzle(feetPerSecondToMeterPerSecond(MUZZLE_VELOCITY), TWIST_DIRECTION, inchesPerTurnToCalibersPerTurn(BARREL_TWIST, CALIBER))
        .withOvertuningCoefficient(OVERTUNING_COEFFICIENT)
        .withLiftCoefficient(LIFT_COEFFICIENT)
        .withMagnusCoefficient(MAGNUS_COEFFICIENT);

    builtin::bodies::ProjectileRigidBody body(specs);
    body.setPosition({0.0, 0.0, 0.0});
    body.setAngles(LAUNCH_ELEVATION, LAUNCH_AZIMUTH);

    // integrator
    math::RK4Integrator integrator;

    // input data block (as in JBM ballistics calculator)
    std::cout << std::fixed;
    std::cout << "Input Data\n";
    std::cout << std::setprecision(3);
    std::cout << "Caliber:           " << CALIBER << " in\n";
    std::cout << std::setprecision(2);
    std::cout << "Drag Function:     " << dragFunctionName(DRAG_FUNCTION) << "\n";
    std::cout << "Bullet Weight:     " << BULLET_WEIGHT << " gr\n";
    std::cout << "----------------------------------------------------\n";
    std::cout << "Muzzle Velocity:   " << MUZZLE_VELOCITY << " ft/s\n";
    std::cout << "----------------------------------------------------\n";
    std::cout << "Barrel Twist:      " << BARREL_TWIST << " in\n";
    std::cout << "Twist Direction:   " << (TWIST_DIRECTION == projectile::Direction::RIGHT ? "Right" : "Left") << "\n";
    std::cout << "Latitude:          " << LATITUDE << " deg\n";
    std::cout << "----------------------------------------------------\n";
    std::cout << "Wind:              (" << WIND.x << ", " << WIND.y << ", " << WIND.z << ") mph\n";
    std::cout << "Temperature:       " << TEMPERATURE << " F\n";
    std::cout << "Pressure:          " << PRESSURE << " in Hg\n";
    std::cout << std::setprecision(1);
    std::cout << "Humidity:          " << REL_HUMIDITY << " %\n";
    std::cout << "----------------------------------------------------\n";
    std::cout << std::setprecision(3);
    std::cout << "Drag Scale Factor: " << DRAG_SCALE_FACTOR << "\n";
    std::cout << "Overtuning Coef.:  " << OVERTUNING_COEFFICIENT << "\n";
    std::cout << "Lift Coef.:        " << LIFT_COEFFICIENT << "\n";
    std::cout << "Magnus Coef.:      " << MAGNUS_COEFFICIENT << "\n";
    std::cout << "\nTrajectory Table\n";

    // header
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "range_m,drop_in,windage_in,time_s\n";

    math::Vec3 origin = body.getPosition();
    double nextRange = MIN_RANGE;
    double time = 0.0;

    while (true)
    {
        math::Vec3 pos = body.getPosition();
        double range = pos.x - origin.x;

        if (range >= nextRange)
        {
            double drop = meterToInch(pos.y - origin.y);
            double drift = meterToInch(pos.z - origin.z);

            std::cout << nextRange << "," << drop << "," << drift << "," << time << "\n";

            nextRange += RANGE_INCREMENT;

            if (nextRange > MAX_RANGE)
                break;
        }

        integrator.step(body, &world, DT);
        time += DT;
    }

    return 0;
}
