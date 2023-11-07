#include "subsystems/catapult.h"

#include <cassert>
#include "constants.h"
#include "comets/math.h"

static inline constexpr double IDLE_VELOCITY_ERROR_RANGE = 10.0;
static inline constexpr double IDLE_POSITION_ERROR_RANGE = 10.0;

namespace arm = constants::catapult;

static double get_next_nearest_position(double curr, double target);
static double remap_360_to_ps180(double curr);

Catapult::Catapult() : m_motor(arm::PORT)
{
    // m_motor.setPosPID(arm::POS_PIDF.F, arm::POS_PIDF.P, arm::POS_PIDF.I, arm::POS_PIDF.D);
    // m_motor.setVelPID(arm::VEL_PIDF.F, arm::VEL_PIDF.P, arm::VEL_PIDF.I, arm::VEL_PIDF.D);
    m_motor.setReversed(arm::REVERSED);
    m_motor.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
    zero_position();
}

void Catapult::zero_position()
{
    m_motor.tarePosition();
}

double Catapult::get_position()
{
    return m_motor.getPosition(); // / static_cast<double>(constants::catapult::MOTOR_GEARSET);
}

void Catapult::wind_back()
{
    const auto curr_pos = get_position();
    if (comets::in_range((fmod(curr_pos, 360)), -arm::TOLERANCE, arm::TOLERANCE))
    {
        std::printf("at zero.");
        return;
    }

    double nearestPosition = get_next_nearest_position(curr_pos, 0);
    std::printf("pos curr %f ; near %f\n", curr_pos, nearestPosition);
    while ((nearestPosition+15) > get_position())
        m_motor.moveVelocity(50);
    std::printf("done winding.\n");
}

void Catapult::fire()
{
    //m_motor.moveRelative(360, 200);
    m_motor.moveVelocity(80);
}

void Catapult::stop()
{
    m_motor.moveVelocity(0);
}

bool Catapult::is_motor_idle() noexcept
{
    const double v_error = m_motor.getVelocityError();
    const double p_error = m_motor.getPositionError();
    static constexpr auto in_range = [](double target, double range)
    {
        return comets::in_range(target, -range / 2, range / 2);
    };
    return in_range(v_error, IDLE_VELOCITY_ERROR_RANGE) &&
           in_range(p_error, IDLE_POSITION_ERROR_RANGE);
}

void Catapult::set_position(double position)
{
    m_motor.moveAbsolute(position, 400);
}

static double get_next_nearest_position(double curr, double target)
{
    const double remainder = 360.0 - fmod(curr, 360.0);
    const double next_zero = curr + remainder;
    const double new_target = next_zero + target;
    assert(new_target > target);
    return new_target;
}

static double remap_360_to_ps180(double curr)
{
    return curr - 180.0;
}