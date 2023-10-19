#include "subsystems/catapult.h"

#include <cassert>
#include "constants.h"
#include "comets/math.h"

static inline constexpr double IDLE_VELOCITY_ERROR_RANGE = 10.0;
static inline constexpr double IDLE_POSITION_ERROR_RANGE = 10.0;

namespace arm = constants::catapult;

Catapult::Catapult() : m_motor(arm::PORT)
{
    m_motor.setPosPID(arm::POS_PIDF.F, arm::POS_PIDF.P, arm::POS_PIDF.I, arm::POS_PIDF.D);
    m_motor.setVelPID(arm::VEL_PIDF.F, arm::VEL_PIDF.P, arm::VEL_PIDF.I, arm::VEL_PIDF.D);
    m_motor.setReversed(arm::REVERSED);
    zero_position();
}

void Catapult::zero_position()
{
    m_motor.tarePosition();
}

void Catapult::wind_arm()
{
    set_position(arm::EXTENDED_POSITION);
}

void Catapult::release_arm()
{
    // although the code would suggest that the release is the same speed as winding
    // but the real robot has elastic bands pulling it torwards the stored position.
    set_position(arm::STORED_POSITION);
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
