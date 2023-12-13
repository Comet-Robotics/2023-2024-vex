#include "subsystems/intake.h"
#include "comets/math.h"

using namespace constants::intake;

Intake::Intake() : m_motorsLeft({LEFT_PORT}), m_motorsRight({RIGHT_PORT})
{
}

void Intake::setVelocity(int velocity) noexcept
{
    m_motorsLeft.moveVelocity(velocity);
    m_motorsRight.moveVelocity(-velocity);
}

void Intake::start() noexcept
{
    setVelocity(static_cast<int>(MOTOR_GEARSET));
}

void Intake::reverse() noexcept
{
    setVelocity(-static_cast<int>(MOTOR_GEARSET));
}

void Intake::stop() noexcept
{
    setVelocity(0);
}

bool Intake::is_running() const noexcept
{
    // Assume left and right groups will both be set to zero if one is set to zero
    return std::abs(m_motorsLeft.getTargetVelocity()) > 0;
}
