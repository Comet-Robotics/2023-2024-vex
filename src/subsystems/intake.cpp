#include "subsystems/intake.h"
#include "comets/math.h"

Intake::Intake() : m_motors({constants::intake::LEFT_PORT, constants::intake::RIGHT_PORT})
{
}

void Intake::start() noexcept
{
    m_motors.moveVelocity(static_cast<int>(constants::intake::MOTOR_GEARSET));
}

void Intake::stop() noexcept
{
    m_motors.moveVelocity(0);
}

bool Intake::is_running() const noexcept
{
    return std::abs(m_motors.getTargetVelocity()) > 0;
}
