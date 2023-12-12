#include "subsystems/intake.h"
#include "comets/math.h"

using namespace constants::intake;

Intake::Intake() : m_motors({LEFT_PORT, RIGHT_PORT})
{
}

void Intake::start() noexcept
{
    m_motors.moveVelocity(static_cast<int>(MOTOR_GEARSET));
}

void Intake::stop() noexcept
{
    m_motors.moveVelocity(0);
}

bool Intake::is_running() const noexcept
{
    return std::abs(m_motors.getTargetVelocity()) > 0;
}
