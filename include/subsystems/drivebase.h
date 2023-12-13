#ifndef __SUBSYSTEMS_DRIVEBASE_H__
#define __SUBSYSTEMS_DRIVEBASE_H__

#include "comets/vendor.h"
#include "constants.h"
#include <memory>

class Drivebase
{
public:
    Drivebase();

    void generatePath(std::initializer_list<okapi::PathfinderPoint> iwaypoints, const std::string &ipathId);
    void arcade(double iforwardSpeed, double iyaw, double ithreshold = 0);
    void tank(double left, double right, double threshold = 0);

    inline auto get_state() noexcept
    {
        return chassis->getState();
    }

    inline auto &get_profile_controller() noexcept
    {
        return profile_controller;
    }
    inline auto &get_chassis() noexcept
    {
        return chassis;
    }

    inline void setTarget(const std::string &target)
    {
        profile_controller->setTarget(target);
    }

    inline void waitUntilSettled()
    {
        profile_controller->waitUntilSettled();
    }

    inline void turnAngle(okapi::QAngle angle)
    {
        const double oldMaxVel = chassis->getMaxVelocity();
        // chassis->setMaxVelocity(oldMaxVel * constants::TURN_VEL_MULT);
        chassis->setMaxVelocity(45);
        chassis->turnAngle(angle);
        chassis->setMaxVelocity(oldMaxVel);
    }

    inline void turnAngleGyro(okapi::QAngle angle)
    {
        targetPositionVelocity = { angle,
                                   90 }
    }

    inline void moveDistance(okapi::QLength length)
    {
        chassis->moveDistance(length);
    }

    inline void driveToPoint(const okapi::Point &ipoint, bool ibackwards = false, const okapi::QLength &ioffset = okapi::QLength(0.0))
    {
        chassis->driveToPoint(ipoint, ibackwards, ioffset);
    }

    inline QAngle get_angle()
    {
        return imu.get();
    }

    inline void periodic()
    {
        if (turningToAngle)
        {
            angleController.setTarget(targetPositionVelocity.first.getValue());
            if (targetPositionVelocity.first > get_angle())
            {
                turn(targetPositionVelocity.second);
            }
            else
            {
                m_motor.moveVelocity(0);
                turningToAngle = false;
                std::printf("done moving to position %f.\n", targetPositionVelocity.first);
            }
        }
    }

private:
    std::shared_ptr<okapi::OdomChassisController> chassis;
    std::shared_ptr<okapi::AsyncMotionProfileController> profile_controller;
    std::shared_ptr<okapi::IMU> imu;
    bool turningToAngle;
    std::pair<QAngle, int16_t> targetPositionVelocity;
    okapi::IterativePosPIDController angleController;
};

#endif
