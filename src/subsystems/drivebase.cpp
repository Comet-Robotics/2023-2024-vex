#include "subsystems/drivebase.h"
#include "constants.h"

Drivebase::Drivebase()
{
    okapi::MotorGroup mgroup_l{{constants::FL_PORT, constants::BL_PORT}};
    okapi::MotorGroup mgroup_r{{constants::FR_PORT, constants::BR_PORT}};

    mgroup_l.setReversed(constants::LEFT_REVERSED);
    mgroup_r.setReversed(constants::RIGHT_REVERSED);

    chassis =
        okapi::ChassisControllerBuilder()
            .withMotors(mgroup_l, mgroup_r)
            .withDimensions(constants::CHASSIS_GEARSET, {constants::CHASSIS_DIMS, constants::CHASSIS_TPR})
            .withOdometry()
            .buildOdometry();

    profile_controller =
        okapi::AsyncMotionProfileControllerBuilder()
            .withLimits(constants::PATH_LIMITS)
            .withOutput(chassis)
            .buildMotionProfileController();
}

void Drivebase::generatePath(std::initializer_list<okapi::PathfinderPoint> iwaypoints, const std::string &ipathId)
{
    profile_controller->generatePath(iwaypoints, ipathId);
}

void Drivebase::arcade(double iforwardSpeed, double iyaw, double ithreshold) {
    chassis->getModel()->arcade(iforwardSpeed, iyaw, ithreshold);
}