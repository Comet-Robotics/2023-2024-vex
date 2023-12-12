#include "main.h"
#include "constants.h"
#include <iostream>

#include "subsystems/catapult.h"
#include "subsystems/drivebase.h"
#include "subsystems/intake.h"

enum class AutonModes
{
	SQUARE,
	PATHS,
	NONE,
};

static AutonModes selectedAuton = AutonModes::NONE;
static std::unique_ptr<Drivebase> drivebase;
static std::unique_ptr<Catapult> catapult;
static std::unique_ptr<Intake> intake;

static inline auto auton_mode_to_string(AutonModes mode) -> std::string
{
	switch (mode)
	{
	case AutonModes::SQUARE:
		return "square";
	case AutonModes::PATHS:
		return "square";
	case AutonModes::NONE:
		return "none";
	}
	// unreachable
	abort();
}

void autonSelectorWatcher()
{
	// this can get mucky if two buttons are pressed at the same time. this does not matter tbh
	const uint8_t buttons = pros::lcd::read_buttons();
	if (buttons == 0)
	{
		return;
	}

	if (buttons & LCD_BTN_LEFT)
	{
		selectedAuton = AutonModes::SQUARE;
	}
	else if (buttons & LCD_BTN_CENTER)
	{
		selectedAuton = AutonModes::PATHS;
	}
	else if (buttons & LCD_BTN_RIGHT)
	{
		selectedAuton = AutonModes::NONE;
	}
	else
	{
		abort();
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Comet Robotics VEX-U!");

	catapult = std::make_unique<Catapult>();
	drivebase = std::make_unique<Drivebase>();
	intake = std::make_unique<Intake>();

	for (const comets::path_plan &plan : constants::PATHS)
	{
		drivebase->generatePath(plan.points, std::string(plan.name));
	}

	pros::Task autonSelectorWatcher_task(autonSelectorWatcher);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
	const auto mode_name = auton_mode_to_string(selectedAuton);
	printf("Starting autonomous routine. (%s)\n", mode_name.c_str());

	auto chassis = drivebase->get_chassis();

	switch (selectedAuton)
	{
	case AutonModes::SQUARE:
	{
		double oldMaxVel = chassis->getMaxVelocity();
		chassis->setMaxVelocity(125.0);		   // affects paths
		drivebase->driveToPoint({1_ft, 1_ft}); // assume starting position of {0, 0, 0} // TODO: figure out what this does
		for (int i = 0; i < 4; i++)
		{
			drivebase->moveDistance(2_ft);
			printf("Finished driving for iter %d\n", i);
			drivebase->turnAngle(90_deg);
			printf("Finished turning for iter %d\n", i);
		}
		chassis->setMaxVelocity(oldMaxVel);
	}
	break;
	case AutonModes::PATHS:
	{
		drivebase->setTarget("right_turn");
		drivebase->waitUntilSettled();
		drivebase->turnAngle(-90_deg);
		drivebase->setTarget("straight");
		drivebase->waitUntilSettled();
		drivebase->setTarget("strafe_right");
		drivebase->waitUntilSettled();
	}
	break;
	case AutonModes::NONE:
	{
	}
	break;
	}

	printf("Done with autonomous routine. (%s)\n", mode_name.c_str());
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	Controller controller;

	while (true)
	{
		pros::lcd::print(0, "Battery: %2.3f V", pros::battery::get_voltage() / 1000.0f);
		pros::lcd::print(1, "arm pos %2.3f deg", catapult->get_motor().getPosition());

		catapult->periodic();

		const auto state = drivebase->get_state();
		// std::printf("%0.2f %0.2f %0.2f\n", state.x.convert(inch), state.y.convert(inch), state.theta.convert(degree));

		if constexpr (constants::USE_TANK)
		{
			drivebase->tank(
				controller.getAnalog(ControllerAnalog::leftY),
				controller.getAnalog(ControllerAnalog::rightY));
		}
		else
		{
			drivebase->arcade(
				controller.getAnalog(ControllerAnalog::leftY),
				controller.getAnalog(ControllerAnalog::rightX));
		}
		if (controller.getDigital(ControllerDigital::R2))
		{
			catapult->fire();
		}
		if (controller.getDigital(ControllerDigital::R1))
		{
			catapult->wind_back();
		}
		if (controller.getDigital(ControllerDigital::B))
		{
			catapult->zero_position();
		}
		if (controller.getDigital(ControllerDigital::X))
		{
			intake->start();
		}
		else
		{
			intake->stop();
		}

		pros::delay(constants::TELEOP_POLL_TIME);
	}
}
