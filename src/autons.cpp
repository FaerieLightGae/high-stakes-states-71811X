#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(8, 0.075, 20);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(1.0, 0, 2, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(8.5, 0.0, 69.5);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.3, 52.0);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.3, 32.0);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(45_ms, 3_deg, 125_ms, 7_deg, 125_ms, 250_ms);
  chassis.pid_swing_exit_condition_set(45_ms, 3_deg, 125_ms, 7_deg, 250_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(45_ms, 1_in, 125_ms, 3_in, 250_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(45_ms, 3_deg, 125_ms, 7_deg, 250_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(45_ms, 1_in, 125_ms, 3_in, 259_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(4_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(1);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(1000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(1000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(1000);
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

//tune 
  void tune() {
//    chassis.pid_turn_set(180, 50, true);
//    chassis.pid_drive_set(24, 50);
//    chassis.pid_swing_set(RIGHT_SWING, -90, 50, 0, true);
    //initializing odom
      chassis.pid_targets_reset();
      chassis.drive_imu_reset();
      chassis.drive_sensor_reset();
      chassis.odom_enable(false);
      //chassis.odom_enable(true);
      chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
      chassis.pid_turn_set(-90, 100);
      chassis.pid_wait();
      pros::delay(100);
      chassis.pid_turn_set(-360, 100);
      chassis.pid_wait();
      chassis.pid_turn_set(-360-90,100);
      chassis.pid_wait();
  }

//red
  //red negative
    //negative side ring rush ladder
      void RedNegativeRushLadder() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(-17);

        //rush
          chassis.pid_drive_set(40, 110);
          DoinkerPiston.set(1);
          Intake1Control(127);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(12, 70);
          chassis.pid_wait();
          pros::delay(250);

        //clamp
          chassis.pid_swing_set(LEFT_SWING, -50, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(-24, 90);
          chassis.pid_wait();
          ClampPiston.set(1);
          Intake1Control(0);

        //score first 3
          chassis.pid_turn_set(-90, 110);
          chassis.pid_wait();
          DoinkerPiston.set(0);
          pros::Task intakeTask(AutonIntakeLoopRed);
          set_intakeRED(110);

          chassis.pid_drive_set(36, 60);
          chassis.pid_wait();
          pros::delay(250);

        //score preload and mid stack
          chassis.pid_swing_set(RIGHT_SWING, -30, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(-23, 110);
          chassis.pid_wait();
          set_intakeRED(-30);

          chassis.pid_turn_set(90, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(22, 90);
          set_intakeRED(110);
          chassis.pid_wait();
          IntakeRaiser.set(1);
          pros::delay(500);

          chassis.pid_drive_set(8, 20);
          chassis.pid_wait();
          pros::delay(500);

          chassis.pid_swing_set(RIGHT_SWING, 0, 90);
          chassis.drive_brake_set(MOTOR_BRAKE_COAST);
          IntakeRaiser.set(0);
          chassis.pid_wait();

          chassis.pid_drive_set(8, 110);
          chassis.pid_wait();
      }

    //negative side ring rush corner
      void RedNegativeRushCorner() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(-17);

        //rush
          chassis.pid_drive_set(40, 110);
          DoinkerPiston.set(1);
          Intake1Control(127);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(12, 70);
          chassis.pid_wait();
          pros::delay(250);
        
        //clamp
          chassis.pid_swing_set(LEFT_SWING, -50, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(-24, 90);
          chassis.pid_wait();
          ClampPiston.set(1);
          Intake1Control(0);

        //score first 3
          chassis.pid_turn_set(-90, 110);
          chassis.pid_wait();
          DoinkerPiston.set(0);
          pros::Task intakeTask(AutonIntakeLoopRed);
          set_intakeRED(110);

          chassis.pid_drive_set(36, 60);
          chassis.pid_wait();
          pros::delay(250);

        //score preload and mid stack
          chassis.pid_swing_set(RIGHT_SWING, -30, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(-23, 110);
          chassis.pid_wait();
          set_intakeRED(-30);

          chassis.pid_turn_set(90, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(80, 60);
          set_intakeRED(110);
          chassis.drive_brake_set(MOTOR_BRAKE_COAST);
          chassis.pid_wait();
      }

    //negative side alliance ladder
      void RedNegativeAllianceLadder() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(90);
          ladybrown_auton_set(2);

        //score alliance
          chassis.pid_turn_set(133, 100);
          chassis.pid_wait();
          ladybrown_alliance();
          chassis.pid_drive_set(8, 100);
          chassis.pid_wait();

        //set up for mogo
          chassis.pid_drive_set(-33, 110);
          chassis.pid_wait();
          chassis.pid_turn_set(215, 110);
          chassis.pid_wait();
          ladybrown_down();

        //drive into mogo and clamp
          chassis.pid_drive_set(-15, 100);
          chassis.pid_wait();
          ClampPiston.set(1);

        //turn toward first ring stack and intake
          chassis.pid_turn_set(-48, 110);
          chassis.pid_wait();
          pros::Task intakeTask(AutonIntakeLoopRed);
          set_intakeRED(110);
          chassis.pid_drive_set(20, 100);
          chassis.pid_wait();
          pros::delay(250);

        //turn toward second ring stack and intake
          chassis.pid_drive_set(-20, 110);
          chassis.pid_wait();
          chassis.pid_turn_set(-90, 110);
          chassis.pid_wait();
          chassis.pid_drive_set(25, 100);
          chassis.pid_wait();

        //turn toward third ring stack and intake
          chassis.pid_turn_set(-10, 110);
          chassis.pid_wait();
          chassis.pid_drive_set(17, 100);
          chassis.pid_wait();
          pros::delay(250);

        //turn toward fourth(mid) ring stack and intake
          chassis.pid_drive_set(-25, 110);
          chassis.pid_wait();
          set_intakeRED(-30);
          chassis.pid_turn_set(101, 110);
          chassis.pid_wait();
          set_intakeRED(110);
          chassis.pid_drive_set(34, 110);
          chassis.pid_wait();
          IntakeRaiser.set(1);
          chassis.pid_drive_set(10, 40);
          chassis.pid_wait();
          pros::delay(500);

        //touch ladder
          chassis.pid_swing_set(RIGHT_SWING, 0, 110);
          chassis.drive_brake_set(MOTOR_BRAKE_COAST);
          IntakeRaiser.set(0);
          chassis.pid_wait_quick_chain();

          chassis.pid_drive_set(12, 110);
          chassis.pid_wait();
      }

    //negative side alliance corner
      void RedNegativeAllianceCorner() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(90);
          ladybrown_auton_set(2);

        //score alliance
          chassis.pid_turn_set(133, 100);
          chassis.pid_wait();
          ladybrown_alliance();
          chassis.pid_drive_set(8, 100);
          chassis.pid_wait();

        //set up for mogo
          chassis.pid_drive_set(-33, 110);
          chassis.pid_wait();
          chassis.pid_turn_set(215, 110);
          chassis.pid_wait();
          ladybrown_down();

        //drive into mogo and clamp
          chassis.pid_drive_set(-15, 100);
          chassis.pid_wait();
          ClampPiston.set(1);

        //turn toward first ring stack and intake
          chassis.pid_turn_set(-48, 110);
          chassis.pid_wait();
          pros::Task intakeTask(AutonIntakeLoopRed);
          set_intakeRED(110);
          chassis.pid_drive_set(20, 100);
          chassis.pid_wait();
          pros::delay(250);

        //turn toward second ring stack and intake
          chassis.pid_drive_set(-20, 110);
          chassis.pid_wait();
          chassis.pid_turn_set(-90, 110);
          chassis.pid_wait();
          chassis.pid_drive_set(25, 100);
          chassis.pid_wait();

        //turn toward third ring stack and intake
          chassis.pid_turn_set(-10, 110);
          chassis.pid_wait();
          chassis.pid_drive_set(17, 100);
          chassis.pid_wait();
          pros::delay(250);

        //turn toward fourth(mid) ring stack and intake
          chassis.pid_drive_set(-25, 110);
          chassis.pid_wait();
          set_intakeRED(-30);
          chassis.pid_turn_set(101, 110);
          chassis.pid_wait();
          set_intakeRED(110);
          chassis.pid_drive_set(90, 60);
          chassis.pid_wait();
      }

  //red positive
    //positive side mogo rush ladder
      void RedPositiveRushLadder() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(15);

        //rush mogo
          chassis.pid_drive_set(40, 110);
          DoinkerPiston.set(1);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(5, 30);
          chassis.pid_wait();
          pros::delay(250);

          chassis.pid_drive_set(-3, 30);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(-15, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(6, 110);
          chassis.pid_wait_quick_chain();

          chassis.pid_drive_set(-12, 110);
          chassis.pid_wait();
          DoinkerPiston.set(0);
        
        //intake ring stack
          chassis.pid_turn_set(50, 110);
          chassis.pid_wait();
          Intake1Control(110);

          chassis.pid_drive_set(20, 100);
          chassis.pid_wait();

        //clamp and score
          chassis.pid_turn_set(88, 110);
          chassis.pid_wait();
          Intake1Control(0);

          chassis.pid_drive_set(-26, 100);
          chassis.pid_wait();
          ClampPiston.set(1);
          pros::delay(250);

        //intake mid ring stack
          chassis.pid_turn_set(220, 110);
          pros::Task intakeTask(AutonIntakeLoopRed);
          set_intakeRED(110);
          chassis.pid_wait();

          chassis.pid_drive_set(16, 90);
          chassis.pid_wait();
          set_intakeRED(0);
          DisableAutonLoops();
          IntakeRaiser.set(1);
          intake2.brake();
          Intake1Control(110);
          chassis.pid_drive_set(8, 40);
          chassis.pid_wait();
          pros::delay(250);
          IntakeRaiser.set(0);

        //drop mogo off and clamp mid mogo
          chassis.pid_swing_set(RIGHT_SWING, -90, 110);
          chassis.pid_wait();
          
          chassis.pid_drive_set(-30, 110);
          chassis.pid_wait();
          ClampPiston.set(0);

          chassis.pid_drive_set(6, 110);
          chassis.pid_wait();
          Intake1Control(0);

          chassis.pid_turn_set(-160, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(-24, 110);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(-5, 80);
          chassis.pid_wait();
          intake2.move(110);
          ClampPiston.set(1);

          chassis.pid_swing_set(LEFT_SWING, -45, 127);
          chassis.drive_brake_set(MOTOR_BRAKE_COAST);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(10, 127);
          chassis.pid_wait();
      }

    //positive side mogo rush middle
      void RedPositiveRushMid() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(15);

        //rush mogo
          chassis.pid_drive_set(40, 110);
          DoinkerPiston.set(1);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(5, 30);
          chassis.pid_wait();
          pros::delay(250);

          chassis.pid_drive_set(-3, 30);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(-15, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(6, 110);
          chassis.pid_wait_quick_chain();

          chassis.pid_drive_set(-12, 110);
          chassis.pid_wait();
          DoinkerPiston.set(0);
        
        //intake ring stack
          chassis.pid_turn_set(50, 110);
          chassis.pid_wait();
          Intake1Control(110);

          chassis.pid_drive_set(20, 100);
          chassis.pid_wait();

        //clamp and score
          chassis.pid_turn_set(88, 110);
          chassis.pid_wait();
          Intake1Control(0);

          chassis.pid_drive_set(-26, 100);
          chassis.pid_wait();
          ClampPiston.set(1);
          pros::delay(250);

        //intake mid ring stack
          chassis.pid_turn_set(220, 110);
          pros::Task intakeTask(AutonIntakeLoopRed);
          set_intakeRED(110);
          chassis.pid_wait();

          chassis.pid_drive_set(16, 90);
          chassis.pid_wait();
          set_intakeRED(0);
          DisableAutonLoops();
          IntakeRaiser.set(1);
          intake2.brake();
          Intake1Control(110);
          chassis.pid_drive_set(8, 40);
          chassis.pid_wait();
          pros::delay(250);
          IntakeRaiser.set(0);

        //drop mogo off and clamp mid mogo
          chassis.pid_swing_set(RIGHT_SWING, -90, 110);
          chassis.pid_wait();
          
          chassis.pid_drive_set(-30, 110);
          chassis.pid_wait();
          ClampPiston.set(0);

          chassis.pid_drive_set(6, 110);
          chassis.pid_wait();
          Intake1Control(0);

          chassis.pid_turn_set(-160, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(-24, 110);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(-5, 80);
          chassis.pid_wait();
          intake2.move(110);
          ClampPiston.set(1);
      }

    //positive side alliance ladder
      void RedPositiveAllianceLadder() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(-90);
          ladybrown_auton_set(2);

        //score alliance
          chassis.pid_turn_set(-133, 115);
          chassis.pid_wait();
          ladybrown_alliance();
          chassis.pid_drive_set(9, 105);
          chassis.pid_wait();
          pros::delay(150);

        //pick up mid ring
          chassis.pid_drive_set(-18, 90);
          chassis.pid_wait();

          chassis.pid_turn_set(-90, 90);
          ladybrown_down();
          chassis.pid_wait();

          chassis.pid_drive_set(11, 90);
          chassis.pid_wait();
          IntakeRaiser.set(1);
          Intake1Control(127);
          chassis.pid_drive_set(11, 80);
          chassis.pid_wait();
          pros::delay(250);
          IntakeRaiser.set(0);

        //clamp mogo
          chassis.pid_turn_set(-140, 90);
          chassis.pid_wait();

          chassis.pid_drive_set(-40, 70);
          chassis.pid_wait();
          ClampPiston.set(1);
          Intake1Control(0);

        //score ring
          chassis.pid_turn_set(110, 90);
          chassis.pid_wait();

          pros::Task intakeTask(AutonIntakeLoopRed);
          set_intakeRED(110);
          chassis.pid_drive_set(28, 70);
          chassis.pid_wait();
          pros::delay(250);

        //touch ladder
          chassis.pid_turn_set(120, 90);
          chassis.pid_wait();

          chassis.pid_drive_set(-3, 120);
          chassis.drive_brake_set(MOTOR_BRAKE_COAST);
          chassis.pid_wait();
      }

    //positive side alliance middle
      void RedPositiveAllianceMiddle() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(-90);
          ladybrown_auton_set(2);

        //score alliance
          chassis.pid_turn_set(-133, 115);
          chassis.pid_wait();
          ladybrown_alliance();
          chassis.pid_drive_set(9, 105);
          chassis.pid_wait();
          pros::delay(150);

        //pick up mid ring
          chassis.pid_drive_set(-18, 90);
          chassis.pid_wait();

          chassis.pid_turn_set(-90, 90);
          ladybrown_down();
          chassis.pid_wait();

          chassis.pid_drive_set(11, 90);
          chassis.pid_wait();
          IntakeRaiser.set(1);
          Intake1Control(127);
          chassis.pid_drive_set(11, 80);
          chassis.pid_wait();
          pros::delay(250);
          IntakeRaiser.set(0);

        //clamp mogo
          chassis.pid_turn_set(-140, 90);
          chassis.pid_wait();

          chassis.pid_drive_set(-40, 70);
          chassis.pid_wait();
          ClampPiston.set(1);
          Intake1Control(0);

        //score ring
          chassis.pid_turn_set(110, 90);
          chassis.pid_wait();

          pros::Task intakeTask(AutonIntakeLoopRed);
          set_intakeRED(110);
          chassis.pid_drive_set(28, 70);
          chassis.pid_wait();
          pros::delay(500);

        //line up with mid mogo
          chassis.pid_turn_set(0, 90);
          chassis.pid_wait();

          chassis.pid_drive_set(-12, 90);
          chassis.pid_wait();
          ClampPiston.set(0);
          set_intakeRED(0);

          chassis.pid_drive_set(6, 90);
          chassis.pid_wait();

          chassis.pid_turn_set(-190, 90);
          chassis.pid_wait();

          chassis.pid_drive_set(-16, 70);
          chassis.pid_wait();
      }

  //red AWP
    //true SAWP
      void RedSAWP() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(90);
          ladybrown_auton_set(2);

        //score alliance
          chassis.pid_turn_set(133, 115);
          chassis.pid_wait();
          ladybrown_alliance();
          chassis.pid_drive_set(9, 105);
          chassis.pid_wait();
          pros::delay(150);

        //set up for mogo
          chassis.pid_drive_set(-33, 115);
          chassis.pid_wait_quick();
          chassis.pid_turn_set(215, 115);
          chassis.pid_wait_quick();
          ladybrown_down();

        //drive into mogo and clamp
          chassis.pid_drive_set(-15, 105);
          chassis.pid_wait();
          ClampPiston.set(1);

        //turn toward first ring stack and intake
          chassis.pid_turn_set(-90, 115);
          chassis.pid_wait_quick();
          chassis.pid_drive_set(28, 100);
          pros::Task intakeTask(AutonIntakeLoopRed);
          set_intakeRED(127);
          chassis.pid_wait();
          pros::delay(250);
        
        //cross and intake mid stack
          chassis.pid_drive_set(-28, 110);
          chassis.pid_wait();

          chassis.pid_turn_set(137, 115);
          chassis.pid_wait();

          chassis.pid_drive_set(18, 115);
          chassis.pid_wait();
          set_intakeRED(0);
          intakeTask.remove();
          ClampPiston.set(0);
          intake2.brake();
          IntakeRaiser.set(1);
          Intake1Control(127);
          chassis.pid_drive_set(11, 80);
          chassis.pid_wait();
          pros::delay(250);
          IntakeRaiser.set(0);

        //clamp next mogo
          chassis.pid_turn_set(-130, 115);
          chassis.pid_wait();

          chassis.pid_drive_set(-36, 105);
          chassis.pid_wait();
          ClampPiston.set(1);

        //score last stack
          chassis.pid_turn_set(95, 115);
          chassis.pid_wait();

          chassis.pid_drive_set(24, 100);
          ContinueAutonLoops();
          set_intakeRED(110);
          intake2.move(110);
          chassis.pid_wait();

        //touch ladder
          chassis.pid_turn_set(121, 127);
          chassis.pid_wait();
          chassis.pid_drive_set(-29.5, 120);
          chassis.drive_brake_set(MOTOR_BRAKE_COAST);
          chassis.pid_wait();
      }

//blue
  //blue negative
    //negative side ring rush ladder
      void BlueNegativeRushLadder() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(17);

        //rush
          chassis.pid_drive_set(40, 110);
          DoinkerPiston.set(1);
          Intake1Control(127);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(12, 70);
          chassis.pid_wait();
          pros::delay(250);

        //clamp
          chassis.pid_swing_set(RIGHT_SWING, 50, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(-24, 90);
          chassis.pid_wait();
          ClampPiston.set(1);
          Intake1Control(0);

        //score first 3
          chassis.pid_turn_set(90, 110);
          chassis.pid_wait();
          DoinkerPiston.set(0);
          pros::Task intakeTask(AutonIntakeLoopBlue);
          set_intakeBLUE(110);

          chassis.pid_drive_set(36, 60);
          chassis.pid_wait();
          pros::delay(250);

        //score preload and mid stack
          chassis.pid_swing_set(LEFT_SWING, 30, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(-23, 110);
          chassis.pid_wait();
          set_intakeBLUE(-30);

          chassis.pid_turn_set(-90, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(22, 90);
          set_intakeBLUE(110);
          chassis.pid_wait();
          IntakeRaiser.set(1);
          pros::delay(500);

          chassis.pid_drive_set(8, 20);
          chassis.pid_wait();
          pros::delay(500);

          chassis.pid_swing_set(LEFT_SWING, 0, 90);
          chassis.drive_brake_set(MOTOR_BRAKE_COAST);
          IntakeRaiser.set(0);
          chassis.pid_wait();

          chassis.pid_drive_set(8, 110);
          chassis.pid_wait();
      }

    //negative side ring rush corner
      void BlueNegativeRushCorner() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(17);

        //rush
          chassis.pid_drive_set(40, 110);
          DoinkerPiston.set(1);
          Intake1Control(127);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(12, 70);
          chassis.pid_wait();
          pros::delay(250);
        
        //clamp
          chassis.pid_swing_set(RIGHT_SWING, 50, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(-24, 90);
          chassis.pid_wait();
          ClampPiston.set(1);
          Intake1Control(0);

        //score first 3
          chassis.pid_turn_set(90, 110);
          chassis.pid_wait();
          DoinkerPiston.set(0);
          pros::Task intakeTask(AutonIntakeLoopBlue);
          set_intakeBLUE(110);

          chassis.pid_drive_set(36, 60);
          chassis.pid_wait();
          pros::delay(250);

        //score preload and mid stack
          chassis.pid_swing_set(LEFT_SWING, 30, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(-23, 110);
          chassis.pid_wait();
          set_intakeBLUE(-30);

          chassis.pid_turn_set(-90, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(80, 60);
          set_intakeBLUE(110);
          chassis.drive_brake_set(MOTOR_BRAKE_COAST);
          chassis.pid_wait();
      }

    //negative side alliance ladder
      void BlueNegativeAllianceLadder() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(-90);
          ladybrown_auton_set(2);

        //score alliance
          chassis.pid_turn_set(-133, 100);
          chassis.pid_wait();
          ladybrown_alliance();
          chassis.pid_drive_set(8, 100);
          chassis.pid_wait();

        //set up for mogo
          chassis.pid_drive_set(-33, 110);
          chassis.pid_wait();
          chassis.pid_turn_set(-215, 110);
          chassis.pid_wait();
          ladybrown_down();

        //drive into mogo and clamp
          chassis.pid_drive_set(-15, 100);
          chassis.pid_wait();
          ClampPiston.set(1);

        //turn toward first ring stack and intake
          chassis.pid_turn_set(48, 110);
          chassis.pid_wait();
          pros::Task intakeTask(AutonIntakeLoopBlue);
          set_intakeBLUE(110);
          chassis.pid_drive_set(20, 100);
          chassis.pid_wait();
          pros::delay(250);

        //turn toward second ring stack and intake
          chassis.pid_drive_set(-20, 110);
          chassis.pid_wait();
          chassis.pid_turn_set(90, 110);
          chassis.pid_wait();
          chassis.pid_drive_set(25, 100);
          chassis.pid_wait();

        //turn toward third ring stack and intake
          chassis.pid_turn_set(10, 110);
          chassis.pid_wait();
          chassis.pid_drive_set(17, 100);
          chassis.pid_wait();
          pros::delay(250);

        //turn toward fourth(mid) ring stack and intake
          chassis.pid_drive_set(-25, 110);
          chassis.pid_wait();
          set_intakeBLUE(-30);
          chassis.pid_turn_set(-101, 110);
          chassis.pid_wait();
          set_intakeBLUE(110);
          chassis.pid_drive_set(34, 110);
          chassis.pid_wait();
          IntakeRaiser.set(1);
          chassis.pid_drive_set(10, 40);
          chassis.pid_wait();
          pros::delay(500);

        //touch ladder
          chassis.pid_swing_set(LEFT_SWING, 0, 110);
          chassis.drive_brake_set(MOTOR_BRAKE_COAST);
          IntakeRaiser.set(0);
          chassis.pid_wait_quick_chain();

          chassis.pid_drive_set(12, 110);
          chassis.pid_wait();
      }

    //negative side alliance corner
      void BlueNegativeAllianceCorner() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(-90);
          ladybrown_auton_set(2);

        //score alliance
          chassis.pid_turn_set(-133, 100);
          chassis.pid_wait();
          ladybrown_alliance();
          chassis.pid_drive_set(8, 100);
          chassis.pid_wait();

        //set up for mogo
          chassis.pid_drive_set(-33, 110);
          chassis.pid_wait();
          chassis.pid_turn_set(-215, 110);
          chassis.pid_wait();
          ladybrown_down();

        //drive into mogo and clamp
          chassis.pid_drive_set(-15, 100);
          chassis.pid_wait();
          ClampPiston.set(1);

        //turn toward first ring stack and intake
          chassis.pid_turn_set(48, 110);
          chassis.pid_wait();
          pros::Task intakeTask(AutonIntakeLoopBlue);
          set_intakeBLUE(110);
          chassis.pid_drive_set(20, 100);
          chassis.pid_wait();
          pros::delay(250);

        //turn toward second ring stack and intake
          chassis.pid_drive_set(-20, 110);
          chassis.pid_wait();
          chassis.pid_turn_set(90, 110);
          chassis.pid_wait();
          chassis.pid_drive_set(25, 100);
          chassis.pid_wait();

        //turn toward third ring stack and intake
          chassis.pid_turn_set(10, 110);
          chassis.pid_wait();
          chassis.pid_drive_set(17, 100);
          chassis.pid_wait();
          pros::delay(250);

        //turn toward fourth(mid) ring stack and intake
          chassis.pid_drive_set(-25, 110);
          chassis.pid_wait();
          set_intakeBLUE(-30);
          chassis.pid_turn_set(-101, 110);
          chassis.pid_wait();
          set_intakeBLUE(110);
          chassis.pid_drive_set(90, 60);
          chassis.pid_wait();
      }

  //blue positive
    //positive side mogo rush ladder
      void BluePositiveRushLadder() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(-15);

        //rush mogo
          chassis.pid_drive_set(40, 110);
          DoinkerPiston.set(1);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(5, 30);
          chassis.pid_wait();
          pros::delay(250);

          chassis.pid_drive_set(-3, 30);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(-15, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(6, 110);
          chassis.pid_wait_quick_chain();

          chassis.pid_drive_set(-12, 110);
          chassis.pid_wait();
          DoinkerPiston.set(0);
        
        //intake ring stack
          chassis.pid_turn_set(-50, 110);
          chassis.pid_wait();
          Intake1Control(110);

          chassis.pid_drive_set(20, 100);
          chassis.pid_wait();

        //clamp and score
          chassis.pid_turn_set(-88, 110);
          chassis.pid_wait();
          Intake1Control(0);

          chassis.pid_drive_set(-26, 100);
          chassis.pid_wait();
          ClampPiston.set(1);
          pros::delay(250);

        //intake mid ring stack
          chassis.pid_turn_set(-220, 110);
          pros::Task intakeTask(AutonIntakeLoopBlue);
          set_intakeBLUE(110);
          chassis.pid_wait();

          chassis.pid_drive_set(16, 90);
          chassis.pid_wait();
          set_intakeBLUE(0);
          DisableAutonLoops();
          IntakeRaiser.set(1);
          intake2.brake();
          Intake1Control(110);
          chassis.pid_drive_set(8, 40);
          chassis.pid_wait();
          pros::delay(250);
          IntakeRaiser.set(0);

        //drop mogo off and clamp mid mogo
          chassis.pid_swing_set(LEFT_SWING, 90, 110);
          chassis.pid_wait();
          
          chassis.pid_drive_set(-30, 110);
          chassis.pid_wait();
          ClampPiston.set(0);

          chassis.pid_drive_set(6, 110);
          chassis.pid_wait();
          Intake1Control(0);

          chassis.pid_turn_set(160, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(-24, 110);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(-5, 80);
          chassis.pid_wait();
          intake2.move(110);
          ClampPiston.set(1);

          chassis.pid_swing_set(RIGHT_SWING, 45, 127);
          chassis.drive_brake_set(MOTOR_BRAKE_COAST);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(10, 127);
          chassis.pid_wait();
      }

    //positive side mogo rush middle
      void BluePositiveRushMid() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(-15);

        //rush mogo
          chassis.pid_drive_set(40, 110);
          DoinkerPiston.set(1);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(5, 30);
          chassis.pid_wait();
          pros::delay(250);

          chassis.pid_drive_set(-3, 30);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(-15, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(6, 110);
          chassis.pid_wait_quick_chain();

          chassis.pid_drive_set(-12, 110);
          chassis.pid_wait();
          DoinkerPiston.set(0);
        
        //intake ring stack
          chassis.pid_turn_set(-50, 110);
          chassis.pid_wait();
          Intake1Control(110);

          chassis.pid_drive_set(20, 100);
          chassis.pid_wait();

        //clamp and score
          chassis.pid_turn_set(-88, 110);
          chassis.pid_wait();
          Intake1Control(0);

          chassis.pid_drive_set(-26, 100);
          chassis.pid_wait();
          ClampPiston.set(1);
          pros::delay(250);

        //intake mid ring stack
          chassis.pid_turn_set(-220, 110);
          pros::Task intakeTask(AutonIntakeLoopBlue);
          set_intakeBLUE(110);
          chassis.pid_wait();

          chassis.pid_drive_set(16, 90);
          chassis.pid_wait();
          set_intakeBLUE(0);
          DisableAutonLoops();
          IntakeRaiser.set(1);
          intake2.brake();
          Intake1Control(110);
          chassis.pid_drive_set(8, 40);
          chassis.pid_wait();
          pros::delay(250);
          IntakeRaiser.set(0);

        //drop mogo off and clamp mid mogo
          chassis.pid_swing_set(LEFT_SWING, 90, 110);
          chassis.pid_wait();
          
          chassis.pid_drive_set(-30, 110);
          chassis.pid_wait();
          ClampPiston.set(0);

          chassis.pid_drive_set(6, 110);
          chassis.pid_wait();
          Intake1Control(0);

          chassis.pid_turn_set(160, 110);
          chassis.pid_wait();

          chassis.pid_drive_set(-24, 110);
          chassis.pid_wait_quick_chain();
          chassis.pid_drive_set(-5, 80);
          chassis.pid_wait();
          intake2.move(110);
          ClampPiston.set(1);
      }

    //positive side alliance ladder
      void BluePositiveAllianceLadder() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(90);
          ladybrown_auton_set(2);

        //score alliance
          chassis.pid_turn_set(133, 115);
          chassis.pid_wait();
          ladybrown_alliance();
          chassis.pid_drive_set(9, 105);
          chassis.pid_wait();
          pros::delay(150);

        //pick up mid ring
          chassis.pid_drive_set(-18, 90);
          chassis.pid_wait();

          chassis.pid_turn_set(90, 90);
          ladybrown_down();
          chassis.pid_wait();

          chassis.pid_drive_set(11, 90);
          chassis.pid_wait();
          IntakeRaiser.set(1);
          Intake1Control(127);
          chassis.pid_drive_set(11, 80);
          chassis.pid_wait();
          pros::delay(250);
          IntakeRaiser.set(0);

        //clamp mogo
          chassis.pid_turn_set(140, 90);
          chassis.pid_wait();

          chassis.pid_drive_set(-40, 70);
          chassis.pid_wait();
          ClampPiston.set(1);
          Intake1Control(0);

        //score ring
          chassis.pid_turn_set(-110, 90);
          chassis.pid_wait();

          pros::Task intakeTask(AutonIntakeLoopBlue);
          set_intakeBLUE(110);
          chassis.pid_drive_set(28, 70);
          chassis.pid_wait();
          pros::delay(250);

        //touch ladder
          chassis.pid_turn_set(-120, 90);
          chassis.pid_wait();

          chassis.pid_drive_set(-3, 120);
          chassis.drive_brake_set(MOTOR_BRAKE_COAST);
          chassis.pid_wait();
      }

    //positive side alliance middle
      void BluePositiveAllianceMiddle() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(90);
          ladybrown_auton_set(2);

        //score alliance
          chassis.pid_turn_set(133, 115);
          chassis.pid_wait();
          ladybrown_alliance();
          chassis.pid_drive_set(9, 105);
          chassis.pid_wait();
          pros::delay(150);

        //pick up mid ring
          chassis.pid_drive_set(-18, 90);
          chassis.pid_wait();

          chassis.pid_turn_set(90, 90);
          ladybrown_down();
          chassis.pid_wait();

          chassis.pid_drive_set(11, 90);
          chassis.pid_wait();
          IntakeRaiser.set(1);
          Intake1Control(127);
          chassis.pid_drive_set(11, 80);
          chassis.pid_wait();
          pros::delay(250);
          IntakeRaiser.set(0);

        //clamp mogo
          chassis.pid_turn_set(140, 90);
          chassis.pid_wait();

          chassis.pid_drive_set(-40, 70);
          chassis.pid_wait();
          ClampPiston.set(1);
          Intake1Control(0);

        //score ring
          chassis.pid_turn_set(-110, 90);
          chassis.pid_wait();

          pros::Task intakeTask(AutonIntakeLoopBlue);
          set_intakeBLUE(110);
          chassis.pid_drive_set(28, 70);
          chassis.pid_wait();
          pros::delay(500);

        //line up with mid mogo
          chassis.pid_turn_set(0, 90);
          chassis.pid_wait();

          chassis.pid_drive_set(-12, 90);
          chassis.pid_wait();
          ClampPiston.set(0);
          set_intakeBLUE(0);

          chassis.pid_drive_set(6, 90);
          chassis.pid_wait();

          chassis.pid_turn_set(190, 90);
          chassis.pid_wait();

          chassis.pid_drive_set(-16, 70);
          chassis.pid_wait();
      }

  //blue AWP
    //true SAWP
      void BlueSAWP() {
        //resetting sensors
          chassis.pid_targets_reset();
          chassis.drive_imu_reset();
          chassis.drive_sensor_reset();
          chassis.odom_enable(false);
          chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
          chassis.drive_angle_set(-90);
          ladybrown_auton_set(2);

        //score alliance
          chassis.pid_turn_set(-133, 115);
          chassis.pid_wait();
          ladybrown_alliance();
          chassis.pid_drive_set(9, 105);
          chassis.pid_wait();
          pros::delay(150);

        //set up for mogo
          chassis.pid_drive_set(-33, 115);
          chassis.pid_wait_quick();
          chassis.pid_turn_set(-215, 115);
          chassis.pid_wait_quick();
          ladybrown_down();

        //drive into mogo and clamp
          chassis.pid_drive_set(-15, 105);
          chassis.pid_wait();
          ClampPiston.set(1);

        //turn toward first ring stack and intake
          chassis.pid_turn_set(90, 115);
          chassis.pid_wait_quick();
          chassis.pid_drive_set(28, 100);
          pros::Task intakeTask(AutonIntakeLoopBlue);
          set_intakeBLUE(127);
          chassis.pid_wait();
          pros::delay(250);
        
        //cross and intake mid stack
          chassis.pid_drive_set(-28, 110);
          chassis.pid_wait();

          chassis.pid_turn_set(-137, 115);
          chassis.pid_wait();

          chassis.pid_drive_set(18, 115);
          chassis.pid_wait();
          set_intakeBLUE(0);
          intakeTask.remove();
          ClampPiston.set(0);
          intake2.brake();
          IntakeRaiser.set(1);
          Intake1Control(127);
          chassis.pid_drive_set(11, 80);
          chassis.pid_wait();
          pros::delay(250);
          IntakeRaiser.set(0);

        //clamp next mogo
          chassis.pid_turn_set(130, 115);
          chassis.pid_wait();

          chassis.pid_drive_set(-36, 105);
          chassis.pid_wait();
          ClampPiston.set(1);

        //score last stack
          chassis.pid_turn_set(-95, 115);
          chassis.pid_wait();

          chassis.pid_drive_set(24, 100);
          ContinueAutonLoops();
          set_intakeBLUE(110);
          intake2.move(110);
          chassis.pid_wait();

        //touch ladder
          chassis.pid_turn_set(-121, 127);
          chassis.pid_wait();
          chassis.pid_drive_set(-29.5, 120);
          chassis.drive_brake_set(MOTOR_BRAKE_COAST);
          chassis.pid_wait();
      }

  //skills
    void skills() {
      //resetting sensors
        chassis.pid_targets_reset();
        chassis.drive_imu_reset();
        chassis.drive_sensor_reset();
        chassis.odom_enable(false);
        chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
        chassis.drive_angle_set(180);
        ladybrown_auton_set(2);

      //score alliance
        ladybrown_alliance();
        pros::delay(750);

      //clamp mogo 1
        chassis.pid_drive_set(-10, 110);
        chassis.pid_wait();

        chassis.pid_turn_set(-90, 110);
        ladybrown_down();
        chassis.pid_wait_quick_chain();

        chassis.pid_drive_set(-20, 60);
        chassis.pid_wait();
        ClampPiston.set(1);

      //score ring 1-1
        chassis.pid_turn_set(0, 110);
        chassis.pid_wait();
        set_intake_blank(127);

        chassis.pid_drive_set(32, 80);
        chassis.pid_wait_quick();

      //score ring 1-2,2 & w-1
        chassis.pid_turn_set(45, 110);
        chassis.pid_wait_quick_chain();

        chassis.pid_drive_set(22, 110);
        chassis.pid_wait_quick_chain();

        chassis.pid_turn_set(0, 110);
        chassis.pid_wait_quick_chain();

        chassis.pid_drive_set(25, 60);
        chassis.pid_wait();
        pros::delay(250);

        chassis.pid_turn_set(25, 110);
        chassis.pid_wait_quick_chain();
        ladybrown.move_absolute(230, 1000);
        intake2.move(80);

        chassis.pid_drive_set(20, 80);
        chassis.pid_wait();

      //do right side wall stakes
        chassis.pid_turn_set(15, 110);
        chassis.pid_wait_quick_chain();

        chassis.pid_drive_set(-36.5, 110);
        chassis.pid_wait_quick_chain();
        intake2.move(0);

        chassis.pid_turn_set(88, 110);
        chassis.pid_wait_quick_chain();
        intake2.move(127);

        chassis.pid_drive_set(17, 80, false, true);
        chassis.pid_wait();

        chassis.pid_drive_set(-2, 30);
        set_intake_blank(0);
        ladybrown.move_absolute(5.5*360, 1000);
        pros::delay(750);
        chassis.pid_wait();
        chassis.pid_drive_set(3, 60);
        ladybrown_prime();
        pros::delay(1000);
        set_intake_blank(127);
        pros::delay(750);
        intake2.move(0);
        pros::delay(250);
        intake2.move(127);
        pros::delay(250);
        chassis.pid_wait();
        chassis.pid_drive_set(-2, 30);
        set_intake_blank(0);
        ladybrown_score();
        chassis.pid_wait();
        pros::delay(500);


      //score rings 1-3,4,5,6
        chassis.pid_drive_set(-4, 110);
        ladybrown_down();
        chassis.pid_wait_quick_chain();

        chassis.pid_turn_set(180, 110);
        chassis.pid_wait_quick_chain();
        set_intake_blank(0);

        chassis.pid_drive_set(50, 90);
        chassis.pid_wait();
    }