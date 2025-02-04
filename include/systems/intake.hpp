#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline pros::Motor intake1(18);  // Make this number negative if you want to reverse the motor
inline pros::Motor intake2(10);
inline pros::Optical optical_sensor(9);


void set_intakeRED(int input);
void REDintake_opcontrol();
void set_intakeBLUE(int input);
void BLUEintake_opcontrol();
void block_intake_extras();
void continue_intake_extras();
void set_intake_blank(int input);
void BLANKintake_opcontrol();

void Intake1Control(int input);
void Intake2Control(int input);

void IntakeBlue();
void IntakeRed();

void AutonIntakeLoopRed();
void AutonIntakeLoopBlue();

void DisableAutonLoops();
void ContinueAutonLoops();