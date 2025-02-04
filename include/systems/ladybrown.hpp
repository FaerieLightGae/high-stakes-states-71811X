#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline pros::Motor ladybrown(-11);  // Make this number negative if you want to reverse the motor

void move_ladybrown();
void ladybrown_opcontrol(); 
void set_ladybrown(int level);

void ladybrown_down();
void ladybrown_prime();
void ladybrown_score();
void ladybrown_alliance();
void ladybrown_auton_set(int input);