#include "main.h"

bool manualmode = false;
int armlevel = 1;

void move_ladybrown() {
    if (armlevel == 1) {
        continue_intake_extras();
        ladybrown.move_absolute(0, 1000);
        ladybrown.set_brake_mode(MOTOR_BRAKE_COAST);
    } else if (armlevel == 2) {
        block_intake_extras();
        ladybrown.move_absolute(220, 1000);
        ladybrown.set_brake_mode(MOTOR_BRAKE_HOLD);
    } else if (armlevel == 3) {
        continue_intake_extras();
        ladybrown.move_absolute(6*360, 1000);
    } else if (armlevel == 100) {
        continue_intake_extras();
        ladybrown.move_absolute(7*360, 1000);
    }
}

void ladybrown_opcontrol() {
    if (armlevel > 3) {
        armlevel = 1;
        move_ladybrown();
    } else if (master.get_digital_new_press(DIGITAL_RIGHT)) {
        manualmode = !manualmode;
    } else if (manualmode) {
        continue_intake_extras();
        if (master.get_digital(DIGITAL_R2)) {
            ladybrown.move(127);
        } else if (master.get_digital(DIGITAL_L2)) {
            ladybrown.move(-127);
        } else {
            ladybrown.set_brake_mode(MOTOR_BRAKE_HOLD);
            ladybrown.brake();
        }
    } else if (master.get_digital_new_press(DIGITAL_R2)) {
        armlevel += 1;
        move_ladybrown();
    }
}

void ladybrown_down() {
    armlevel = 1;
    move_ladybrown();
}

void ladybrown_prime() {
    armlevel = 2;
    move_ladybrown();
}

void ladybrown_score() {
    armlevel = 3;
    move_ladybrown();
}

void ladybrown_alliance() {
    armlevel = 100;
    move_ladybrown();
}

void set_ladybrown(int level) {
    if (level == 1) {
        armlevel = 1;
        move_ladybrown();
    } else if (level == 2) {
        armlevel = 2;
        move_ladybrown();
    } else if (level == 3) {
        armlevel = 3;
        move_ladybrown();
    } else if (level == 4) {
        armlevel = 4;
        move_ladybrown();
    }
}

void ladybrown_auton_set(int input) {
    armlevel = input;

    if (input == 2) {
        ladybrown.set_zero_position(-220);

    }
}