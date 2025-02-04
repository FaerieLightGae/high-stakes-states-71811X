#include "main.h"
bool rejectring = false;
bool blockExtras = false;
int RedIntakeSpeed = 0;
int BlueIntakeSpeed = 0;
bool DriverControl = false;

void DisableAutonLoops() {
    DriverControl = true;
}
void ContinueAutonLoops() {
    DriverControl = false;
}

void set_intakeRED(int input) {
    RedIntakeSpeed = input;
}

void REDintake_opcontrol() {
    if (master.get_digital(DIGITAL_B)) {
        set_intakeRED(-127);
        block_intake_extras();
        IntakeRed();
    } else if (master.get_digital(DIGITAL_R1)) {
        set_intakeRED(127);
        continue_intake_extras();
        IntakeRed();
    } else if (rejectring) {
        set_intakeRED(127);
        IntakeRed();
    } else {
        set_intakeRED(0);
        continue_intake_extras();
        IntakeRed();
    }
}

void set_intakeBLUE(int input) {
    BlueIntakeSpeed = input;
}

void BLUEintake_opcontrol() {
    if (master.get_digital(DIGITAL_B)) {
        set_intakeBLUE(-127);
        block_intake_extras();
        IntakeBlue();
    } else if (master.get_digital(DIGITAL_R1)) {
        set_intakeBLUE(127);
        continue_intake_extras();
        IntakeBlue();
    } else if (rejectring) {
        set_intakeBLUE(127);
        IntakeBlue();
    } else {
        set_intakeBLUE(0);
        continue_intake_extras();
        IntakeBlue();
    }
}

void block_intake_extras() {
    blockExtras = true;
}

void continue_intake_extras() {
    blockExtras = false;
}

void set_intake_blank(int input) {
        intake1.move(input);
        intake2.move(input);
}

void BLANKintake_opcontrol() {
    if (master.get_digital(DIGITAL_B)) {
        set_intake_blank(-127);
        block_intake_extras();
    } else if (master.get_digital(DIGITAL_R1)) {
        set_intake_blank(127);
        continue_intake_extras();
    } else if (rejectring) {
        set_intake_blank(127);
    } else {
        set_intake_blank(0);
        continue_intake_extras();
    }
}


void Intake1Control(int input) {
    intake1.move(input);

    if (input != 0) {
        DriverControl = true;
    } else {
        DriverControl = false;
    }
}

void IntakeBlue() {
    optical_sensor.set_led_pwm(100);

        if ((rejectring == true) && !master.get_digital(DIGITAL_B) && (blockExtras == false)) {
            intake1.move(BlueIntakeSpeed);
            if (intake2.get_torque() >= 0.75) {
                intake2.brake();
                pros::delay(250);
                intake2.move(BlueIntakeSpeed);
                rejectring = false;
                }
        } else if ((rejectring == false) && (optical_sensor.get_proximity() >= 200) && (blockExtras == false)) {
            if (100 > optical_sensor.get_hue()) {
            rejectring = true;
            }
            else {
                intake1.move(BlueIntakeSpeed);
                intake2.move(BlueIntakeSpeed);
            }
        } else {
            intake1.move(BlueIntakeSpeed);
            intake2.move(BlueIntakeSpeed);
        }
        pros::delay(ez::util::DELAY_TIME);
}

void IntakeRed() {
    optical_sensor.set_led_pwm(100);

        if ((rejectring == true) && !master.get_digital(DIGITAL_B) && (blockExtras == false)) {
            intake1.move(RedIntakeSpeed);
            if (intake2.get_torque() >= 0.75) {
                intake2.brake();
                pros::delay(250);
                intake2.move(RedIntakeSpeed);
                rejectring = false;
                }
        } else if ((rejectring == false) && (optical_sensor.get_proximity() >= 200) && (blockExtras == false)) {
            if (200 < optical_sensor.get_hue()) {
            rejectring = true; 
            }
            else {
                intake1.move(RedIntakeSpeed);
                intake2.move(RedIntakeSpeed);
            }
        } else {
            intake1.move(RedIntakeSpeed);
            intake2.move(RedIntakeSpeed);
        }
        pros::delay(ez::util::DELAY_TIME);
}

void AutonIntakeLoopRed() {
    if (DriverControl == false) {
    while (!DriverControl) {
        IntakeRed();
    }
    }
}

void AutonIntakeLoopBlue() {
    if (DriverControl == false) {
    while (!DriverControl) {
        IntakeBlue();
    }
    }
}