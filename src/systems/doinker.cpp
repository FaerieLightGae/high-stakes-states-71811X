#include "main.h"
bool DoinkerToggle = false;

void doinker_opcontrol() {
    if (master.get_digital_new_press(DIGITAL_A)) {
        DoinkerToggle = !DoinkerToggle;
        DoinkerPiston.set(DoinkerToggle);
    }
}