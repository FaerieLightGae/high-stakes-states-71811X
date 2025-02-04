#include "main.h"
bool ClampToggle = false;

void clamp_opcontrol() {
    if (master.get_digital_new_press(DIGITAL_L1)) {
        ClampToggle = !ClampToggle;
        ClampPiston.set(ClampToggle);
    }
}

void clamp_set() {
    ClampToggle = ClampPiston.get();
}