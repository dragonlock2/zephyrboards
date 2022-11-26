#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init();
void motor_write(double duty, bool forward);

#endif // MOTOR_H
