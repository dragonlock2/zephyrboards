#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init();
void motor_write(double duty, bool forward);
uint32_t motor_cycle_time_us();

#endif // MOTOR_H
