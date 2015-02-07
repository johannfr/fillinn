#ifndef MAIN_HPP_
#define MAIN_HPP_

void setup();
void loop();

void writePCF(uint8_t);

double getUpdatedAverage(double, double*, int*, int*);

void arm_interrupt();

void cmd_setpoint(int, char**);
void cmd_run(int, char**);
void cmd_stop(int, char**);
void cmd_brake(int, char**);
void cmd_direction(int, char**);
void cmd_pcf(int, char**);
void cmd_p(int, char**);
void cmd_i(int, char**);
void cmd_d(int, char**);
void cmd_arm_left(int, char**);
void cmd_arm_right(int, char**);
void cmd_arm_rotate(int, char**);
void cmd_arm_motor_pwm(int, char**);
void cmd_arm_servo_pwm(int, char**);
void cmd_arm_magnet(int, char**);
void cmd_manual_pwm(int, char**);
#endif
