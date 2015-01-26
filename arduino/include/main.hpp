#ifndef MAIN_HPP_
#define MAIN_HPP_

void setup();
void loop();

void writePCF(uint8_t);

double getUpdatedAverage(double, double*, int*, int*);

void cmd_setpoint(int, char**);
void cmd_run(int, char**);
void cmd_stop(int, char**);
void cmd_brake(int, char**);
void cmd_direction(int, char**);
void cmd_pcf(int, char**);


#endif
