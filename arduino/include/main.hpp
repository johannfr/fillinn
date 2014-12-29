#ifndef MAIN_HPP_
#define MAIN_HPP_

void setup();
void loop();
void cmd_setpoint(int, char**);
void cmd_run(int, char**);
void cmd_stop(int, char**);
void cmd_brake(int, char**);
void cmd_direction(int, char**);

#endif
