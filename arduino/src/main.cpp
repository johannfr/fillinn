#include "Arduino.h"
#include <main.hpp>

#include <Cmd.h>


#define pPWMFront 9
#define pDirectionFrontA 7
#define pDirectionFrontB 8
#define pEncoderFront 4

#define pPWMBack 10
#define pDirectionBackA 12
#define pDirectionBackB 11
#define pEncoderBack 2


double setpoint = 0;

bool pidRunning = false;

bool directionA = false;
bool directionB = false;


void setup()
{
    Serial.begin(115200);
    pinMode(pPWMFront, OUTPUT);
    pinMode(pDirectionFrontA, OUTPUT);
    pinMode(pDirectionFrontB, OUTPUT);
    pinMode(pEncoderFront, INPUT);
    digitalWrite(pEncoderFront, HIGH); // Enable pull-ups

    pinMode(pPWMBack, OUTPUT);
    pinMode(pDirectionBackA, OUTPUT);
    pinMode(pDirectionBackB, OUTPUT);
    pinMode(pEncoderBack, INPUT);
    digitalWrite(pEncoderBack, HIGH); // Enable pull-ups


    cmdInit(&Serial);
    cmdAdd("setpoint", cmd_setpoint);
}

void loop()
{
    cmdPoll();

    //TODO Add PID stuff here.

    digitalWrite(pDirectionFrontA, directionA);
    digitalWrite(pDirectionFrontB, directionB);
    digitalWrite(pDirectionBackA, !directionA); // Invert, motor is other way around.
    digitalWrite(pDirectionBackB, !directionB);


}

void cmd_setpoint(int arg_cnt, char **args)
{

    if (arg_cnt < 2)
    {
        Serial.println(setpoint, DEC);
        return;
    }

    setpoint = cmdStr2Num(args[1], 10);

}



void cmd_run(int arg_cnt, char **args)
{

    if (arg_cnt < 2)
    {
        Serial.println(pidRunning, DEC);
        return;
    }

    // TODO Reset PID values.
    pidRunning = (cmdStr2Num(args[1], 10) == 1 ? true : false);
}

void cmd_stop(int arg_cnt, char **args)
{
    pidRunning = false;
    directionA = false;
    directionB = false;
}

void cmd_brake(int arg_cnt, char **args)
{
    pidRunning = false;
    directionA = true;
    directionB = true;
}

void cmd_direction(int arg_cnt, char **args)
{
    if (arg_cnt < 2)
    {
        Serial.println(directionA, DEC);
        return;
    }

    directionA = (cmdStr2Num(args[1], 10) == 1 ? true : false);
    directionB = !directionA;
}
