#include "Arduino.h"
#include <main.hpp>

#include <Cmd.h>
#include <Wire.h>


#define pPWMFront 3
#define pEncoderFront 2

#define pPWMBack 5
#define pEncoderBack 4

#define inputAvgSize 10

#define PCFAddress 0x38
volatile uint8_t PCFValue = 0x00;

double setpoint = 10000.0;
double error_front = 0, error_back = 0;
double PID_front = 0, PID_back = 0;

bool pidRunning = false;

double
    P_front = 0.0,
    I_front = 0.0,
    D_front = 0.0;

double
    kP_front = 0.3,
    kI_front = 0.01,
    kD_front = 0.0;

double
    P_back = 0.0,
    I_back = 0.0,
    D_back = 0.0;

double
    kP_back = 0.3,
    kI_back = 0.01,
    kD_back = 0.0;

double inputAvgFront[inputAvgSize];
int inputAvgFrontIndex = 0;
int inputAvgFrontWritten = 0;

double inputAvgBack[inputAvgSize];
int inputAvgBackIndex = 0;
int inputAvgBackWritten = 0;

unsigned long measurement_front = 0,  measurement_back = 0;
double rpm_front = 0, rpm_back = 0;
double lastRPM_front = 0, lastRPM_back = 0;
double avgMeasurement_front = 0, avgMeasurement_back = 0;

int outputPWM_front = 0, outputPWM_back = 0;

void setup()
{
    Serial.begin(115200);
    pinMode(pPWMFront, OUTPUT);
    pinMode(pEncoderFront, INPUT);
    digitalWrite(pEncoderFront, HIGH); // Enable pull-ups

    pinMode(pPWMBack, OUTPUT);
    pinMode(pEncoderBack, INPUT);
    digitalWrite(pEncoderBack, HIGH);


    cmdInit(&Serial);
    cmdAdd("setpoint", cmd_setpoint);
    cmdAdd("run", cmd_run);
    cmdAdd("stop", cmd_stop);
    cmdAdd("brake", cmd_brake);
    cmdAdd("direction", cmd_direction);
    cmdAdd("pcf", cmd_pcf);


}

void loop()
{
    cmdPoll();

    measurement_front = pulseIn(pEncoderFront, HIGH, 5000);
    avgMeasurement_front = getUpdatedAverage(measurement_front, inputAvgFront, &inputAvgFrontIndex, &inputAvgFrontWritten);
    rpm_front = (avgMeasurement_front > 0.0 ? (60000000 / (avgMeasurement_front * 12)) : 0);

    measurement_back = pulseIn(pEncoderBack, HIGH, 5000);
    avgMeasurement_back = getUpdatedAverage(measurement_back, inputAvgBack, &inputAvgBackIndex, &inputAvgBackWritten);
    rpm_back = (avgMeasurement_back > 0.0 ? (60000000 / (avgMeasurement_back * 12)) : 0);

    if (pidRunning)
    {
        // PID styring

        error_front = setpoint - rpm_front;
        // P
        P_front = error_front * kP_front;
        // I
        I_front += error_front * kI_front;
        // D
        D_front = (lastRPM_front - rpm_front) * kD_front;
        PID_front = P_front + I_front + D_front;
        outputPWM_front = constrain(map(PID_front, 0, 22000, 0, 255), 0, 255);


        error_back = setpoint - rpm_back;
        // P
        P_back = error_back * kP_back;
        // I
        I_back += error_back * kI_back;
        // D
        D_back = (lastRPM_back - rpm_back) * kD_back;
        PID_back = P_back + I_back + D_back;
        outputPWM_back = constrain(map(PID_back, 0, 22000, 0, 255), 0, 255);
    
    }

    analogWrite(pPWMFront, outputPWM_front);
    analogWrite(pPWMBack, outputPWM_back);

    lastRPM_front = rpm_front;
    lastRPM_back = rpm_back;

}

double getUpdatedAverage(double newValue, double *array, int *index, int *written)
{
    array[*index] = newValue;

    if (*written < inputAvgSize)
        (*written)++;
    
    double updatedAverage = 0;
    for (int i = 0 ; i < *written ; i++)
    {
        updatedAverage += array[i];
    }
    updatedAverage = updatedAverage / *written;
    *index = (*index + 1) % inputAvgSize;

    return updatedAverage;
}

void writePCF(uint8_t newValue)
{
    Wire.beginTransmission((uint8_t)PCFAddress);
    Wire.write(newValue);
    Wire.endTransmission();
}

void cmd_setpoint(int arg_cnt, char **args)
{

    if (arg_cnt < 2)
    {
        Serial.println(setpoint, DEC);
        return;
    }

    error_front = 0;
    error_back = 0;
    PID_front = 0;
    PID_back = 0;
    P_front = 0;
    I_front = 0;
    D_front = 0;
    P_back = 0;
    I_back = 0;
    D_back = 0;
    outputPWM_front = 0;
    outputPWM_back = 0;
    setpoint = cmdStr2Num(args[1], 10);

}



void cmd_run(int arg_cnt, char **args)
{

    if (arg_cnt < 2)
    {
        Serial.println(pidRunning, DEC);
        return;
    }

    error_front = 0;
    error_back = 0;
    PID_front = 0;
    PID_back = 0;
    P_front = 0;
    I_front = 0;
    D_front = 0;
    P_back = 0;
    I_back = 0;
    D_back = 0;
    pidRunning = (cmdStr2Num(args[1], 10) == 1 ? true : false);
    outputPWM_front = 0;
    outputPWM_back = 0;
}

void cmd_stop(int arg_cnt, char **args)
{
    pidRunning = false;
    //FIXME directionA = false;
    //FIXME directionB = false;
}

void cmd_brake(int arg_cnt, char **args)
{
    pidRunning = false;
    //FIXME directionA = true;
    //FIXME directionB = true;
}

void cmd_direction(int arg_cnt, char **args)
{
    /* if (arg_cnt < 2)
    {
        Serial.println(directionA, DEC);
        return;
    }
    */

    //FIXME directionA = (cmdStr2Num(args[1], 10) == 1 ? true : false);
    //FIXME directionB = !directionA;
}

void cmd_pcf(int arg_cnt, char **args)
{
    if (arg_cnt < 2)
    {
        return;
    }
    uint8_t value = (uint8_t)cmdStr2Num(args[1], 10);
    writePCF(value);
}
