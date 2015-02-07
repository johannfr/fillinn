#include "Arduino.h"
#define ARDUINO_ARCH_AVR
#include <main.hpp>

#include <Cmd.h>
#include <Wire.h>
#include <Servo.h>

#define pPWMArm 3
#define pEncoderArm 2

#define pPWMArmA 10
#define pPWMArmB 11
#define pAnalogArm 0

#define pPWMBack 5
#define pEncoderBack 4

#define pPWMFront 6
#define pEncoderFront 7

#define inputAvgSize 10

#define PCFAddress 0x38
volatile uint8_t PCFValue = 0x00;
volatile bool direction = false;
volatile int arm_steps = 0;

int arm_motor_pwm = 50;
int arm_servo_pwm = 200;

Servo magnet_servo;

double setpoint = 10000.0;
double error_front = 0, error_back = 0;
double PID_front = 0, PID_back = 0;

bool pidRunning = false;

double
    P_front = 0.0,
    I_front = 0.0,
    D_front = 0.0;

double
    kP_front = 0.7,
    kI_front = 0.09,
    kD_front = 0.0;

double
    P_back = 0.0,
    I_back = 0.0,
    D_back = 0.0;

double
    kP_back = 0.7,
    kI_back = 0.09,
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
int manual_pwm = 0;

void setup()
{
    Serial.begin(9600);
    pinMode(pPWMFront, OUTPUT);
    pinMode(pEncoderFront, INPUT);
    digitalWrite(pEncoderFront, HIGH); // Enable pull-ups

    pinMode(pPWMBack, OUTPUT);
    pinMode(pEncoderBack, INPUT);
    digitalWrite(pEncoderBack, HIGH);
    writePCF(PCFValue);

    pinMode(pPWMArm, OUTPUT);
    pinMode(pPWMArmA, OUTPUT);
    pinMode(pPWMArmB, OUTPUT);
    pinMode(pEncoderArm, INPUT);

    magnet_servo.attach(9);
    magnet_servo.write(12);


    cmdInit(&Serial);
    cmdAdd("setpoint", cmd_setpoint);
    cmdAdd("r", cmd_run);
    cmdAdd("s", cmd_stop);
    cmdAdd("b", cmd_brake);
    cmdAdd("dir", cmd_direction);
    cmdAdd("pcf", cmd_pcf);
    cmdAdd("p", cmd_p);
    cmdAdd("i", cmd_i);
    cmdAdd("d", cmd_d);
    cmdAdd("ar", cmd_arm_left);
    cmdAdd("al", cmd_arm_right);
    cmdAdd("arot", cmd_arm_rotate);
    cmdAdd("ampwm", cmd_arm_motor_pwm);
    cmdAdd("aspwm", cmd_arm_servo_pwm);
    cmdAdd("mservo", cmd_arm_magnet);
    cmdAdd("mpwm", cmd_manual_pwm);
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
        manual_pwm = 0;
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

    if (manual_pwm == 0)
    {
        analogWrite(pPWMFront, outputPWM_front);
        analogWrite(pPWMBack, outputPWM_back);

        lastRPM_front = rpm_front;
        lastRPM_back = rpm_back;
    }
    else
    {
        analogWrite(pPWMFront, manual_pwm);
        analogWrite(pPWMBack, manual_pwm);
    }

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
    if (pidRunning)
    {
        analogWrite(pPWMFront, 70);
        analogWrite(pPWMBack, 70);
        delay(400);
    }
}

void cmd_stop(int arg_cnt, char **args)
{
    pidRunning = false;
    PCFValue &= 0xc3;
    writePCF(PCFValue);
}

void cmd_brake(int arg_cnt, char **args)
{
    pidRunning = false;
    PCFValue |= 0x3c;
    writePCF(PCFValue);
}

void cmd_direction(int arg_cnt, char **args)
{
    if (arg_cnt < 2)
    {
        Serial.println(direction, DEC);
        return;
    }
    
    direction = (cmdStr2Num(args[1], 10) == 1 ? true : false);
    if (direction)
    {
        PCFValue &= ~0x3c;
        PCFValue |= 0x24;
    }
    else
    {
        PCFValue &= ~0x3c;
        PCFValue |= 0x18;
    }
    writePCF(PCFValue);
}

void cmd_arm_left(int arg_cnt, char **args)
{
    if (arg_cnt < 2)
    {
        return;
    }

    int arm_setpoint = cmdStr2Num(args[1], 10);
    arm_steps = 0;

    attachInterrupt(0, arm_interrupt, FALLING);
    PCFValue &= ~0x3;
    PCFValue |= 0x1;
    writePCF(PCFValue);
    analogWrite(pPWMArm, arm_motor_pwm);

    unsigned long starttime = millis();
    while (millis() < (starttime+3000) && arm_steps < arm_setpoint);


    PCFValue &= ~0x3;
    PCFValue |= 0x3;
    writePCF(PCFValue);
    analogWrite(pPWMArm, 0);
    detachInterrupt(0);
}

void cmd_arm_right(int arg_cnt, char **args)
{
    if (arg_cnt < 2)
    {
        return;
    }

    int arm_setpoint = cmdStr2Num(args[1], 10);
    arm_steps = 0;

    attachInterrupt(0, arm_interrupt, FALLING);
    PCFValue &= ~0x3;
    PCFValue |= 0x2;
    writePCF(PCFValue);
    analogWrite(pPWMArm, arm_motor_pwm);

    unsigned long starttime = millis();
    while (millis() < (starttime+3000) && arm_steps < arm_setpoint);


    PCFValue &= ~0x3;
    PCFValue |= 0x3;
    writePCF(PCFValue);
    analogWrite(pPWMArm, 0);

    detachInterrupt(0);
}

void arm_interrupt()
{
    arm_steps++;
}

void cmd_arm_rotate(int arg_cnt, char **args)
{

    int position = analogRead(0);
    int degrees = map(position, 69, 920, 0, 180);

    if (arg_cnt < 2)
    {
        Serial.println(degrees, DEC);
        return;
    }

    int setpoint = constrain(map(cmdStr2Num(args[1], 10), 0, 180, 69, 920), 69, 920);

    if (setpoint < position)
    {
        digitalWrite(pPWMArmA, HIGH);
        analogWrite(pPWMArmB, arm_servo_pwm);
        unsigned long starttime = millis();
        while (millis() < (starttime+10000) && position > setpoint)
        {
            position = analogRead(0);
        }
    }
    else if (setpoint > position)
    {
        digitalWrite(pPWMArmB, HIGH);
        analogWrite(pPWMArmA, arm_servo_pwm);
        unsigned long starttime = millis();
        while (millis() < (starttime+10000) && setpoint > position)
        {
            position = analogRead(0);
        }
    }
    analogWrite(pPWMArmA, 0);
    analogWrite(pPWMArmB, 0);
    digitalWrite(pPWMArmA, HIGH);
    digitalWrite(pPWMArmB, HIGH);
}

void cmd_arm_motor_pwm(int arg_cnt, char **args)
{
    if (arg_cnt < 2)
    {
        Serial.println(arm_motor_pwm, DEC);
        return;
    }
    arm_motor_pwm = cmdStr2Num(args[1], 10);
}

void cmd_arm_servo_pwm(int arg_cnt, char **args)
{
    if (arg_cnt < 2)
    {
        Serial.println(arm_servo_pwm, DEC);
        return;
    }
    arm_servo_pwm = cmdStr2Num(args[1], 10);
}

void cmd_arm_magnet(int arg_cnt, char **args)
{
    magnet_servo.write(cmdStr2Num(args[1], 10));
}

void cmd_pcf(int arg_cnt, char **args)
{
    if (arg_cnt < 2)
    {
        return;
    }
    PCFValue = (uint8_t)cmdStr2Num(args[1], 10);
    writePCF(PCFValue);
}

void cmd_p(int arg_cnt, char**args)
{
    if (arg_cnt < 2)
    {
        Serial.println(kP_front, DEC);
        return;
    }

    kP_front = strtod(args[1], NULL);
    kP_back = kP_front;
}

void cmd_i(int arg_cnt, char**args)
{
    if (arg_cnt < 2)
    {
        Serial.println(kI_front, DEC);
        return;
    }

    kI_front = strtod(args[1], NULL);
    kI_back = kI_front;
}

void cmd_d(int arg_cnt, char **args)
{
    if (arg_cnt < 2)
    {
        Serial.println(kD_front, DEC);
        return;
    }

    kD_front = strtod(args[1], NULL);
    kD_back = kD_front;
}

void cmd_manual_pwm(int arg_cnt, char **args)
{
    if (arg_cnt < 2)
    {
        return;
    }
    manual_pwm = cmdStr2Num(args[1], 10);
}
