/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       MSI                                                       */
/*    Created:      3/9/2025, 14:38:33                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

brain Brain;
controller Controller1;

// Motores del lado izquierdo (puertos 1-4)
motor LeftMotor1(PORT7, true);
motor LeftMotor2(PORT8, false);
motor LeftMotor3(PORT9, true);
motor LeftMotor4(PORT10, false);
motor_group Left(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);

// Motores del lado derecho (puertos 7-10)
motor RightMotor1(PORT1, true);
motor RightMotor2(PORT2, false);
motor RightMotor3(PORT3, true);
motor RightMotor4(PORT4, false);
motor_group Right(RightMotor1, RightMotor2, RightMotor3, RightMotor4);

// Sensor de Inercia
inertial imuSensor(PORT20);

struct PID
{
    double kp = 0;
    double ki = 0;
    double kd = 0;

    double error = 0;
    double prevError = 0;
    double Integral = 0;
};

double computerPID(PID &pid, double setpoint, double current, double dt)
{
    pid.error = setpoint - current;
    pid.Integral += pid.error * dt;

    double derivative = (pid.error - pid.prevError) / dt;
    double output = pid.kp * pid.error + pid.ki * pid.Integral + pid.kd * derivative;
    pid.prevError = pid.error;
    return output;
}

int main()

{

    imuSensor.calibrate();

    while (imuSensor.isCalibrating())
    {
        wait(20, msec);
    }

    // Crear e inicializar PID
    PID pid;
    pid.kp = 0.3;

    pid.ki = 0.009;

    pid.kd = 0.035;

    double target = 180;

    timer t;

    double lastTime = t.time(msec);

    while (true)

    {

        double currentTime = t.time(msec);

        double dt = (currentTime - lastTime) / 1000.0;

        lastTime = currentTime;

        double angle = imuSensor.rotation(degrees);

        double power = computerPID(pid, target, angle, dt);

        if (power > 100)
            power = 100;

        if (power < -100)
            power = -100;

        Left.spin(forward, power, percent);

        Right.spin(forward, -power, percent);

        Brain.Screen.clearLine(1);                         // Limpia la primera fila de la pantalla
        Brain.Screen.setCursor(1, 1);                      // Coloca el cursor en la fila 1, columna 1
        Brain.Screen.print("Angulo: %.2f", angle); // Imprime el texto y el valor de la variable

        wait(20, msec);
    }
}
