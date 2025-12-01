/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       MSI                                                       */
/*    Created:      8/9/2025, 12:24:07                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

brain Brain;
controller Controller1;

// Motores del lado izquierdo (puertos 1-4)
motor LeftMotor1(PORT6, true);
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

// Sensor de Distancia
distance SensorDeDistancia = distance(PORT19);

// Estructura pra definir el PID
struct PID
{
    double kp = 0;
    double ki = 0;
    double kd = 0;

    double error = 0;
    double prevError = 0;
    double Integral = 0;
};

// Función para calcular el PID
double computerPID(PID &pid, double setpoint, double current, double dt)
{
    pid.error = setpoint - current;
    pid.Integral += pid.error * dt;

    double derivative = (pid.error - pid.prevError) / dt;
    double output = pid.kp * pid.error + pid.ki * pid.Integral + pid.kd * derivative;
    pid.prevError = pid.error;
    return output;
}

// Función para reiniciar datos del PID y evitar acumulación de errores
void resetPID(PID &pid)
{
    pid.Integral = 0;
    pid.prevError = 0;
}

int main()
{
    imuSensor.calibrate();

    while (imuSensor.isCalibrating())
    {
        wait(20, msec);
    }

    // Crear las estructuras PID para los dos sensores
    PID pidDistancia;
    PID pidInercia;

    // Definir los parametros de cada sensor
    pidDistancia.kp = 0.05;
    pidDistancia.ki = 0.0004;
    pidDistancia.kd = 0.005;
    double targetDistancia = 300;

    pidInercia.kp = 0.1;
    pidInercia.ki = 0.00;
    pidInercia.kd = 0.0;
    double targetInercia1 = 0;
    double targetInercia2 = -90;

    timer t;
    double lastTime = t.time(msec);

    double angle;
    double distancia;
    bool realizarGiro = false;

    while (true)
    {
        double currentTime = t.time(msec);

        double dt = (currentTime - lastTime) / 1000.0;

        lastTime = currentTime;

        if (realizarGiro)
        {
            // Se realiza la fase del giro
            resetPID(pidInercia);
            do
            {
                currentTime = t.time(msec);
                dt = (currentTime - lastTime) / 1000.0;
                lastTime = currentTime;
                angle = imuSensor.angle(degrees);

                double powerGiro = computerPID(pidInercia, targetInercia1, angle, dt);
                if (powerGiro > 100)
                    powerGiro = 100;
                if (powerGiro < -100)
                    powerGiro = -100;

                Left.spin(forward, powerGiro, percent);

                Right.spin(forward, -powerGiro, percent);

                Brain.Screen.clearLine(1);                       // Limpia la primera fila de la pantalla
                Brain.Screen.setCursor(2, 1);                    // Coloca el cursor en la fila 1, columna 1
                Brain.Screen.print("Angulo: %.2f", angle);       // Imprime el texto y el valor de la variable
                Controller1.Screen.clearScreen();                // Limpia la pantalla del control
                Controller1.Screen.setCursor(2, 1);              // Fila 1, Columna 1
                Controller1.Screen.print("Angulo: %.2f", angle); // Imprime con 1 decimal

                wait(20, msec);
            } while (fabs(angle - targetInercia1) > 2.0);

            // Giro completado, detener motores y reiniciar para el próximo ciclo
            Left.stop();
            Right.stop();

            wait(1000, msec); // Pausa

            resetPID(pidDistancia); // Prepara el PID de avance para la próxima vez
            realizarGiro = false;   // Vuelve a la fase de avance
        }
        else
        {
            // Se realiza la fase de avance
            distancia = SensorDeDistancia.objectDistance(mm);
            Brain.Screen.clearLine(1);
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("Distancia: %.2f", distancia);     // Imprime el texto y el valor de la variable
            Controller1.Screen.clearScreen();                     // Limpia la pantalla del control
            Controller1.Screen.setCursor(1, 1);                   // Fila 1, Columna 1
            Controller1.Screen.print("Dist: %.2f mm", distancia); //
            if (distancia >= 300 && distancia <= 310)
            {
                angle = imuSensor.angle(degrees);
                targetInercia1 = angle + 90;
                realizarGiro = true; // Activa la fase de giro para la siguiente iteración
                Left.stop();
                Right.stop();
            }
            else
            {
                double powerDistancia = computerPID(pidDistancia, targetDistancia, distancia, dt);
                if (powerDistancia > 100)
                    powerDistancia = 100;

                if (powerDistancia < -100)
                    powerDistancia = -100;

                Left.spin(forward, -powerDistancia, percent);

                Right.spin(forward, -powerDistancia, percent);
            }
        }
        wait(20, msec);
    }
}
