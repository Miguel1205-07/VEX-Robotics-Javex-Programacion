/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       MSI                                                       */
/*    Created:      30/9/2025, 7:18:50                                        */
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
inertial imuSensor(PORT11);

// Sensor de Distancia
distance SensorDeDistancia = distance(PORT19);

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

void resetPID(PID &pid)
{
    pid.error = 0;
    pid.prevError = 0;
    pid.Integral = 0;
}

int main()
{
    imuSensor.calibrate();

    while (imuSensor.isCalibrating())
    {
        wait(20, msec);
    }

    double distanciaAvanzar1_cm = 102;
    double gradosMotorObjetivo1 = (distanciaAvanzar1_cm * 3.444);

    double distanciaAvanzar2_cm = 62;
    double gradosMotorObjetivo2 = (distanciaAvanzar2_cm * 3.444);

    double distanciaAvanzar3_cm = 85;
    double gradosMotorObjetivo3 = (distanciaAvanzar3_cm * 3.444);

    double distanciaAvanzar4_cm = 125;
    double gradosMotorObjetivo4 = (distanciaAvanzar4_cm * 3.444);

    PID pidRecto;
    pidRecto.kd = 0;
    pidRecto.ki = 0;
    pidRecto.kp = 0.1;

    PID pidGiro;
    pidGiro.kd = 0.01;
    pidGiro.ki = 0.018;
    pidGiro.kp = 0.2;

    PID pidDistancia;
    pidDistancia.kp = 0.2;
    pidDistancia.ki = 0.001;
    pidDistancia.kd = 0.003;

    double setpointAngulo = 0;
    timer t;
    double lastTime = t.time(msec);
    double targetAngulo1 = 90;

    Left.resetPosition();
    Right.resetPosition();

    // Bucle para corrección mientras estén girando
    while (fabs(gradosMotorObjetivo1 - ((Left.position(degrees) + Right.position(degrees)) / 2)) > 5)
    {
        // --- Cálculo del delta de tiempo (dt) ---
        double currentTime = t.time(msec);
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        // --- 1. Calcular la potencia base con el PID de Distancia ---
        double posicionPromedioActual = ((Left.position(degrees) + Right.position(degrees)) / 2);
        double potenciaBase = computerPID(pidDistancia, gradosMotorObjetivo1, posicionPromedioActual, dt);

        // --- 2. Calcular la corrección con el PID de Giro ---
        double anguloActual = imuSensor.rotation();
        double correccionGiro = computerPID(pidRecto, setpointAngulo, anguloActual, dt);

        // --- Limitar la potencia y la corrección para evitar valores extremos ---
        if (potenciaBase > 100)
            potenciaBase = 100;
        if (potenciaBase < -100)
            potenciaBase = -100;

        // --- 3. Combinar y aplicar las potencias a los motores ---
        double leftPower = potenciaBase - correccionGiro;
        double rightPower = potenciaBase + correccionGiro;

        Left.spin(forward, leftPower, percent);
        Right.spin(forward, rightPower, percent);

        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("PromActual %.1f", posicionPromedioActual);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", potenciaBase);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Objetivo: %.1f", gradosMotorObjetivo1);
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Izquierda: %.1f", Left.position(degrees));
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Derecha: %.1f", Right.position(degrees));

        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    resetPID(pidRecto);
    Brain.Screen.clearScreen();

    while (fabs(targetAngulo1 - imuSensor.rotation(degrees)) > 2.0)
    {
        double currentTime = t.time(msec);

        double dt = (currentTime - lastTime) / 1000.0;

        lastTime = currentTime;

        double angle = imuSensor.rotation(degrees);
        Brain.Screen.setFont(mono40); // Letra grande para verla bien
        double power = computerPID(pidGiro, targetAngulo1, angle, dt);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Angulo: %.1f", angle);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", power);

        if (power > 100)
            power = 100;

        if (power < -100)
            power = -100;

        Left.spin(forward, power, percent);
        Right.spin(forward, -power, percent);
        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    Left.resetPosition();
    Right.resetPosition();

    while (fabs(gradosMotorObjetivo2 - ((Left.position(degrees) + Right.position(degrees)) / 2)) > 5)
    {
        // --- Cálculo del delta de tiempo (dt) ---
        double currentTime = t.time(msec);
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        // --- 1. Calcular la potencia base con el PID de Distancia ---
        double posicionPromedioActual = ((Left.position(degrees) + Right.position(degrees)) / 2);
        double potenciaBase = computerPID(pidDistancia, gradosMotorObjetivo2, posicionPromedioActual, dt);

        // --- Limitar la potencia y la corrección para evitar valores extremos ---
        if (potenciaBase > 100)
            potenciaBase = 100;
        if (potenciaBase < -100)
            potenciaBase = -100;

        // --- 3. Combinar y aplicar las potencias a los motores ---
        double leftPower = potenciaBase;
        double rightPower = potenciaBase;

        Left.spin(forward, leftPower, percent);
        Right.spin(forward, rightPower, percent);

        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("PromActual %.1f", posicionPromedioActual);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", potenciaBase);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Objetivo: %.1f", gradosMotorObjetivo1);
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Izquierda: %.1f", Left.position(degrees));
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Derecha: %.1f", Right.position(degrees));

        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    resetPID(pidDistancia);

    double targetAngulo2 = 45;

    while (fabs(targetAngulo2 - imuSensor.rotation(degrees)) > 4.0)
    {
        double currentTime = t.time(msec);

        double dt = (currentTime - lastTime) / 1000.0;

        lastTime = currentTime;

        double angle = imuSensor.rotation(degrees);
        Brain.Screen.setFont(mono40); // Letra grande para verla bien
        double power = computerPID(pidGiro, targetAngulo2, angle, dt);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Angulo: %.1f", angle);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", power);

        if (power > 100)
            power = 100;

        if (power < -100)
            power = -100;

        Left.spin(forward, power, percent);
        Right.spin(forward, -power, percent);
        wait(20, msec);
    }

    Left.stop(brake);
    Right.stop(brake);
    Left.resetPosition();
    Right.resetPosition();

    while (fabs(gradosMotorObjetivo3 - ((Left.position(degrees) + Right.position(degrees)) / 2)) > 5)
    {
        // --- Cálculo del delta de tiempo (dt) ---
        double currentTime = t.time(msec);
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        // --- 1. Calcular la potencia base con el PID de Distancia ---
        double posicionPromedioActual = ((Left.position(degrees) + Right.position(degrees)) / 2);
        double potenciaBase = computerPID(pidDistancia, gradosMotorObjetivo3, posicionPromedioActual, dt);

        // --- Limitar la potencia y la corrección para evitar valores extremos ---
        if (potenciaBase > 100)
            potenciaBase = 100;
        if (potenciaBase < -100)
            potenciaBase = -100;

        // --- 3. Combinar y aplicar las potencias a los motores ---
        double leftPower = potenciaBase;
        double rightPower = potenciaBase;

        Left.spin(forward, leftPower, percent);
        Right.spin(forward, rightPower, percent);

        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("PromActual %.1f", posicionPromedioActual);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", potenciaBase);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Objetivo: %.1f", gradosMotorObjetivo1);
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Izquierda: %.1f", Left.position(degrees));
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Derecha: %.1f", Right.position(degrees));

        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    resetPID(pidDistancia);
    double targetAngulo3 = -45;

    while (fabs(targetAngulo3 - imuSensor.rotation(degrees)) > 2.0)
    {
        double currentTime = t.time(msec);

        double dt = (currentTime - lastTime) / 1000.0;

        lastTime = currentTime;

        double angle = imuSensor.rotation(degrees);
        Brain.Screen.setFont(mono40); // Letra grande para verla bien
        double power = computerPID(pidGiro, targetAngulo3, angle, dt);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Angulo: %.1f", angle);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", power);

        if (power > 100)
            power = 100;

        if (power < -100)
            power = -100;

        Left.spin(forward, power, percent);
        Right.spin(forward, -power, percent);
        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    Left.resetPosition();
    Right.resetPosition();

    while (fabs(gradosMotorObjetivo3 - ((Left.position(degrees) + Right.position(degrees)) / 2)) > 5)
    {
        // --- Cálculo del delta de tiempo (dt) ---
        double currentTime = t.time(msec);
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        // --- 1. Calcular la potencia base con el PID de Distancia ---
        double posicionPromedioActual = ((Left.position(degrees) + Right.position(degrees)) / 2);
        double potenciaBase = computerPID(pidDistancia, gradosMotorObjetivo3, posicionPromedioActual, dt);

        // --- Limitar la potencia y la corrección para evitar valores extremos ---
        if (potenciaBase > 100)
            potenciaBase = 100;
        if (potenciaBase < -100)
            potenciaBase = -100;

        // --- 3. Combinar y aplicar las potencias a los motores ---
        double leftPower = potenciaBase;
        double rightPower = potenciaBase;

        Left.spin(forward, leftPower, percent);
        Right.spin(forward, rightPower, percent);

        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("PromActual %.1f", posicionPromedioActual);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", potenciaBase);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Objetivo: %.1f", gradosMotorObjetivo1);
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Izquierda: %.1f", Left.position(degrees));
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Derecha: %.1f", Right.position(degrees));

        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    resetPID(pidDistancia);

    double targetAngulo4 = -90;

    while (fabs(targetAngulo4 - imuSensor.rotation(degrees)) > 2.0)
    {
        double currentTime = t.time(msec);

        double dt = (currentTime - lastTime) / 1000.0;

        lastTime = currentTime;

        double angle = imuSensor.rotation(degrees);
        Brain.Screen.setFont(mono40); // Letra grande para verla bien
        double power = computerPID(pidGiro, targetAngulo4, angle, dt);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Angulo: %.1f", angle);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", power);

        if (power > 100)
            power = 100;

        if (power < -100)
            power = -100;

        Left.spin(forward, power, percent);
        Right.spin(forward, -power, percent);
        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    Left.resetPosition();
    Right.resetPosition();

    while (fabs(gradosMotorObjetivo4 - ((Left.position(degrees) + Right.position(degrees)) / 2)) > 5)
    {
        // --- Cálculo del delta de tiempo (dt) ---
        double currentTime = t.time(msec);
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        // --- 1. Calcular la potencia base con el PID de Distancia ---
        double posicionPromedioActual = ((Left.position(degrees) + Right.position(degrees)) / 2);
        double potenciaBase = computerPID(pidDistancia, gradosMotorObjetivo4, posicionPromedioActual, dt);

        // --- Limitar la potencia y la corrección para evitar valores extremos ---
        if (potenciaBase > 100)
            potenciaBase = 100;
        if (potenciaBase < -100)
            potenciaBase = -100;

        // --- 3. Combinar y aplicar las potencias a los motores ---
        double leftPower = potenciaBase;
        double rightPower = potenciaBase;

        Left.spin(forward, leftPower, percent);
        Right.spin(forward, rightPower, percent);

        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("PromActual %.1f", posicionPromedioActual);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", potenciaBase);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Objetivo: %.1f", gradosMotorObjetivo1);
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Izquierda: %.1f", Left.position(degrees));
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Derecha: %.1f", Right.position(degrees));

        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    resetPID(pidDistancia);

    double targetAngulo5 = -135;

    while (fabs(targetAngulo5 - imuSensor.rotation(degrees)) > 2.0)
    {
        double currentTime = t.time(msec);

        double dt = (currentTime - lastTime) / 1000.0;

        lastTime = currentTime;

        double angle = imuSensor.rotation(degrees);
        Brain.Screen.setFont(mono40); // Letra grande para verla bien
        double power = computerPID(pidGiro, targetAngulo5, angle, dt);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Angulo: %.1f", angle);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", power);

        if (power > 100)
            power = 100;

        if (power < -100)
            power = -100;

        Left.spin(forward, power, percent);
        Right.spin(forward, -power, percent);
        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    Left.resetPosition();
    Right.resetPosition();

    while (fabs(gradosMotorObjetivo3 - ((Left.position(degrees) + Right.position(degrees)) / 2)) > 5)
    {
        // --- Cálculo del delta de tiempo (dt) ---
        double currentTime = t.time(msec);
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        // --- 1. Calcular la potencia base con el PID de Distancia ---
        double posicionPromedioActual = ((Left.position(degrees) + Right.position(degrees)) / 2);
        double potenciaBase = computerPID(pidDistancia, gradosMotorObjetivo3, posicionPromedioActual, dt);

        // --- Limitar la potencia y la corrección para evitar valores extremos ---
        if (potenciaBase > 100)
            potenciaBase = 100;
        if (potenciaBase < -100)
            potenciaBase = -100;

        // --- 3. Combinar y aplicar las potencias a los motores ---
        double leftPower = potenciaBase;
        double rightPower = potenciaBase;

        Left.spin(forward, leftPower, percent);
        Right.spin(forward, rightPower, percent);

        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("PromActual %.1f", posicionPromedioActual);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", potenciaBase);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Objetivo: %.1f", gradosMotorObjetivo1);
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Izquierda: %.1f", Left.position(degrees));
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Derecha: %.1f", Right.position(degrees));

        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    resetPID(pidDistancia);

    double targetAngulo6 = -225;

    while (fabs(targetAngulo6 - imuSensor.rotation(degrees)) > 2.0)
    {
        double currentTime = t.time(msec);

        double dt = (currentTime - lastTime) / 1000.0;

        lastTime = currentTime;

        double angle = imuSensor.rotation(degrees);
        Brain.Screen.setFont(mono40); // Letra grande para verla bien
        double power = computerPID(pidGiro, targetAngulo6, angle, dt);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Angulo: %.1f", angle);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", power);

        if (power > 100)
            power = 100;

        if (power < -100)
            power = -100;

        Left.spin(forward, power, percent);
        Right.spin(forward, -power, percent);
        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    Left.resetPosition();
    Right.resetPosition();

    while (fabs(gradosMotorObjetivo3 - ((Left.position(degrees) + Right.position(degrees)) / 2)) > 5)
    {
        // --- Cálculo del delta de tiempo (dt) ---
        double currentTime = t.time(msec);
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        // --- 1. Calcular la potencia base con el PID de Distancia ---
        double posicionPromedioActual = ((Left.position(degrees) + Right.position(degrees)) / 2);
        double potenciaBase = computerPID(pidDistancia, gradosMotorObjetivo3, posicionPromedioActual, dt);

        // --- Limitar la potencia y la corrección para evitar valores extremos ---
        if (potenciaBase > 100)
            potenciaBase = 100;
        if (potenciaBase < -100)
            potenciaBase = -100;

        // --- 3. Combinar y aplicar las potencias a los motores ---
        double leftPower = potenciaBase;
        double rightPower = potenciaBase;

        Left.spin(forward, leftPower, percent);
        Right.spin(forward, rightPower, percent);

        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("PromActual %.1f", posicionPromedioActual);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", potenciaBase);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Objetivo: %.1f", gradosMotorObjetivo1);
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Izquierda: %.1f", Left.position(degrees));
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Derecha: %.1f", Right.position(degrees));

        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    resetPID(pidDistancia);

    double targetAngulo7 = -315;

    while (fabs(targetAngulo7 - imuSensor.rotation(degrees)) > 2.0)
    {
        double currentTime = t.time(msec);

        double dt = (currentTime - lastTime) / 1000.0;

        lastTime = currentTime;

        double angle = imuSensor.rotation(degrees);
        Brain.Screen.setFont(mono40); // Letra grande para verla bien
        double power = computerPID(pidGiro, targetAngulo7, angle, dt);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Angulo: %.1f", angle);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", power);

        if (power > 100)
            power = 100;

        if (power < -100)
            power = -100;

        Left.spin(forward, power, percent);
        Right.spin(forward, -power, percent);
        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
    Left.resetPosition();
    Right.resetPosition();
    while (fabs(gradosMotorObjetivo3 - ((Left.position(degrees) + Right.position(degrees)) / 2)) > 5)
    {
        // --- Cálculo del delta de tiempo (dt) ---
        double currentTime = t.time(msec);
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        // --- 1. Calcular la potencia base con el PID de Distancia ---
        double posicionPromedioActual = ((Left.position(degrees) + Right.position(degrees)) / 2);
        double potenciaBase = computerPID(pidDistancia, gradosMotorObjetivo3, posicionPromedioActual, dt);

        // --- Limitar la potencia y la corrección para evitar valores extremos ---
        if (potenciaBase > 100)
            potenciaBase = 100;
        if (potenciaBase < -100)
            potenciaBase = -100;

        // --- 3. Combinar y aplicar las potencias a los motores ---
        double leftPower = potenciaBase;
        double rightPower = potenciaBase;

        Left.spin(forward, leftPower, percent);
        Right.spin(forward, rightPower, percent);

        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("PromActual %.1f", posicionPromedioActual);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", potenciaBase);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Objetivo: %.1f", gradosMotorObjetivo1);
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Izquierda: %.1f", Left.position(degrees));
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Derecha: %.1f", Right.position(degrees));

        wait(20, msec);
    }
    Left.stop(brake);
    Right.stop(brake);
}
