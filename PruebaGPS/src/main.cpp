/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       MSI                                                       */
/*    Created:      27/10/2025, 15:42:21                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <cmath> // Necesitamos esta librería para matemáticas (atan2, hypot)

using namespace vex;

// --- Definiciones de Dispositivos ---
// (Estos se crean automáticamente en robot-config.h cuando los configuras)
brain Brain;
controller Controller1;
//
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

gps GPS(PORT10, 200, 190, mm, 90);

void Avanzar(int targetX, int targetY);

int main()
{
    GPS.calibrate();
    while (GPS.isCalibrating())
    {
        wait(20, msec);
    }
    Avanzar(30, 0);
}

void Avanzar(int targetX, int targetY)
{
    double errorDeAvance = 999;
    double errorDeGiro = 999;

    // Bucle principal: Repetir hasta que estemos cerca (ej. a 10mm)
    while (fabs(errorDeAvance) > 10)
    {

        // --- 1. Obtener estado actual ---
        double currentX = GPS.xPosition(mm);
        double currentY = GPS.yPosition(mm);
        double currentHeading = GPS.heading(degrees); // 0-360

        // --- 2. Calcular Deltas ---
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        // --- 3. Calcular Error de Avance (Distancia) ---
        errorDeAvance = hypot(deltaX, deltaY);

        // --- 4. Calcular Ángulo Deseado (hacia el objetivo) ---
        double anguloDeseado_rad = atan2(deltaY, deltaX);
        double anguloDeseado_deg = anguloDeseado_rad * (180.0 / M_PI);

        // Convertir el ángulo del sistema matemático(0 = derecha)
        // al sistema de rumbo del GPS VEX (0=arriba).
        double anguloDeseado_VEX = 90.0 - anguloDeseado_deg;

        // Convertir atan2 (-180 a 180) a (0 a 360) para coincidir con el GPS
        if (anguloDeseado_deg < 0)
        {
            anguloDeseado_deg += 360;
        }

        // --- 5. Calcular Error de Giro (Rumbo) ---
        errorDeGiro = anguloDeseado_deg - currentHeading;

        if (errorDeGiro > 180)
        {
            errorDeGiro -= 360;
        }
        if (errorDeGiro < -180)
        {
            errorDeGiro += 360;
        }

        // --- 6. Calcular Potencia ---
        double potenciaDeAvance = errorDeAvance;
        double potenciaDeGiro = errorDeGiro;

        // --- 7. Mover Motores ---
        Left.spin(forward, potenciaDeAvance + potenciaDeGiro, percent);
        Right.spin(forward, potenciaDeAvance - potenciaDeGiro, percent);
        wait(20,msec);
    }
    Left.stop(brake);
Right.stop(brake);
}