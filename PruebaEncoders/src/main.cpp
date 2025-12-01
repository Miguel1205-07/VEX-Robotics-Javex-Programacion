/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       MSI                                                       */
/*    Created:      1/10/2025, 13:10:07                                       */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

brain Brain;
controller Controller1;

motor MotorPrueba(PORT20, true);

int main()
{
    // --- Constantes del Robot ---
    // NOTA: El diámetro de las ruedas de 4 pulgadas es 10.16 cm, no 10.5.
    // Usar el valor correcto mejorará la precisión en distancias reales.
    double diametroRueda_cm = 10.16; 
    double circunferenciaRueda_cm = diametroRueda_cm * 3.14159;

    // --- Objetivo del Movimiento ---
    // Para dar una vuelta exacta, la distancia debe ser la circunferencia.
    double distanciaAvanzar_cm = 50; 
    
    // Esto calculará exactamente 360 grados.
    double gradosMotorObjetivo = (distanciaAvanzar_cm / circunferenciaRueda_cm) * 360;

    // --- Ejecución ---
    MotorPrueba.resetPosition();
    
    // spinFor es la mejor opción: es preciso y simple.
    // Gira hacia adelante, la cantidad de grados objetivo, a una velocidad de 50%.
    MotorPrueba.spinFor(forward, gradosMotorObjetivo, degrees, 50, velocityUnits::pct);

    // El programa esperará aquí hasta que el motor termine y luego continuará.
    // No necesitas un bucle ni un comando .stop() manual.
}