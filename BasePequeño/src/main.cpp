/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Miguel Casallas                                           */
/*    Created:      19/11/2024, 8:35:53                                       */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

// Inicialización de objetos del dispositivo VEX
using namespace vex;

// Configuración del cerebro, controlador y motores
brain Brain;
controller Controller1;

//  ________
// |  [LA1]    [RA1]    |
// |[LB2]         [RB2] |
// |                    |
// |  [LA3]    [RA3]    |
// |[L4B]         [RB4] |
//  --------------------

// Motores del lado izquierdo (puertos 1-4)
motor LeftMotor1(PORT5, true);
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

// Motores Recolección
motor Recolección1(PORT11, false);
motor Recolección2(PORT15, true);
motor Recolección3(PORT18, true);
motor Recolección4(PORT20, false);
motor Recolección5(PORT19, false);
motor MotBloqueo(PORT12, false);
motor_group Recoleccion(Recolección1, Recolección2, Recolección3, Recolección4);
digital_out ValvulaNeumaticaRecol = digital_out(Brain.ThreeWirePort.A);

// Control Recolección
bool ActivacionRecoleccion = false;
bool ActivacionRecoleccionReversa = false;
bool ActivacionRecoleccionCenter = false;
bool Bloqueo = false;
bool RecoleccionNeumatica = false;

// Modo de control
int controlMode = 0;

// Función para cambiar el modo de control
/*void switchControlMode()
{
    controlMode = (controlMode + 1) % 4;
    Brain.Screen.clearScreen();
    Brain.Screen.print("Modo de control: %d", controlMode);
}*/

// Función para controlar con dos joysticks
void twoJoysticksControl()
{
    int leftSpeed = Controller1.Axis3.position() + 0.5*Controller1.Axis1.position();
    int rightSpeed = Controller1.Axis3.position() - 0.5*Controller1.Axis1.position();
    Left.spin(forward, leftSpeed, percent);
    Right.spin(forward, rightSpeed, percent);
    // Cambiar el modo de control con el botón A
    /*if (Controller1.ButtonA.pressing())
    {
        switchControlMode();
        while (Controller1.ButtonA.pressing())
        {
            // Esperar a que se suelte el botón A
            wait(10, msec);
        }
    }*/

    if (Controller1.ButtonB.pressing())
    {
        ActivacionRecoleccion = !ActivacionRecoleccion;
        ActivacionRecoleccionReversa = false;
        ActivacionRecoleccionCenter = false;
        while (Controller1.ButtonB.pressing())
        {
            wait(10, msec);
        }
    }

    if (Controller1.ButtonY.pressing())
    {
        ActivacionRecoleccionReversa = !ActivacionRecoleccionReversa;
        ActivacionRecoleccion = false;
        ActivacionRecoleccionCenter = false;
        while (Controller1.ButtonY.pressing())
        {
            wait(10, msec);
        }
    }

    if (Controller1.ButtonX.pressing())
    {
        ActivacionRecoleccionCenter = !ActivacionRecoleccionCenter;
        ActivacionRecoleccion = false;
        ActivacionRecoleccionReversa = false;
        while (Controller1.ButtonX.pressing())
        {
            wait(10, msec);
        }
    }

    if(Controller1.ButtonR1.pressing())
    {
        RecoleccionNeumatica = !RecoleccionNeumatica;
         while (Controller1.ButtonR1.pressing())
        {
            wait(10, msec);
        }

    }

    if (ActivacionRecoleccion == true)
    {
        Recoleccion.spin(forward, 100, percent);
        Recolección5.spin(forward, 100, percent);
    }
    else if (ActivacionRecoleccionReversa == true)
    {
        Recoleccion.spin(reverse, 100, percent);
        Recolección5.spin(reverse, 100, percent);
    }
    else if (ActivacionRecoleccionCenter == true)
    {
        Recoleccion.spin(reverse, 100, percent);
        Recolección5.spin(forward, 100, percent);
    }
    else
    {
        Recoleccion.stop();
        Recolección5.stop(brake);
    }

    if(RecoleccionNeumatica== false)
    {
        ValvulaNeumaticaRecol.set(false);
    }
    if(RecoleccionNeumatica== true)
    {
        ValvulaNeumaticaRecol.set(true);
    }
}

// Función para controlar con un joystick
void singleJoystickControl()
{
    int forwardSpeed = Controller1.Axis3.position();
    int turnSpeed = Controller1.Axis4.position();
    Left.spin(forward, forwardSpeed + turnSpeed, percent);
    Right.spin(forward, forwardSpeed - turnSpeed, percent);
}

// Función para controlar con las flechas
void arrowControl()
{
    if (Controller1.ButtonUp.pressing())
    {
        Left.spin(forward, 100, percent);
        Right.spin(forward, 100, percent);
    }
    else if (Controller1.ButtonDown.pressing())
    {
        Left.spin(reverse, 100, percent);
        Right.spin(reverse, 100, percent);
    }
    else if (Controller1.ButtonLeft.pressing())
    {
        Left.spin(reverse, 100, percent);
        Right.spin(forward, 100, percent);
    }
    else if (Controller1.ButtonRight.pressing())
    {
        Left.spin(forward, 100, percent);
        Right.spin(reverse, 100, percent);
    }
    else
    {
        Left.stop();
        Right.stop();
    }
}
void joystickNewControl()
{
    int leftSpeed = Controller1.Axis3.position();
    int rightSpeed = Controller1.Axis2.position();
    Left.spin(forward, leftSpeed, percent);
    Right.spin(forward, rightSpeed, percent);
}

// Función principal
int main()
{
    while (true)
    {

        // Control del robot basado en el modo seleccionado
        if (controlMode == 0)
        {
            twoJoysticksControl();
        }
        else if (controlMode == 1)
        {
            singleJoystickControl();
        }
        else if (controlMode == 2)
        {
            arrowControl();
        }
        else if (controlMode == 3)
        {
            joystickNewControl();
        }

        // Espera para evitar saturar el CPU
        wait(20, msec);
    }
}
