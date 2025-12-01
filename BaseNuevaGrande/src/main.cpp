/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       aulasingenieria                                           */
/*    Created:      3/14/2025, 11:47:31 AM                                    */
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
// | [L1]         [R1] |
// |                   |
// | [L2]         [R2] |
// |  A[L3]     A[R3]  |
// | [L4]         [R4] |
//  --------------------

// Motores del lado izquierdo (puertos 1-3)
motor LeftMotor1(PORT1, false);
motor LeftMotor2(PORT2, false);
motor LeftMotor3(PORT3, true);
motor LeftMotor4(PORT4, true);
motor_group Left(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);

// Motores del lado derecho (puertos 9-10)
motor RightMotor1(PORT11, false);
motor RightMotor2(PORT12, true);
motor RightMotor3(PORT13, true);
motor RightMotor4(PORT14, false);
motor_group Right(RightMotor1, RightMotor2, RightMotor3, RightMotor4);

//Motor Central 
/*motor CentralMotor1(PORT11,false);
motor CentralMotor2(PORT11,true);
motor_group Centro(CentralMotor1,CentralMotor2);*/

// Modo de control
int controlMode = 0;
bool pistonAbierto = false;
bool pistonAbierto1 = false;

// Función para cambiar el modo de control
void switchControlMode()
{
    controlMode = (controlMode + 1) % 4;
    Brain.Screen.clearScreen();
    Brain.Screen.print("Modo de control: %d", controlMode);
}

// Función para controlar con dos joysticks
void twoJoysticksControl()
{
    int rightSpeed = Controller1.Axis3.position()-Controller1.Axis1.position();
    int leftSpeed = Controller1.Axis3.position()+Controller1.Axis1.position();
    int lateralSpeed = Controller1.Axis1.position();
    Left.spin(forward, leftSpeed, percent);
    Right.spin(forward, rightSpeed, percent);
   // Centro.spin(forward,lateralSpeed, percent);
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
        // Cambiar el modo de control con el botón A
        if (Controller1.ButtonA.pressing())
        {
            switchControlMode();
            while (Controller1.ButtonA.pressing())
            {
                // Esperar a que se suelte el botón A
                task::sleep(10);
            }
        }

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
        task::sleep(20);
    }
}
