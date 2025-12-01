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
motor LeftMotor1(PORT18, false);
motor LeftMotor2(PORT14, false);
motor LeftMotor3(PORT15, false);
motor LeftMotor4(PORT19, true);
motor_group Left(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);

// Motores del lado derecho (puertos 7-10)
motor RightMotor1(PORT2, true);
motor RightMotor2(PORT1, true);
motor RightMotor3(PORT6, true);
motor RightMotor4(PORT13, true);
motor_group Right(RightMotor1, RightMotor2, RightMotor3, RightMotor4);

// Motor para el sistema de recolección
motor RampaArriba(PORT20, false);
motor RampaAbajo(PORT16, true);

vex::pneumatics Pinza(Brain.ThreeWirePort.A);
vex::pneumatics RecolectorNeumatica(Brain.ThreeWirePort.B);
vex::pneumatics BrazoNeumatica(Brain.ThreeWirePort.C);

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
    int leftSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
    int rightSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
    Left.spin(forward, leftSpeed, percent);
    Right.spin(forward, rightSpeed, percent);
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

        // Control del motor recolector y rampa usando L1 y L2
        if (Controller1.ButtonL1.pressing())
        {
            RampaAbajo.spin(directionType::fwd, 100, velocityUnits::pct);
            RampaArriba.spin(directionType::fwd, 100, velocityUnits::pct);
        }
        else if (Controller1.ButtonL2.pressing())
        {
            RampaAbajo.spin(directionType::rev, 100, velocityUnits::pct);
            RampaArriba.spin(directionType::rev, 100, velocityUnits::pct);
        }
        else
        {
            RampaAbajo.stop(brakeType::hold);
            RampaArriba.stop(brakeType::hold);
        }

        // Control del motor recolector y rampa usando L1 y L2
        /* if (Controller1.ButtonX.pressing()) {
             Garra.spin(directionType::fwd, 100, velocityUnits::pct);
         } else if (Controller1.ButtonY.pressing()) {
             Garra.spin(directionType::rev, 100, velocityUnits::pct);
         } else {
             Garra.stop(brakeType::hold);
         }*/

        if (Controller1.ButtonR2.pressing())
        {
            // Esperamos a que el botón sea liberado para evitar múltiples activaciones en una sola pulsación
            while (Controller1.ButtonR2.pressing())
            {
                // Espera a que el botón se suelte
            }

            // Cambiamos el estado del pistón
            pistonAbierto = !pistonAbierto;

            // Ejecutamos la acción correspondiente
            if (pistonAbierto)
            {
                Pinza.open();
            }
            else
            {
                Pinza.close();
            }
        }

        if (Controller1.ButtonR1.pressing())
        {
            RecolectorNeumatica.open();
        }
        else
        {
            RecolectorNeumatica.close();
        }

        //Brazo Neumatico 
        if (Controller1.ButtonUp.pressing())
        {
            // Esperamos a que el botón sea liberado para evitar múltiples activaciones en una sola pulsación
            while (Controller1.ButtonUp.pressing())
            {
                // Espera a que el botón se suelte
            }

            // Cambiamos el estado del pistón
            pistonAbierto1 = !pistonAbierto1;

            // Ejecutamos la acción correspondiente
            if (pistonAbierto1)
            {
                BrazoNeumatica.open();
            }
            else
            {
                BrazoNeumatica.close();
            }
        }

        // Espera para evitar saturar el CPU
        task::sleep(20);
    }
}
