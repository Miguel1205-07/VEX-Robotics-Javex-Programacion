/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       MSI                                                       */
/*    Created:      1/8/2025, 15:41:15                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "vex.h"
#include <iostream>

using namespace vex;

/* Funciones
void moveForward(int speed, int duration);
void moveBackward(int speed, int duration);
void turnRight90(int speed);
void turnLeft90(int speed);
void stop(); */

// Configuración del cerebro y los motores
brain Brain;

// Crear instancias de los motores y asignarlos a los puertos correspondientes del cerebro
motor MotorL1(PORT5, false); 
motor MotorL2(PORT3, false);
motor MotorL3(PORT20, false); 

motor MotorR1(PORT1, true); 
motor MotorR2(PORT4, true);  
motor MotorR3(PORT11, true);  

// Sensor de inercia
inertial InertialSensor(PORT10);


// Función para mover el robot hacia adelante sin tiempo
void moveForward(int speed)
{
    MotorL1.spin(forward, speed, percent);
    MotorL2.spin(forward, speed, percent);
    MotorL3.spin(forward, speed, percent);
    MotorR1.spin(forward, speed, percent);
    MotorR2.spin(forward, speed, percent);
    MotorR3.spin(forward, speed, percent);
}

// Función para girar a la derecha
void turnRight(int speed)
{
    MotorL1.spin(forward, speed, percent);
    MotorL2.spin(forward, speed, percent);
    MotorL3.spin(forward, speed, percent);
    MotorR1.spin(reverse, speed, percent);
    MotorR2.spin(reverse, speed, percent);
    MotorR3.spin(reverse, speed, percent);
}

// Función para girar a la izquierda
void moveLeft(int speed)
{
    MotorL1.spin(reverse, speed, percent);
    MotorL2.spin(reverse, speed, percent);
    MotorL3.spin(reverse, speed, percent);
    MotorR1.spin(forward, speed, percent);
    MotorR2.spin(forward, speed, percent);
    MotorR3.spin(forward, speed, percent);
}

// Función para mover el robot hacia la izquierda (giro en arco)
void moveLeft(int speed, int duration)
{
    MotorL1.spin(reverse, speed, percent);
    MotorL2.spin(reverse, speed, percent);
    MotorL3.spin(reverse, speed, percent);
    MotorR1.spin(forward, speed, percent);
    MotorR2.spin(forward, speed, percent);
    MotorR3.spin(forward, speed, percent);
    wait(duration, seconds);
    MotorL1.stop();
    MotorL2.stop();
    MotorL3.stop();
    MotorR1.stop();
    MotorR2.stop();
    MotorR3.stop();
}

// Funcion detener
void stop()
{
    MotorL1.stop();
    MotorL2.stop();
    MotorL3.stop();
    MotorR1.stop();
    MotorR2.stop();
    MotorR3.stop();
}


void moveForward(int speed, double duration)
{
    MotorL1.spin(forward, speed, percent);
    MotorL2.spin(forward, speed, percent);
    MotorL3.spin(forward, speed, percent);
    MotorR1.spin(forward, speed, percent);
    MotorR2.spin(forward, speed, percent);
    MotorR3.spin(forward, speed, percent);
    wait(duration, seconds);
    stop();
}

int main() {
    InertialSensor.calibrate();
    while (InertialSensor.isCalibrating()) wait(100, msec);
  
    InertialSensor.setRotation(0, degrees);
  
    double targetAngle = 90;
    double threshold = 1; // margen de error
    double slowZone = 20; // zona para bajar velocidad
    int fastSpeed = 30;   // velocidad alta
    int slowSpeed = 10;   // velocidad baja
  
    while (true) {
      double angle = InertialSensor.rotation();
      double error = targetAngle - angle;
  
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("Angle: %.2f", angle);
  
      if (fabs(error) <= threshold) {
        stop();
        break;
      }
  
      if (error > 0) {
        if (error > slowZone) {
          turnRight(fastSpeed);
        } else {
          turnRight(slowSpeed);
        }
      } else {
        if (error < -slowZone) {
          moveLeft(fastSpeed);
        } else {
          moveLeft(slowSpeed);
        }
      }
  
      wait(20, msec);
    }
  
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Giro Terminado!");
  }