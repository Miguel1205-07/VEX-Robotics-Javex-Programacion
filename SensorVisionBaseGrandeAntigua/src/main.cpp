/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Miguel Casallas                                           */
/*    Created:      3/11/2024, 9:44:26                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
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
motor MotorL1(PORT14, true);  // Motor izquierdo en el puerto 1, con engranaje estándar y sin inversión de dirección
motor MotorL2(PORT18, false); // Motor izquierdo en el puerto 2, con engranaje estándar y sin inversión de dirección
motor MotorL3(PORT19, false); // Motor izquierdo en el puerto 3, con engranaje estándar y sin inversión de dirección
motor MotorL4(PORT20, true);  // Motor izquierdo en el puerto 3, con engranaje estándar y sin inversión de dirección

motor MotorR1(PORT7, false);  // Motor derecho en el puerto 4, con engranaje estándar y dirección invertida
motor MotorR2(PORT8, true);   // Motor derecho en el puerto 5, con engranaje estándar y dirección invertida
motor MotorR3(PORT9, true);   // Motor derecho en el puerto 6, con engranaje estándar y dirección invertida
motor MotorR4(PORT10, false); // Motor derecho en el puerto 6, con engranaje estándar y dirección invertida

motor Recolector(PORT1, true);
motor Rampa(PORT17, true);

vex::pneumatics Pinza(Brain.ThreeWirePort.A);
vex::pneumatics Recoleccion(Brain.ThreeWirePort.B);

// Visión
vision::signature DONAAZUL(1, -3339, -2457, -2898, 4163, 6649, 5406, 2.5, 0);
vision::signature DONAROJA(2, 6329, 9599, 7964, -901, -201, -551, 2.5, 0);
vision::signature ESTACA(3, -2287, -1627, -1957, -7241, -6429, -6835, 2.5, 0);
vex::vision Vision1(vex::PORT15, 90, DONAAZUL, DONAROJA, ESTACA);

// Stop
vex::bumper Bumper(Brain.ThreeWirePort.C);

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
  MotorL4.stop();
  MotorR1.stop();
  MotorR2.stop();
  MotorR3.stop();
  MotorR4.stop();
  Recolector.stop();
  Rampa.stop();
}

void Recoleccionmov(int speed, int duration, int speed1, int duration1)
{
  MotorL1.spin(forward, speed, percent);
  MotorL2.spin(forward, speed, percent);
  MotorL3.spin(forward, speed, percent);
  MotorR1.spin(forward, speed, percent);
  MotorR2.spin(forward, speed, percent);
  MotorR3.spin(forward, speed, percent);
  wait(duration1, seconds);
  Recolector.spin(reverse, speed1, percent);
  Rampa.spin(reverse, speed1, percent);
  wait(duration1, seconds);
}

void moveForward(int speed, double duration)
{
  MotorL1.spin(forward, speed, percent);
  MotorL2.spin(forward, speed, percent);
  MotorL3.spin(forward, speed, percent);
  MotorL4.spin(forward, speed, percent);
  MotorR1.spin(forward, speed, percent);
  MotorR2.spin(forward, speed, percent);
  MotorR3.spin(forward, speed, percent);
  MotorR4.spin(forward, speed, percent);
  wait(duration, seconds);
  stop();
}

void moveForward1(int speed, double duration)
{
  MotorL1.spin(forward, speed, percent);
  MotorL2.spin(forward, speed, percent);
  MotorL3.spin(forward, speed, percent);
  MotorL4.spin(forward, speed, percent);
  MotorR1.spin(forward, speed, percent);
  MotorR2.spin(forward, speed, percent);
  MotorR3.spin(forward, speed, percent);
  MotorR4.spin(forward, speed, percent);
  wait(duration, seconds);
  stop();
}

void moveForward2(int speedLeft, int speedRight, double duration)
{
  MotorL1.spin(forward, speedLeft, percent);
  MotorL2.spin(forward, speedLeft, percent);
  MotorL3.spin(forward, speedLeft, percent);
  MotorL4.spin(forward, speedLeft, percent);
  MotorR1.spin(forward, speedRight, percent);
  MotorR2.spin(forward, speedRight, percent);
  MotorR3.spin(forward, speedRight, percent);
  MotorR4.spin(forward, speedRight, percent);
  wait(duration, seconds);
  stop();
}

// Función para mover el robot hacia atrás
void moveBackward(int speed, int duration)
{
  MotorL1.spin(reverse, speed, percent);
  MotorL2.spin(reverse, speed, percent);
  MotorL3.spin(reverse, speed, percent);
  MotorL4.spin(reverse, speed, percent);
  MotorR1.spin(reverse, speed, percent);
  MotorR2.spin(reverse, speed, percent);
  MotorR3.spin(reverse, speed, percent);
  MotorR4.spin(reverse, speed, percent);
  wait(duration, seconds);
  stop();
}

void moveBackward1(int speedLeft, int speedRight, double duration)
{
  MotorL1.spin(reverse, speedLeft, percent);
  MotorL2.spin(reverse, speedLeft, percent);
  MotorL3.spin(reverse, speedLeft, percent);
  MotorL4.spin(reverse, speedLeft, percent);
  MotorR1.spin(reverse, speedRight, percent);
  MotorR2.spin(reverse, speedRight, percent);
  MotorR3.spin(reverse, speedRight, percent);
  MotorR4.spin(reverse, speedRight, percent);
  wait(duration, seconds);
  stop();
}
// Función para girar el robot 90 grados a la derecha
void turnRight90(int speed)
{
  MotorL1.spin(forward, speed, percent);
  MotorL2.spin(forward, speed, percent);
  MotorL3.spin(forward, speed, percent);
  MotorL4.spin(forward, speed, percent);
  MotorR1.spin(reverse, speed, percent);
  MotorR2.spin(reverse, speed, percent);
  MotorR3.spin(reverse, speed, percent);
  MotorR4.spin(reverse, speed, percent);
  wait(1.5, seconds);
  stop();
}
// Función para girar el robot 90 grados a la derecha
void turnLeft90(int speed)
{
  MotorL1.spin(reverse, speed, percent);
  MotorL2.spin(reverse, speed, percent);
  MotorL3.spin(reverse, speed, percent);
  MotorL4.spin(reverse, speed, percent);
  MotorR1.spin(forward, speed, percent);
  MotorR2.spin(forward, speed, percent);
  MotorR3.spin(forward, speed, percent);
  MotorR4.spin(forward, speed, percent);
  wait(1.5, seconds);
  stop();
}

void recoleccion(int speed)
{
  Recolector.spin(reverse, speed, percent);
  Rampa.spin(reverse, speed, percent);
}

void pinza(int duration)
{
  Pinza.close();
  wait(duration, seconds);
  stop();
}

int main()
{
  while (true)
  {
    if(Bumper.value() == 1)
    {
      stop();
    }
    while (Bumper.value() == 0)
    {
      // Establece el brillo
      Vision1.setBrightness(90);
      // Toma una captura para buscar el objeto con la firma DONAROJA
      Vision1.takeSnapshot(DONAROJA);
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("Object Count: %d", Vision1.objectCount);
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("Width: %d", Vision1.objects[0].width);
      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("Center x: %d", Vision1.objects[0].centerX);
      if (Vision1.objectCount == 0)
      {
       turnRight(30);
      }
      if (Vision1.objectCount > 0)
      {
        moveForward(10);
        recoleccion(100);
        int x = Vision1.objects[0].centerX;
        int y = Vision1.objects[0].centerY;
        int tam = Vision1.objects[0].width;
        // Ajusta la posición en función de la posición X
        if (x < 60)
        {
          moveLeft(10); // El objeto está a la izquierda
        }
        else if (x > 250)
        {
          turnRight(10); // El objeto está a la derecha
        }
        else
        {
          if (tam > 100)
          {
            moveForward(50);
            recoleccion(100);
          }
          else
          {
            moveForward(50);
          }
        }
      }
    }
    wait(20, msec); // Pausa breve para la próxima captura
  }
}
