// INVERSE KINEMATICS for Scara arm. This code not used - code incorporated in robotGeometry.cpp
#include <math.h>
#include <Arduino.h>

double x = 10.0;
double y = 10.0;
double L1 = 228; // L1 = 228mm
double L2 = 136.5; // L2 = 136.5mm
double theta1, theta2, z;

void inverseKinematics(float x, float y) {
  theta2 = acos((sq(x) + sq(y) - sq(L1) - sq(L2)) / (2 * L1 * L2));
  if (x < 0 & y < 0) {
    theta2 = (-1) * theta2;
  }
  
  theta1 = atan(x / y) - atan((L2 * sin(theta2)) / (L1 + L2 * cos(theta2)));
  
  theta2 = (-1) * theta2 * 180 / PI;
  theta1 = theta1 * 180 / PI;

 // Angles adjustment depending in which quadrant the final tool coordinate x,y is
  if (x >= 0 & y >= 0) {       // 1st quadrant
    theta1 = 90 - theta1;
  }
  if (x < 0 & y > 0) {       // 2nd quadrant
    theta1 = 90 - theta1;
  }
  if (x < 0 & y < 0) {       // 3d quadrant
    theta1 = 270 - theta1;
  }
  if (x > 0 & y < 0) {       // 4th quadrant
    theta1 = -90 - theta1;
  }
  if (x < 0 & y == 0) {
    theta1 = 270 + theta1;
  }
  
}
