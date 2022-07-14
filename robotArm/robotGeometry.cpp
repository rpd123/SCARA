#include "robotGeometry.h"
#include "config.h"
#include <math.h>
#include <Arduino.h>

RobotGeometry::RobotGeometry() {

}

void RobotGeometry::set(float axmm, float aymm, float azmm) {
  xmm = axmm;
  ymm = aymm;
  zmm = azmm;
  calculateGrad();
}

float RobotGeometry::getXmm() const {
  return xmm;
}

float RobotGeometry::getYmm() const {
  return ymm;
}

float RobotGeometry::getZmm() const {
  return zmm;
}

float RobotGeometry::getRotRad() const {
  return rot;
}

float RobotGeometry::getLowRad() const {
  return low;
}

float RobotGeometry::getHighRad() const {
  return high;
}

void RobotGeometry::calculateGrad() {

  if (!SCARA) {
      float rrot =  sqrt((xmm * xmm) + (ymm * ymm));    //radius from Top View
      float rside = sqrt((rrot * rrot) + (zmm * zmm));  //radius from Side View. Use rrot instead of ymm..for everything

      rot = asin(xmm / rrot);
      //Angle of Higher Stepper Motor
      //high = acos((rside * 0.5) / 120.0) * 2.0;  //120mm shank length
      high = acos((rside * 0.5) / 180.0) * 2.0; //180mm shank length

      //Angle of Lower Stepper Motor  (asin()=Angle To Gripper)
      if (zmm > 0) {
        low =      asin(rrot / rside) + ((PI - high) / 2.0) - (PI / 2.0);
      } else {
        low = PI - asin(rrot / rside) + ((PI - high) / 2.0) - (PI / 2.0);
      }

      //correct higher Angle as it is mechanically bounded width lower Motor
      high = high + low;
  }else{
      // modified from howtomechatronics.com/projects/scara-robot-how-to-build-your-own-arduino-based-robot/
      
      high = acos((sq(xmm) + sq(ymm) - sq(L1) - sq(L2)) / (2 * L1 * L2));
      if (xmm < 0 & ymm < 0) {
        high = (-1) * high;
      }
      
      low = atan(xmm / ymm) - atan((L2 * sin(high)) / (L1 + L2 * cos(high)));
      
     // high = (-1) * high * 180 / PI;
      high = (-1) * high;
      //low = low * 180 / PI;

     // Angles adjustment depending in which quadrant the final tool coordinate x,y is
      if (xmm >= 0 & ymm >= 0) {       // 1st quadrant
        low = (PI / 2.0) - low;
      }
      if (xmm < 0 & ymm > 0) {       // 2nd quadrant
        low = (PI / 2.0) - low;
      }
      if (xmm < 0 & ymm < 0) {       // 3d quadrant
        low = (PI * 1.5) - low;
      }
      if (xmm > 0 & ymm < 0) {       // 4th quadrant
        low = -(PI / 2.0) - low;
      }
      if (xmm < 0 & ymm == 0) {
        low = (PI * 1.5) + low;
      }
      //low=round(low);  //rounding radians would be mad!
      //high=round(high);
      //SERIALX.println("L" + String(low));
      rot = round((PI * 2) * zmm / LEAD);   // height in radians
  }
}
