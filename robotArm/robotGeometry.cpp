#include "robotGeometry.h"
#include "config.h"
#include <math.h>
#include <Arduino.h>

float lawOfCosines(float a, float b, float c) 
{
  return acosf((a*a + b*b - c*c) / (2.0f * a * b));
}

bool RobotGeometry::elbow = 0;
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
      // 
      float dist = sqrt(xmm*xmm +ymm*ymm);
      float D1,D2;
      if (dist > (L1+L2)){
        dist = (L1+L2)-0.001f;
        SERIALX.println("IK overflow->limit");
      }
      //bool elbow = 0; // elbow is initially static and = 0
      
      if (xmm > 0 && ymm < (L1+L2)) {
        elbow = 0;
      }
      if (xmm > 135) {   // parked
        elbow = 0;
      }
      if (xmm < 0 && ymm < (L1+L2)) {
        elbow = 1;
      }
      
      if (elbow==1)  // inverse elbow solution: reverse X axis, and final angles.
        xmm = -xmm;
         
      D1 = atan2(ymm,xmm); 
      D2 = lawOfCosines(dist, L1, L2);   
      low = D1 + D2 -(PI / 2.0);
      high = lawOfCosines(L1,L2,dist) - PI;  
      if (elbow==1){
        low = -low;
        high = -high;
      }
      float highgearing = 33.0/62.0;
      high = high + (highgearing * low);    //aha!
     // Angles adjustment depending in which quadrant the final tool coordinate x,y is
      if (xmm >= 0 & ymm >= 0) {       // 1st quadrant
        //low = (PI / 2.0) - low;
      }
      if (xmm < 0 & ymm > 0) {       // 2nd quadrant
        //low = (PI / 2.0) - low;
      }
      if (xmm < 0 & ymm < 0) {       // 3d quadrant
        //low = (PI * 1.5) - low;
      }
      if (xmm > 0 & ymm < 0) {       // 4th quadrant
        //low = -(PI / 2.0) - low;
      }
      if (xmm < 0 & ymm == 0) {
        //low = (PI * 1.5) + low;
      }

      rot = (PI * 2) * zmm / LEAD;   // height in radians
  }
}
