#include "config.h"
#include <Arduino.h>
#include <Stepper.h>
#include <Servo.h>
//#include <VarSpeedServo.h>
#include "pinout.h"
#include "logger.h"
#include "robotGeometry.h"
#include "interpolation.h"
#include "fanControl.h"
#include "RampsStepper.h"
#include "queue.h"
#include "command.h"
#include "endstop.h"

//STEPPER OBJECTS
RampsStepper stepperHigher(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, INVERSE_X_STEPPER);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, INVERSE_Y_STEPPER);
RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, INVERSE_Z_STEPPER);


Stepper stepper(2400, STEPPER_GRIPPER_PIN_0, STEPPER_GRIPPER_PIN_1, STEPPER_GRIPPER_PIN_2, STEPPER_GRIPPER_PIN_3);

RampsStepper stepperExtruder(E_STEP_PIN, E_DIR_PIN, E_ENABLE_PIN, INVERSE_E0_STEPPER);
FanControl fan(FAN_PIN);
//ENDSTOP OBJECTS
Endstop endstopX(X_MIN_PIN, X_DIR_PIN, X_STEP_PIN, X_ENABLE_PIN, X_MIN_INPUT, X_HOME_STEPS, HOME_DWELL);
Endstop endstopY(Y_MIN_PIN, Y_DIR_PIN, Y_STEP_PIN, Y_ENABLE_PIN, Y_MIN_INPUT, Y_HOME_STEPS, HOME_DWELL);
Endstop endstopZ(Z_MIN_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN, Z_MIN_INPUT, Z_HOME_STEPS, HOME_DWELL);
RobotGeometry geometry;
Interpolation interpolator;
Queue<Cmd> queue(15);
Command command;

Servo servo;
//int angle = 170;
int angle = 45;
int angle_offset = 0; // offset to compensate deviation from 90 degree(middle position)
// which should gripper should be full closed.

void setup() {

  SERIALX.begin(BAUD);

  //various pins..
  pinMode(HEATER_0_PIN  , OUTPUT);
  pinMode(HEATER_1_PIN  , OUTPUT);
  pinMode(LED_PIN       , OUTPUT);

  //unused Stepper..
  pinMode(E_STEP_PIN   , OUTPUT);
  pinMode(E_DIR_PIN    , OUTPUT);
  pinMode(E_ENABLE_PIN , OUTPUT);

  //unused Stepper..
  pinMode(Q_STEP_PIN   , OUTPUT);
  pinMode(Q_DIR_PIN    , OUTPUT);
  pinMode(Q_ENABLE_PIN , OUTPUT);

  //GripperPins
  pinMode(STEPPER_GRIPPER_PIN_0, OUTPUT);
  pinMode(STEPPER_GRIPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_GRIPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_GRIPPER_PIN_3, OUTPUT);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);

  //  vaccum motor control
  pinMode(MOTOR_IN1  , OUTPUT);
  pinMode(MOTOR_IN2  , OUTPUT);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  servo.attach(SERVO_PIN);
  servo.write(angle + angle_offset);
  //servo.slowmove (angle + angle_offset, 50);

  //reduction of steppers..

  if (SCARA) {
      //SERIALX.println("here");
      //stepperHigher.setReductionRatio(62.0 / 20.0, 200 * 16);
      stepperHigher.setReductionRatio((62.0 / 16.0) * (62.0 / 33.0), 200 * 16);   
      //stepperLower.setReductionRatio( 62.0 / 20.0, 200 * 16);
      stepperLower.setReductionRatio( 72.0 / 16.0, 200 * 16);
      stepperRotate.setReductionRatio(1.0, 200 * 16);
  } else{
      stepperHigher.setReductionRatio(32.0 / 9.0, 200 * 16);  //big gear: 32, small gear: 9, steps per rev: 200, microsteps: 16
      stepperLower.setReductionRatio( 32.0 / 9.0, 200 * 16);
      stepperRotate.setReductionRatio(32.0 / 9.0, 200 * 16);
      stepperExtruder.setReductionRatio(32.0 / 9.0, 200 * 16);    
  }

  //start positions..
  stepperHigher.setPositionRad(PI / 2.0);  //90°
  stepperLower.setPositionRad(0);          // 0°
  stepperRotate.setPositionRad(0);         // 0°
  stepperExtruder.setPositionRad(0);

 if (SCARA) {
      //stepperHigher.setPositionRad(-PI / 2.0);          // -90°
      stepperHigher.setPositionRad(0);
      stepperLower.setPositionRad(PI / 2.0);  //90°
      stepperRotate.setPositionRad(0);         // 0°
 }
  //enable and init..
  setStepperEnable(false);
  
    if (HOME_ON_BOOT) { //HOME DURING SETUP() IF HOME_ON_BOOT ENABLED
    homeSequence(); 
    Logger::logINFO("ROBOT ONLINE");
  } else {
    setStepperEnable(false); //ROBOT ADJUSTABLE BY HAND AFTER TURNING ON
    if (HOME_X_STEPPER && HOME_Y_STEPPER && !HOME_Z_STEPPER){
      Logger::logINFO("ROBOT ONLINE");
      Logger::logINFO("ROTATE ROBOT TO FACE FRONT CENTRE & SEND G28 TO CALIBRATE");
    }
    if (HOME_X_STEPPER && HOME_Y_STEPPER && HOME_Z_STEPPER){
      Logger::logINFO("ROBOT ONLINE");
      Logger::logINFO("SEND G28 TO CALIBRATE");
    }
    if (!HOME_X_STEPPER && !HOME_Y_STEPPER){
      Logger::logINFO("ROBOT ONLINE");
      Logger::logINFO("HOME ROBOT MANUALLY");
    }
  }
  
  //interpolator.setInterpolation(0, 180, 180, 0, 0, 180, 180, 0);
  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);

  SERIALX.println("started");
  //Serial2.println("started");
}


void setStepperEnable(bool enable) {
  stepperRotate.enable(enable);
  stepperLower.enable(enable);
  stepperHigher.enable(enable);
  stepperExtruder.enable(enable);
  fan.enable(enable);
}

void loop () {
  //update and Calculate all Positions, Geometry and Drive all Motors...
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  //Logger::logDEBUG(String(geometry.getLowRad()) + " " + String(geometry.getHighRad()) + " " + String(geometry.getRotRad()));
  stepperRotate.stepToPositionRad(geometry.getRotRad());
  stepperLower.stepToPositionRad (geometry.getLowRad());
  stepperHigher.stepToPositionRad(geometry.getHighRad());
  stepperExtruder.stepToPositionRad(interpolator.getEPosmm());
  stepperRotate.update();
  stepperLower.update(STEPPERDELAY);
  stepperHigher.update(STEPPERDELAY);
  fan.update();

  if (!queue.isFull()) {
    if (command.handleGcode()) {
      queue.push(command.getCmd());
      printOk();
    }
  }
  if ((!queue.isEmpty()) && interpolator.isFinished()) {
	  //Serial.println("moreOK");
    executeCommand(queue.pop());
  }

  if (millis() % 500 < 250) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}



void cmdMove(Cmd (&cmd)) {
//  Serial.println(cmd.valueX);
//  Serial.println(cmd.valueY);
//  Serial.println(cmd.valueZ);
  interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE, cmd.valueF);
}
void cmdDwell(Cmd (&cmd)) {
  delay(int(cmd.valueT * 1000));
}
void cmdGripperOn(Cmd (&cmd)) {
  //Serial.print("Gripper on ");

  // vaccum griiper
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);

  angle = int(cmd.valueT);
  servo.write(angle + -angle_offset);

  // stepper gripper
  stepper.setSpeed(5);
  stepper.step(int(cmd.valueT));
  delay(50);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);
  //printComment("// NOT IMPLEMENTED");
  //printFault();
}
void cmdGripperOff(Cmd (&cmd)) {
  //Serial.print("Gripper off ");

  // vaccum griiper
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  angle = int(cmd.valueT);
  servo.write(angle + -angle_offset);

  // stepper gripper
  stepper.setSpeed(5);
  stepper.step(-int(cmd.valueT));
  delay(50);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);
  //printComment("// NOT IMPLEMENTED");
  //printFault();
}
void cmdStepperOn() {
  setStepperEnable(true);
}
void cmdStepperOff() {
  setStepperEnable(false);
}
void cmdFanOn() {
  fan.enable(true);
}
void cmdFanOff() {
  fan.enable(false);
}

void handleAsErr(Cmd (&cmd)) {
  printComment("Unknown Cmd " + String(cmd.id) + String(cmd.num) + " (queued)");
  printFault();
}

void executeCommand(Cmd cmd) {
  if (cmd.id == -1) {
    String msg = "parsing Error";
    printComment(msg);
    handleAsErr(cmd);
    return;
  }

  if (cmd.valueX == NAN) {
    cmd.valueX = interpolator.getXPosmm();
  }
  if (cmd.valueY == NAN) {
    cmd.valueY = interpolator.getYPosmm();
  }
  if (cmd.valueZ == NAN) {
    cmd.valueZ = interpolator.getZPosmm();
  }
  if (cmd.valueE == NAN) {
    cmd.valueE = interpolator.getEPosmm();
  }

  //decide what to do
  if (cmd.id == 'G') {
    switch (cmd.num) {
      case 0: cmdMove(cmd); break;
      case 1: cmdMove(cmd); break;
      case 4: cmdDwell(cmd); break;
	  case 28: homeSequence(); break;
      //case 21: break; //set to mm
      //case 90: cmdToAbsolute(); break;
      //case 91: cmdToRelative(); break;
      //case 92: cmdSetPosition(cmd); break;
      default: handleAsErr(cmd);
    }
  } else if (cmd.id == 'M') {
    switch (cmd.num) {
      //case 0: cmdEmergencyStop(); break;
      case 3: cmdGripperOn(cmd); break;
      case 5: cmdGripperOff(cmd); break;
      case 17: cmdStepperOn(); break;
      case 18: cmdStepperOff(); break;
      case 106: cmdFanOn(); break;
      case 107: cmdFanOff(); break;
      default: handleAsErr(cmd);
    }
  } else {
    handleAsErr(cmd);
  }
}
void homeSequence(){
  setStepperEnable(false);
  //fan.enable(true);
  fan.enable(false);
  if (HOME_Y_STEPPER){
    endstopY.home(!INVERSE_Y_STEPPER); //INDICATE STEPPER HOMING DIRECTION
  }
  if (HOME_X_STEPPER){
    endstopX.home(!INVERSE_X_STEPPER); //INDICATE STEPPER HOMING DIRECTION
  }
  if (HOME_Z_STEPPER){
    endstopZ.home(INVERSE_Z_STEPPER); //INDICATE STEPPER HOMING DIRECDTION
  }

  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
  Logger::logINFO("HOMING COMPLETE");
}
