#include <Arduino.h>
#include "RampsStepper.h"

RampsStepper::RampsStepper(int aStepPin, int aDirPin, int aEnablePin, bool aInverse) {
  setReductionRatio(1, 3200);
  stepPin = aStepPin;
  dirPin = aDirPin;
  enablePin = aEnablePin;
  inverse = aInverse;
  stepperStepPosition = 0;
  stepperStepTargetPosition;
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  disable();
}

void RampsStepper::enable(bool value) {
  digitalWrite(enablePin, !value);
}

void RampsStepper::disable() {
  digitalWrite(enablePin, HIGH);
}

bool RampsStepper::isOnPosition() const {
  return stepperStepPosition == stepperStepTargetPosition;
}

int RampsStepper::getPosition() const {
  return stepperStepPosition;
}

void RampsStepper::setPosition(int value) {
  stepperStepPosition = value;
  stepperStepTargetPosition = value;
}

void RampsStepper::stepToPosition(int value) {
  stepperStepTargetPosition = value;
}

void RampsStepper::stepRelative(int value) {
  value += stepperStepPosition;
  stepToPosition(value);
}

float RampsStepper::getPositionRad() const {
  return stepperStepPosition / radToStepFactor;
}

void RampsStepper::setPositionRad(float rad) {
  setPosition(rad * radToStepFactor);
}

void RampsStepper::stepToPositionRad(float rad) {
  stepperStepTargetPosition = rad * radToStepFactor;
}

void RampsStepper::stepRelativeRad(float rad) {
  stepRelative(rad * radToStepFactor);
}

void RampsStepper::update(int aDelay) {   
  while (stepperStepTargetPosition < stepperStepPosition) {  
    digitalWrite(dirPin, !inverse);
    delayMicroseconds(10);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(aDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(10);
    stepperStepPosition--;
  }
  while (stepperStepTargetPosition > stepperStepPosition) {    
    digitalWrite(dirPin, inverse);
    delayMicroseconds(10);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(aDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(10);
    stepperStepPosition++;
  }
}

void RampsStepper::setReductionRatio(float gearRatio, int stepsPerRev) {
  radToStepFactor = gearRatio * stepsPerRev / 2 / PI;
};
