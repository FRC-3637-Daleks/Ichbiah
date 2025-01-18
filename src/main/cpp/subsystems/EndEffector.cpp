#include "subsystems/EndEffector.h"

EndEffector::EndEffector() {

};

void EndEffector::MotorBack() {
endeffectorMotor.SetVoltage(-12_V);
};

void EndEffector::MotorForward() {
endeffectorMotor.SetVoltage(12_V);
};

bool EndEffector::getBreakBeamState() {

};

void EndEffector::MotorStop() {
    endeffectorMotor.SetVoltage(0_V);
};