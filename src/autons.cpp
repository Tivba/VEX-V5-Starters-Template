#include "robot-config.h"
#include "vex.h"
#include "PID.h"
#include "arm.h"
#include "functions.h"

using namespace vex;


void rightAuton() {

  drivePID(12, 0.06, 0, 0.01); 
  
  
}

void leftAuton() {
  spinLeftDT(50);
  turnPID(90, 0.0, 0.001, 0.3);
  drivePID(6, 0.5, 0.003, 0.3);
  clamp();
}

void skillsAuton() {
  drivePID(12, 0.5, 0.003, 0.3);
  turnPID(90, 0.06, 0.001, 0.3);
  clamp();
  liftArm(30, 5);
  drivePID(-12, 0.5, 0.003, 0.3);
  turnPID(90, 0.06, 0.001, 0.3);
  drivePID(6, 0.5, 0.003, 0.3);
  clamp();
}
