#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

//servo specs
#define MIN_PULSE_WIDTH 400 //in micro seconds
#define MAX_PULSE_WIDTH 2400 //in micro seconds
#define FREQUENCY 50 //Hz

//robot dimensions
#define CoxaLength 20
#define FemurLength 70
#define TibiaLength 106

//servo naming
int Coxas[] = {0, 3, 6, 9};
int Femurs[] = {1, 4, 7, 10};
int Tibias[] = {2, 5, 8, 11};
int DegreeCoxas[4];
int DegreeFemurs[4];
int DegreeTibias[4];

//commands for direction and rotation of body
int dirX = 0;
int dirY = 0;
int dirZ = 0;
int rotZ = 0;

//x y z positions for next step
int neutralX = 65;
int neutralY = 65;
int neutralZ = 70;
int nextX = neutralX;
int nextY = neutralY;
int nextZ = neutralZ;

//x y positions for rotations
int nextX2;
int nextY2;

//maximum steping lengths (140 max)
int maxStepLength = 110; //max steping length (140mm max)
int maxHeightLength = 30; //max body height length (30mm max)
int legLift = 30; //leg lifting height

//delay time for slower walking cycle
int cycleSpeed = 5;

//leg index for walking gait incrementing each leg 128 steps in 1 cycle
unsigned int legIndex = 0;
//leg coordinates inverting
int WHATLEG_X = 0;
int WHATLEG_Y = 0;

//analog values for PWM sent to servo
int analog_value;
int ANALOG_MIN = 4095 / (20000 / MIN_PULSE_WIDTH);
int ANALOG_MAX = 4095 / (20000 / MAX_PULSE_WIDTH);

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  delay(1000);

  InverseKinematics(0, nextX, nextY, nextZ);
  InverseKinematics(1, nextX, nextY, nextZ);
  InverseKinematics(2, nextX, nextY, nextZ);
  InverseKinematics(3, nextX, nextY, nextZ);
}

void loop() {
  ReceiveCommand();

  WHATLEG_X = 0;
  WHATLEG_Y = 0;
  WalkingCycle();
  Serial.print("Leg 0 coordinates : X:");
  Serial.print(nextX);
  Serial.print(" Y:");
  Serial.print(nextY);
  Serial.print(" Z:");
  Serial.println(nextZ);
  Serial.print("LegIndex0: ");
  Serial.println(legIndex);
  if (rotZ != 0) {
    InverseKinematics(0, nextX2, nextY2, nextZ);
  }
  else if (rotZ == 0) {
    InverseKinematics(0, nextX, nextY, nextZ);
  }

  legIndex = (legIndex + 31) % 128; //+31 for taking into account increment inside walking cycle
  WHATLEG_X = 1;
  WHATLEG_Y = 1;
  WalkingCycle();
  InverseKinematics(2, nextX, nextY, nextZ);

  legIndex = (legIndex + 31) % 128; //+31 for taking into account increment inside walking cycle
  WHATLEG_X = 1;
  WHATLEG_Y = 0;
  WalkingCycle();
  InverseKinematics(1, nextY, nextX, nextZ);

  legIndex = (legIndex + 31) % 128; //+31 for taking into account increment inside walking cycle
  WHATLEG_X = 0;
  WHATLEG_Y = 1;
  WalkingCycle();
  InverseKinematics(3, nextY, nextX, nextZ);

  legIndex = (legIndex + 32) % 128;

  pwm.setPWM(Coxas[0], 0, pulseWidthInverse(DegreeCoxas[0])-10);
  pwm.setPWM(Femurs[0], 0, pulseWidthInverse(DegreeFemurs[0])-2);
  pwm.setPWM(Tibias[0], 0, pulseWidthInverse(DegreeTibias[0])-4);
  pwm.setPWM(Coxas[1], 0, pulseWidthInverse(DegreeCoxas[1])-4);
  pwm.setPWM(Femurs[1], 0, pulseWidthNormal(DegreeFemurs[1])-4);
  pwm.setPWM(Tibias[1], 0, pulseWidthNormal(DegreeTibias[1])+3);
  pwm.setPWM(Coxas[2], 0, pulseWidthInverse(DegreeCoxas[2])-12);
  pwm.setPWM(Femurs[2], 0, pulseWidthInverse(DegreeFemurs[2])-6);
  pwm.setPWM(Tibias[2], 0, pulseWidthInverse(DegreeTibias[2])-22);
  pwm.setPWM(Coxas[3], 0, pulseWidthInverse(DegreeCoxas[3])+4);
  pwm.setPWM(Femurs[3], 0, pulseWidthNormal(DegreeFemurs[3])+4);
  pwm.setPWM(Tibias[3], 0, pulseWidthNormal(DegreeTibias[3])+14);
  delay(cycleSpeed);
  Serial.println();
}

//------------------------------------------------------------------------
void ReceiveCommand() { //walking command from controller
  //eventually analog inputs from controler. for now values from -100 to 100 in each direction
  dirX = 0; //+100 move right at high speed, -100 move left at high speed
  dirY = 100; //+100 move forward at high speed, -100 move backward at high speed
  dirZ = 0; //+100 body higher up to threshold, -100 body lower up to threshold
  rotZ = 0; //+100 rotate right at high speed, -100 rotate left at high speed
  Serial.println("Command");
}

//-------------------------------------------------------------------------
void WalkingCycle() { //define what step in each leg cycle and what position each leg has to take
  if (legIndex < 96) {
    if (dirX > 0 || 0 > dirX) {
      int StepLength = dirX;
      if (WHATLEG_X == 0) {
        StepLength = map(StepLength, -100, 100, -maxStepLength, maxStepLength); //defining step length in mm
      }
      if (WHATLEG_X == 1) {
        StepLength = map(StepLength, -100, 100, maxStepLength, -maxStepLength);
      }
      int StepX = legIndex * abs(StepLength) / 96; //giving x position in step length
      int StepMinPos = neutralX - (StepLength / 2); //defining x coordinates of begining of step
      int StepMaxPos = neutralX + (StepLength / 2); //defining x coordinates of end of step
      nextX = map (StepX, 0, abs(StepLength), StepMinPos, StepMaxPos); //new x position
    }
    if (dirY > 0 || 0 > dirY) {
      int StepLength = dirY;
      if (WHATLEG_Y == 0) {
        StepLength = map(StepLength, -100, 100, -maxStepLength, maxStepLength); //defining step length in mm
      }
      if (WHATLEG_Y == 1) {
        StepLength = map(StepLength, -100, 100, maxStepLength, -maxStepLength);
      }
      int StepY = legIndex * abs(StepLength) / 96; //giving y position in step length
      int StepMinPos = neutralY - (StepLength / 2); //defining y coordinates of begining of step
      int StepMaxPos = neutralY + (StepLength / 2); //defining y coordinates of end of step
      nextY = map (StepY, 0, abs(StepLength), StepMinPos, StepMaxPos); //new y position
    }
  }

  else if (96 <= legIndex) { //LEG LIFTING MOTION
    if (dirX > 0 || 0 > dirX) {
      int StepLength = dirX;
      if (WHATLEG_X == 0) {
        StepLength = map(StepLength, -100, 100, -maxStepLength, maxStepLength); //defining step length in mm
      }
      if (WHATLEG_X == 1) {
        StepLength = map(StepLength, -100, 100, maxStepLength, -maxStepLength);
      }
      int StepX = (legIndex - 96) * abs(StepLength) / 32; //giving x position in step length
      int StepMinPos = neutralX + (StepLength / 2); //defining x coordinates of begining of step
      int StepMaxPos = neutralX - (StepLength / 2); //defining x coordinates of end of step
      nextX = map (StepX, 0, abs(StepLength), StepMinPos, StepMaxPos); //new x position
    }
    if (dirY > 0 || 0 > dirY) {
      int StepLength = dirY;
      if (WHATLEG_Y == 0) {
        StepLength = map(StepLength, -100, 100, -maxStepLength, maxStepLength); //defining step length in mm
      }
      if (WHATLEG_Y == 1) {
        StepLength = map(StepLength, -100, 100, maxStepLength, -maxStepLength);
      }
      int StepY = (legIndex - 96) * abs(StepLength) / 32; //giving y position in step length
      int StepMinPos = neutralY + (StepLength / 2); //defining y coordinates of begining of step
      int StepMaxPos = neutralY - (StepLength / 2); //defining y coordinates of end of step
      nextY = map (StepY, 0, abs(StepLength), StepMinPos, StepMaxPos); //new y position
    }
  }

  int BodyHeight = dirZ;
  if (dirZ > 0 || 0 > dirZ) {
    BodyHeight = map(BodyHeight, -100, 100, -maxHeightLength, maxHeightLength);
    nextZ = neutralZ + BodyHeight; // Z COORDINATE TO SEND TO IK for
  }

  if (legIndex < 96) { // Z COORDINATE TO SEND TO IK for keeping leg on ground
    nextZ = neutralZ + BodyHeight;
  }
  else if (96 <= legIndex && dirX != 0 || dirY != 0) {
    nextZ = 0.1 * sq(legIndex - 112) - legLift + (neutralZ + BodyHeight); // Z COORDINATE TO SEND TO IK for lift parabola
  }
  /*
    if (legIndex < 96) {
      if (rotZ > 0 || 0 > rotZ) {
        Serial.println("walking cycle rotZ");
        float RotSpeed = map_float(rotZ, -100, 100, -HALF_PI, HALF_PI); //define angle step length
        Serial.println(RotSpeed);
        float RotAngle = legIndex * (RotSpeed / 96); //giving angle position in step length
        Serial.println(RotAngle);
        float CurCoxaAngle = atan2(nextY, nextX); //coxa angle after translation movement
        Serial.println(CurCoxaAngle);
        float LegLength = sqrt(sq(nextX) + sq(nextY));
        Serial.println(LegLength);
        nextX2 = nextX + ((LegLength * cos(CurCoxaAngle + RotAngle)) - neutralX); // X COORDINATE TO SEND TO IK
        Serial.print("walking cycle nextX ROT ");
        Serial.println(nextX2);
        nextY2 = nextY + ((LegLength * sin(CurCoxaAngle + RotAngle)) - neutralY); // Y COORDINATE TO SEND TO IK
        Serial.print("walking cycle nextY ROT ");
        Serial.println(nextY2);
      }
    }
    if (legIndex <= 96) {
      if (rotZ > 0 || 0 > rotZ) {
        Serial.println("walking cycle rotZ");
        float RotSpeed = map_float(rotZ, -100, 100, HALF_PI, -HALF_PI); //define angle step length
        Serial.println(RotSpeed);
        float RotAngle = (legIndex - 96) * (RotSpeed / 32); //giving angle position in step length
        Serial.println(RotAngle);
        float CurCoxaAngle = atan2(nextY, nextX); //coxa angle after translation movement
        Serial.println(CurCoxaAngle);
        float LegLength = sqrt(sq(nextX) + sq(nextY));
        Serial.println(LegLength);
        nextX2 = nextX + ((LegLength * cos(CurCoxaAngle + RotAngle)) - neutralX); // X COORDINATE TO SEND TO IK
        Serial.print("walking cycle nextX ROT ");
        Serial.println(nextX2);
        nextY2 = nextY + ((LegLength * sin(CurCoxaAngle + RotAngle)) - neutralY); // Y COORDINATE TO SEND TO IK
        Serial.print("walking cycle nextY ROT ");
        Serial.println(nextY2);
      }
    }*/

  legIndex = (legIndex + 1) % 128; // individual leg indexes automatically update
}

float map_float(float val, float in_min, float in_max, float out_min, float out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int pulseWidthNormal(int radian) {
  analog_value = map(radian, 0, 3141, ANALOG_MIN, ANALOG_MAX);
  return analog_value;
}

int pulseWidthInverse(int radian) {
  analog_value = map(radian, 3141, 0, ANALOG_MIN, ANALOG_MAX);
  return analog_value;
}

//-----------------------------------------------------------------------
void InverseKinematics(int numPatte, int x, int y, int z) { //inverse kinematics calculating next x y z position for each leg
  //--------------------------leg IK---------------------
  float CoxaAngle = atan2(y, x);
  float LegLength = sqrt(sq(x) + sq(y));
  float Hf = sqrt(sq(LegLength - CoxaLength) + sq(z));
  float A1 = atan2(LegLength - CoxaLength, z);
  float A2 = acos((sq(TibiaLength) - sq(FemurLength) - sq(Hf)) / (-2 * FemurLength * Hf));
  float FemurAngle = HALF_PI - (A1 + A2);
  float C1 = acos((sq(Hf) - sq(TibiaLength) - sq(FemurLength)) / (-2 * FemurLength * TibiaLength));
  float TibiaAngle = HALF_PI - C1;

  DegreeCoxas[numPatte] = (CoxaAngle + (HALF_PI / 2)) * 1000; //Coxa in radians x1000
  DegreeFemurs[numPatte] = (FemurAngle + (HALF_PI)) * 1000; //Femur in radians x1000
  DegreeTibias[numPatte] = ((HALF_PI) - TibiaAngle) * 1000; //Tibia in radians x1000
}
