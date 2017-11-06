#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

//servo specs
#define MIN_PULSE_WIDTH 400 //in micro seconds
#define MAX_PULSE_WIDTH 2400 //in micro seconds
#define FREQUENCY 50 //Hz

#define CoxaLength 20
#define FemurLength 70
#define TibiaLength 106

int Coxas[] = {0, 3, 6, 9};
int Femurs[] = {1, 4, 7, 10};
int Tibias[] = {2, 5, 8, 11};

int DegreeCoxas[4];
int DegreeFemurs[4];
int DegreeTibias[4];

// leg positions
int x = 60;
int y = 60;
int z = 70;

//comands for direction and rotation of body
int ComDirX = 0;
int ComDirY = 0;
int ComDirZ = 0;
int ComRotZ = 0;

//leg index for walking gait incrementing each leg 128 steps in 1 cycle
int legIndex = 0;
int LB = legIndex;
int RB = legIndex + 32;
int RF = legIndex + 64;
int LF = legIndex + 96;

//analog values for PWM sent to servo
int analog_value;
int ANALOG_MIN = 4095 / (20000 / MIN_PULSE_WIDTH);
int ANALOG_MAX = 4095 / (20000 / MAX_PULSE_WIDTH);

//controller input
int CommandInput;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  delay(1000);
}

void loop() {
  ReceiveCommand(ComDirX, ComDirY, ComDirZ, ComRotZ);
  Serial.println(ComDirX);
  ComDirY = 100;
  Serial.println(ComDirY);
  Serial.println(ComDirZ);
  Serial.println(ComRotZ);
  WalkingCycle(x, y, z);

  InverseKinematics(0, x, y, z);
  pwm.setPWM(Coxas[0], 0, pulseWidthInverse(DegreeCoxas[0]));
  pwm.setPWM(Femurs[0], 0, pulseWidthInverse(DegreeFemurs[0]));
  pwm.setPWM(Tibias[0], 0, pulseWidthInverse(DegreeTibias[0]));
}

//------------------------------------------------------------------------
//walking command from controller
void ReceiveCommand(int dirX, int dirY, int dirZ, int rotZ) {
  if (Serial.available() > 0) {
    CommandInput = Serial.read();
    if (CommandInput == 0) {
      dirX = 0;
      dirY = 0;
      dirZ = 0;
      rotZ = 0;
    }
     else if (CommandInput == 0) {
      //eventually analog inputs from controler. for now values from -100 to 100 in each direction
      dirX = 0; //+100 move right at high speed, -100 move left at high speed
      dirY = 100; //+100 move forward at high speed, -100 move backward at high speed
      dirZ = 0; //+100 body higher up to threshold, -100 body lower up to threshold
      rotZ = 0; //+100 rotate right at high speed, -100 rotate left at high speed
    }
  }
  Serial.print("Command ");
  Serial.println(CommandInput);
}

//-------------------------------------------------------------------------
//define what step in each leg cycle and what position each leg has to take
void WalkingCycle(int nextX, int nextY, int nextZ) {
  if (ComDirZ > 0 > ComDirZ) {
    int BodyHeight = ComDirZ;
    map(BodyHeight, -100, 100, -30, 30);
    nextZ = z + BodyHeight; // Z COORDINATE TO SEND TO IK
  }
  if (legIndex < 96) {
    if (ComDirX > 0 > ComDirX) { //
      int StepLength = ComDirX;
      map(StepLength, -100, 100, -140, 140); //defining step length in mm
      int StepX = legIndex * sqrt(sq(StepLength)) / 96; //giving x position in step length
      int StepMinPos = 60 - (StepLength / 2); //defining x coordinates of begining of step
      int StepMaxPos = 60 - (StepLength / 2); //defining x coordinates of end of step
      nextX = StepX;
      map (nextX, 0, StepLength, StepMinPos, StepMaxPos); //new x position
    }
    if (ComDirY > 0 > ComDirY) {
      int StepLength = ComDirY;
      map(StepLength, -100, 100, -140, 140); //defining step length in mm
      int StepY = legIndex * sqrt(sq(StepLength)) / 96; //giving y position in step length
      int StepMinPos = 60 - (StepLength / 2); //defining y coordinates of begining of step
      int StepMaxPos = 60 - (StepLength / 2); //defining y coordinates of end of step
      nextY = StepY;
      map (nextY, 0, StepLength, StepMinPos, StepMaxPos); //new y position
    }
    nextX = ((nextX - 60) / 2) + 60; //next x position for front back and translate movement combination
    nextY = ((nextY - 60) / 2) + 60; //next y position for front back and translate movement combination
    legIndex = (legIndex + 1) % 128; // individual leg indexes automatically update
  }
  else if (96 <= legIndex) {
    if (ComDirX > 0 > ComDirX) { //
      int StepLength = ComDirX;
      map(StepLength, -100, 100, 140, -140); //defining step length in mm
      int StepX = legIndex * sqrt(sq(StepLength)) / 32; //giving x position in step length
      int StepMinPos = 60 - (StepLength / 2); //defining x coordinates of begining of step
      int StepMaxPos = 60 - (StepLength / 2); //defining x coordinates of end of step
      nextX = StepX;
      map (nextX, 0, StepLength, StepMinPos, StepMaxPos); //new x position
    }
    if (ComDirY > 0 > ComDirY) {
      int StepLength = ComDirY;
      map(StepLength, -100, 100, 140, -140); //defining step length in mm
      int StepY = legIndex * sqrt(sq(StepLength)) / 32; //giving y position in step length
      int StepMinPos = 60 - (StepLength / 2); //defining y coordinates of begining of step
      int StepMaxPos = 60 - (StepLength / 2); //defining y coordinates of end of step
      nextY = StepY;
      map (nextY, 0, StepLength, StepMinPos, StepMaxPos); //new y position
    }
    nextZ = -0.1 * sq(legIndex - 112) + 20; // Z COORDINATE TO SEND TO IK for lift parabola
    nextX = ((nextX - 60) / 2) + 60; //next x position for front back and translate movement combination
    nextY = ((nextY - 60) / 2) + 60; //next y position for front back and translate movement combination
    legIndex = (legIndex + 1) % 128; // individual leg indexes automatically update
  }
  if (ComRotZ > 0 > ComRotZ) {
    int RotSpeed = ComRotZ;
    map (RotSpeed, -100, 100, -PI / 4, PI / 4);
    int RotAngle = RotSpeed / 64; //defining micro angle steps
    double CurCoxaAngle = atan2(nextY, nextX); //coxa angle after translation movement
    double LegLength = sqrt(sq(x) + sq(y));
    nextX = LegLength * cos(CurCoxaAngle + RotAngle); // X COORDINATE TO SEND TO IK
    nextY = LegLength * sin(CurCoxaAngle + RotAngle); // Y COORDINATE TO SEND TO IK
  }
}

int pulseWidthNormal(int radian) {
  analog_value = map(radian, 0, 3141, ANALOG_MIN, ANALOG_MAX);
  Serial.print("Servo analog value : ");
  Serial.println(analog_value);
  return analog_value;
}

int pulseWidthInverse(int radian) {
  analog_value = map(radian, 3141, 0, ANALOG_MIN, ANALOG_MAX);
  Serial.print("Servo analog value : ");
  Serial.println(analog_value);
  return analog_value;
}

//-----------------------------------------------------------------------
//inverse kinematics calculating next x y z position for each leg
void InverseKinematics(int numPatte, int x, int y, int z) {
  //--------------------------leg IK---------------------
  double CoxaAngle = atan2(y, x);
  double LegLength = sqrt(sq(x) + sq(y));
  double Hf = sqrt(sq(LegLength - CoxaLength) + sq(z));
  double A1 = atan2(LegLength - CoxaLength, z);
  double A2 = acos((sq(TibiaLength) - sq(FemurLength) - sq(Hf)) / (-2 * FemurLength * Hf));
  double FemurAngle = PI / 2 - (A1 + A2);
  double C1 = acos((sq(Hf) - sq(TibiaLength) - sq(FemurLength)) / (-2 * FemurLength * TibiaLength));
  double TibiaAngle = PI / 2 - C1;

  DegreeCoxas[numPatte] = (CoxaAngle + (PI / 4)) * 1000; //Coxa in radians x1000
  DegreeFemurs[numPatte] = (FemurAngle + (PI / 2)) * 1000; //Femur in radians x1000
  DegreeTibias[numPatte] = ((PI / 2) - TibiaAngle) * 1000; //Tibia in radians x1000
}
