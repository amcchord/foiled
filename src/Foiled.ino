
#include <SPI.h>
#include <i2c_t3.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <PulsePosition.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <PWMServo.h>
#include <Adafruit_DotStarMatrix.h>
#include <Adafruit_DotStar.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//Reboot Mode
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

Adafruit_SSD1306 display(29);
String modeString = "";
String armString = "";
#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

//AHRS Vars
#define BNO055_SAMPLERATE_DELAY_MS (1)
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);
bool inverted = 0;
double targetHeading = 0;
double targetBalance = 5;
double headingDelta = 0;

//Heading PID
double consKp=0.7, consKi=.2, consKd=0.01;
double Setpoint, Input, Output;
PID headingPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

#define TURN_SENSITIVITY 60.0 //The lower this number the more sensitive we are to
                              //Turn Inputs
#define THROTTLESTART 40      //This is the PWM level that actually makes the wheels
                              //move
#define H_HOLD_TIME 500       //Milliseconds to hold in heading mode after 0 throttle

unsigned long hstart = 0;


//Balance PIDs
double balKp=7, balKi=50, balKd=0.10;
double balSet, balIn, balOut;
PID balancePID(&balIn, &balOut, &balSet, balKp, balKi, balKd, REVERSE);

//Travel PIDs
//double trvKp=.70, trvKi=.2, trvKd=0.00;

double trvKp=.5, trvKi=10, trvKd=0.00;

double trvSet, trvIn, trvOut = 0;
PID travelPID(&trvIn, &trvOut, &trvSet, trvKp, trvKi, trvKd, DIRECT);
double thrustDrift = 0;


//Trim for RC Inputs
const int rcMin = 1099;
const int rcMax = 1920;
int rcScale = rcMax - rcMin;
#define FAILSAFE false //Failsafe is disabled for now
#define DEADBAND 20 //If thrust values are within +/-10 of 0 assume they are 0
#define REJECTTHRESH 2200 //Rc values above this number are considered invalid
#define RCVR_PPM 5 //Pin where the PPM comes in

//Create some global variables to store the state of RC Reciver Channels
double rc1 = 0; // Turn
double rc2 = 0; // Thrust
double rc3 = 0; //
double rc4 = 0; //
double rc5 = 0; // Mode
double rc6 = 0; // Safety

//Define the PPM decoder object
PulsePositionInput myIn;

//Define the ports that control the motors
//Motor Driver Outputs



Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);





//Some globals for handling mode switching
int lastMode = 0;
int pixelTicker = 0;

int left = 0;
int right = 0;

int count = 10;

imu::Vector<3> euler ;
IntervalTimer gyroSafety;

void setup()   {

  pinMode(RCVR_PPM, INPUT_PULLDOWN);

  pinMode(13,OUTPUT); //Just make sure we can use the onboard LED for stuff

  Serial.begin(9600);
  myIn.begin(RCVR_PPM);
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Wire.setClock(3200000);
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Booting...");
  display.display();

  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Gyro Calibartion...");
  display.display();


  delay(100);
  gyroSafety.begin(gSafety, 2000000); // Call the safety function after 2 seocnds
  gyroSafety.priority(10);
  if(!bno.begin())
  {
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.println("Gyro FAIL!");
    display.display();
    delay(1000);
  }
  gyroSafety.end();

  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);



  //Okay lets start the PID
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  targetHeading = euler.x();

  //Arm the motors!
  AFMS.begin();

  //levelFlippers();
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetSampleTime(10);
  headingPID.SetOutputLimits(-200, 200);

  balancePID.SetMode(AUTOMATIC);
  balancePID.SetSampleTime(10);
  balancePID.SetOutputLimits(-200, 200);

  travelPID.SetMode(AUTOMATIC);
  travelPID.SetSampleTime(10);
  travelPID.SetOutputLimits(-10, 10);

}

int thrust;
int turn;
void loop() {
  updateChannels();
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  display.clearDisplay();

  //Scale the raw RC input
   thrust = round(((rc2 - rcMin)/rcScale) * 500) - 250; //Cast to -250-0-250
   turn = round(((rc1 - rcMin)/rcScale) * 500) - 250; //Cast to -250-0-250
  int flipper = round(((rc3 - rcMin)/rcScale) * 180) - 90; //Cast to 0-256
  int yaw = round(((rc4 - rcMin)/rcScale) * 180) - 90; //Cast to 0-256
  int mode = round(((rc5 - rcMin)/rcScale) * 4); //Cast to 0-256
  int safety = round(((rc6 - rcMin)/rcScale) * 256); //Cast to 0-256



  if (mode == 0){
    modeString = "Direct";
  }
  else if (mode == 2){
    modeString = "Heading";
  }

  else if (mode == 4){
    modeString = "Full";
  }

  if (safety > 200){
    armString = "ARM";
  } else {
    armString = "Safe";
    //Stop all the motors.
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
  }


  //Apply Deadband Correction
  if (thrust < DEADBAND && thrust > (DEADBAND * -1)){
    thrust = 0;
  }
  if (turn < DEADBAND && turn > (DEADBAND * -1)){
    turn = 0;
  }

  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);


  if (count > 1 ){ //Only update the display once every 10 loops
    updateDisplay();
    count = 0;
  }
  count++;


  if (mode == 0){
    simpleDrive(thrust, turn);
  }

  else if (mode == 2){
    //driveAsist(thrust, turn);
    simpleDrive(thrust, turn);
  }

  else if (mode == 4){
    //fullAuto(thrust, turn);
    simpleDrive(thrust, turn);
  }



}
void doBalance(){
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    if (euler.z() < -50 && euler.z() > -140){
      balSet = -87.5 + trvOut;
      balIn = euler.z();
      balancePID.Compute();
      thrust = balOut;
      thrustDrift += thrust / 30.0;
      trvIn = thrustDrift;
      if (trvIn > 30){
        trvIn = 30;
      }
      if (trvIn < -30){
        trvIn = -30;
      }
      trvSet = 0;
      travelPID.Compute();
      if (thrust > 0){
        thrust = thrust + 15;
      } else if (thrust < 0){
        thrust = thrust - 15;
      }

      simpleDrive(thrust, turn);

    } else {
      leftMotor->run(RELEASE);
      rightMotor->run(RELEASE);

    }

}



void updateDisplay(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.setRotation(2);
  display.setTextSize(1);
  display.print(armString);
  display.setCursor(64,0);
  display.setTextSize(1);
  display.print(modeString);
  display.setCursor(0,8);
  display.setTextSize(1);
  display.print("");
  display.print((int)thrust);
  display.print(" / ");
  display.println((int)turn);


  display.print("Lft:  ");
  display.println(left);
  display.print("Rgt: ");
  display.println(right);

  display.setCursor(64,8);
  display.print("X: ");
  display.println(euler.x());
  display.setCursor(64,16);
  display.print("Z: ");
  display.println(euler.z());
  display.setCursor(64,24);
  display.print("T: ");
  display.print(balSet);
  display.print(" ");
  display.print(targetHeading);
  display.display();

}

void fullAuto(double thrust, double turn){

  //Lets run away from areas where we might fall over!
  double tilt = euler.y();
  double thrustAdjust;
  if (tilt < -20){
    thrustAdjust = -100 + tilt * 2;
  }
  else if (tilt > 20){
    thrustAdjust = 100 + tilt * 2;
  }
  thrust = thrust + thrustAdjust;
  if (thrust > 250){
    thrust = 250;
  }
  else if (thrust < -250){
    thrust = -250;
  }
  driveAsist(thrust, turn);

}

void driveAsist(double thrust, double turn){
  double currentHeading = euler.x();
  int throttleAssist = THROTTLESTART;

  if (thrust < 20 && hstart + H_HOLD_TIME < millis()){
    targetHeading = currentHeading;
    if (turn < 0 ){
      turn = ((turn * turn) / 250) * -1;
    }
    else if (turn > 0 ){
      turn = ((turn * turn) / 250);
    }
    simpleDrive(thrust, turn);
  }
  else {
    hstart = millis();

    //Turn input now shifts our target heading
    targetHeading = targetHeading + (turn/TURN_SENSITIVITY) * -1;
    if (targetHeading < 0){
      targetHeading = targetHeading + 360;
    }
    if (targetHeading > 360){
      targetHeading = targetHeading - 360;
    }

    //Figure out how far off couse we are.. heading different needs to correct
    //for the fact we can never be more than 180 degrees off target.
    if (targetHeading - currentHeading < -180){
      headingDelta = targetHeading - currentHeading + 360;
    }
    else if (targetHeading - currentHeading > 180 ){
      headingDelta = targetHeading - currentHeading - 360;
    }
    else {
      headingDelta = targetHeading - currentHeading;
    }

    Setpoint = 0;
    Input = headingDelta;
    headingPID.Compute();
    turn = Output;
  }

  double finalOutput = 0;
  //If our outputs are below the throttleAssist level lets give them a boost
  if (thrust < 0){
    throttleAssist = throttleAssist + thrust;
  } else if (thrust > 0) {
    throttleAssist = throttleAssist - thrust;
  }
  if (throttleAssist < 0){
    throttleAssist = 0;
  }
  if (turn > 0){
    finalOutput = turn + throttleAssist;
  } else if (turn < 0) {
    finalOutput = turn - throttleAssist;
  }

  simpleDrive(thrust, finalOutput);

}

void simpleDrive(double thrust, double turn){
  left = 0;
  right = 0;
  turn = turn;

  //This is where the turning logic is.. That's it.
  left = thrust + turn;
  right = thrust - turn;


  //Safety checks!
  if (left > 255){
    left = 255;
  }
  else if (left < -255){
    left = -255;
  }

  //If the left motor needs to go forward.
    if (left > 0){
      leftMotor->setSpeed(left);
      leftMotor->run(FORWARD);



    } else { //Left motor needs to spin backward
      leftMotor->setSpeed(left * -1);
      leftMotor->run(BACKWARD);


    }


    //Same thing for the right side
    if (right > 255){
      right = 255;
    }
    else if (right < -255){
      right = -255;
    }

    if (right > 0){
      rightMotor->setSpeed(right);
      rightMotor->run(BACKWARD);



    } else {
      rightMotor->setSpeed(right * -1);
      rightMotor->run(FORWARD);

    }
}

//Read in the channels from the RC reciver
void updateChannels(){

  int num = myIn.available();
  if (num > 0) {

    int rc1t = myIn.read(1);
    int rc2t = myIn.read(2);
    int rc3t = myIn.read(3);
    int rc4t = myIn.read(4);
    int rc5t = myIn.read(5);
    int rc6t = myIn.read(6);

    //Don't register weird outliers!
    if (rc1t > 0 && rc1t < REJECTTHRESH){
      rc1 = rc1t;
    }
    if (rc2t > 0 && rc2t < REJECTTHRESH){
      rc2 = rc2t;
    }
    if (rc3t > 0 && rc3t < REJECTTHRESH){
      rc3 = rc3t;
    }
    if (rc4t > 0 && rc4t < REJECTTHRESH){
      rc4 = rc4t;
    }
    if (rc5t > 0 && rc5t < REJECTTHRESH){
      rc5 = rc5t;
    }
    if (rc6t > 0 && rc6t < REJECTTHRESH){
      rc6 = rc6t;
    }

    if (rc6 > 2000 && FAILSAFE){ //Will shutdown is reciever is programed correctly for failsafe
      rc1 = 0;
      rc2 = 0;
      rc3 = 0;
      rc4 = 0;
      rc5 = 0;
      rc6 = 0;
    }
  }
}


void gSafety(){
  //We are here because the Gyro hung... we are going to reboot!
   CPU_RESTART
   delay(1000);
}
