
#define CBUFFER_SIZE 100
#include "BasicPIDLibraryClass.h"
#include "math.h"
#include "Wire.h"
#include "sensorbar.h"
#include <Adafruit_MotorShield.h>


const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
double gSetpoint=0;
double Kp=0.5, Ki=0, Kd=20;

BasicPIDLibrary myPID(Kp, Ki, Kd);
#define tdelay 5

SensorBar mySensorBar(SX1509_ADDRESS);
CircularBuffer positionHistory(CBUFFER_SIZE);

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(2);


void setup() {
  AFMS.begin();
  Serial.begin(115200);  // start serial for output
  Serial.println("Program started.");
  Serial.println();
  myPID.SetOutputLimits(-90/5,90/5);
  myPID.SetSampleTime(tdelay);
  
  mySensorBar.setBarStrobe();
  
  mySensorBar.clearInvertBits();
  
  uint8_t returnStatus = mySensorBar.begin();
  if(returnStatus)
  {
    Serial.println("sx1509 IC communication OK");
  }
  else
  {
    Serial.println("sx1509 IC communication FAILED!");
    while(1);
  }
  Serial.println();

}

void loop() {
  int temp = mySensorBar.getDensity();
    if( (temp < 4)&&(temp > 0) )
  {
    positionHistory.pushElement( mySensorBar.getPosition());
  }
  int16_t avePos = positionHistory.averageLast( 10 );
  Serial.println(temp);
  //Serial.println(avePos);//this is the line location under the follower from -127 to positive 127

  double dOutput = 0.0;
  double dInput = avePos;
  myPID.Compute(gSetpoint, dInput, dOutput);
  double t = millis() / 1000.0;
  double dt = 1.0/tdelay;
  static double corrector = 0;
  corrector = corrector + dOutput;
  
  //Serial.println(dOutput);
  delay(tdelay);

  static int otime = millis()+5000;
  if(millis() > otime)
  { 
    gSetpoint = -gSetpoint;
    otime = millis()+5000;
  }

  LeftMotor->run(FORWARD);
  RightMotor->run(FORWARD); 

  LeftMotor->setSpeed(100 - dOutput);
  RightMotor->setSpeed(100 + dOutput);

  if (temp >= 5)
  {
  LeftMotor->run(RELEASE);
  RightMotor->run(RELEASE);
  exit(0);
  }
  
  
}
