#define servoincrement 15 //microseconds, approximately 2.7 degrees
#define crashangle 30 //degrees
#define PI 3.14159265358979323
#define obstacleturntime 3000 //3 seconds

#include <Servo.h>

Servo aileronservo;
Servo elevatorservo;
Servo rudderservo;
Servo stopperservo;

// global variables
int earthradius = 6367460; //earth's radius in m for Wolfville, Nova Scotia
int timetogohome = 300000; //time in milliseconds after which the aircraft will return to its start position

//PID controller variables
double olderrorL = 0;
double olderrorR = 0;
double olderrorS = 0;
int pgain = 1;
int igain = 0.5;
int dgain = 2;

//servo positions in microseconds
int aileronpos = 1500;
int elevatorpos = 1500;
int rudderpos = 1500;
//turning bank angle
int leftbankangle = -20;
int rightbankangle = 20;

//aircraft controls and sensors
double IMUrollangle = 0; //roll angle of the aircraft from -90 to 90 degrees
double IMUpitchangle = 0; //pitch angle of the aircraft from -180 to 180 degrees
float airspeed = 0;
float altitude = 0;
int leftdistance = 0;
int rightdistance = 0;
int bottomdistance = 0;
int altitudeflag = 0;

//GPS variables
double homelatitude = 0, homelongitude = 0;
double currentlatitude = 0, currentlongitude = 0;
double oldxpos = 0, oldypos = 0;
float groundspeed = 0;


void setup() {
  // put your setup code here, to run once:

  //runmotor();

}

void loop() {
  // put your main code here, to run repeatedly:

  avoidobstacles();

  if (millis() > timetogohome || altitudeflag == 1){
    returntohome();
  }

}


void avoidobstacles(){
  unsigned long starttime = millis(); //start time equals time after last reset

  if (leftdistance < rightdistance && (leftdistance!=0||rightdistance!=0)){
    while (millis() < starttime + obstacleturntime ){ // turn for 1 second
      turnright();
    }
  }
  if (leftdistance > rightdistance && (leftdistance!=0||rightdistance!=0)){
    while (millis() < starttime + obstacleturntime){ // turn for 1 second
    turnleft();
    }
  }
  return;
}

int turnleft(){ //loop this function as much as necessary

  double errorL = IMUrollangle - leftbankangle;
  double errorproportionL = errorL / leftbankangle;
  double errorsumL = errorsumL + errorL;
  double changeinerrorL = errorL - olderrorL;

  double PIDoutputL = pgain * errorproportionL + igain * errorsumL + dgain * changeinerrorL;

  if (PIDoutputL > 0 && aileronpos<1585){ //1585 is approximately +16 degrees; aileron servo must not extend past this
    aileronservo.writeMicroseconds(aileronpos + servoincrement); //If the PIDouput indicates the plane is banked too far right, shift the servo 2.7 degrees left (15 microseconds)
    aileronpos += servoincrement;

  }else if (PIDoutputL < 0 && aileronpos>1415){ //1585 is approximately -16 degrees; aileron servo must not extend past this
    aileronservo.writeMicroseconds(aileronpos - servoincrement); //If the PIDouput indicates the plane is banked too far left, shift the servo 2.7 degrees right (15 microseconds)
    aileronpos -= servoincrement;
  }
  for (rudderpos; rudderpos > 1465; rudderpos -= servoincrement){
    rudderservo.writeMicroseconds(rudderpos -= servoincrement);
  }
  //rotate rudderservo in 2.7 degree increments until it reaches 7 degrees left (-35 microseconds)

  olderrorL = errorL;

  return;
}

int turnright(){ //loop this function as much as necessary

  double errorR = IMUrollangle - rightbankangle;
  double errorproportionR = errorR / rightbankangle;
  double errorsumR = errorsumR + errorR;
  double changeinerrorR = errorR - olderrorR;

  double PIDoutputR = pgain * errorproportionR + igain * errorsumR + dgain * changeinerrorR;

  if (PIDoutputR > 0 && aileronpos>1415){  
    aileronservo.writeMicroseconds(aileronpos - servoincrement); //If the PIDouput indicates the plane is banked too far right, shift the servo 2.7 degrees left (15 microseconds)
    aileronpos -= servoincrement;
    
  }else if (PIDoutputR < 0 && aileronpos<1585){
    aileronservo.writeMicroseconds(aileronpos + servoincrement); //If the PIDouput indicates the plane is banked too far left, shift the servo 2.7 degrees right (15 microseconds)
    aileronpos += servoincrement;

  }
  for (rudderpos; rudderpos < 1535; rudderpos += servoincrement){ 
    rudderservo.writeMicroseconds(rudderpos += servoincrement);
  //rotate rudderservo in 2.7 degree increments until it reaches 7 degrees right (35 microseconds)
  }
  olderrorR = errorR;

  return;
}
void setrollstraight(){ //run this function only once to set the aircraft's position straight; do not loop

  rudderservo.writeMicroseconds(1500); //set the rudder back to its straight position
  rudderpos = 1500;

  double errorS = IMUrollangle;
  double errorsumS = igain * errorsumS + pgain * errorS; //any roll angle is error in this case (roll angle - 0 = roll angle)

  double PIoutput = errorS + errorsumS;

  while (PIoutput > 1 && aileronpos<1585){ 
    aileronservo.writeMicroseconds(aileronpos + servoincrement); //If the PIDouput indicates the plane is banked too far right, increment the servo 2.7 degrees left (15 microseconds)
    aileronpos += servoincrement;

  }while (PIoutput < 1 && aileronpos>1415){
    aileronservo.write(aileronpos - servoincrement); //If the PIDouput indicates the plane is banked too far left, shift the servo 2.7 degrees right (15 microseconds)
    aileronpos -= servoincrement;
  }

  return;
}
/*
void runmotor(){
  
  while (rotaryencoderturns < initial + 200){
    stopperservo.writeMicroseconds(1400); //jam stopping gear
    AnalogWrite(motorpin, 1023 ); //rev motor at full speed
  }
  delay (3000);
  AnalogWrite(motorpin, 0 ); // stop motor
  stopperservo.writeMicroseconds(1500); //unjam stoppping gear

  return;
}
*/
void avoidcrash(){
  if (IMUrollangle < (-crashangle)){
    aileronservo.writeMicroseconds(aileronpos + servoincrement); //if roll angle is too large, add servo increments
    aileronpos += servoincrement;
  }
  if (IMUrollangle > crashangle){
    aileronservo.writeMicroseconds(aileronpos - servoincrement); //if roll angle is too far right, go back the other way to 20 degrees
    aileronpos -= servoincrement;
  }
  if (IMUpitchangle > crashangle){
    elevatorservo.write(elevatorpos - servoincrement); //if pitch angle is too high, push elevators down 
    elevatorpos -= servoincrement;
    
  }
  if (IMUpitchangle < (-crashangle)){
    elevatorservo.write(elevatorpos + servoincrement); //if pitch angle is too low, push elevators up
    elevatorpos += servoincrement;
  }
  return;
}
void returntohome(){
  int flag = 0;
  //homelatitude = latitude of starting position
  //homelongitude = longitude of starting position

  // get sensor data first
  homelatitude = homelatitude * PI/180; //convering to radians
  homelongitude = homelongitude * PI/180;

  currentlatitude = currentlatitude * PI/180; 
  currentlongitude = currentlatitude * PI/180;

  double hxpos = earthradius * cos(homelatitude) * cos(homelongitude); //approximating the earth as a sphere to get home cartesian coordinates
  double hypos = earthradius * cos(homelatitude) * sin(homelongitude);

  double xpos = earthradius * cos(currentlatitude) * cos(currentlongitude);
  double ypos = earthradius * cos(currentlatitude) * sin(currentlongitude);

  double xvectortohome = hxpos - xpos;
  double yvectortohome = hypos - ypos;
  float magtohome = sqrt(xvectortohome * xvectortohome + yvectortohome * yvectortohome); //in meters
  double uxvectortohome = xvectortohome / magtohome;
  double uyvectortohome = yvectortohome / magtohome;


  double xvector = xpos - oldxpos;
  double yvector = ypos - oldypos;
  float mag = sqrt(xvector * xvector + yvector * yvector);
  double uxvector = xvector / mag;
  double uyvector = yvector / mag;


  float dotprod = uxvectortohome * uxvector + uyvectortohome * uyvector;
  float angle = acos(dotprod);

  if (angle > 0.07){ //error allowance of 0.07 rad (4 degrees)
    turnleft();
  }else if (flag !=1){
    setrollstraight();
    flag = 1;
  }

  oldxpos = xpos;
  oldypos = ypos;

  return;
}
