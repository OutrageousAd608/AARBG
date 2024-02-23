#define servoincrement 15 //microseconds, approximately 2.7 degrees
#define crashangle 30 //degrees
#define PI 3.14159265358979323

#include <Servo.h>

Servo aileronservo;
Servo elevatorservo;
Servo rudderservo;
Servo stopperservo;

// global variables
double IMUrollangle = 0;
double IMUpitchangle = 0;

double errorL = 0;
double errorproportionL = 0;
double errorsumL = 0;
double changeinerrorL = 0;
double olderrorL = 0;
double PIDoutputL = 0;

double errorR = 0;
double errorproportionR = 0;
double errorsumR = 0;
double changeinerrorR = 0;
double olderrorR = 0;
double PIDoutputR = 0;

double errorS = 0;
double errorsumS = 0;
double olderrorS = 0;
double PIoutput = 0;

int aileronpos = 1500;
int elevatorpos = 1500;
int rudderpos = 1500;
int leftbankangle = -20;
int rightbankangle = 20;

double homelatitude = 0, homelongitude = 0;
double currentlatitude = 0, currentlongitude = 0;
double oldxpos = 0, oldypos = 0;

float airspeed = 0;
float altitude = 0;

int leftdistance = 0;
int rightdistance = 0;

void setup() {
  // put your setup code here, to run once:
  runmotor();

}

void loop() {
  // put your main code here, to run repeatedly:




}

void avoidobstacles(){
  unsigned long starttime = millis(); //start time equals time after last reset

  if (leftdistance < rightdistance && (leftdistance!=0||rightdistance!=0)){
    while (millis() < starttime + 1000 ){ // turn for 1 second
      turnright();
    }
  }
  if (leftdistance > rightdistance && (leftdistance!=0||rightdistance!=0)){
    while (millis() < starttime + 1000){ // turn for 1 second
    turnleft();
    }
  }
  return;
}

int turnleft(){ //loop this function as much as necessary

  errorL = IMUrollangle - leftbankangle;
  errorproportionL = errorL / leftbankangle;
  errorsumL = errorsumL + errorL;
  changeinerrorL = errorL - olderrorL;

  PIDoutputL = errorproportionL + errorsumL + 2*changeinerrorL;

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

  errorR = IMUrollangle - rightbankangle;
  errorproportionR = errorR / rightbankangle;
  errorsumR = errorsumR + errorR;
  changeinerrorR = errorR - olderrorR;

  PIDoutputR = errorproportionR + errorsumR + 2*changeinerrorR;

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

  errorS = IMUrollangle;
  errorsumS = errorsumS + errorS; //any roll angle is error in this case (roll angle - 0 = roll angle)

  PIoutput = errorS + errorsumS;

  while (PIoutput > 1 && aileronpos<1585){ 
    aileronservo.writeMicroseconds(aileronpos + servoincrement); //If the PIDouput indicates the plane is banked too far right, increment the servo 2.7 degrees left (15 microseconds)
    aileronpos += servoincrement;

  }while (PIoutput < 1 && aileronpos>1415){
    aileronservo.write(aileronpos - servoincrement); //If the PIDouput indicates the plane is banked too far left, shift the servo 2.7 degrees right (15 microseconds)
    aileronpos -= servoincrement;
  }

  return;
}

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
  int earthradius = 6367460; //Earth's radius in m for Wolfville, Nova Scotia

  //homelatitude = latitude of starting position
  //homelongitude = longitude of starting position

  // get sensor data first
  homelatitude = homelatitude * PI/180; //convering to radians
  homelongitude = homelongitude * PI/180;

  currentlatitude = currentlatitude*PI/180; 
  currentlongitude = currentlatitude*PI/180;

  double hxpos = earthradius * cos(homelatitude) * cos(homelongitude); //approximating the earth as a sphere
  double hypos = earthradius * cos(homelatitude) * sin(homelongitude);

  double xpos = earthradius * cos(currentlatitude) * cos(currentlongitude);
  double ypos = earthradius * cos(currentlatitude) * sin(currentlongitude);

  double xvectortohome = hxpos - xpos;
  double yvectortohome = hypos - ypos;
  float magtohome = sqrt(xvectortohome * xvectortohome + yvectortohome * yvectortohome); //in meters
  double uxvectortohome = xvectortohome / magtohome;
  double uyvectortohome = yvectortohome / magtohome;


  double currentxvector = xpos - oldxpos;
  double currentyvector = ypos - oldypos;
  float currentmag = sqrt(currentxvector * currentxvector + currentyvector * currentyvector);
  double ucurrentxvector = currentxvector / currentmag;
  double ucurrentyvector = currentyvector / currentmag;


  float dotprod = xvectortohome * currentxvector + yvectortohome * currentyvector;
  float angle = acos(dotprod);

  if (angle > 0.09){
    turnleft();
  }else{
    setrollstraight();
  }

  oldxpos = xpos;
  oldypos = ypos;

  return;
}




