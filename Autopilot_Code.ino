/*
AARBG code, 2023
- Wire NEO-6M GPS to Serial1
- Wire Benwake TF-Mini's to Serial2 & Serial3
- Wire Benewake TF-Luna, BNO055 & BMP388 via I2C
- Wire SD card MISO to pin 50, MOSI to 51, SCK to 52
- Wire MPXV7002DP pitot tube analog output to A1

- Wire aileron, elevator, rudder servos to PWM pins 2, 3, 4 respectively
- Wire motor stopper pin to PWM pin 5
- Wire motor control pin to PWM pin 6
- Wire buzzer to PWM pin 7
- Wire RGB LED to PWM 8, 9, 10 respectively

- Wire RF reciever to pin 11

- Set global definitions according to your needs
*/
#define PI 3.14159265358979323
#define MaxMem 500 //max number of each flight variable to store
#define SEALEVELPRESSURE_HPA 1014.4 //sea level pressure in Hpa
#define aircraftmass 0.5 //kg

#define servoincrement 12.0 //microseconds, approximately 2.2 degrees
#define crashangle 30.0 //degrees
#define mingrounddist 300 //minimum ground distance in centimeters to make a turn
#define obstacleturntime 3000 //in milliseconds
#define minspeed 4.0 //minimum speed to turn in m/s
#define timetogohome 180000 //time in milliseconds after which the aircraft will return to its starting position

#include <ServoTimer2.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TFMPlus.h>
#include <TFLI2C.h>
#include "printf.h"
#include <RH_ASK.h>

ServoTimer2 aileronservo;
ServoTimer2 elevatorservo;
ServoTimer2 rudderservo;
ServoTimer2 stopperservo;
ServoTimer2 motorcontrol;

//motor controls
int rotaryencoderturns = 0;
int const rotaryencoderpin = 23;
int const motorcontrolpin = 6;
int currentStateCLK = 0;
int lastStateCLK = 0;

int const aileronservopin = 2;
int const elevatorservopin = 3;
int const rudderservopin = 4;
int const stopperservopin = 5;

int const buzzerpin = 7;
int const redLED = 8; //needs 145 ohm resistor
int const greenLED = 9; //needs 90 ohm resistor
int const blueLED = 10; //needs 95 ohm resistor

//analog pins
int const voltagecheckerpin = A0;
int const airspeedpin = A1;

float batteryvoltage = 0;

//outside conditions
int Date = 0; // DDMMYY
int Time = 0; // HHMMSSCC
float temperature = 0;
float airpressure = 0;
float airdensity = 0;
int takeofftime = 0;

// global variables
const int earthradius = 6367460; //earth's radius in m for Wolfville, Nova Scotia

//PID controller variables
float olderrorL = 0;
float olderrorR = 0;
float olderrorS = 0;
float olderrorP = 0;
float olderrorPD = 0;
float errorsumL = 0;
float errorsumR = 0;
float errorsumS = 0;
float errorsumP = 0;
int pgain = 2;
int igain = 0.5;
int dgain = 1;
unsigned long currenttime = 0;
unsigned long previoustime = 0;
float elapsedtime = 0;

//servo positions in microseconds
int aileronpos = 1500;
int elevatorpos = 1500;
int rudderpos = 1500;
//turning bank angle
int leftbankangle = -20;
int rightbankangle = 20;
//pitching angles for flaring (degrees)
int initialflareangle = -5;
int finalflareangle = 5;

//aircraft controls and sensors
float airspeed = 0.0;
float pressurealtitude = 0.0;
float oldpressurealtitude = 0.0;
float GPSaltitude = 0.0;
float initialpressurealtitude = 0.0;
int turnflag = 0;

//BNO055 variables
float IMUrollangle = 0.00; //roll angle of the aircraft from -90 to 90 degrees
float IMUpitchangle = 0.00; //pitch angle of the aircraft from -180 to 180 degrees

float yawacceleration = 0;
float rollacceleration = 0;
float pitchacceleration = 0;
float accely = 0;
float accelx = 0;

//GPS variables
float homelatitude = 0.00, homelongitude = 0.00;
float currentlatitude = 0.00, currentlongitude = 0.00;
float oldxpos = 0.00, oldypos = 0.00;
float groundspeed = 0.0;
float magtohome = 0;

//Lidar sensor variables
int16_t leftdist = 0; // distance to object in centimeters
int16_t oldleftdist = 0;
int16_t changeindistL = 0; 

int16_t leftsensorflux = 0; // Strength or quality of return signal
int16_t lefttemp = 0; // Internal temperature of Lidar sensor chip

int16_t rightdist = 0; // distance to object in centimeters
int16_t oldrightdist = 0;
int16_t changeindistR = 0; 

int16_t rightsensorflux = 0; // Strength or quality of return signal
int16_t righttemp = 0;

int16_t tfAddr = TFL_DEF_ADR;    // default I2C address
int16_t bottomdist = 0 ;   // distance in centimeters
int16_t tfFlux = 0 ;   // signal quality in arbitrary units
int16_t tfTemp = 0 ;   // temperature in 0.01 degree Celsius

int16_t bottomtemp = 0; // temperature in 0.01 degree Celsius
int16_t tfladdr = 0;

//Data storage arrays
int Datestrg[MaxMem];
int Timestrg[MaxMem];
float Xaccelstrg[MaxMem];
float Yaccelstrg[MaxMem];
float Airspeedstrg[MaxMem];
float Grounspeedstrg[MaxMem];
float PresAltstrg[MaxMem];
float GPSAltstrg[MaxMem];
float Bankanglestrg[MaxMem];
float Pitchanglestrg[MaxMem];
float GPSlatstrg[MaxMem];
float GPSlonstrg[MaxMem];
float Tempstrg[MaxMem];
float Airdstystrg[MaxMem];
int16_t LeftLidarstrg[MaxMem];
int16_t RightLidarstrg[MaxMem];
int16_t BottomLidarstrg[MaxMem];
int i = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;

TFMPlus tfmPL;
TFMPlus tfmPR;
TFLI2C tflI2C;

File SDcard;
String SDheader = "TIME\tXacc\tYacc\tAIRSPD\tPRESALT\tBNKANGLE\tPTCHANGLE\tGPSLAT\tGPSLON\tTEMP\tAIRDSTY\tLLIDARDIST\tRLIDARDIST\tBLIDARDIST";

RH_ASK receiver;
bool gohome = false;
bool failure = false;

void setup() {
  // put your setup code here, to run once:
  bool BNOflag = true;
  bool GPSflag = true;
  bool BMP388flag = true;
  bool SDflag = true;
  bool radioflag = true;

  Serial.begin(115200);
  analogWrite(redLED, 255);
  delay(20);
  analogWrite(redLED, 0);

  Serial1.begin(9600);
  analogWrite(greenLED, 255);
  delay(20);
  analogWrite(greenLED, 0);

  Serial2.begin(115200);
  analogWrite(blueLED, 255);
  delay(20);
  analogWrite(blueLED, 0);
  tfmPL.begin(&Serial2);
  delay(20);

  Serial3.begin(115200);
  analogWrite(redLED, 255);
  analogWrite(greenLED, 255);
  analogWrite(blueLED, 255);
  delay(100);
  analogWrite(redLED, 0);
  analogWrite(greenLED, 0);
  analogWrite(blueLED, 0);
  tfmPR.begin(&Serial3);
  delay(20);

  //pin setup

  //aileronservo.attach(aileronservopin);
  elevatorservo.attach(elevatorservopin);
  //rudderservo.attach(rudderservopin);
  //stopperservo.attach(stopperservopin);
  //motorcontrol.attach(motorcontrolpin);

  //aileronservo.write(1500);
  elevatorservo.write(1500);
  //rudderservo.write(1500);
  //stopperservo.write(1500);
  delay(200);

  //rotary encoder pins
  //pinMode(rotaryencoderpin, INPUT);   // Set encoder pins as inputs
  //lastStateCLK = digitalRead(rotaryencoderpin);  // Read the initial state of CLK

  pinMode(buzzerpin, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);

  pinMode(airspeedpin, INPUT);

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    BMP388flag = false;
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_1_5_HZ);

  //BNO055 setup
  if (!bno.begin()){
    BNOflag = false;
  }

  printf_begin(); // Initialize printf

  //SD card initialization
  if (!SD.begin(53)) {
    SDflag = false;
  }

  //Radio reciever initialization
  /*if (!receiver.init()){
    radioflag = false;
  }*/

  //error protocol
  if (BMP388flag == false || BNOflag == false || SDflag == false || radioflag == false){
    while(1){
      analogWrite(redLED, 255); //light error LED
      tone(buzzerpin, 1000, 1000);
      Serial.println(F("Sensor initialization failure"));
    }
  }
  uint8_t system, gyro, accel, mag = 0;

  /*while (system < 3 || gyro < 3 || accel < 3 || mag < 3){
    bno.getCalibration(&system, &gyro, &accel, &mag);
    analogWrite(blueLED, 255); //calibration not finished LED indicator
  }
  analogWrite(blueLED, 0);
  delay(1000);
*/
  //get initial values
  bmp.performReading();
  delay(20);
  initialpressurealtitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  Serial.print(F("Initial altitude = "));
  Serial.println(initialpressurealtitude);
  delay(1000);

  analogWrite(redLED, 255);
  analogWrite(greenLED, 255);
  analogWrite(blueLED, 255);

  //while(homelatitude == 0 || homelongitude == 0){
    while (Serial1.available() > 0){
    gps.encode(Serial1.read());

      if (gps.location.isUpdated()){
        homelatitude = (gps.location.lat());
        homelongitude = (gps.location.lng());
        Date = (gps.date.value());
      }
    }
 // }

  analogWrite(redLED, 0);
  analogWrite(greenLED, 0);
  analogWrite(blueLED, 0);

  Serial.print(F("Home latitude, longitude = "));
  Serial.print(homelatitude);
  Serial.print(F("\t"));
  Serial.println(homelongitude);
  Serial.print(F("Date = "));
  Serial.println(Date);

  display_freeram();

  analogWrite(greenLED, 255);
  
  //intitialize motor
  motorcontrol.write(1900);
  delay(7000);
  motorcontrol.write(1100);
  analogWrite(greenLED, 0);

  //takeoff
  Serial.println(F("Taking off!"));
  //runmotor();

  delay(1000); //time to get into the air
  takeofftime = millis(); //time that the airplane first started to take off

}

void loop() {
  currenttime = millis();
  elapsedtime = currenttime - previoustime;

  // Set buffer to size of expected message
  /*uint8_t buf[4]; //character space to recieve the word "stop"
  uint8_t buflen = sizeof(buf);
  // Check if received packet is correct size
  if (receiver.recv(buf, &buflen)){
    gohome = true;
  }
*/
  getsystemdata();
  display_freeram();
  analogWrite(redLED, 255);
  delay(50);
  analogWrite(redLED, 0);
  delay(1000);
  savetoMemory();
  if (millis() > 50000){
    writetoSD();
    analogWrite(greenLED, 255);
    while(1);
  }
/*
  failure = checkfailure();
  while (failure == true){
    returntohome();

    tone(buzzerpin, 1000);
    analogWrite(redLED, 255);
    Serial.print("Failure detected");
    if (bottomdist < 500){ //dist in cm
      flare();
    }
  }

  //ssd1309 LCD display
  
  turnflag = avoidobstacles();
  if (turnflag == 1){
    setrollstraight();
    turnflag = 0;
  }

  if (millis() > timetogohome || gohome == true){
    Serial.println("Returning to home!");
    returntohome();
  }

  if (bottomdist < 500){ //dist in cm
    Serial.print("FLARING!");
    flare();
  }

  if (bottomdist < 20 && pressurealtitude < 0.2){ //aircraft has landed
    writetoSD();
    while(1){
      tone(buzzerpin, 5000, 1);
      analogWrite(greenLED, 255);
      analogWrite(redLED, 255);
      analogWrite(blueLED, 255);
      delay(1000);
    }
  }
  */
  previoustime = currenttime;
  
}

void getsystemdata(){
  static unsigned long previousMillisbmp = 0;
  const int BMPinterval = 1500; // in ms

  unsigned long currentMillis = millis();

  batteryvoltage = analogRead(voltagecheckerpin);

  //Lidar readings
  tfmPL.getData(leftdist, leftsensorflux, lefttemp);
  delay(500);
  tfmPR.getData(rightdist, rightsensorflux, righttemp);
  delay(2000);

  if(tflI2C.getData(bottomdist, tfFlux, tfTemp, tfAddr)){
    bottomtemp = int16_t( tfTemp / 100.0);
  }
  delay(200);

  //BMP388 readings
  if ((currentMillis - previousMillisbmp) >= BMPinterval) {
    previousMillisbmp = currentMillis;

    bmp.performReading();
    temperature = bmp.temperature; //in degrees C
    airpressure = bmp.pressure; //pressure in Pa
    pressurealtitude = (bmp.readAltitude(SEALEVELPRESSURE_HPA)) - initialpressurealtitude; //actual altitude in m
  }

  //MPXV7002DP readings
  float airspeedvin = analogRead(airspeedpin); //raw voltage data from MPXV7002DP

  float diffpressure = 1000*((airspeedvin / 1023.0) - 0.04)/0.09 ; // Transfer function converted in Pa (Check datasheet page 6)
  airdensity = airpressure / (287.058 * (temperature + 273.15)); //using gas constant for dry air
  airspeed = sqrt(2 * diffpressure / airdensity); //in m/s
  delay(50);

  //NEO-6M GPS readings
  if (Serial1.available() > 0){
    gps.encode(Serial1.read());
    
    if (gps.location.isUpdated()){
        currentlatitude = (gps.location.lat());
        currentlongitude = (gps.location.lng());
        Time = (gps.time.value());
        groundspeed = (gps.speed.mps());
        GPSaltitude = (gps.altitude.meters());
    }
  }

  //BNO055 readings
  sensors_event_t orientationData , angVelocityData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  delay(500);
  
  IMUrollangle = orientationData.orientation.y;
  IMUpitchangle = orientationData.orientation.z;

  yawacceleration = angVelocityData.gyro.x;
  rollacceleration = angVelocityData.gyro.y;
  pitchacceleration = angVelocityData.gyro.z;

  accelx = (linearAccelData.acceleration.x);
  accely = linearAccelData.acceleration.y;
  delay(500);

  //print all the data to serial monitor
  Serial.print(F("left distance = "));
  Serial.print(leftdist);
  Serial.print(F("\tleft sensor flux = "));
  Serial.print(leftsensorflux);
  Serial.print(F("\tLeft sensor temperature = "));
  Serial.println(lefttemp);

  Serial.print(F("right distance = "));
  Serial.print(rightdist);
  Serial.print(F("\tright sensor flux = "));
  Serial.print(rightsensorflux);
  Serial.print(F("\tRight sensor temperature = "));
  Serial.println(righttemp);

  Serial.print(F("Bottom distance = "));
  Serial.print(bottomdist);
  Serial.print(F("\tSensor flux = "));
  Serial.print(tfFlux);
  Serial.print(F("\tBottom sensor temperature = "));
  Serial.println(bottomtemp);

  Serial.print(F("Pressure altitude = "));
  Serial.print(pressurealtitude);
  Serial.print(F("\tAir pressure = "));
  Serial.print(airpressure);
  Serial.print(F("\tTemperature = "));
  Serial.println(temperature);

  Serial.print(F("airspeed = "));
  Serial.println(airspeed);

  Serial.print(F("current latitude, longitude = "));
  Serial.print(currentlatitude);
  Serial.print(F("\t"));
  Serial.println(currentlongitude);
  Serial.print(F("GPS groundspeed = "));
  Serial.print(groundspeed);
  Serial.print(F("\tGPS altitude = "));
  Serial.println(GPSaltitude);

  Serial.print(F("Roll, pitch = "));
  Serial.print(IMUrollangle);
  Serial.println(IMUpitchangle);
  
  Serial.print(F("Yaw, roll, pitch acceleration = "));
  Serial.print(yawacceleration);
  Serial.print(F("\t"));
  Serial.print(rollacceleration);
  Serial.print(F("\t"));
  Serial.println(pitchacceleration);
  
  Serial.print(F("Acceleration in x, y = "));
  Serial.print(accelx);
  Serial.print(F("\t"));
  Serial.println(accely);

  Serial.print(F("Rotary encoder turns = "));
  Serial.println(rotaryencoderturns);

  Serial.print(F("Battery voltage = "));
  Serial.println(batteryvoltage);
  Serial.println("");
  
  return;
}

void savetoMemory(){
  static unsigned long previousMillis = 0;
  const int saveinterval = 3000; // save data every 3 seconds

  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis) >= saveinterval) {
    previousMillis = currentMillis;
  
    Timestrg[i] = Time;
    Xaccelstrg[i] = accelx;
    Yaccelstrg[i] = accely;
    Airspeedstrg[i] = airspeed;
    Grounspeedstrg[i] = groundspeed;
    PresAltstrg[i] = pressurealtitude;
    GPSAltstrg[i] = GPSaltitude;
    Bankanglestrg[i] = IMUrollangle;
    Pitchanglestrg[i] = IMUpitchangle;
    GPSlatstrg[i] = currentlatitude;
    GPSlonstrg[i] = currentlongitude;
    Tempstrg[i] = temperature;
    Airdstystrg[i] = airdensity;
    LeftLidarstrg[i] = leftdist;
    RightLidarstrg[i] = rightdist;
    BottomLidarstrg[i] = bottomdist;

    i++;
  }

  return;
}


bool checkfailure(){

  if (batteryvoltage < 780){ //can't directly measure 6.8V, so the voltage must be dropped by 3V. Arduino ADC 0-1024, 780 approx. 3.8V
    return true;
  }
  if (lefttemp > 70 || righttemp > 70 || bottomtemp > 70){ //LiDAR sensor temperatures
    return true;
  }
  if (temperature > 35){ //aircraft interior temperature
    return true;
  }
  if (magtohome > 50){ //horizontal distance to home greater than 50m
    return true;
  }
  if (pressurealtitude > 12){ //in meters
    return true;
  }

  return false;
}

int avoidobstacles(){
  unsigned long starttime = millis();

  if (leftdist < (rightdist + 10) && (leftdist != 0 || rightdist != 0)){ //distance in centimeters
    while (millis() < starttime + obstacleturntime ){
      turnright();
    }
  }
  if ((leftdist + 10) > rightdist && (leftdist != 0 || rightdist != 0)){
    while (millis() < starttime + obstacleturntime){
    turnleft();
    }
  }
  return 1;
}

void turnleft(){ //loop this function as much as necessary

  if (airspeed > minspeed && bottomdist > mingrounddist){ //ensure aircraft is not stalled and approaching ground
    double errorL = IMUrollangle - leftbankangle;
    double errorproportionL = errorL / leftbankangle;
    errorsumL = errorsumL + errorL;
    double changeinerrorL = (errorL - olderrorL) / elapsedtime;

    double PIDoutputL = pgain * errorproportionL + igain * errorsumL + dgain * changeinerrorL;

    if (PIDoutputL > 0 && aileronpos<1585){ //1585 is approximately +16 degrees; aileron servo must not extend past this
      aileronservo.write(aileronpos + servoincrement); //If the PIDouput indicates the plane is banked too far right, shift the servo 2.7 degrees left (15 microseconds)
      aileronpos += servoincrement;

    }else if (PIDoutputL < 0 && aileronpos>1415){ //1585 is approximately -16 degrees; aileron servo must not extend past this
      aileronservo.write(aileronpos - servoincrement); //If the PIDouput indicates the plane is banked too far left, shift the servo 2.7 degrees right (15 microseconds)
      aileronpos -= servoincrement;
    }
    for (rudderpos; rudderpos > 1465; rudderpos -= servoincrement){
      rudderservo.write(rudderpos -= servoincrement);
    }
    //rotate rudderservo in 2.7 degree increments until it reaches 7 degrees left (-35 microseconds)

    olderrorL = errorL;

  }
  
  return;
}

void turnright(){ //loop this function as much as necessary

  if ((airspeed > minspeed) && (bottomdist > mingrounddist)){

    double errorR = IMUrollangle - rightbankangle;
    double errorproportionR = errorR / rightbankangle;
    errorsumR = errorsumR + errorR;
    double changeinerrorR = (errorR - olderrorR) / elapsedtime;

    double PIDoutputR = pgain * errorproportionR + igain * errorsumR + dgain * changeinerrorR;

    if (PIDoutputR > 0 && aileronpos>1415){  
      aileronservo.write(aileronpos - servoincrement); //If the PIDouput indicates the plane is banked too far right, shift the servo 2.7 degrees left (15 microseconds)
      aileronpos -= servoincrement;
    
    }else if (PIDoutputR < 0 && aileronpos<1585){
      aileronservo.write(aileronpos + servoincrement); //If the PIDouput indicates the plane is banked too far left, shift the servo 2.7 degrees right (15 microseconds)
      aileronpos += servoincrement;
    }

    for (rudderpos; rudderpos < 1535; rudderpos += servoincrement){ 
      rudderservo.write(rudderpos += servoincrement);
    //rotate rudderservo in 2.7 degree increments until it reaches 7 degrees right (35 microseconds)
    }
    olderrorR = errorR;
  }

  return;
}
void setrollstraight(){ //run this function only once to set the aircraft's position straight; do not loop

  rudderservo.write(1500); //set the rudder back to its straight position
  rudderpos = 1500;

  double errorS = IMUrollangle;
  errorsumS = errorsumS + errorS; //any roll angle is error in this case (roll angle - 0 = roll angle)

  double PIoutput = pgain*errorS + igain*errorsumS;

  while (PIoutput > 0.5 && aileronpos<1585){ 
    aileronservo.write(aileronpos + servoincrement); //If the PIDouput indicates the plane is banked too far right, increment the servo 2.7 degrees left (15 microseconds)
    aileronpos += servoincrement;

  }while (PIoutput < -0.5 && aileronpos>1415){
    aileronservo.write(aileronpos - servoincrement); //If the PIDouput indicates the plane is banked too far left, shift the servo 2.7 degrees right (15 microseconds)
    aileronpos -= servoincrement;
  }

  return;
}

void runmotor(){
  //rotary encoder
  rotaryencoderturns = 0;

  currentStateCLK = digitalRead(rotaryencoderpin);  // Read the current state of CLK
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    rotaryencoderturns++;
  }
  lastStateCLK = currentStateCLK;

  if (rotaryencoderturns < 300){ //number of turns of the rubber band
    stopperservo.write(1400); //jam stopping gear
    motorcontrol.write(1900); //rev motor at full speed
  }
  motorcontrol.write(1100); // stop motor
  delay(100);
  stopperservo.write(1500); //unjam stoppping gear

  return;
}

void avoidcrash(){
  unsigned long currenttime = millis();
  int turntime = 1500; //time in milliseconds to turn for

  if (IMUrollangle < (-crashangle)){
    while (millis() < (currenttime + turntime)){
      aileronservo.write(1530); //if roll angle is too large, turn right
    }
    aileronservo.write(1500);
  }

  if (IMUrollangle > crashangle){
    while (millis() < (currenttime + turntime)){ 
      aileronservo.write(1470); //if roll angle is too large, turn left
    }
    aileronservo.write(1500);
  }

  if (IMUpitchangle > crashangle){
    while (millis() < (currenttime + turntime)){
      elevatorservo.write(1470);
    }
    elevatorservo.write(1500);
  }
  if (IMUpitchangle < (-crashangle)){
    while (millis() < (currenttime + turntime)){
      elevatorservo.write(1530);
    }
    elevatorservo.write(1500);
  }
  if (accely > 1){ //1 m/s/s
    while (millis() < (currenttime + turntime)){
      aileronservo.write(1470); //if starting to slip right, roll left
    }
    aileronservo.write(1500);
  }
    if (accely < -1){
    while (millis() < (currenttime + turntime)){
      aileronservo.write(1530); //if starting to slip left, roll right
    }
    aileronservo.write(1500);
  }

  return;
}

void flare(){

  if (bottomdist > 100 && bottomdist < 300){  //distance to ground greater than 1 meter but less than 3 meters
    double initialpitcherror = initialflareangle - IMUpitchangle;
    double initialproportion = initialpitcherror / initialflareangle;
    errorsumP = errorsumP + initialpitcherror;
    double changeinerrorP = (initialpitcherror - olderrorP) / elapsedtime;

    double PIDoutput = pgain * initialproportion + igain * errorsumP + dgain * changeinerrorP;

    olderrorP = initialpitcherror;
    if (PIDoutput > 0){
      elevatorservo.write(elevatorpos - servoincrement);
      elevatorpos -= servoincrement;

    }else if (PIDoutput < 0){
      elevatorservo.write(elevatorpos + servoincrement);
      elevatorpos += servoincrement;
    }

  }else if (bottomdist < 50){
    double finalpitcherror = finalflareangle - IMUpitchangle;
    double finalproportion = finalpitcherror / finalflareangle;
    errorsumP = errorsumP + finalpitcherror;
    double changeinerrorP = finalpitcherror - olderrorPD;

    double finalPIDoutput = pgain * finalproportion + igain * errorsumP + dgain * changeinerrorP;

    olderrorPD = finalpitcherror;

    if (finalPIDoutput > 0){
      elevatorservo.write(elevatorpos - servoincrement);
      elevatorpos -= servoincrement;

    }else if (finalPIDoutput < 0){
      elevatorservo.write(elevatorpos + servoincrement);
      elevatorpos += servoincrement;
    }
  }

  return;
}

void returntohome(){
  //homelatitude = latitude of starting position
  //homelongitude = longitude of starting position

  // get sensor data first
  homelatitude = homelatitude * PI/180; //convering to radians
  homelongitude = homelongitude * PI/180;

  currentlatitude = currentlatitude * PI/180; 
  currentlongitude = currentlatitude * PI/180;

  float hxpos = earthradius * cos(homelatitude) * cos(homelongitude); //approximating the earth as a sphere to get home cartesian coordinates
  float hypos = earthradius * cos(homelatitude) * sin(homelongitude);

  float xpos = earthradius * cos(currentlatitude) * cos(currentlongitude);
  float ypos = earthradius * cos(currentlatitude) * sin(currentlongitude);

  float xvectortohome = hxpos - xpos;
  float yvectortohome = hypos - ypos;
  magtohome = sqrt(xvectortohome * xvectortohome + yvectortohome * yvectortohome); //in meters
  float uxvectortohome = xvectortohome / magtohome;
  float uyvectortohome = yvectortohome / magtohome;


  float xvector = xpos - oldxpos;
  float yvector = ypos - oldypos;
  float mag = sqrt(xvector * xvector + yvector * yvector);
  float uxvector = xvector / mag;
  float uyvector = yvector / mag;

  float angletohome = atan2(uyvectortohome, uxvectortohome);
  float currentangle = atan2(uyvector, uxvector);
  float angledifference = currentangle - angletohome;

  if (angledifference > 0.017) { //error allowance of 0.017 rad (1 degree) (4 m/s)
   turnleft();  
  }
  if (angledifference < -0.017) { //error allowance of 0.017 rad (1 degree) (4 m/s)
   turnright();  
  }
  
}
void writetoSD(){
  int j = 0; //counter
  String SDheader = "TIME    Xacc    Yacc    AIRSPD    PRESALT    BNKANGLE    PTCHANGLE    GPSLAT    GPSLON    TEMP    AIRDSTY    LLIDARDIST    RLIDARDIST    BLIDARDIST";

  if (SDcard){
    SDcard = SD.open("flightdata.txt", FILE_WRITE);
    SDcard.print(SDheader);

    for (j = 0; j < i; j++){

      SDcard.println(Time);
      SDcard.print("\t");

      SDcard.print(accelx);
      SDcard.print("\t");

      SDcard.print(accely);
      SDcard.print("\t");

      SDcard.print(airspeed);
      SDcard.print("\t");

      SDcard.print(groundspeed);
      SDcard.print("\t");

      SDcard.print(pressurealtitude);
      SDcard.print("\t");

      SDcard.print(GPSaltitude);
      SDcard.print("\t");

      SDcard.print(IMUrollangle);
      SDcard.print("\t");

      SDcard.print(IMUpitchangle);
      SDcard.print("\t");

      SDcard.print(currentlatitude);
      SDcard.print("\t");

      SDcard.print(currentlongitude);
      SDcard.print("\t");

      SDcard.print(temperature);
      SDcard.print("\t");

      SDcard.print(airdensity);
      SDcard.print("\t");

      SDcard.print(leftdist);
      SDcard.print("\t");

      SDcard.print(rightdist);
      SDcard.print("\t");

      SDcard.print(bottomdist);

    }
    SDcard.println("\nFlight concluded!");
  }
  
  // close the file:
  SDcard.close();

  return;
}

void display_freeram() {
  Serial.print(F("- SRAM left: "));
  Serial.println(freeRam());
}

int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0  
    ? (int)&__heap_start : (int) __brkval);  
}