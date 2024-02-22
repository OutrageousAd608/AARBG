
float airspeed;
double rollangle, pitchangle, yawangle;

void setup() {
  // put your setup code here, to run once:

  runmotor();
}

void loop() {
  // put your main code here, to run repeatedly:




}

void avoidobstacles(){
  unsigned long starttime = millis(); //start time equals time after last reset

  if (leftsensordistance<rightsensordistance&&(leftsensordistance!=0||rightsensordistance!=0)){
    while (millis() < starttime + 1000 ){ // turn for 1 second
      turnright();
    }
  }
  if (leftsensordistance>rightsensordistance&&(leftsensordistance!=0||rightsensordistance!=0)){
    while (millis() < starttime + 1000){ // turn for 1 second
    turnleft();
    }
  }

  return;
}

void turnleft(){

  if (rollangle>-20degrees){    //bank aircraft 30 degrees left and apply rudder
    aileronservo.write(80); //rotate ailerons 10 degrees in the left turn direction
  }
  rudderservo.write(100); //rotate rudder 10 degrees in the left turn direction
  if (rollangle<-20degrees){
    aileronservo.write(100); //if roll angle is getting too large, roll back to 20 degrees
  }
  return;
}

void turnright(){

  if (rollangle<20 degrees){    //bank aircraft 20 degrees right and apply rudder
   aileronservo.write(100); //rotate ailerons 10 degrees in the right turn direction
  }
  rudderservo.write(80); //rotate rudder 10 degrees in the right turn direction
  if (rollangle>20degrees){
    aileronservo.write(80); //if roll angle is getting too large, roll back to 20 degrees
  }
  return;
}
void pitchup(){
  if (pitchangle<25degrees){
    elevatorservo.write(80); //rotate elevators by 10 degrees in the upwards direction
  }
  if (pitchangle>25degrees){
    elevatorservo.write(100);
  }
  return;
}
void pitchdown(){
  if (pitchangle>-25degrees){
    elevatorservo.write(100); //rotate elevators 10 degrees in the downwards direction
  }
  if (pitchangle<-25degrees){
    elevatorservo.write(80);
  }
  return;
}
void flystraight(){
  if (yawangularacceleration>0){
    rudderservo.write(95); //if aircraft is starting to yaw right, apply 5 degrees left rudder
  }

  if (yawangularacceleration<0){
    rudderservo.write(85); //if aircraft is starting to yaw left, apply 5 degrees right rudder
  }

  return;
}
void runmotor(){
  
  while (rotaryencoderturns>initial + 200){
    stopperservo.write(70); //jam stopping gear
    AnalogWrite(motorpin, 1023 ); //rev motor at full speed
  }

    AnalogWrite(motorpin, 0 ); // stop motor
    stopperservo.write(90); //unjam stoppping gear

  return;
}

void avoidcrash(){
  if (rollangle<-25 degrees){
    aileronservo.write(100); //if roll angle is too far left, go back the other way to -20 degrees
  }
  if (rollangle>25 degrees){
    aileronservo.write(80); //if roll angle is too far right, go back the other way to 20 degrees
  }
  if (pitchangle>25degrees){
    elevatorservo.write(100); //if pitch angle is too high, push elevators down 
  }
  if (pitchangle<-25degrees){
    elevatorservo.write(80); //if pitch angle is too low, push elevators up
  }
  if (verticalacceleration<-10m/s^2 &&airspeed<1m/s){ //aircraft has stalled
    
  }
  return;
}
void returntohome(){

  //homelatitude = latitude of starting position
  //homelongitude = longitude of starting position
  
  currentlatitude = sensor reading;
  currentlongitude = sensor reading;


  while(ulatitudetohome!=ulatide && ulongitudetohome!=ulongitude)){
    turnleft();
  }


  oldlatitude = currentlatitude;
  oldlongitude = currentlongitude;
  return;
}




