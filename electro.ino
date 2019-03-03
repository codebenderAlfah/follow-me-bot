#include <Servo.h>  // servo library, note that this library disables PWM on pins 9 and 10!
#define TRIG 11 // the SRF05 Trig pin
#define ECHO 10 // the SRF05 Echo pin
#define SENSOR_STEP 10 // 5 degrees
#define NUMBER_OF_REPEATED_MEASURES 3
int numberOfSteps = int(180 / SENSOR_STEP);
unsigned long pulseTime;
unsigned long srfDistanceArray[180 / SENSOR_STEP]; // 180/step (where step is equal to 5)
unsigned long minimumDistanceThreshold = 5;
unsigned long maximumDistanceThreshold = 100;
//unsigned long SRFmeasurementsArray[NUMBER_OF_REPEATED_MEASURES];

int robotTurningCounter = 0;


Servo servo1;  // servo control object

void setup(){
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT);

  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  
  servo1.attach(3);
  Serial.begin(9600);
}// void setup()

float measurement(){
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  // wait for the pulse to return. The pulse
  // goes from low to HIGH to low, so we specify
  // that we want a HIGH-going pulse below:
  pulseTime = pulseIn(ECHO,HIGH);
  return pulseTime / 58.00;
}// float measurement()

boolean checkNeighborhood(int arrayIndex){
  unsigned long loweNeighborDiff = abs(srfDistanceArray[arrayIndex]-srfDistanceArray[arrayIndex-1]);
  unsigned long upperNeighborDiff = abs(srfDistanceArray[arrayIndex]-srfDistanceArray[arrayIndex+1]);
  if (loweNeighborDiff<20 || upperNeighborDiff<20){
    return true;
  }
  else{
    return false;
  }
}// boolean checkNeighborhood(int arrayIndex)

int measurementsDataMin(){
  unsigned long minDistance = maximumDistanceThreshold; // srfDistanceArray[0];
  int index = 0;
  for (int i=1; i<numberOfSteps-2; i++){
    if(srfDistanceArray[i]>minimumDistanceThreshold && srfDistanceArray[i] < minDistance){
      if(checkNeighborhood(i)){ // if TRUE
        index = i;
        minDistance = srfDistanceArray[i];
      }
    }  
  }// for
  index = index*SENSOR_STEP;
  return index;
}

/*
void turnRobotToObstaclePosition(int obstaclePosition){
  if(robotTurningCounter < 10 ){
    if(obstaclePosition == 0) go_forward();
    else if(obstaclePosition < 90) turn_right();    
    else turn_left();
    
    robotTurningCounter++;
  }
  
  else{
    stopRobot();
    delay(100);
  }     
}
*/

void goFront() {
  Serial.println("Front");
  
  analogWrite(5, 200);
  analogWrite(6, 200);
  delay(500);
}

void goLeft() {
  Serial.println("Front");
  
  analogWrite(5, HIGH);
  analogWrite(6, HIGH);
  //delay(500);
}

void goRight() {
  Serial.println("Front");
  
  analogWrite(5, HIGH);
  analogWrite(6, HIGH);
  //delay(500);
}

void stop() {
  Serial.println("Front");
  
  analogWrite(5, 0);
  analogWrite(6, 0);
  delay(500);
}

void loop(){

  //////// find me
  int position;
  int nearestObstaclePosition;
  int arrayIndex = 0;
  servo1.write(0);     // Tell servo to go to 0 degrees
  delay(31);         // Pause to get it time to move

  for(position = 0; position < 180; position += SENSOR_STEP){ // Tell servo to go to 180 degrees, stepping by 5 degrees
    servo1.write(position);  // Move to next position
    delay(31);               // Short pause to allow it to move, min 20ms
    if(arrayIndex<numberOfSteps-1){
       srfDistanceArray[arrayIndex] = measurement(); //getAverageDistance();
       //arrayIndex++;
    }
    Serial.println("Angle: "+String(position)+ ", Distance: "+String(srfDistanceArray[arrayIndex])+" cm");
    arrayIndex++;
  }
  nearestObstaclePosition = measurementsDataMin();
  servo1.write(nearestObstaclePosition);     // Tell servo to go to nearest obstacle position
  Serial.println("nearestObstaclePosition: "+String(nearestObstaclePosition));
  delay(500);

   //////////// bot, follow me
   if (nearestObstaclePosition >= 50 && nearestObstaclePosition <= 110) goFront();
   /*
   while (nearestObstaclePosition > 110 && nearestObstaclePosition <= 170) goLeft();
   while (nearestObstaclePosition >  10 && nearestObstaclePosition <   70) goRight();
   if    (nearestObstaclePosition == 0) {
        stop(); 
        servo.write(0); 
   }
   */
   else stop();
}
