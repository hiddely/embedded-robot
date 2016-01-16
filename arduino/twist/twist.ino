
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

const int PIN_LED = 13;
const int PIN_1_FWD = 6;
const int PIN_1_REV = 7;
const int PIN_1_ENA = 24;
const int PIN_2_FWD = 2;
const int PIN_2_REV = 3;
const int PIN_2_ENA = 25;
const int PIN_US_TRIG = 23;
const int PIN_US_ECHO = 22;

long distance;

class NewHardware : public ArduinoHardware {
public : NewHardware ( ) : ArduinoHardware (&Serial1,
57600){};
}; ros::NodeHandle_<NewHardware> nh; 

int noInputCounter;
int sonarCounter;

void messageCb( const geometry_msgs::Twist& geometry_msg){
  
  noInputCounter = 0;
  
  geometry_msgs::Vector3 linear = geometry_msg.linear;
  geometry_msgs::Vector3 angular = geometry_msg.angular;
  
  if(distance <= 10) {
    analogWrite(PIN_1_FWD, 0);
    analogWrite(PIN_2_FWD, 0);
    analogWrite(PIN_1_REV, 0);
    analogWrite(PIN_2_REV, 0);
    return;
  }
  
  float linear_speed = linear.x;
  
  float angular_speed = angular.z;
 
  float lft_speed;
  float rgt_speed;
  if (distance <= 20) {
    linear_speed = linear_speed * (distance - 10) / 10;
    angular_speed = angular_speed * (distance - 10) / 10;
  }
  if (linear_speed >= 0) {
    lft_speed = min(max(linear_speed - angular_speed, 0), 255);
    rgt_speed = min(max(linear_speed + angular_speed, 0), 255);
    analogWrite(PIN_1_FWD, lft_speed);
    //writeLeftFwd(lft_speed);
    analogWrite(PIN_2_FWD, rgt_speed);
    //writeRightFwd(rgt_speed);
    analogWrite(PIN_1_REV, 0);
    //writeLeftBwd(0);
    analogWrite(PIN_2_REV, 0);
    //writeRightBwd(0);
  } else {
    lft_speed = min(max(-linear_speed + angular_speed, 0), 255);
    rgt_speed = min(max(-linear_speed - angular_speed, 0), 255);
    analogWrite(PIN_1_REV, lft_speed);
    //writeLeftBwd(lft_speed);
    analogWrite(PIN_2_REV, rgt_speed);
    //writeRightBwd(rgt_speed);
    analogWrite(PIN_1_FWD, 0);
    //writeLeftFwd(0);
    analogWrite(PIN_2_FWD, 0);
    //writeRightFwd(0);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void setup() {
  
  sonarCounter = 0;
  noInputCounter = 0;
  
  pinMode(PIN_LED, OUTPUT);
  
  pinMode(PIN_1_FWD, OUTPUT);
  pinMode(PIN_1_REV, OUTPUT);
  pinMode(PIN_1_ENA, OUTPUT);
  pinMode(PIN_2_FWD, OUTPUT);
  pinMode(PIN_2_REV, OUTPUT);
  pinMode(PIN_2_ENA, OUTPUT);
    
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.spinOnce();
    
  digitalWrite(PIN_1_ENA, HIGH);
  digitalWrite(PIN_2_ENA, HIGH);
  
  TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(WGM30);
  TCCR3B = _BV(CS32);
  //OCR3A = 180;

  // Set up 
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  
  TIMSK1 = (1 << TOIE1);
  // Cycle every ±4ms
  TCCR1B |= (1 << CS10);
  sei();
}

ISR(TIMER1_OVF_vect) {
  // After 1 s (250 * ±4ms)
  if (noInputCounter >= 250) {
    analogWrite(PIN_1_FWD, 0);
    analogWrite(PIN_2_FWD, 0);
    analogWrite(PIN_1_REV, 0);
    analogWrite(PIN_2_REV, 0);
  } else {
    noInputCounter++;
  }

  checkSonar();
}

void loop() {  
  nh.spinOnce();
}

void writeRightFwd(int x) {
  OCR3A = x;
}

void writeRightBwd(int x) {
  OCR3B = x;
}

void writeLeftFwd(int x) {
  OCR4A = x;
}

void writeLeftBwd(int x) {
  OCR4B = x;
}

void checkSonar() {
  // Complete Cycle consists of 200ms (50 * ±4ms).
  if (++sonarCounter == 1) {
    // First loop, make sure trigger is low.
    digitalWrite(PIN_US_TRIG, LOW);
  } else if (sonarCounter == 2) {
    // Second loop, write trigger high.
    digitalWrite(PIN_US_TRIG, HIGH);
  } else if (sonarCounter == 5) {
    // After roughly three cycles, write trigger low.
    digitalWrite(PIN_US_TRIG, LOW);
    // Calculate distance from the duration of the pulses.
    long duration = pulseIn(PIN_US_ECHO, HIGH);
    distance = (duration / 2) / 29.1;
  } else if (sonarCounter >= 50) {
    // Reset sonar loop.
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    sonarCounter = 0;
  }
}


