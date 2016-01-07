
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

const int PIN_1_FWD = 6;
const int PIN_1_REV = 7;
const int PIN_1_ENA = 24;
const int PIN_2_FWD = 2;
const int PIN_2_REV = 3;
const int PIN_2_ENA = 25;
const int PIN_LED = 13;

ros::NodeHandle  nh;

void messageCb( const geometry_msgs::Twist& geometry_msg){
  
  geometry_msgs::Vector3 linear = geometry_msg.linear;
  geometry_msgs::Vector3 angular = geometry_msg.angular;
  
  float fwd_speed = linear.x;
  float lft_speed;
  float rgt_speed;
  
  float th = angular.z;
 
  
  /*if (fwd_speed == 0 && th == 0) {
    digitalWrite(PIN_1_ENA, LOW);
    digitalWrite(PIN_2_ENA, LOW);
    return;
  } else {
    // TODO not every loop
    digitalWrite(PIN_1_ENA, HIGH);
    digitalWrite(PIN_2_ENA, HIGH);
  }*/
  
  rgt_speed = fwd_speed - th;
  lft_speed = fwd_speed + th;
  
  if (fwd_speed < 0) {
    analogWrite(PIN_1_REV, lft_speed * -100);
    analogWrite(PIN_1_FWD, 0);
    analogWrite(PIN_2_REV, rgt_speed * -100);
    analogWrite(PIN_2_FWD, 0);
  } else {
    analogWrite(PIN_1_FWD, lft_speed * 100);
    analogWrite(PIN_1_REV, 0);
    analogWrite(PIN_2_FWD, rgt_speed * 100);
    analogWrite(PIN_2_REV, 0);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

/*ISR(TIMER1_COMPA_vect) {
  
}*/

/*class NewHardware: public ArduinoHardware {
  public: NewHardware():ArduinoHardware(&Serial1 , 57600){};
  );
  
ros::NodeHandle_<NewHardware> nh1;*/

void setup() {
  //Serial.begin(9600);
  //Keyboard.begin(); // enable when connected

  pinMode(PIN_1_FWD, OUTPUT);
  pinMode(PIN_1_REV, OUTPUT);
  pinMode(PIN_1_ENA, OUTPUT);
  pinMode(PIN_2_FWD, OUTPUT);
  pinMode(PIN_2_REV, OUTPUT);
  pinMode(PIN_2_ENA, OUTPUT);
  
  pinMode(PIN_LED, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);
  
  digitalWrite(PIN_LED, HIGH);
  
  digitalWrite(PIN_1_ENA, HIGH);
  digitalWrite(PIN_2_ENA, HIGH);

// initialize timer1 
  /*noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 31250;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();*/             // enable all interrupts

}

void loop() {
  nh.spinOnce();
  delay(1);
}

void controlMotors(char inChar) {
  switch(inChar) {
    
    case 'w': // front
      //
      analogWrite(PIN_1_FWD, 127);
      analogWrite(PIN_2_FWD, 127);
      analogWrite(PIN_1_REV, 0);
      analogWrite(PIN_2_REV, 0);
      break;
    case 'a': // left
      //
      
      analogWrite(PIN_1_FWD, 0);
      analogWrite(PIN_2_FWD, 127);
      analogWrite(PIN_1_REV, 0);
      analogWrite(PIN_2_REV, 0);
      break;
    case 's': // reverse
      //
      
      analogWrite(PIN_1_FWD, 0);
      analogWrite(PIN_2_FWD, 0);
      analogWrite(PIN_1_REV, 127);
      analogWrite(PIN_2_REV, 127);
      break;
    case 'd': // right
      //
      analogWrite(PIN_1_FWD, 127);
      analogWrite(PIN_2_FWD, 0);
      analogWrite(PIN_1_REV, 0);
      analogWrite(PIN_2_REV, 0);
      break;
    
  }
}

