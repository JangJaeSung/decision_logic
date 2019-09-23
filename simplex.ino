#include <stdio.h>
#include <Servo.h>
#include <math.h>
#include <RPlidar.h>

#include <ros.h>

#include <geometry_msgs/Twist.h>


//#define DEBUG
#define pin_throttle 10 // set pin number 8 of arduino mega as rc_throttle pin 
#define pin_steer 9 // set pin number 7 of arduino mega as rc_steer pin
#define RPLIDAR_MOTOR 3
#define LED 13
#define ZERO_STEER 1500
#define ZERO_SPEED 1500

Servo steer;
Servo throttle;
RPLidar lidar;

int steer_val = ZERO_STEER;
int throttle_val = ZERO_SPEED;
int lane_steer_val_old, lane_steer_val_new = ZERO_STEER;
int lane_throttle_val = ZERO_SPEED;

float tx_steer;
float tx_throttle;

bool mode = true;

float Distance_[360] = {0,};



void rosTwistCallback(const geometry_msgs::Twist& twist_msg){
  tx_throttle = twist_msg.linear.x;
  tx_steer = twist_msg.angular.z;


  throttle.writeMicroseconds(throttle_val);
  steer.writeMicroseconds(steer_val);

/*
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
*/
  /*
  Serial.print(twist_msg.angular.z);
  Serial.print(",");
  Serial.println(twist_msg.linear.x);
  */
}

ros::NodeHandle nh;


ros::Subscriber<geometry_msgs::Twist> sub_twist("twist_msg", &rosTwistCallback);





void setup() {
  nh.initNode();
  nh.subscribe(sub_twist);

  //Serial.begin(57600);
  lidar.begin(Serial);

  pinMode(RPLIDAR_MOTOR, OUTPUT);
  
  steer.attach(pin_steer);
  throttle.attach(pin_throttle);

  steer.writeMicroseconds(1500);
  throttle.writeMicroseconds(1500);
  
}

void loop() {
  //High Performance Mode
  if (mode == true){
    nh.spinOnce();
    delay(10);

  }

  // High Safety Mode
  else if (mode == false){
    if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    
    //perform data processing here...   
    } 
    else {
      analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
      // try to detect RPLIDAR... 
      rplidar_response_device_info_t info;
      if (IS_OK(lidar.getDeviceInfo(info, 100))) {
        // detected...
        lidar.startScan();
       
        // start motor rotating at max allowed speed
        analogWrite(RPLIDAR_MOTOR, 255);
        delay(1000);
      }
    }

    if (angle > 40 && angle <= 50){
      Distance_[angle] = distance;
    }
   
 
  }
}
