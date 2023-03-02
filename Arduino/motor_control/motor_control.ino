#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

int ledPin = 3;
int value;

void pwm_callback(const std_msgs::Float32 &msg){
  value = msg.data;
}

ros::NodeHandle motor;
ros::Subscriber<std_msgs::Float32> pwm_sub("cmd_pwm", &pwm_callback);

std_msgs::Float32 pwm_signal;
ros::Publisher motor_pwm("motor_pwm", &pwm_signal);

void setup() {
  
  motor.initNode();
  motor.advertise(motor_pwm);
  pinMode(ledPin, OUTPUT);
  motor.subscribe(pwm_sub);
}

void loop() {
  //pwm_signal = pwm_sub.msg;
  pwm_signal.data = value;
  analogWrite(ledPin, pwm_signal.data);
  motor_pwm.publish(&pwm_signal);
  motor.spinOnce();
  delay(1000);
}
