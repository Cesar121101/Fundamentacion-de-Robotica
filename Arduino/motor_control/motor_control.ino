#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

int Enable = 3;
int Izquierda = 5;
int Derecha = 6;
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
  pinMode(Enable, OUTPUT);
  pinMode(Izquierda, OUTPUT);
  pinMode(Derecha, OUTPUT);
  motor.subscribe(pwm_sub);
  analogWrite(Enable, 255);
}

void loop() {
  pwm_signal.data = value;
  if (value > 0){
    analogWrite(Derecha, value);
    analogWrite(Izquierda, 0);
  } else {
    analogWrite(Izquierda, abs(value));
    analogWrite(Derecha, 0);
  }
  motor_pwm.publish(&pwm_signal);
  motor.spinOnce();
  delay(1000);
}
