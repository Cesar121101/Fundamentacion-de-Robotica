#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

//Pines del Puente H
int Enable = 3;
int Izquierda = 5;
int Derecha = 6;

//Variable para guardar el valor que recibimos del nodo input
int value;

//Funcion callback para la subscripcion
void input_callback(const std_msgs::Float32 &msg){
  value = msg.data;
}

//Creamos el nodo 
ros::NodeHandle motor;
//Subscripcion para el topico cmd_pwm
ros::Subscriber<std_msgs::Float32> input_sub("motor_input", &input_callback);

//Variable para publicar el valor actual del puente H
std_msgs::Float32 pwm_signal;

//Crear el topico motor_pwm para publicar su valor
ros::Publisher motor_pwm("motor_pwm", &pwm_signal);

//Iniciamos el nodo, nos subscribimos al topico cmd_pwm e inicializamos los pines del puente H
void setup() {
  motor.initNode();
  motor.advertise(motor_pwm);
  pinMode(Enable, OUTPUT);
  pinMode(Izquierda, OUTPUT);
  pinMode(Derecha, OUTPUT);
  motor.subscribe(input_sub);
  analogWrite(Enable, 255);
}

void loop() {
  //Asignamos el valor que recibimos en callback hacia la variable que publicamos de regreso
  pwm_signal.data = value;

  //Si el valor que recibimos de la señal es positivo, giramos en hacia la derecha con ese valor
  if (value > 0){
    analogWrite(Derecha, value);
    analogWrite(Izquierda, 0);
  } 
  //Si el valor que recibimos de la señal es negativo, giramos hacia la izquierda con el valor absoluto del valor
  else {
    analogWrite(Izquierda, abs(value));
    analogWrite(Derecha, 0);
  }

  //Publicamos el valor del puente HS
  motor_pwm.publish(&pwm_signal);
  motor.spinOnce();
  delay(1000);
}
