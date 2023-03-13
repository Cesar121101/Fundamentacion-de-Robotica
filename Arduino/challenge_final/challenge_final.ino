#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

//Pines del Puente H
int Enable = 9;
int Izquierda = 5;
int Derecha = 6;

// Pines de Encoders
int encoderA = 2;
int encoderB = 3;
float protectedCount = 0;
float previousCount = 0;


// Contador
volatile int count = 0;

//Variable para guardar el valor que recibimos del nodo input
int value;

// New CODE
#define ENC_COUNT_REV 455

const float count_to_rads = 0.0138091984773;
const float rpm_to_radians = 678.584;
const float rad_to_deg = 57.29578;

float roc = 0;
float rpm = 0;
float ang_velocity = 0;
float ang_velocity_deg = 0;

long previousMillis = 0;
long currentMillis = 0;

int interval = 10;

#define readA bitRead(PIND,2)//faster than digitalRead()
#define readB bitRead(PIND,3)//faster than digitalRead()

//Funcion callback para la subscripcion
void input_callback(const std_msgs::Float32 &msg){
  value = msg.data*255;
}

//Creamos el nodo 
ros::NodeHandle motor;
//Subscripcion para el topico cmd_pwm
ros::Subscriber<std_msgs::Float32> input_sub("motor_input", &input_callback);

//Variable para publicar el valor actual del puente H
std_msgs::Float32 pwm_signal;

//Crear el topico motor_pwm para publicar su valor
ros::Publisher motor_output("motor_output", &pwm_signal);

//Iniciamos el nodo, nos subscribimos al topico cmd_pwm e inicializamos los pines del puente H
void setup() {

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA), ACallback, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB), BCallback, RISING);

  motor.initNode();
  motor.advertise(motor_output);
  pinMode(Enable, OUTPUT);
  pinMode(Izquierda, OUTPUT);
  pinMode(Derecha, OUTPUT);
  motor.subscribe(input_sub);
  analogWrite(Enable, 255);
}

void loop() {
  
  noInterrupts();
  protectedCount = count;
  interrupts();

  currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    
    previousMillis = currentMillis;
    //pwm_signal.data = protectedCount;

    roc = (float)((protectedCount-previousCount)/(interval));
    //rpm = (float)(roc*60/ ENC_COUNT_REV);
    //ang_velocity = rpm * rpm_to_radians;
    //ang_velocity_deg = ang_velocity * rad_to_deg;

    pwm_signal.data = roc * 2 * 3.1416;

    previousCount = protectedCount;
    protectedCount = 0;
  }
  
  //Asignamos el valor que recibimos en callback hacia la variable que publicamos de regreso
  //pwm_signal.data = value;

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
  motor_output.publish(&pwm_signal);
  motor.spinOnce();
  //delay(5);
}

void ACallback(){
  if(readB != readA) {
    count --;
  } else {
    count ++;
  }
}

void BCallback(){
  if (readA == readB) {
    count --;
  } else {
    count ++;
  }
}
