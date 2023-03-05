//angular velocity PID
#include <Servo.h>
Servo ESC1;

#define m A5
#define D A1     //  input
#define d A0     //should feel the force //will always follow D //output
#define pwm_pin 9

//PID constants
double kp = 1;
double ki = 0;
double kd = 0;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error ;
double lastError;
int output, setPoint, last_theta_d, theta_d, last_theta_D, theta_D;
double cumError, rateError, rateErrorR;


/* motors specifications */
int T1, T2, read_D, read_d, w_D, w_d, w_exp, target;
int P1 = 150;   //Watt
int P2 = 150; 
int max_wD = 12000;   //rpm
int max_wd = 12000;
int val = 900;
int countD = 0;
int countd = 0;
void setup()
{  
  pinMode(m, INPUT);
  pinMode(D, INPUT_PULLUP);
  pinMode(pwm_pin, OUTPUT);
  ESC1.attach(8,1000,2000);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(D),COUNT,CHANGE);
}


void loop(){
//main loop
  Serial.print("start ");  

  
  // -----------MASTER MOTOR---------//
  theta_D = analogRead(D);
  theta_D = theta_D;
  Serial.print(" theta_D ");
  Serial.print(theta_D);
  int  potValue = analogRead(A5);
  ESC1.write(potValue);
  currentTime = millis();                         //get current time
  elapsedTime = currentTime - previousTime;       //compute time elapsed from previous computation
  
  w_D = (theta_D - last_theta_D)/elapsedTime;

  Serial.print(" ");
  Serial.println(countD);
  last_theta_D = theta_D;                         //remember current error
  previousTime = currentTime;                     //remember current time   
}
void COUNT() {
   //ISR function
    countD = countD + 1;
}
