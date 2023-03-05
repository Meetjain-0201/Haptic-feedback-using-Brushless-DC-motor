/* 2bldc motor both interlinked. driver moves, driven moves with same speed same direction
 *  in bldc speed and torque are inversely proportional. P=TW
 *  pwm varies speed of bldc using esc. 0-100 duty cycle - 0 to max speed
 *  hall effect sensor is used to derive angular speed of bldc motor
*/

//angular velocity PID
#include <Servo.h>
Servo ESC1;
Servo ESC2; 

#define m PA5
#define D PA1     //  input
#define d PA0     //should feel the force //will always follow D //output

double masterspeed();
double slavespeed();
double PID();

//PID constants
double kp = 1;
double ki = 0;
double kd = 0;
 
double error;
double lastError;
double setPoint, setpoint_pwm, current_pwm, current_omega, theta_d, theta_D;
double cumError, rateError, rateErrorR;

unsigned long currentTime, previousTime;
double elapsedTime;
int VAL = 700;
int last_theta_D = 0;
int countD = 0;
int countd = 0;
int last_theta_d = 0;

void setup()
{  
  pinMode(m, INPUT);
  pinMode(D, INPUT_PULLUP);
  pinMode(d, INPUT_PULLUP);
  ESC1.attach(PB7,1000,2000);
  ESC2.attach(PB6,1000,2000);
  Serial.begin(115200);
}


void loop(){
// --------------main loop-----------------//
  
  currentTime = micros();
  Serial.print(currentTime);
  Serial.print(" ");
  Serial.print(previousTime);
  Serial.print(" ");
  elapsedTime = micros() - previousTime;
  elapsedTime = elapsedTime;
  Serial.print(elapsedTime);
  
  Serial.print(" start ");  
  int potValue = analogRead(A5);
  potValue = map(potValue, 0, 1023, 0, 255);
  ESC1.write(potValue);
  
  theta_D = analogRead(D);
  int setpoint_pwm = masterspeed();
//  Serial.print(" setPointpwm ");
//  Serial.print(setpoint_pwm);
  
  theta_d = analogRead(d);
  int current_pwm = slavespeed();
//  Serial.print(" current_pwm ");
//  Serial.println(current_pwm);
  
  int pwm = PID();
  ESC2.write(pwm);
  
  previousTime = currentTime;    
}

double masterspeed(){
  // -----------MASTER MOTOR---------//
  
  theta_D = theta_D + 1024*countD; 
  int dtheta1 = theta_D - last_theta_D;
  int setpoint = dtheta1/elapsedTime; 
  
  if (dtheta1 < -VAL){
    countD = countD + 1;
  }
  else if (dtheta1 > VAL)
  {
    countD = countD - 1;
  }
  Serial.print(" setpoint ");
  Serial.print(setpoint);  
  setpoint_pwm = map(setpoint, 0, 170667, 0, 255);
  Serial.print(" setpoint_pwm ");
  Serial.print(setpoint_pwm);
  last_theta_D = theta_D;
  return setpoint_pwm;
}

double slavespeed(){
  // -----------SLAVE MOTOR---------//
  
  theta_d = theta_d + 1024*countd;
  int dtheta2 = theta_d - last_theta_d; 
  int omega = dtheta2/elapsedTime; 
  
  if (dtheta2 < -VAL){
    countd = countd + 1;
  }
  else if (dtheta2 > VAL)
  {
    countd = countd - 1;
  }
  Serial.print(" omega ");
  Serial.print(omega);
  current_pwm = map(omega, 0, 170667, 0, 255);
  Serial.print(" current_pwm ");
  Serial.println(current_pwm);
  last_theta_d = theta_d;
  return current_pwm;
}

double PID(){  
  // -----------PID OF VELOCITY---------//
  error = setpoint_pwm - current_pwm;               // determine error
//  Serial.print(" error ");
//  Serial.println(error);   
  cumError += error * elapsedTime;                // compute integral
  rateError = (error - lastError)/elapsedTime;    // compute derivative   

  int pwm = kp*error + ki*cumError + kd*rateError;//PID output  
  lastError = error;                              //remember current error
  return pwm;
}
