/* 2bldc motor both interlinked. driver moves, driven moves with same speed same direction
 *  in bldc speed and torque are inversely proportional. P=TW
 *  pwm varies speed of bldc using esc. 0-100 duty cycle - 0 to max speed
 *  hall effect sensor is used to derive angular speed of bldc motor
*/

//position + angular PID
#include <Servo.h>
Servo ESC; 
#define D A5     //  input
#define d A0     //should feel the force //will always follow D //output
#define pwm_pin 9

//PID constants
double kp = 1;
double ki = 0;
double kd = 0;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
int output, setPoint, last_theta_d, theta_d, last_theta_D, theta_D;
double cumError, rateError, rateErrorR;


/* motors specifications */
int T1, T2, read_D, read_d, w_D, w_d, w_exp, target;
int P1 = 150;   //Watt
int P2 = 150; 
int max_wD = 12000;   //rpm
int max_wd = 12000;


void setup()
{  
  pinMode(D, INPUT_PULLUP);
  pinMode(d, INPUT_PULLUP);
  pinMode(pwm_pin, OUTPUT);
  ESC.attach(pwm_pin,1000,2000);
  Serial.begin(19200);
}

void loop(){
//main loop
  Serial.print("start ");  
  read_D = analogRead(D);
  Serial.print(read_D);
//  currentTime = millis();                         //get current time
//  elapsedTime = currentTime - previousTime;       //compute time elapsed from previous computation
//  int read_d = map(read_d,0,1023,0,180);
//  w_D = (theta_D - last_theta_D)/elapsedTime;
//  T1 =  P1 / w_D;                  //torque applied by driver
//  T1 = T2;                         //same torque
//  w_exp = P2 / T2;                 //w_d required to reach the same position
//  int setPoint = read_D;                //target set
  read_d = analogRead(d);
  Serial.print(" ");
  Serial.print(read_d);
//  currentTime = millis();                         //get current time
//  elapsedTime = currentTime - previousTime;       //compute time elapsed from previous computation
//  int theta_d = map(read_d,0,1023,0,180);
//  w_d = (theta_d - last_theta_d)/elapsedTime;

  currentTime = millis();                         //get current time
  elapsedTime = currentTime - previousTime;       //compute time elapsed from previous computation

  error = read_D - read_d;                      // determine error
  cumError += error * elapsedTime;                // compute integral
  rateError = (error - lastError)/elapsedTime;    // compute derivative   

  int output = kp*error + ki*cumError + kd*rateError; //PID output               

  lastError = error;                              //remember current error
  previousTime = currentTime;                     //remember current time       
//  output = computePID();
   
//  last_theta_D = theta_D;                              //remember current error
//  previousTime = currentTime;  

  int pwm = output/4;
  Serial.print(" ");
  Serial.println(pwm);
  ESC.write(pwm);
  angular();
}
void angular(){
  Serial.print("start ");  
  theta_D = analogRead(D);
  Serial.print(" theta_D ");
  Serial.print(theta_D);
  currentTime = millis();                         //get current time
  elapsedTime = currentTime - previousTime;       //compute time elapsed from previous computation
//  int theta_D = map(read_D,0,1023,0,360);
  w_D = (theta_D - last_theta_D)/elapsedTime;
//  T1 =  P1 / w_D;                  //torque applied by driver
//  T1 = T2;                         //same torque
//  w_exp = P2 / T2;                 //w_d required to reach the same position
  int setPoint = w_D;                //target set
  Serial.print(" setPoint ");
  Serial.print(setPoint);
  theta_d = analogRead(d);
  Serial.print(" theta_d ");
  Serial.print(theta_d);
//  int theta_d = map(read_d,0,1023,0,360);
  w_d = (theta_d - last_theta_d)/elapsedTime;
  int current_posn = w_d;
  Serial.print(" w_d ");
  Serial.print(w_d);
  error = setPoint - current_posn;                      // determine error
  cumError += error * elapsedTime;                // compute integral
  rateError = (error - lastError)/elapsedTime;    // compute derivative   

  int output = kp*error + ki*cumError + kd*rateError; //PID output               
   
  int pwm = output;
  Serial.print(" ");
  Serial.println(pwm);
  ESC.write(pwm);
  
  last_theta_D = theta_D;                              //remember current error
  last_theta_d = theta_d;
  lastError = error;                              //remember current error
  previousTime = currentTime;                     //remember current time   
}
