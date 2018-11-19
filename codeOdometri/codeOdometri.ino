#include <math.h>

#define en1 6
#define in1 7
#define in2 8
#define en2 11
#define in3 9
#define in4 10

const byte interruptPin3 = 3;
const byte interruptPin2 = 2;
int rotaryRight, rotaryLeft;

float kRoda,pulseEveryMmLeft,pulseEveryMmRight; 
float rRoda = 3.25;
float resolusiEncLeft = 15;
float resolusiEncRight = 15;
float wheelBase = 7.5;
float currentDistance, sudutOrientasi,heading, xPos, yPos;
float errorX, errorY, targetDistance, beta, alpha, lamda;
float xDistance = 100;
float yDistance = 100;
float error, last_error, MV,rpm,D,D1,D2,D3,I,I1,I2,I3,P,Pd, pid_l, pid_r;
float Max_MV = 500;
float max_pwm_ref = 180;
float Kp=50.0;
float Kd=10.0;
float Ki=0.1;
float Ts=0.1;


void setup() 
{
  Serial.begin(9600);
  for(int i=7; i<=10; i++)
  {
    pinMode(i, OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(interruptPin3), countRight, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), countLeft, RISING);
}

void loop() 
{
  kRoda = 2*3.14*3.25;
  pulseEveryMmLeft = resolusiEncLeft / kRoda;
  pulseEveryMmRight = resolusiEncRight / kRoda;

  float leftRotaryMm = float(rotaryLeft)*pulseEveryMmLeft;
  float rightRotaryMm = float(rotaryRight)*pulseEveryMmLeft;
  
  currentDistance = (leftRotaryMm + rightRotaryMm )/ 2;
  sudutOrientasi = (leftRotaryMm - rightRotaryMm ) / wheelBase;
  heading = sudutOrientasi * (180/3.14);

  xPos = currentDistance * sin(heading);
  yPos = currentDistance * cos(heading);

  errorX = xDistance - xPos;
  errorY = yDistance - yPos;

  targetDistance = sqrt((pow(xDistance,2)+0.5) + (pow(yDistance,2)+0.5));

  beta = atan((yDistance-yPos)/(xDistance-xPos));

  lamda = tan(yPos/xPos);
  alpha = beta - lamda;

  error = alpha;

  if(xPos >= xDistance && yPos >= yDistance)
  {
    while(1)
    {
      backward(0,0);
    }
  }
  
  P = Kp * error; //output nilai Proportional 
        
  D1 = Kd*8;      // konstanta derivative dijadikan 8 bit, dikarenakan mikrokontroller hanya 8 bit                        
  D2 = D1 / Ts; //KD/Time sampling
  D3 = error - last_error;  //pengurangan nilai error sekarang dengan nilai error sebelumhya
  D = D2 * D3;  //output nilai Derivative

  I1 = Ki/8;  //Ki di ubah ke 8 bit
  I2 = error + last_error;  //summing dari nilai error sekarang dan nilai error sebelumnya
  I3 = I1 * I2; 
  I = I3 * Ts; // output nilai Integral

  last_error = error; //error sekarang dijadikan error sebelumnya, kemudian akan di loop ke atas lagi

  Pd = P + D; 
  MV = Pd + I;  //output PID

  //=======================================//  
  if(MV>=-Max_MV && MV<=Max_MV)         //jika output PID tidak melebihi nilai pembatasan output PID maka robot akan berjalan
  {                                     //maju dengan pengaturan PWM kanan dan kiri
      pid_l  = max_pwm_ref + MV; 
      pid_r  = max_pwm_ref - MV; 
                          
      if (pid_l < 0) pid_l = 0;
      if (pid_l > 150) pid_l = 150;
      if (pid_r < 0) pid_r = 0;
      if (pid_r > 150) pid_r = 150;
      forward(pid_r,pid_l);  
  }  
  else if(MV<-Max_MV)                   //jika output PID lebih kecil dari nilai batasan output PID maka robot akan putar KANAN
  { 
     forward(100,10);
  }   
  else if(MV>Max_MV)                    //jika output PID lebih besar dari nilai batasan output PID maka robot akan putar KIRI
  {
     forward(10,100); 
  }
  else
  {
     forward(75,75);
  } 
  
  Serial.print(rotaryLeft); Serial.print("\t");
  Serial.print(rotaryRight); Serial.print("\t");
  Serial.print(currentDistance); Serial.print("\t");
  Serial.print(sudutOrientasi); Serial.print("\t");
  Serial.print(heading); Serial.print("\t");
  Serial.print(xPos); Serial.print("\t");
  Serial.print(yPos); Serial.print("\t");
  Serial.print(errorX); Serial.print("\t");
  Serial.print(errorY); Serial.print("\t");
  Serial.print(targetDistance); Serial.print("\t");
  Serial.print(beta); Serial.print("\t");
  Serial.print(lamda); Serial.print("\t");
  Serial.print(alpha); Serial.print("\t");
  Serial.print(MV); Serial.print("\t");
  Serial.print(pid_l); Serial.print("\t");
  Serial.println(pid_r);
  
}
void countLeft() 
{
  attachInterrupt(digitalPinToInterrupt(interruptPin2), countLeftFalling, FALLING);
}
void countLeftFalling() 
{
  rotaryLeft++;
  attachInterrupt(digitalPinToInterrupt(interruptPin2), countLeft, RISING);
}
void countRight() 
{
  attachInterrupt(digitalPinToInterrupt(interruptPin3), countRightFalling, FALLING);
}
void countRightFalling() 
{
  rotaryRight++;
  attachInterrupt(digitalPinToInterrupt(interruptPin3), countRight, RISING);
}

void forward(int pwm1, int pwm2)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(en1, pwm1);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en2, pwm2);
}
void backward(int pwm1, int pwm2)
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(en1, pwm1);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(en2, pwm2);
}

