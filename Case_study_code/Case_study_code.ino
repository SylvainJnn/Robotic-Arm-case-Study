#include <Servo.h>

#define   M_PI   3.14159265358979323846 /* pi */


// ============= Program variables =============
Servo servo_alpha, servo_beta, servo_gama, servo_q4; // create servo object to control a servo
const int potpin_alpha = A0;  // analog pin used to connect the potentiometer
const int potpin_beta  = A1;  // analog pin used to connect the potentiometer
const int potpin_gama  = A2;  // analog pin used to connect the potentiometer
                   
const int servo_pin_alpha = 9;
const int servo_pin_beta  = 3;
const int servo_pin_gama  = 4;
const int servo_pin_q4  = 5;

const int LED = 11;
const int Button_Calibration = 12; // ASK for Switch --> easier to implemant
const int Button_Action = 13;


float d = 0.3;
float l1 = 0.1;
float l2 = 0.1;
float l3 = 0.15;


int alpha_0, beta_0, gama_0, q4_0; // Home angle positions

bool Read_button(int pin)           //return High if a button is pressed for two secods
{
  if(digitalRead(pin) == HIGH)      //check if the button is pressed
    {
      delay(2500);                  //wait 2,5 seconds
      if(digitalRead(pin) == HIGH)  //check if it is still the case
      {
        return(HIGH);
      }
    }
    return(LOW);
}

void Calibration()
{
  int new_alpha_0, new_beta_0, new_gama_0;
  //Read Values
  new_alpha_0 = analogRead(potpin_alpha);             // reads the value of the potentiometer (value between 0 and 1023)
  new_alpha_0 = map(new_alpha_0, 0, 1023, 0, 180);    // scale it to use it with the servo (value between 0 and 180)

  new_beta_0  = analogRead(potpin_beta);              // reads the value of the potentiometer (value between 0 and 1023)
  new_beta_0  = map(new_beta_0, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)

  new_gama_0  = analogRead(potpin_gama);              // reads the value of the potentiometer (value between 0 and 1023)
  new_gama_0  = map(new_gama_0, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
 
  //Write Values
  servo_alpha.write(new_alpha_0);                  // sets the servo position according to the scaled value
  alpha_0 = servo_alpha.read();

  servo_beta.write(new_beta_0);                  // sets the servo position according to the scaled value
  beta_0 = servo_beta.read();
  
  servo_gama.write(new_gama_0);                  // sets the servo position according to the scaled value
  gama_0 = servo_gama.read();
  
  delay(15);
  
}


// ===================================== action & poses =====================================

void pose_home()// home position
{
  servo_alpha.write(alpha_0);
  servo_beta.write(beta_0);
  servo_gama.write(gama_0);
}

void pose_1()
{
  servo_alpha.write(alpha_0 - 45);
  servo_beta.write(beta_0);
  servo_gama.write(gama_0);
}


void pose_2()
{
  servo_alpha.write(alpha_0 - 45);
  servo_beta.write(beta_0 - 30);
  servo_gama.write(gama_0 - 20);
}

// Other side 

void pose_11() // opposite
{
  servo_alpha.write(alpha_0 + 45);
  servo_beta.write(beta_0);
  servo_gama.write(gama_0);
}
// =============================================

float compute_phi(float x)
{
  if(x>0)
    return(0);
  return(-M_PI);
}

float compute_r3(float r,float  l3,float phi)
{//check
    return(r-l3*cos(phi));
}    
float compute_z3(float z,float  l3,float phi)
{//check
    return(z-l3*sin(phi));
}    

float compute_q3(float r,float z,float l1,float l2)
{
  float numerator1, numerator2, denominator;

  numerator1 =(float)(r*r + z*z);
  numerator2 = l1*l1 + l2*l2;
  denominator = 2 * l1 * l2;
  return(-acos((numerator1 - numerator2) / (denominator)));
}


float compute_q2(float r,float z,float l1,float l2, float q3)
{
  float a,b_numerator,b_denominator,b;
  a = atan2(z,r);
  b_numerator = l2*sin(q3);//change it to q3
  b_denominator = l1 + l2*cos(q3);
  b = atan2(b_numerator, b_denominator);
  //print(a,b);
  return(a + b -M_PI/2); // to have origin on s axis we have to change it from 90 degrees, i don't know why // maybesomething wrong here too
}


void action_equation(float q1, float q2, float q3, float q4)// write
{
  

  servo_alpha.write(q1 + alpha_0);
  delay(1000);
  servo_beta.write(q2 + beta_0);
  delay(1000);
  servo_gama.write(q3 + gama_0);
  delay(1000);
  servo_q4.write(q4 + q4_0);
  delay(1000);
  
}


void pose(float *direct_model)//takes an array 
{
  /*
  Write the inverse kinematic equation : 
  alpha = ... direct_model[0] ou x = direct_model[0]...
  beta  = ...
  
  alpha = atan2(direct_model[1], direct_model[0]); // atan2(y,x) == arctan(y/x)
  beta  = asin((direct_model[2] + d)/(l1 + l2));
  gama  = log(-direct_model[3]/sin(alpha))*M_PI;// log(x) = ln(x); //might be worng, but there is another solution
  
  //old one above
  */
  float q1,q2,q3,q4;

  float x = direct_model[0];
  float y = direct_model[1];
  float z = direct_model[2] - d;//take off the d from DH parameter
  float phi = compute_phi(x); //in our case phi is always 0

  float r =(float) sqrt(x*x + y*y); //I don't know how to decreibe it yes baisicly, in this on this is line that the planer is based

  q1 = atan2(y,x); // atan2(z,x) == arctan(z/x);
  float r3, z3;
  r3 = compute_r3(r, l3, phi);//this is position of servo4
  z3 = compute_z3(z, l3, phi);
  
  q3  = compute_q3(r3, z3, l1, l2);
  q2  = compute_q2(r3, z3, l1, l2, q3);

  q2 = q2 + M_PI/2; //add offset
  q4 = phi - (q2+q3);

  //if (q4 > 180)//for the limit but i don't think we need it now
  //  q4 = 360 - q4;

  
  if(q2 <0)
    q2 = -q2; // I don't know how to handle this properly yet but at least if the value is negative, we change it to positive

  //pass q to angles
  q1 = q1 * 180/M_PI;
  q2 = q2 * 180/M_PI;
  q2 = q2 * 180/M_PI;
  q2 = q2 * 180/M_PI;
  
  action_equation(q1, q2, q3, q4);//I feel like something is wrong with q2 but for q3 it is the way i foud to have a positive value 

}

// Gripper
void pick()
{
  //nothing for the moment, basicly we want to set the other servos to grip somthing
}

void drop()
{
  //nothing for the moment, set the servos to drop
}


void Action()
{
  pose_home();
  delay(1000);

  pose_1();
  delay(1000);

  pose_2();
  delay(1000);
  pick();

  pose_1();
  delay(1000);

  pose_home();
  delay(1000);

  pose_11();
  delay(1000);
  drop();

}

void Action_test() //for the moment, let's just check if it is working properly
{
  servo_alpha.write(alpha_0 - 35);
  delay(500);

  servo_alpha.write(alpha_0 - 35);
  delay(500);

  servo_alpha.write(alpha_0 + 55);
  servo_beta.write(beta_0 + 20);
  delay(500);
  
  servo_alpha.write(alpha_0 - 55);
  servo_beta.write(beta_0 - 20);
  servo_gama.write(gama_0 + 45);
  delay(500);

  servo_gama.write(gama_0 - 45);
  delay(500);
}



void setup() 
{
  Serial.begin(115200);
  // Set Servos
  servo_alpha.attach(servo_pin_alpha);  
  servo_beta.attach(servo_pin_beta);  
  servo_gama.attach(servo_pin_gama); 
  servo_q4.attach(servo_pin_q4); 
  
  pinMode(servo_pin_alpha, OUTPUT);
  pinMode(servo_pin_beta, OUTPUT);
  pinMode(servo_pin_gama, OUTPUT);
  pinMode(servo_pin_q4, OUTPUT);
  
  pinMode(Button_Calibration, INPUT);
  pinMode(Button_Action, INPUT);
  
  pinMode(LED, OUTPUT);    
}


void loop() 
{

  float directmodel[3];
//save home position (init pose)
  alpha_0 = servo_alpha.read();
  beta_0 = servo_beta.read();
  gama_0 = servo_gama.read();
  q4_0 = servo_q4.read();
  
  delay(500); 
  while(1)
  {   

    
    directmodel[0] = 0 ;
    directmodel[1] = 0.2 ;
    directmodel[2] = 0.1 ;
    digitalWrite(LED, LOW); 
    pose(directmodel);
    
    Serial.println("POSE 1");
    

    
    delay(5000);
    directmodel[0] = 0.1 ;
    directmodel[1] = 0 ;
    directmodel[2] = 0.15 ;
    
    digitalWrite(LED, HIGH); 
    delay(5000);
  }
  
/*
  if(digitalRead(Button_Calibration) == HIGH)
  {
    Calibration();  
    digitalWrite(LED, HIGH);              //turn on LED to show that calibration is running
      
  }
  else
  {    
    digitalWrite(LED, LOW);              //turn on LED to show that calibration is running
    Action();
  }
*/
  delay(1000);                           // waits for the servo to get there
}
