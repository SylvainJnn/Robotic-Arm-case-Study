#include <Servo.h>

#define   M_PI   3.14159265358979323846 /* pi */


// ============= Program variables =============
Servo servo_q1, servo_q2, servo_q3, servo_q4, servo_gripper; // create servo object to control a servo
const int potpin_q1 = A0;  // analog pin used to connect the potentiometer
const int potpin_q2  = A1;  // analog pin used to connect the potentiometer
const int potpin_q3  = A2;  // analog pin used to connect the potentiometer
                   
const int servo_pin_q1  = 3;
const int servo_pin_q2  = 5;
const int servo_pin_q3  = 6;
const int servo_pin_q4  = 9;
const int servo_pin_gripper = 10;

const int LED = 11;
const int Button_Calibration = 12; // ASK for Switch --> easier to implemant
const int Button_Action = 13;

// Arms lengths
float d = 0.03;
float l1 = 0.1;
float l2 = 0.1;
float l3 = 0.15;


int q1_0, q2_0, q3_0, q4_0; // Home angle positions

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
  int new_q1_0, new_q2_0, new_q3_0;
  //Read Values
  new_q1_0 = analogRead(potpin_q1);             // reads the value of the potentiometer (value between 0 and 1023)
  new_q1_0 = map(new_q1_0, 0, 1023, 0, 180);    // scale it to use it with the servo (value between 0 and 180)

  new_q2_0  = analogRead(potpin_q2);              // reads the value of the potentiometer (value between 0 and 1023)
  new_q2_0  = map(new_q2_0, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)

  new_q3_0  = analogRead(potpin_q3);              // reads the value of the potentiometer (value between 0 and 1023)
  new_q3_0  = map(new_q3_0, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
 
  //Write Values
  servo_q1.write(new_q1_0);                  // sets the servo position according to the scaled value
  q1_0 = servo_q1.read();

  servo_q2.write(new_q2_0);                  // sets the servo position according to the scaled value
  q2_0 = servo_q2.read();
  
  servo_q3.write(new_q3_0);                  // sets the servo position according to the scaled value
  q3_0 = servo_q3.read();
  
  delay(15);
  
}


// ===================================== action & poses =====================================


float compute_phi(float x)//comoute phi angle for the end effector
{
  if(x>=0)
    return(0);
  return(-M_PI);
}

//calculate r3 and z3 positions, it correspond to the (r,z) at the end of l2 arm (=at the 4th servos)
float compute_r3(float r, float l3, float phi)
{
    return(r-l3*cosf(phi));
}    
float compute_z3(float z, float l3, float phi)
{//check
    return(z-l3*sinf(phi));
}    

//compute q3 based on plannar inverse model
float compute_q3(float r,float z,float l1,float l2)
{
  float numerator1, numerator2, denominator;
  
  numerator1 =(float)(r*r + z*z);
  numerator2 = l1*l1 + l2*l2;
  denominator = 2 * l1 * l2;
  
  return(-acosf((numerator1 - numerator2) / (denominator)));
}

//compute q2 based on plannar inverse model
float compute_q2(float r,float z,float l1,float l2, float q3)
{
  float a,b_numerator,b_denominator,b;
  
  a = atan2f(z,r);
  b_numerator = l2*sinf(q3);//change it to q3
  b_denominator = l1 + l2*cosf(q3);
  b = atan2f(b_numerator, b_denominator);
  
  return(a + b +M_PI/2); //add an offset of PI/2 so that 0radient is the "x axis" and PI/2 is the y "axis"
}

//make the servos move by writing the angle
void action_equation(float q1, float q2, float q3, float q4)
{
  /*
  Serial.println("angles");
  Serial.println(q1);
  Serial.println(q2);
  Serial.println(q3);
  Serial.println(q4);
  */
  
  //Serial.println("q1");
  servo_q1.write(q1 );
  delay(1000);
  //Serial.println("q4");
  servo_q4.write(q4 );
  delay(1000);
  //Serial.println("q2");
  servo_q2.write(q2 );
  delay(1000);
  //Serial.println("q3");
  servo_q3.write(q3 );
  delay(1000);
}
void action_equation2(float q1, float q2, float q3, float q4)// write
{
  
  Serial.println("angles");
  Serial.println(q1);
  Serial.println(q2);
  Serial.println(q3);
  Serial.println(q4);

  Serial.println("with 0");
  Serial.println(q1 +q1_0);
  Serial.println(q2 +q2_0);
  Serial.println(q3 + q3_0);
  Serial.println(q4 +q4_0);
  
  
  //Serial.println("q1");
  servo_q1.write(q1 +q1_0);
  delay(2000);
  //Serial.println("q2");
  servo_q2.write(q2 +q2_0);
  delay(2000);
  //Serial.println("q3");
  servo_q3.write(q3 + q3_0);
  delay(2000);
  //Serial.println("q4");
  servo_q4.write(q4 +q4_0);
  delay(2000);
}

void pose(float *direct_model)//takes an array 
{
  float q1,q2,q3,q4;
  //take x y and z from direct model
  float x = direct_model[0];
  float y = direct_model[1];
  float z = direct_model[2] - d;//take off the d from DH parameter
  float phi = compute_phi(x); //in our case phi is always 0

  float r =(float) sqrt(sq(x) + sq(y)); //we take the hypotenus because except the first servo, all the other servo works in the plan plan

  q1 = atan2f(y,x); // comoute q1, it give the direction so that the other servo works only on the same plan
  
  //calculate r3 and z3 positions, it correspond to the (r,z) at the end of l2 arm (=at the 4th servos)
  float r3, z3;
  r3 = compute_r3(r, l3, phi);//this is position of servo4
  z3 = compute_z3(z, l3, phi);
  
  //compute q
  q3  = compute_q3(r3, z3, l1, l2);
  q2  = compute_q2(r3, z3, l1, l2, q3);
  q4 = phi - (q2+q3);

  //add offsets
  if(q2 <0)
    q2 = -q2; // the angles is negative in our case, but we want it positive ( it is negative based on the trigonometrique cercle) because we make the servo move on the opposite sens

  q3 = q3 + M_PI/2;//add pi/2 offset
  
  if(q3<0)
    q3 = 0;//if the angle is negative, we add a limit because servos can't go under 0°
 
  if(q4 <0)
      q4 = 0; //if the angle is negative, we add a limit because servos can't go under 0°
  
  //pass q from radient to angles*
  q1 = q1 * 180/M_PI;
  q2 = q2 * 180/M_PI;
  q3 = q3 * 180/M_PI;
  q4 = q4 * 180/M_PI + 80;


  action_equation(0, q2, q3, q4);// move the servos

}



// Gripper - picking and dropping 
void pick()
{
  Serial.println("picking");
  servo_gripper.write(180);// We don't know fot the moment which one of these two is the good one 
}

void drop()
{
  Serial.println("Dropping");
  servo_gripper.write(10);
}


void home_pose()
{
  Serial.println("home_pose");
  float directmodel[3];
  directmodel[0] = (l2 + l3)*0.95;
  directmodel[1] = 0;//(l1+l3+l2)*0.95;
  directmodel[2] = d+l1;//d + l1;

  drop();
  pose(directmodel);
  
}

void pose_1()//ready to pick
{
  float directmodel[3];
  Serial.println("pose 1: move and pick");
  directmodel[0] = l1+l3 ;
  directmodel[1] = 0;
  directmodel[2] = d-l2;
  
  //drop();
  pose(directmodel);
  pick();
  delay(5000);
  
}

void pose_2()//took somthing and go up
{
  float directmodel[3];
  Serial.println("pose 2: go up");
  directmodel[0] = (l1+l3+l2)*0.95;
  directmodel[1] = 0;
  directmodel[2] = d;

  pick();
  pose(directmodel);
  pick();
  delay(5000);
}

void pose_3()//move
{
  float directmodel[3];
  Serial.println("pose 3: Rotate");
  directmodel[0] = 0;
  directmodel[1] = (l2 + l3)*0.95;
  directmodel[2] = d+l1;//d + l1;

  pose(directmodel);
  delay(5000);
}

void pose_4()// move and ready to drop
{
  float directmodel[3];
  Serial.println("pose 4: drop object");
  directmodel[0] =  0;
  directmodel[1] = l1+l3;
  directmodel[2] = d-l2;
  
  pose(directmodel);
  drop();
  delay(5000);
}

void Action()
{
  home_pose();
  pose_1();
  pose_2();
  pose_3();
  pose_4();
  home_pose();
}


void setup() 
{
  Serial.begin(115200);
  // Set Servos
  servo_q1.attach(servo_pin_q1);  
  servo_q2.attach(servo_pin_q2);  
  servo_q3.attach(servo_pin_q3); 
  servo_q4.attach(servo_pin_q4);
  servo_gripper.attach(servo_pin_gripper); 
  
  pinMode(servo_pin_q1, OUTPUT);
  pinMode(servo_pin_q2, OUTPUT);
  pinMode(servo_pin_q3, OUTPUT);
  pinMode(servo_pin_q4, OUTPUT);
  pinMode(servo_pin_gripper, OUTPUT);
  
  pinMode(Button_Calibration, INPUT);
  pinMode(Button_Action, INPUT);
  
  pinMode(LED, OUTPUT);    
}


void loop() 
{
/*
    float directmodel[3];//set a array of 3 for the direct model (x,y,z)
    //save home position (init pose)
    delay(500); 
    q1_0 = servo_q1.read();
    q2_0 = servo_q2.read();
    q3_0 = servo_q3.read();
    q4_0 = servo_q4.read();

    
    delay(500); 
*/
    
    
    
    /*
    while(1)
    {
    //action();
    //delay(10000);
    Serial.println("yo");
    pick();
    delay(2000);
    drop();
    delay(2000);
    }
    */

    Action();
    /*
    while(1)
    {
      Serial.println("POSE 1");
  
      directmodel[0] = l1+l3 ;
      directmodel[1] = 0;
      directmodel[2] = d-l2;
      pose(directmodel);
      delay(5000); 
     
      Serial.println("POSE 2");
      directmodel[0] = (l2 + l3)*0.95;
      directmodel[1] = 0;//(l1+l3+l2)*0.95;
      directmodel[2] = d+l1;//d + l1;
      Serial.println(directmodel[1]);
      Serial.println(directmodel[2]);
      pose(directmodel);
      
      delay(5000); 
      Serial.println("POSE 3");
      directmodel[0] = 0;
      directmodel[1] = (l1+l3+l2)*0.95;
      directmodel[2] = d;
      pose(directmodel);
  
      
      delay(5000);    
      Serial.println("POSE 4"); 
      directmodel[0] =  0;
      directmodel[1] = l1+l3;
      directmodel[2] = d-l2;
      pose(directmodel);
      delay(5000); 
    }*/
}
