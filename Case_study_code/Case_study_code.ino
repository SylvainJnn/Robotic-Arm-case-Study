#include <Servo.h>

Servo servo_alpha, servo_beta, servo_gama; // create servo object to control a servo

const int potpin_alpha = A0;  // analog pin used to connect the potentiometer
const int potpin_beta  = A1;  // analog pin used to connect the potentiometer
const int potpin_gama  = A2;  // analog pin used to connect the potentiometer
                   
const int servo_pin_alpha = 2;
const int servo_pin_beta  = 3;
const int servo_pin_gama  = 4;

const int LED = 1;
const int Button_Calibration = 12; // ASK for Switch --> easier to implemant
const int Button_Action = 13;

int alpha_0, beta_0, gama_0; // Home angle positions

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
  // Set Servos
  servo_alpha.attach(servo_pin_alpha);  
  servo_beta.attach(servo_pin_beta);  
  servo_gama.attach(servo_pin_gama); 
  
  
  pinMode(servo_pin_alpha, INPUT);
  pinMode(servo_pin_beta, INPUT);
  pinMode(servo_pin_gama, INPUT);
  
  pinMode(Button_Calibration, INPUT);
  pinMode(Button_Action, INPUT);
  
  pinMode(LED, OUTPUT);    
}


void loop() 
{

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

  delay(1000);                           // waits for the servo to get there
}