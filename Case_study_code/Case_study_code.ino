#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int potpin = A0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

const int servo_pin = 2;
const int LED = 1;
const int Button_Calibration = 12; // ASK for Switch --> easier to implemant
const int Button_Action = 13;

int alpha_0;

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
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
 
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);
  
}

void Action() //for the moment, let's just check if it is working properly
{
  myservo.write(alpha_0 - 35);
  delay(500);

  myservo.write(alpha_0 - 45);
  delay(500);

  myservo.write(alpha_0 + 35);
  delay(500);

  myservo.write(alpha_0 +45);
  delay(500);
}



void setup() 
{
  myservo.attach(servo_pin);  // attaches the servo on pin 9 to the servo object
  
  pinMode(Button_Calibration, INPUT);
  pinMode(Button_Action, INPUT);
  
  pinMode(LED, OUTPUT);    
}


void loop() 
{
  bool Button_Calibration_state, Button_Calibration_action;
  
  Button_Calibration_state = Read_button(Button_Calibration);
  //if else or while ? 
  while(Button_Calibration_state)         //Pressed the button for 2.5seconds -> calibration starts. Repressed this button to end the loop
  {
    Button_Calibration_state = LOW;       //Once the button is pressed we need to set it low. The idea is: we calibrate if user press the button, if he repressed the button it means the calibration is over. So once we enter the calibration we set ti to low, we re read the button at the end of the while loops to see if the calibration continues or not.
    digitalWrite(LED, HIGH);              //turn on LED to show that calibration is running
    Calibration();
    Button_Calibration_state = Read_button(Button_Calibration);

    alpha_0 = myservo.read();
    
  }
  digitalWrite(LED, LOW);                     //turn off LED to show that calibration is over
  Button_Calibration_action = Read_button(Button_Action);

  if(Button_Calibration_action)
  {
    Action();
  }


  delay(1000);                           // waits for the servo to get there
}
