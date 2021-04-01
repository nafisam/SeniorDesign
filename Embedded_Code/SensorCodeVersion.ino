#include <SoftwareSerial.h>
#include "Definitions.h"

// Rx, Tx pings for Bluetooth
SoftwareSerial bluetoothModule(BLUETOOTH_TX, BLUETOOTH_RX);

// Should the car be enabled or disabled based on Bluetooth
bool car_enabled = true;

// Speed values
signed int ReveseSpeed = 0;
signed int ForwardSpeed = 0;

// Bluetooth inputs will come suddenly and infrequently, and we don't want to suddently jolt the car
// Instead of sending data constantly we will get a single command and repeat it so many times
int BL_ForwCount = 0;
int BL_RevCount = 0;

//Define functions
bool SensorsDetectWall(int trigPin, int echoPin);
bool BluetoothControls();
void ReadJoystick();
void SetMotorForwardSpeed(bool forward);
void SetMotorReveseSpeed(bool reverse);
void SetMotorIdle();
bool SetMotorTurning();
void SetMotorForwardIdle();
void SetMotorReverseIdle();

// variables for distance sensors
bool LeftSensor = false;
bool CenterSensor = false;
bool RightSensor = false;
bool BackSensor = false;

//variable to check if the kid moved the joystick 0 -no, 1-left, 2-right
int did_child_turn = 0;

bool forwardAllowed = true;
bool reverseAllowed = true;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    while(!Serial) { ; } // For using USB Serial

    // Setup the motor pins for output only
    pinMode(MOTOR1_PWM_CW, OUTPUT);
    pinMode(MOTOR1_PWM_CCW, OUTPUT);
    pinMode(MOTOR2_PWM_CW, OUTPUT);
    pinMode(MOTOR2_PWM_CCW, OUTPUT);
    pinMode(STEPPER_PWM_CW, OUTPUT);
    pinMode(STEPPER_PWM_CCW, OUTPUT);
   
    //Setup the Sensor echo pins as input and the triggers as output
    pinMode(SENSOR0_ECHO, INPUT);
    pinMode(SENSOR0_TRIG, OUTPUT);
    pinMode(SENSOR1_ECHO, INPUT);
    pinMode(SENSOR1_TRIG, OUTPUT);
    pinMode(SENSOR2_ECHO, INPUT);
    pinMode(SENSOR2_TRIG, OUTPUT);
    pinMode(SENSOR3_ECHO, INPUT);
    pinMode(SENSOR3_TRIG, OUTPUT);
    pinMode(SENSOR4_ECHO, INPUT);
    pinMode(SENSOR4_TRIG, OUTPUT);
    pinMode(SENSOR5_ECHO, INPUT);
    pinMode(SENSOR5_TRIG, OUTPUT);
    
    //Setup the button;
    pinMode(BUTTON, OUTPUT); // Generally, in push-button we take INPUT as a parameter but here we take OUTPUT because ANALOG PIN 

    digitalWrite(BUTTON, HIGH); // Make button condition HIGH


  
    Serial.println("App Started");
  
    //When it comes to the bluetooth this is going to be the biggest hickup
    //Depending on the module that is used this can be any baud rate
    //This was the baud rate for my Module, but their's might be 9600
    bluetoothModule.begin(9600);
}

void loop() {

    // If the car is disabled, check for input from bluetooth and loop again
    if(!car_enabled)
    {
        SetMotorIdle();
        BluetoothControls();
        delayMicroseconds(100);
        
        return;
    }
    //Check sensors, if we are approaching a wall stop
    Serial.println("we are here");
    LeftSensor = SensorsDetectWall(SENSOR0_TRIG, SENSOR0_ECHO);
    CenterSensor = SensorsDetectWall(SENSOR1_TRIG, SENSOR1_ECHO);
    RightSensor = SensorsDetectWall(SENSOR2_TRIG, SENSOR2_ECHO);
    BackSensor = SensorsDetectWall(SENSOR3_TRIG, SENSOR3_ECHO);
   
    forwardAllowed = true;
    reverseAllowed = true;
    if(LeftSensor)
    {
      forwardAllowed = false;
      SetMotorForwardIdle();
      bluetoothCmd parentData = (bluetoothCmd)bluetoothModule.read();
      did_child_turn = ReadJoystickTurn();
      if((parentData == Right) || (did_child_turn == 2)) //did the parents or the kid turn right?
      {
        forwardAllowed = true;
      }
      else if ((!CenterSensor) && (!RightSensor))
      {
        forwardAllowed = true;
      }
      else
      {
        forwardAllowed = false;
      }
    }
    if(CenterSensor)
    {
      SetMotorForwardIdle();
      bluetoothCmd parentData = (bluetoothCmd)bluetoothModule.read();
      did_child_turn = ReadJoystickTurn();
      if((parentData == Right) || (did_child_turn == 2)) //did the parents or the kid turn right?
      {
        forwardAllowed = true;
      }
      else if((!CenterSensor) && (!RightSensor))
      {
        forwardAllowed = true;
      }
      else if((parentData == Left) || (did_child_turn == 1)) //did the parents or the kid turn left?
      {
        forwardAllowed = true;
      }
      else if((!CenterSensor) && (!LeftSensor))
      {
        forwardAllowed = true;
      }
      else
      {
        forwardAllowed = false;
      }
    }
    
    if(RightSensor)
    {
      forwardAllowed = false;
      SetMotorForwardIdle();
      bluetoothCmd parentData = (bluetoothCmd)bluetoothModule.read();
      did_child_turn = ReadJoystickTurn();
      if((parentData == Left) || (did_child_turn == 1)) //did the parents or the kid turn left?
      {
        forwardAllowed = true;
      }
      else if((!CenterSensor) && (!LeftSensor))
      {
        forwardAllowed = true;
      }
      else
      {
        forwardAllowed = false;
      }
    }

    if(BackSensor)
    {
      reverseAllowed = false;
      SetMotorReverseIdle();
    }

    // Attempt to read in values from bluetooth
    BluetoothControls();
    

    if(BL_ForwCount > 0)
    {
        //Serial.println(BL_ForwCount);
        Serial.println("Bluetooth: ForwardSpeed");
        BL_ForwCount--;   
        SetMotorForwardSpeed(forwardAllowed);
    }
    else if (BL_RevCount > 0)
    {
        Serial.println("Bluetooth: ReveseSpeed");
        BL_RevCount--;
        SetMotorReveseSpeed(reverseAllowed);
    }
    // Read from joystick if nothing else has stopped us
    else
    {
        //Serial.println("Reading Joysticks");
        //ReadJoystick();
        Serial.println("Reading Button");
        ReadButton();
    }
    
    // Let everything breath for a moment
    //delayMicroseconds(50);
}


// Stop the motors if they start to detect a wall
bool SensorsDetectWall(int trigPin, int echoPin)
{
    float duration, sensorDistance;
    
    
    digitalWrite(trigPin, LOW);
    delayMicroseconds(1);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    sensorDistance = (duration * .0343)/2;
    Serial.print("Distance: ");
    Serial.println(sensorDistance);
    if (sensorDistance < Distance)
    {
        return true;
    }
    return false;

}

// Read in the bluetooth commands and set the bluetooth values as needed
bool BluetoothControls()
{
    if(bluetoothModule.available())
    {
        // Read in the command from bluetooth and process the command
        bluetoothCmd data = (bluetoothCmd)bluetoothModule.read();
        switch(data)
        {
            // For both ForwardSpeed and ReveseSpeed we are pulsing the car
            // This is in case data is not constantly coming in
            case Forward:
                BL_ForwCount = 17;
                BL_RevCount = 0;
                break;
            case Backward:
                BL_RevCount = 17;
                BL_ForwCount = 0;
                break;
            case Left:
                SetMotorTurning(LeftCmd);
                break;
            case Right:
                SetMotorTurning(RightCmd);
                break;
            case Stop:
                car_enabled = false;
                BL_ForwCount = 0;
                BL_RevCount = 0;
                break;
            case Start:
                car_enabled = true;
                BL_ForwCount = 0;
                BL_RevCount = 0;
                break;
            default:
                // Do nothing, garbage data
                break;
        }
        return true;
    }
    Serial.println(BL_ForwCount);
    return false; 
}

// Read in the joystick values and set the motor valuse as needed
void ReadJoystick()
{
    int Joystick_xPos = analogRead(JOYSTICK_XPOS);
    int Joystick_yPos = analogRead(JOYSTICK_YPOS);

    // ForwardSpeed & Reverse
    if(Joystick_yPos > JOYSTICK_HIGH_THRES)
        SetMotorForwardSpeed(forwardAllowed);
    else if(Joystick_yPos < JOYSTICK_LOW_THRES)
        SetMotorReveseSpeed(reverseAllowed);
    else
        SetMotorIdle();

    // Turning 
    if(Joystick_yPos < JOYSTICK_TURN_LEFT)
    {
      
    }
    else if(Joystick_yPos > JOYSTICK_TURN_RIGHT)
    {
      
    }
}

// Read in the joystick values to see if we turned right or left
int ReadJoystickTurn()
{
    int Joystick_xPos = analogRead(JOYSTICK_XPOS);
    int Joystick_yPos = analogRead(JOYSTICK_YPOS);

    // Turning 
    if(Joystick_yPos < JOYSTICK_TURN_LEFT)
    {
      return 1;
    }
    else if(Joystick_yPos > JOYSTICK_TURN_RIGHT)
    {
      return 2;
    }
    else
    {
      return 0;
    }
}

//read button
void ReadButton()
{
    if(digitalRead(BUTTON) == LOW)  // If button pressed
    {
      SetMotorForwardSpeed(forwardAllowed);
    }
    else
    {
        SetMotorIdle();
    }
}

// Increase the ForwardSpeed speed of both of the motors and decrease their ReveseSpeed speed
void SetMotorForwardSpeed(bool forward)
{
  if(!forward)
  {
    Serial.println("can't move forward");
  }

  else{
    Serial.println("Forwarding");
    ForwardSpeed = lim_min(MaxSpeed, ForwardSpeed+Accleration);
    ReveseSpeed = lim_max(0, ReveseSpeed-Deccleration);

    analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
    analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
    analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
    analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);
  }
}


// Increase the ReveseSpeed speed of both of the motors and decrease their ForwardSpeed speed
void SetMotorReveseSpeed(bool reverse)
{
  if(!reverse)
  {
    return;
  }
  else{
    Serial.println("Revering");
    ReveseSpeed = lim_min(MaxSpeed, ReveseSpeed+Accleration);
    ForwardSpeed = lim_max(0, ForwardSpeed-Deccleration);
    
    analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
    analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
    analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
    analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);
  }
}

// Slowly break the motors, this prevents sudden breaking
void SetMotorIdle()
{
    Serial.println("Motor idling");
    ForwardSpeed = lim_max(0, (ForwardSpeed-Deccleration));
    ReveseSpeed = lim_max(0, (ReveseSpeed-Deccleration));

    analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
    analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);   
    analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
    analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
}

// Slowly break the motor, this prevents sudden breaking
void SetMotorForwardIdle()
{
   // Serial.println("Motor idling");
   ForwardSpeed = lim_max(0, (ForwardSpeed-Deccleration));
    //ReveseSpeed = lim_max(0, (ReveseSpeed-Deccleration));

    analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
    analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);   
  //  analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
  //  analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
}

// Slowly break the motor, this prevents sudden breaking
void SetMotorReverseIdle()
{
   // Serial.println("Motor idling");
   //ForwardSpeed = lim_max(0, (ForwardSpeed-Deccleration));
    ReveseSpeed = lim_max(0, (ReveseSpeed-Deccleration));

   // analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
   // analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);   
    analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
    analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
}

// Turn steering motor either right or left
bool SetMotorTurning(turnCmd turn)
{
    switch(turn)
    {
      LeftCmd:
          // Rotate the stepper motor once Left
          return true;
      RightCmd:
          // Rotate the stepper motor once Right
          return true;
      default:
          // Garbage data, do nothing
          return false;
    }
}
