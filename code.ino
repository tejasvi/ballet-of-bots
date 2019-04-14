////////////////////////////// PREPROCESSOR ////////////////////////////////////

//PID control
#include <PID_v1.h>
//Custom size int for saving memory
#include <stdint.h>
//For analog transfer
#include <Wire.h>
//Servo library
#include <Servo.h>

//ball path mode eg. ellipse
#define MODE 0

////////////////////////////////////////////////////////////////////////////////


///////////////////////////////// VARIABLES ////////////////////////////////////

// Definitions TOUCH PINS
bool touch;
int noTouchCount = 0; //to detect ball presence
float x, y;           //touch coordinates

//increment variable for path
double k = 0;

// PID values
//Input: current state, Output: target state (more than desired
//Setpoint: desired state
double Setpoint, Input, Output;    //for X
double Setpoint1, Input1, Output1; //for Y

// servos variables
Servo servo1; //X axis
Servo servo2; //Y axis

//Raw x, y values are mapped to dimensions of screen
//using manual calibration

float convertX = 154.08 / 790.0; // converts raw x values to length
float convertY = 85.92 / 470.0;  // converts raw y values to length

//time sample (increase to reduce sensitivity)
int Ts = 0;
unsigned long Stable = 0;

//PID const (can be tweaked to improve maneuver)
//x
float Kp = 0.3;
float Ki = 0.03;
float Kd = 0.13;
//y
float Kp1 = 0.3;
float Ki1 = 0.08;
float Kd1 = 0.13;

//Initiallize PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVER);         //x
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT); //y

////////////////////////////////////////////////////////////////////////////////


///////////////////////////////// FUNCTIONS ////////////////////////////////////

/* The coordinate data has three consecutive 255 bit signal 
as seperator. For example, 255 255 255 a b m n 255 255 255
means coordinates are (255*a +b), (255b+a). */

//get x, y coordinates from screen matrix
void getxy()
{
  int serial[18];
  for (int i = 0; i < 9; i++)
  {
    int num = Serial.read();
    serial[i] = num;
  }
  for (int i = 9; i < 18; i++)
  {
    serial[i] = serial[i - 9];
  }
  for (int i = 0; i < 18; i++)
  {
    if (i > 5)
      if (serial[i] == 255)
        if (serial[i + 1] == 255)
          if (serial[i + 2] == 255)
          {
            x = serial[i - 5] * 255 + serial[i - 4];
            y = serial[i - 3] * 255 + serial[i - 2];
            touch = serial[i - 1];
          }
  }
}

//Set desired position of ball (lay path)
void setDesiredPosition()
{
  switch (MODE)
  {
  //center
  case 0:
    Setpoint = 120;
    Setpoint1 = 70;
    break;

  //Other shapes like 8, elipse, circle. More can be added.
  case 1:
    Setpoint = 85 + (50 * cos(k)) / (1 + sin(k) * sin(k));
    Setpoint1 = 55 + (50 * sin(k) * cos(k)) / (1 + sin(k) * sin(k));
    k = k + 0.008;
    break;
  case 2:
    Setpoint = 85 + 25 * cos(k);
    Setpoint1 = 55 + 25 * sin(k);
    k = k - 0.02;
    break;
  case 3:
    Setpoint = 85 + 40 * cos(k);
    Setpoint1 = 55 + 25 * sin(k);
    k = k - 0.02;
    break;
  case 4:
    Setpoint = 85 + 18 * cos(k) + 12 * cos(k * 150);  //
    Setpoint1 = 55 + 18 * sin(k) - 12 * sin(k * 150); //
    k = k + 0.01;
  }
}

////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////// CONFIGURE //////////////////////////////////

//Seting up controller pins
void setup()
{
  pinMode(0, INPUT);
  servo1.attach(5);
  servo2.attach(6);
  //set servos to make screen flat
  Output = 60;
  Output1 = 4;
  servo1.write(Output);
  servo2.write(Output1);

  //begin serial with 9600 baud rate to match with display
  Serial.begin(9600);

  //INIT OF TOUSCHSCREEN
  getxy();
  Input = 120; //10
  Input1 = 65; //3
  //INIT SETPOINT
  Setpoint = 120;
  Setpoint1 = 65;

  //Zapnutie PID
  //limit max tilt of screen
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(20, 160);
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(20, 160);
  // TIME SAMPLE
  myPID1.SetSampleTime(Ts);
  myPID.SetSampleTime(Ts);

  //slight delay to let screen boot
  delay(100);
}

////////////////////////////////////////////////////////////////////////////////


/////////////////////////////// CONTROL LOOP ///////////////////////////////////

void loop()
{
  while (Stable < 125) //REGULATION LOOP
  {
    int oldx = x, oldy = y;
    getxy();

    //Check ball is on plate (since screen sends last touch coordinates)
    if ((x != oldx) || (y != oldy)) 
     {
      servo1.attach(5); //connect servos
      servo2.attach(6);
      setDesiredPosition();
      noTouchCount = 0;
      getxy();                 // measure actual position
      Input = (x * convertX);  // read and convert X coordinate
      Input1 = (y * convertY); // read and convert Y coordinate

      //Quantifying ball stability from coordinate variations
      if ((Input > Setpoint - 2 && Input < Setpoint + 2 && Input1 > Setpoint1 -\
      2 && Input1 < Setpoint1 + 2)) //if ball is close to setpoint
      {
        Stable = Stable + 1; //increment STABLE
      }

      myPID.Compute();  //action control X compute
      myPID1.Compute(); //   action control  Y compute
    }
    else //if there is no ball on plate
    {
      noTouchCount++; //increment no touch count

      //If coordinates remain constant for long
      if (noTouchCount == 75)
      {
        noTouchCount++;
        Output = 60; //make plate flat
        Output1 = 4;
        servo1.write(Output);
        servo2.write(Output1);
      }
    }
    servo1.write(Output);  //control
    servo2.write(Output1); //control

    //Output current coordinates to serial
    Serial.print(Setpoint);
    Serial.print(",");
    Serial.print(Setpoint1);
    Serial.print(",");
    Serial.print(Input);
    Serial.print(",");
    Serial.println(Input1);

  } ////END OF REGULATION LOOP///

  /*   servo1.detach(); //detach servos
  servo2.detach(); */

  ///control STABILITY////
  while (Stable == 125) //if is stable
  {
    //still measure actual postiion
    setDesiredPosition();
    getxy();
    Input = (x * convertX); //read X
    Input1 = (y * convertY); //read Y
    if (Input < Setpoint - 2 || Input > Setpoint + 2 || Input1 > (Setpoint1 + 2)/
    || Input1 < Setpoint1 - 2) //if ball isnt close to setpoint
    {
      Stable = 0; //change STABLE state
    }

  } //end of STABLE LOOP
} //loop end

////////////////////////////////////////////////////////////////////////////////


/*///////////////////////////////////////////////////////////////////////////////
Limitaions: Coordinates don't update if screen contact is maintained
throughout motion of the ball. This leads to slight loss in aaccuracy.
Current solution is to reverse engineer the screen firmware more deeply
to get updated coordinates. 
Add-ons: Control ball position using joystick (nunchuck.h)
//////////////////////////////////////////////////////////////////////////////*/
