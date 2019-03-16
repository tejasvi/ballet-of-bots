///Libraries///
#include <PID_v1.h>
#include <stdint.h>
// #include "TouchScreen.h"
//#include <SPI.h>
#include <Wire.h>
//#include <wiinunchuk.h>
#include <Servo.h>

#define MODE 0

// Definitions TOUCH PINS
/* #define YP A0 //0
  #define XM A1 //1
  #define YM 3  //3
  #define XP 4  //4
  TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
  int buttonPushCounter = 1; // counter for the number of button presses
  int lastButtonState = 0;   // previous state of the button */
// int flag, flagZ;
bool touch;
int noTouchCount=0;
float x, y;
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
// int cCount = 0;
// int flagC = 0;
// int flagK = 0;
// float kk = 0;
// int fl = 0;
// double l = 0.00;
// unsigned int noTouchCount = 0; //viariable for noTouch
double k = 0;

// PID values
double Setpoint, Input, Output;    //for X
double Setpoint1, Input1, Output1; //for Y

// int Modulo;
// long lastcas = 0;

// servos variables
Servo servo1; //X axis
Servo servo2; //Y axis
/*
  uint16_t homeX = 550; // raw data value for center of touchscreen
  uint16_t homeY = 550; // raw data value for center of touchscreen */

float convertX = 15.408 / 790.0; // converts raw x values to mm. found through manual calibration
float convertY = 8.592 / 470.0;  // converts raw y values to mm. found through manual calibration

/////TIME SAMPLE
int Ts = 50;
unsigned long Stable = 0;

//PID const
float Kp = 0.3;
float Ki = 0.03;
float Kd = 0.13;

float Kp1 = 0.3;
float Ki1 = 0.08;
float Kd1 = 0.13;
// long cas = 0;

//INIT PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

void setup()
{
  pinMode(0, INPUT);
  servo1.attach(3);
  servo2.attach(4);
  Output = 90;
  Output1 = 0;
  servo1.write(Output);
  servo2.write(Output1);

  /*   //init NUN
    nunchuk_setpowerpins();
    nunchuk_init();
    nunchuk_get_data(); */

  //INIT PINS
  /*     pinMode(9, OUTPUT);
    pinMode(8, OUTPUT);
    digitalWrite(9, LOW); //LED INIT
    digitalWrite(8, LOW); */

  Serial.begin(9600);

  //INIT OF TOUSCHSCREEN
  getxy();
  Input = 120;
  Input1 = 65;
  //INIT SETPOINT
  Setpoint = 395;
  Setpoint1 = 235;
  //// Make plate flat
  /*  servo1.attach(5);
    servo2.attach(6);
    Output = 95;
    Output1 = 95;
    servo1.write(Output);
    servo2.write(Output1); */

  //Zapnutie PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(80, 100);
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-10, 10);
  // TIME SAMPLE
  myPID1.SetSampleTime(Ts);
  myPID.SetSampleTime(Ts);

  delay(100);
}

void loop()
{
  while (Stable < 125) //REGULATION LOOP
  {
    int oldx = x, oldy = y;
    getxy();
    if ((x != oldx) || (y != oldy)) //ball is on plate
    {
      servo1.attach(5); //connect servos
      servo2.attach(6);
      setDesiredPosition();
      noTouchCount = 0;
      getxy(); // measure actual position
      Input = (x * convertX);    // read and convert X coordinate
      Input1 = (y * convertY);   // read and convert Y coordinate

      if ((Input > Setpoint - 2 && Input < Setpoint + 2 && Input1 > Setpoint1 - 2 && Input1 < Setpoint1 + 2)) //if ball is close to setpoint
      {
        Stable = Stable + 1; //increment STABLE
        // digitalWrite(9, HIGH);
      }
      /* else
        {
          digitalWrite(9, LOW);
        } */
      myPID.Compute();  //action control X compute
      myPID1.Compute(); //   action control  Y compute
    }
    else //if there is no ball on plate
    {
      noTouchCount++; //increment no touch count

      if (noTouchCount == 75)
      {
        noTouchCount++;
        Output = 90; //make plate flat
        Output = 0;
        servo1.write(Output);
        servo2.write(Output1);
      }
      /* if (noTouchCount == 150) //if there is no ball on plate longer
        {
          servo1.detach(); //detach servos
          servo2.detach();
        } */
    }
    servo1.write(Output);  //control
    servo2.write(Output1); //control
    Serial.print(Setpoint);
    Serial.print(",");
    Serial.print(Setpoint1);
    Serial.print(",");
    Serial.print(Input);
    Serial.print(",");
    Serial.println(Input1);

  } ////END OF REGULATION LOOP///

  servo1.detach(); //detach servos
  servo2.detach();

  ///control STABILITY////
  while (Stable == 125) //if is stable
  { //still measure actual postiion
    setDesiredPosition();
    getxy();                                                                            //alternative
    Input = (x * convertX);                                                                               //read X
    Input1 = (y * convertY);                                                                              //read Y
    if (Input < Setpoint - 2 || Input > Setpoint + 2 || Input1 > Setpoint1 + 2 || Input1 < Setpoint1 - 2) //if ball isnt close to setpoint
    {
      servo1.attach(5); //again attach servos
      servo2.attach(6);
      // digitalWrite(9, LOW);
      Stable = 0; //change STABLE state
    }

  } //end of STABLE LOOP
} //loop end

////////////////////////Functions//////////////////
///// DESIRED POSITION
void setDesiredPosition()
{

  // nunchuk_get_data();
  //if zbutton is pressed, zero positions

  /* int c = nunchuk_zbutton();
    if (c != lastButtonState)
    {
      // if the state has changed, increment the counter
      if (c == HIGH && digitalRead(11) == 0)
      {
          // if the current state is HIGH then the button
          // wend from off to on:
          buttonPushCounter++;
      }
    }
    lastButtonState = c; */

  /* switch (buttonPushCounter)
    {
    case 1:
      Setpoint = 120;
      Setpoint1 = 70;
      fl = 1;
      break;
    case 2:
      Setpoint = 52;
      Setpoint1 = 70;
      fl = 2;
      break;
    case 3:
      Setpoint = 52;
      Setpoint1 = 40;
      fl = 3;
      break;
    case 4:
      Setpoint = 120;
      Setpoint1 = 40;
      buttonPushCounter = 0;
      fl = 4;
      break;
    } */

  switch (MODE)
  {
    case 0:
      Setpoint = 395;
      Setpoint1 = 235;
      break;
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
