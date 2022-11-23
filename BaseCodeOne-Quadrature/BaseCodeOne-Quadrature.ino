int b = 0; //reading the time for main loop to be run for 15s
int c = 0; //memory for the time in mainloop

float s = 0; //built-in encoder counts
float s_2;   //built-in encoder counts for RPM calculation for PI controler

float rpmm; //rpm obtained each 5s from built-in encoder

int s1 = 0;         //built-in encoder chanel one outpot
int s2 = 0;         //built-in encoder chanel two outpot
int r = 0;          //repetition indicator for reading counts of bult-in encoder
int s2m = 0;        //memory of built-in encoder chanel two outpot
int directionm = 0; //indicator for direction read by built-in encoder
int dirm;           //indicator for direction read by built-in encode
int RPM;            //Commanded RPM

int exitt = 0; //mainloop exit condition

float ctrl;     //PI controller outpot
float kp = .4;  //proportional gain of PI controller
float ki = .01; //integral gain of PI controller
float eri;      //integral of error of PI controller

int repc = 1;   //repetition condition of PI controller
int t0;         //memory of time for the Purpose of displaying the results
int repeat = 0; //repeat indicator to only let the memory of time for the Purpose of displaying the results be updated once

/* 
 *  Added variables
 */
#define ADCONV 5.0 / 1024.0
#define TOTAL_SEG 32         // Number of segments in outer row of disk
#define DPE 360.0 / (TOTAL_SEG) // Disk resolution in dpe

const int ChannelAPin = A3;
const int ChannelBPin = A4;

const float ChannelAHigh = 1.5;
const float ChannelALow = 1;

const float ChannelBHigh = 1.5;
const float ChannelBLow = 1;

bool hasInterrupted = false;
bool isClockwise;
bool dirKnown = false;

int numEdges = 0;

float quadRPM;

int prevTime;
#define MINTIME 0

void setup()
{
    // put your setup code here, to run on
    Serial.begin(250000); //Baud rate of communication

    Serial.println("Enter the desired RPM.");

    while (Serial.available() == 0)
    {
        //Wait for user input
    }

    RPM = Serial.readString().toFloat(); //Reading the Input string from Serial port.
    if (RPM < 0)
    {
        analogWrite(3, 255); //changing the direction of motor's rotation
    }
    RPM = abs(RPM);

    pinMode(ChannelAPin, INPUT);
    pinMode(ChannelBPin, INPUT);

    //attachInterrupt(digitalPinToInterrupt(photoPin1), DETECTEDGE, RISING);
}

int wasHigh = false;

void DETECTEDGE()
{
    float ChannelAVoltage = analogRead(ChannelAPin) * ADCONV;
    if (ChannelAVoltage >= ChannelAHigh && !wasHigh){
      numEdges++;
      wasHigh = true;
      isClockwise = analogRead(ChannelBPin)*ADCONV >= ChannelBHigh ? 1 : 0;
    }else if (ChannelAVoltage <= ChannelALow && wasHigh){
      numEdges++;
      wasHigh = false;
      isClockwise = analogRead(ChannelBPin)*ADCONV >= ChannelBHigh ? 0 : 1;
    }
    
}

//#define COEFF1 0.000086514
//#define COEFF2 -0.0271
//#define COEFF3 0.5665

#define COEFF11 0.000071456
#define COEFF12 -0.0213
#define COEFF13 0.3094

#define COEFF21 -0.000015068
#define COEFF22 0.0058
#define COEFF23 -0.2603

double AdjustError(float RPMIn, float Coeff1, float Coeff2, float Coeff3){
  return (Coeff1 * (RPMIn*RPMIn) + Coeff2 * RPMIn + Coeff3);
}

void loop()
{

    b = millis(); //reading time
    c = b;        //storing the current time

    while ((b >= c) && (b <= (c + 15500)) && exitt == 0) //let the main loop to be run for 15s
    {
        DETECTEDGE();
        if (b % 13 == 0 && repc == 1) //PI controller
        {
            eri = ki * (RPM - rpmm) + eri;
            ctrl = 50 + kp * (RPM - rpmm) + eri;
            analogWrite(6, ctrl);
            repc = 0;
        }
        if (b % 13 == 1)
        {
            repc = 1;
        }

        s1 = digitalRead(7); //reading Chanel 1 of builtin encoder
        s2 = digitalRead(8); //reading Chanel 2 of builtin encoder
        if (s1 != s2 && r == 0)
        {
            s = s + 1;     //counters for rpm that displyed every 5s
            s_2 = s_2 + 1; //counters for rpm that used in PI contoller
            r = 1;         // this indicator wont let this condition, (s1 != s2), to be counted until the next condition, (s1 == s2), happens
        }

        if (s1 == s2 && r == 1)
        {
            s = s + 1;     //counters for rpm that displyed every 5s
            s_2 = s_2 + 1; //counters for rpm that used in PI contoller
            r = 0;         // this indicator wont let this condition, (sm1 == sm2), to be counted until the next condition, (sm1 != sm2), happens
        }

        b = millis(); //updating time
        if (b % 100 <= 1 && repeat == 0)
        {
            t0 = b; //storing the current time once
            repeat = 1;
        }

        if (b % 100 == 0)
        {
            Serial.print("time in ms: ");
            Serial.print(b - t0);

            Serial.print("  spontaneous speed from builtin encoder:  ");
            rpmm = (s_2 / (2 * 114)) * 600; //formulation for rpm in each 100ms for PI controller
            Serial.println(rpmm);
            s_2 = 0; //reseting the counters of PI controller rpm meter

//            Serial.println(numEdges);

            if ((b - t0) % 5000 == 0)
            {
                Serial.println();
                Serial.print("RPM from builtin encoder: ");
                Serial.println((s / (228)) * 12); //formula for rpm in each 5s

                Serial.print("RPM from optical quadrature encoder: ");
                quadRPM = (DPE * numEdges) / (5.0*6);
                
                Serial.println(quadRPM);
                
                Serial.print("RPM from optical quadrature encoder (linearised): ");
                float quadRPMLin = quadRPM - AdjustError(quadRPM, COEFF11, COEFF12, COEFF13);
                float quadRPMLin2 = quadRPMLin - AdjustError(quadRPMLin, COEFF21, COEFF22, COEFF23);
                Serial.println(quadRPMLin);

                Serial.print("Counts: ");
                Serial.println(numEdges);

                Serial.print("Error: ");
                Serial.println(quadRPM - (s / (228)) * 12);

                Serial.print("Error (linearised): ");
                Serial.println(quadRPMLin2 - (s / (228)) * 12);

                Serial.print("direction read by motor's sensor: ");
                if (dirm == 0)
                {
                    Serial.print("CW");
                }
                else
                {
                    Serial.print("CCW");
                }
                Serial.print("  ,   ");

                Serial.print("direction read by sensor:  ");
                Serial.println(isClockwise ? "CW" : "CCW");

                s = 0;
                numEdges = 0;
                directionm = 0;
            }
            delay(1);
        }

        if ((s1 == HIGH) && (s2 == HIGH) && (s2m == LOW)) //reading the direction of motor by cheaking which chanel follows which
        {
            directionm = directionm + 1;
        }

        if ((s1 == LOW) && (s2 == LOW) && (s2m == HIGH))
        {
            directionm = directionm + 1;
        }

        s2m = s2; //memory of the previous builtin encoder chanel 2

        if (directionm > 100)
        {
            dirm = 0;
        }
        if (directionm < 20)
        {
            dirm = 1;
        }

        b = millis(); //updating time
    }
    analogWrite(6, 0); //turning off the motor
    exitt = 1;         //changing the exit condition to prevent the motor to run after 15s
}
