float deg = 45; // Rotation degree
float s = 0;    //Encoder counts
int sm1 = 0;    //Built-in chanel 1
int sm2 = 0;    //Built-in chanel 2
int r = 0;      //indicator for reading builtin encoder to avoid the reading redundancy
float er;       //Proportional error for PI controller
float eri;      //Integral error for PI controller

int t = 0;  //time in ms
int t0 = 0; //memory for time in ms

int finish = 0; //finish indicator
int rep = 1;    //Repetition indicator

#define TOTAL_SEG 32   

const int photoPin1 = 2;
const int photoPin2 = 11;
const int photoPin3 = 12;
const int photoPin4 = 9;
const int photoPin5 = 10;

void setup()
{

    Serial.begin(250000); //Baud rate of communication

    Serial.println("Enter the desired rotation in degree.");

    while (Serial.available() == 0) //Obtaining data from user
    {
        //Wait for user input
    }

    deg = Serial.readString().toFloat(); //Reading the Input string from Serial port.
    if (deg < 0)
    {
        analogWrite(3, 255); //change the direction of rotation by applying voltage to pin 3 of arduino
    }
    deg = abs(deg);

    pinMode(photoPin1, INPUT);
    pinMode(photoPin2, INPUT);
    pinMode(photoPin3, INPUT);
    pinMode(photoPin4, INPUT);
    pinMode(photoPin5, INPUT);
}

float kp = .6 * 90 / deg; //proportional gain of PI
float ki = .02;           //integral gain of PI




float initialAngle = 0;
int GreyCode[5];

String LookupTable[32][2] ={{"00000", "00000"} ,
                            {"00001", "00001"},
                            {"00010", "00011"},
                            {"00011", "00010"},
                            {"00100", "00110"},
                            {"00101", "00111"},
                            {"00110", "00101"},
                            {"00111", "00100"},
                            {"01000", "01100"},
                            {"01001", "01101"},
                            {"01010", "01111"},
                            {"01011", "01110"},
                            {"01100", "01010"},
                            {"01101", "01011"},
                            {"01110", "01001"},
                            {"01111", "01000"},
                            {"10000", "11000"},
                            {"10001", "11001"},
                            {"10010", "11011"},
                            {"10011", "11010"},
                            {"10100", "11110"},
                            {"10101", "11111"},
                            {"10110", "11101"},
                            {"10111", "11100"},
                            {"11000", "10100"},
                            {"11001", "10101"},
                            {"11010", "10111"},
                            {"11011", "10110"},
                            {"11100", "10010"},
                            {"11101", "10011"},
                            {"11110", "10001"},
                            {"11111", "10000"}};

void CheckAngle(){
  GreyCode[0] = digitalRead(photoPin1);
  GreyCode[1] = digitalRead(photoPin2);
  GreyCode[2] = digitalRead(photoPin3);
  GreyCode[3] = digitalRead(photoPin4);
  GreyCode[4] = digitalRead(photoPin5);
}

void PrintLookupTable(){
  for (int i = 0; i < TOTAL_SEG; i++){
    Serial.print(LookupTable[i][0]);
    Serial.print("      ");
    Serial.println(LookupTable[i][1]);
  }
}


float FindAngle(){
   String greyCode = String(GreyCode[0]) + String(GreyCode[1]) + String(GreyCode[2]) + String(GreyCode[3]) + String(GreyCode[4]); 
   float decimalValue = 0;
   for (int i = 0; i < TOTAL_SEG; i++){
    if (LookupTable[i][1] == greyCode){
      decimalValue += 11.25 * (LookupTable[i][0].charAt(0) - '0');
      decimalValue += 22.5 * (LookupTable[i][0].charAt(1) - '0');
      decimalValue += 45 * (LookupTable[i][0].charAt(2) - '0');
      decimalValue += 90 * (LookupTable[i][0].charAt(3) - '0');
      decimalValue += 180 * (LookupTable[i][0].charAt(4) - '0');
      return decimalValue;
    }
   }
}

void loop()
{
    // put your main code here, to run repeatedly:
    CheckAngle();
    initialAngle = FindAngle();    

    t = millis();                      //reading time
    t0 = t;                            //sving the current time in memory
    while (t < t0 + 4000 && rep <= 10) //let the code to ran for 4 seconds each with repetitions of 10
    {
        
        
        if (t % 10 == 0) //PI controller that runs every 10ms
        {
            if (s < deg * 114 * 2 / 360)
            {
                er = deg - s * 360 / 228;
                eri = eri + er;
                analogWrite(6, kp * er + ki * eri);
            }

            if (s >= deg * 228 / 360)
            {
                analogWrite(6, 0);
                eri = 0;
            }
            delay(1);
        }

        sm1 = digitalRead(7); //reading chanel 1
        sm2 = digitalRead(8); //reading chanel 2

        if (sm1 != sm2 && r == 0)
        { //counting the number changes for both chanels
            s = s + 1;
            r = 1; // this indicator wont let this condition, (sm1 != sm2), to be counted until the next condition, (sm1 == sm2), happens
        }
        if (sm1 == sm2 && r == 1)
        {
            s = s + 1;
            r = 0; // this indicator wont let this condition, (sm1 == sm2), to be counted until the next condition, (sm1 != sm2), happens
        }

        t = millis(); //updating time
        finish = 1;   //cghanging finish indicator
    }

    if (finish == 1)
    {                  //this part of the code is for displaying the result

        
      
        delay(500);    //half second delay

        CheckAngle();
        float currentAngle = FindAngle();
        float totalAngle = currentAngle - initialAngle;

        if (currentAngle < initialAngle){
          totalAngle += 360;
        }
        
        rep = rep + 1; // increasing the repetition indicator
        Serial.print("shaft possition from optical absolute sensor from home position: ");
        Serial.println(initialAngle);

        Serial.print("shaft displacement from optical absolute sensor (current): ");
        Serial.println(currentAngle);

        Serial.print("shaft displacement from optical absolute sensor: ");
        Serial.println(totalAngle);

        Serial.print("Shaft displacement from motor's builtin encoder: ");
        Serial.println(s * 360 / 228); //every full Revolution of the shaft is associated with 228 counts of builtin
                                       //encoder so to turn it to degre we can use this formula (s * 360 / 228), "s" is the number of  built-in encoder counts

        float Error = totalAngle - s * 360 / 228;
        Serial.print("Error :");
        Serial.println(Error); //displaying error
        Serial.println();
        s = 0;
        finish = 0;
    }
    analogWrite(6, 0); //turning off the motor
}
